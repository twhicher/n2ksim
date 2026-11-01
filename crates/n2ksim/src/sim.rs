//! The whole simulation: world, devices, the bus between them, and the
//! analyser listening to it. Time advances in 1 ms steps; physics runs at
//! 100 Hz; frames are timed to the bit.

use crate::analyser::Analyser;
use crate::bus::frame_bits;
use crate::checks::{self, Check};
use crate::device::{DevState, Device, Origin, TxFrame};
use crate::network::{DeviceKind, Network};
use crate::power::{voltages, Load, LEN_AMPS, MIN_VOLTS};
use crate::rng::Rng;
use crate::world::{EnvCfg, HelmCfg, World};
use n2k::Frame;
use std::collections::VecDeque;

pub const DT: f64 = 0.001;
const PHYS_DT: f64 = 0.01;
const RING: usize = 200_000;

#[derive(Clone, Debug)]
pub struct FrameRec {
    pub seq: u64,
    /// End of the frame on the wire, simulation seconds.
    pub t: f64,
    pub frame: Frame,
    pub origin: Origin,
    /// Destroyed by a bus error; the sender will try again.
    pub error: bool,
    /// The gateway was on the bus, so its TCP clients saw this frame.
    pub gw: bool,
}

#[derive(Clone, Debug, Default, serde::Serialize)]
pub struct HistPoint {
    pub t: f64,
    pub load: f64,
    pub fps: f64,
    pub eps: f64,
}

pub struct Replay {
    pub name: String,
    pub frames: Vec<(f64, Frame)>,
    pub idx: usize,
    pub start_t: f64,
    pub mute: bool,
}

pub struct Sim {
    pub net: Network,
    pub devices: Vec<Device>,
    pub world: World,
    pub analyser: Analyser,
    pub t: f64,
    pub paused: bool,
    pub speed: f64,
    /// Simulated time owed to the wall clock, for whoever runs the clock.
    pub owed: f64,
    rng: Rng,
    pub bus_free_at: f64,
    busy: VecDeque<(f64, f64)>,
    pub ring: VecDeque<FrameRec>,
    pub frame_seq: u64,
    pub console: VecDeque<TxFrame>,
    replay_q: VecDeque<TxFrame>,
    /// Frames from TCP clients waiting to go into the gateway's queue.
    pub inbox: Vec<(u32, Frame)>,
    pub client_dropped: u64,
    pub clients: u32,
    pub replay: Option<Replay>,
    pub frames_total: u64,
    pub errors_total: u64,
    pub history: VecDeque<HistPoint>,
    sec_start: f64,
    sec_frames: u64,
    sec_errors: u64,
    pub checks: Vec<Check>,
    /// The backbone as it was before a wiring drill, for "Clear faults".
    pub pre_drill_backbone: Option<crate::network::Backbone>,
    phys_acc: f64,
    next_power_t: f64,
    next_check_t: f64,
    seed: u64,
}

impl Sim {
    pub fn new(net: Network, env: EnvCfg, helm: HelmCfg) -> Self {
        let seed = env.seed;
        let mut sim = Sim {
            devices: Vec::new(),
            world: World::new(env, helm),
            analyser: Analyser::default(),
            t: 0.0,
            paused: false,
            speed: 1.0,
            owed: 0.0,
            rng: Rng::new(seed.wrapping_mul(31)),
            bus_free_at: 0.0,
            busy: VecDeque::new(),
            ring: VecDeque::new(),
            frame_seq: 0,
            console: VecDeque::new(),
            replay_q: VecDeque::new(),
            inbox: Vec::new(),
            client_dropped: 0,
            clients: 0,
            replay: None,
            frames_total: 0,
            errors_total: 0,
            history: VecDeque::new(),
            sec_start: 0.0,
            sec_frames: 0,
            sec_errors: 0,
            checks: Vec::new(),
            pre_drill_backbone: None,
            phys_acc: 0.0,
            next_power_t: 0.0,
            next_check_t: 1.0,
            seed,
            net: Network::default(),
        };
        sim.set_network(net);
        sim
    }

    /// Apply a new network configuration, keeping the running state of
    /// devices whose identity has not changed.
    pub fn set_network(&mut self, net: Network) {
        let mut old: Vec<Device> = std::mem::take(&mut self.devices);
        for (i, cfg) in net.devices.iter().enumerate() {
            let dev = match old.iter().position(|d| d.cfg.id == cfg.id) {
                Some(j) => {
                    let mut d = old.swap_remove(j);
                    let identity_changed = d.cfg.preferred_address != cfg.preferred_address || d.cfg.name != cfg.name || d.cfg.kind != cfg.kind;
                    if identity_changed {
                        // A new address set by the installer replaces the stored one.
                        d.stored_address = None;
                    }
                    let pgns_changed = d.cfg.pgns.len() != cfg.pgns.len();
                    d.cfg = cfg.clone();
                    d.index = i;
                    for f in d.tx.iter_mut() {
                        if let Origin::Device(_) = f.origin {
                            f.origin = Origin::Device(i);
                        }
                    }
                    if (identity_changed && d.state.powered()) || pgns_changed {
                        d.boot(self.t);
                    }
                    d
                }
                None => Device::new(cfg.clone(), i, self.seed.wrapping_add(i as u64 * 7919)),
            };
            self.devices.push(dev);
        }
        self.net = net;
        self.next_power_t = self.t;
        self.update_power();
    }

    pub fn gateway_index(&self) -> Option<usize> {
        self.devices.iter().position(|d| d.cfg.kind == DeviceKind::Gateway && d.cfg.enabled)
    }

    pub fn gateway_up(&self) -> bool {
        self.gateway_index().is_some_and(|i| self.devices[i].state.on_bus())
    }

    fn update_power(&mut self) {
        let loads: Vec<Load> = self
            .devices
            .iter()
            .map(|d| Load {
                tee_m: d.cfg.tee_m,
                drop_m: d.cfg.drop_m,
                amps: if d.cfg.enabled && d.state.powered() { d.cfg.len as f64 * LEN_AMPS } else { 0.0 },
            })
            .collect();
        let v = voltages(&self.net.backbone, &loads);
        let t = self.t;
        for (d, v) in self.devices.iter_mut().zip(v) {
            d.voltage = v;
            if !d.cfg.enabled {
                if d.state != DevState::Off {
                    d.power_off();
                }
            } else if v < MIN_VOLTS {
                if d.state.powered() || d.state == DevState::Off {
                    d.state = if v <= 0.0 { DevState::Off } else { DevState::Brownout { until: t + 2.0 } };
                    d.tx.clear();
                }
            } else if d.state == DevState::Off {
                d.boot(t);
            }
        }
    }

    /// Bus errors caused by the wiring. Illustrative figures: a missing
    /// terminator makes reflections, both missing makes the bus unusable.
    pub fn wiring_error_prob(&self) -> f64 {
        let bb = &self.net.backbone;
        let terms = bb.term_start as u8 + bb.term_end as u8;
        let mut p = match terms {
            2 => 0.0,
            1 => 0.02,
            _ => 0.4,
        };
        let over = bb.length_m / bb.cable.max_backbone_m();
        if over > 1.0 {
            p += 0.5 * (over - 1.0);
        }
        for d in &self.devices {
            if d.cfg.enabled && d.cfg.drop_m > 6.0 {
                p += 0.002 * (d.cfg.drop_m - 6.0);
            }
        }
        p.min(0.9)
    }

    /// Bus load over the last second, 0 to 1.
    pub fn bus_load(&self) -> f64 {
        let from = self.t - 1.0;
        let busy: f64 = self.busy.iter().map(|&(s, d)| (s + d).min(self.t) - s.max(from)).filter(|x| *x > 0.0).sum();
        busy.min(1.0)
    }

    pub fn start_replay(&mut self, name: String, frames: Vec<(f64, Frame)>, mute: bool) {
        self.replay = Some(Replay { name, frames, idx: 0, start_t: self.t, mute });
    }

    /// Advance by one millisecond.
    pub fn step(&mut self) {
        self.t += DT;
        let t = self.t;

        self.phys_acc += DT;
        while self.phys_acc >= PHYS_DT - 1e-9 {
            self.phys_acc -= PHYS_DT;
            if let Some(d) = self.devices.iter().find(|d| d.cfg.kind == DeviceKind::Rudder && d.cfg.library.is_empty() && d.cfg.enabled) {
                self.world.external_order = d.last_order.map(|(at, a, _)| (at, a));
            }
            self.world.step(PHYS_DT);
        }

        if t >= self.next_power_t {
            self.next_power_t = t + 0.1;
            self.update_power();
        }

        // Frames typed by TCP clients go out through the gateway.
        let inbox = std::mem::take(&mut self.inbox);
        match self.gateway_index() {
            Some(g) if self.devices[g].state.on_bus() => {
                for (c, f) in inbox {
                    self.devices[g].push(TxFrame { frame: f, ready_t: t, origin: Origin::Client(c) });
                }
            }
            _ => self.client_dropped += inbox.len() as u64,
        }

        let mut mute = false;
        if let Some(r) = &mut self.replay {
            mute = r.mute;
            while r.idx < r.frames.len() && r.start_t + r.frames[r.idx].0 <= t {
                self.replay_q.push_back(TxFrame { frame: r.frames[r.idx].1, ready_t: t, origin: Origin::Replay });
                r.idx += 1;
            }
            if r.idx >= r.frames.len() && self.replay_q.is_empty() {
                self.replay = None;
            }
        }

        for d in &mut self.devices {
            d.tick(t, &self.world, mute);
        }
        self.advance_bus(t);

        if t - self.sec_start >= 1.0 {
            let span = t - self.sec_start;
            self.history.push_back(HistPoint {
                t,
                load: self.bus_load(),
                fps: self.sec_frames as f64 / span,
                eps: self.sec_errors as f64 / span,
            });
            if self.history.len() > 300 {
                self.history.pop_front();
            }
            self.sec_start = t;
            self.sec_frames = 0;
            self.sec_errors = 0;
        }
        if t >= self.next_check_t {
            self.next_check_t = t + 1.0;
            self.checks = checks::run(self);
        }
        while self.busy.front().is_some_and(|&(s, d)| s + d < t - 1.0) {
            self.busy.pop_front();
        }
    }

    pub fn run_for(&mut self, secs: f64) {
        let n = (secs / DT).round() as u64;
        for _ in 0..n {
            self.step();
        }
    }

    fn head(&self, q: usize) -> Option<&TxFrame> {
        let n = self.devices.len();
        if q < n {
            self.devices[q].tx.front()
        } else if q == n {
            self.console.front()
        } else {
            self.replay_q.front()
        }
    }

    fn pop(&mut self, q: usize) -> Option<TxFrame> {
        let n = self.devices.len();
        if q < n {
            self.devices[q].tx.pop_front()
        } else if q == n {
            self.console.pop_front()
        } else {
            self.replay_q.pop_front()
        }
    }

    fn push_front(&mut self, q: usize, f: TxFrame) {
        let n = self.devices.len();
        if q < n {
            self.devices[q].tx.push_front(f)
        } else if q == n {
            self.console.push_front(f)
        } else {
            self.replay_q.push_front(f)
        }
    }

    /// Put queued frames on the wire up to `t_end`. Lowest identifier wins
    /// arbitration among frames ready when the bus goes idle.
    fn advance_bus(&mut self, t_end: f64) {
        let queues = self.devices.len() + 2;
        let bitrate = self.net.backbone.bitrate.max(1000.0);
        let wiring_p = self.wiring_error_prob();
        loop {
            let mut min_ready = f64::INFINITY;
            for q in 0..queues {
                if let Some(h) = self.head(q) {
                    min_ready = min_ready.min(h.ready_t);
                }
            }
            if !min_ready.is_finite() {
                break;
            }
            let start = self.bus_free_at.max(min_ready);
            if start >= t_end {
                break;
            }
            let mut win: Option<(usize, u32)> = None;
            for q in 0..queues {
                if let Some(h) = self.head(q) {
                    if h.ready_t <= start && win.is_none_or(|(_, id)| h.frame.id < id) {
                        win = Some((q, h.frame.id));
                    }
                }
            }
            let (q, _) = win.expect("a frame is ready at start");
            let tf = self.pop(q).expect("head exists");
            let bits = frame_bits(&tf.frame) as f64;
            let dev = (q < self.devices.len()).then_some(q);
            let p = wiring_p + dev.map(|i| self.devices[i].cfg.faults.tx_error_pct / 100.0).unwrap_or(0.0);
            let gw = self.gateway_up();

            if p > 0.0 && self.rng.chance(p) {
                let err_bits = self.rng.range(20.0, bits) + 17.0;
                self.bus_free_at = start + err_bits / bitrate;
                self.busy.push_back((start, err_bits / bitrate));
                self.errors_total += 1;
                self.sec_errors += 1;
                self.record(self.bus_free_at, tf.frame, tf.origin, true, false);
                for (i, d) in self.devices.iter_mut().enumerate() {
                    if Some(i) != dev && d.state.on_bus() {
                        d.rec += 1;
                    }
                }
                match dev {
                    Some(i) => {
                        let d = &mut self.devices[i];
                        d.tec += 8;
                        d.error_frames += 1;
                        if d.tec > 255 {
                            d.state = DevState::BusOff { until: start + 1.0 };
                            d.tx.clear();
                        } else {
                            d.tx.push_front(tf);
                        }
                    }
                    None => self.push_front(q, tf),
                }
                continue;
            }

            let dur = bits / bitrate;
            self.bus_free_at = start + dur;
            let te = self.bus_free_at;
            self.busy.push_back((start, dur));
            self.frames_total += 1;
            self.sec_frames += 1;
            if let Some(i) = dev {
                let d = &mut self.devices[i];
                d.tec = d.tec.saturating_sub(1);
                d.frames_tx += 1;
            }
            for (i, d) in self.devices.iter_mut().enumerate() {
                if Some(i) != dev {
                    d.rec = d.rec.saturating_sub(1);
                    d.rx(te, &tf.frame);
                }
            }
            self.analyser.ingest(te, &tf.frame);
            self.record(te, tf.frame, tf.origin, false, gw);
        }
    }

    fn record(&mut self, t: f64, frame: Frame, origin: Origin, error: bool, gw: bool) {
        self.frame_seq += 1;
        self.ring.push_back(FrameRec { seq: self.frame_seq, t, frame, origin, error, gw });
        if self.ring.len() > RING {
            self.ring.pop_front();
        }
    }

    /// Send a frame from the web UI's own bus interface.
    pub fn inject(&mut self, f: Frame) {
        self.console.push_back(TxFrame { frame: f, ready_t: self.t, origin: Origin::Console });
    }

    /// Restart every device and clear the analyser, as if the network had
    /// just been switched on.
    pub fn power_cycle(&mut self) {
        for d in &mut self.devices {
            d.power_off();
        }
        self.analyser.reset();
        self.next_power_t = self.t;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::default_network;
    use n2k::pgn;

    fn sim() -> Sim {
        Sim::new(default_network(), EnvCfg::default(), HelmCfg::default())
    }

    #[test]
    fn devices_claim_and_data_flows_at_the_expected_rates() {
        let mut s = sim();
        s.run_for(10.0);
        let online = s.devices.iter().filter(|d| d.state == DevState::Online).count();
        assert_eq!(online, s.devices.len());
        // Every device's preferred address is free, so each keeps it.
        for d in &s.devices {
            assert_eq!(d.address, d.cfg.preferred_address, "{}", d.cfg.id);
        }
        let wind = &s.analyser.stats[&(pgn::WIND, 5)];
        let r = wind.rate(s.t);
        assert!((9.0..=11.0).contains(&r), "wind at {r} Hz");
        let load = s.bus_load();
        assert!(load > 0.02 && load < 0.2, "bus load {load}");
        // The display asked for product information and got it.
        assert!(s.analyser.srcs[&2].product.as_ref().is_some_and(|p| p.model.contains("GNSS")));
        assert!(s.checks.iter().all(|c| c.level != "error"), "{:?}", s.checks);
    }

    #[test]
    fn address_conflict_lower_name_wins() {
        let mut net = default_network();
        let mut extra = crate::network::template(DeviceKind::Dst, "dst2", "Second sounder", 1, );
        extra.preferred_address = 3; // the heading sensor's address
        extra.name.arbitrary_address_capable = false;
        net.devices.push(extra);
        let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
        s.run_for(5.0);
        let compass = s.devices.iter().find(|d| d.cfg.id == "compass").unwrap();
        let dst2 = s.devices.iter().find(|d| d.cfg.id == "dst2").unwrap();
        // The fixed-address sounder has the lower NAME, so it keeps 3 and
        // the heading sensor moves.
        assert_eq!(dst2.address, 3);
        assert_ne!(compass.address, 3);
        assert_ne!(compass.address, 254);
        assert_eq!(compass.state, DevState::Online);
    }

    #[test]
    fn missing_terminator_makes_errors_and_babbler_saturates() {
        let mut net = default_network();
        net.backbone.term_end = false;
        let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
        s.run_for(5.0);
        assert!(s.errors_total > 0);
        let mut net = default_network();
        net.devices.iter_mut().find(|d| d.cfg_is("battery")).unwrap().faults.babble_hz = 3000.0;
        let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
        s.run_for(5.0);
        assert!(s.bus_load() > 0.95, "{}", s.bus_load());
        let starved = s.devices.iter().filter(|d| d.dropped > 0).count();
        assert!(starved >= 2, "other devices lose frames");
    }

    #[test]
    fn brownout_at_the_end_of_a_long_thin_backbone() {
        let mut net = default_network();
        net.backbone.cable = crate::network::Cable::Micro;
        net.backbone.length_m = 95.0;
        net.backbone.power_tap_m = 0.0;
        net.backbone.supply_v = 11.0;
        for d in &mut net.devices {
            d.len = 6;
        }
        net.devices.iter_mut().find(|d| d.cfg_is("wind")).unwrap().tee_m = 94.0;
        let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
        s.run_for(3.0);
        let wind = s.devices.iter().find(|d| d.cfg.id == "wind").unwrap();
        assert!(wind.voltage < MIN_VOLTS || wind.boots > 1, "{} V", wind.voltage);
    }

    #[test]
    fn external_rudder_orders_steer_the_boat() {
        let mut s = sim();
        s.world.helm.mode = crate::world::HelmMode::External;
        s.run_for(2.0);
        let order = n2k::Builder::new(pgn::RUDDER).num("instance", 0.0).num("angleOrder", 0.2).encode();
        let id = n2k::CanId::new(2, pgn::RUDDER, 50, 255).encode();
        for _ in 0..30 {
            s.inbox.push((1, Frame::new(id, &order)));
            s.run_for(0.1);
        }
        assert!((s.world.s.rudder - 0.2).abs() < 0.01, "rudder {}", s.world.s.rudder);
        assert!(!s.world.external_timed_out);
        s.run_for(2.0);
        assert!(s.world.external_timed_out);
    }

    impl crate::network::DeviceCfg {
        fn cfg_is(&self, id: &str) -> bool {
            self.id == id
        }
    }
}
