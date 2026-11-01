//! A simulated device on the bus: it claims an address, answers requests,
//! and sends its data on schedule with sensor noise and any faults.

use crate::network::{DeviceCfg, DeviceKind, PgnCfg};
use crate::rng::{Ou, Rng};
use crate::world::{wrap_2pi, wrap_pi, World};
use n2k::canid::{BROADCAST, NULL_ADDRESS};
use n2k::pgn::{self, Builder};
use n2k::{frames_for, CanId, Frame, Name};
use std::collections::{HashMap, HashSet, VecDeque};

/// PGNs a simulated device can generate from the world.
pub const SIMULATED: &[u32] = &[
    pgn::POSITION_RAPID,
    pgn::COG_SOG,
    pgn::GNSS_POSITION,
    pgn::SYSTEM_TIME,
    pgn::TIME_DATE,
    pgn::GNSS_DOPS,
    pgn::MAGNETIC_VARIATION,
    pgn::HEADING,
    pgn::RATE_OF_TURN,
    pgn::ATTITUDE,
    pgn::HEAVE,
    pgn::WIND,
    pgn::SPEED,
    pgn::DEPTH,
    pgn::DISTANCE_LOG,
    pgn::TEMPERATURE,
    pgn::TEMPERATURE_EXT,
    pgn::ENV_OBSOLETE,
    pgn::ENV_PARAMETERS,
    pgn::HUMIDITY,
    pgn::PRESSURE,
    pgn::DIRECTION_DATA,
    pgn::RUDDER,
    pgn::BATTERY,
];

/// Flavours of a PGN a device can send, first is the usual default.
pub const VARIANTS: &[(u32, &[&str])] = &[
    (pgn::HEADING, &["magnetic", "true"]),
    (pgn::WIND, &["apparent", "true_boat", "true_water", "true_north"]),
    (pgn::SPEED, &["water", "ground"]),
    (pgn::TEMPERATURE, &["sea", "air"]),
    (pgn::TEMPERATURE_EXT, &["sea", "air"]),
    (pgn::ENV_OBSOLETE, &["all", "sea"]),
    (pgn::ENV_PARAMETERS, &["air", "sea"]),
];

/// Network management PGNs every simulated device handles itself; a
/// library entry may list them but they are not data.
pub const MANAGEMENT: &[u32] = &[59392, 59904, 60928, 65240, 126208, 126464, 126993, 126996, 126998];

pub fn default_variant(kind: DeviceKind, p: u32) -> &'static str {
    match (kind, p) {
        (DeviceKind::Gnss, pgn::HEADING) => "true",
        (DeviceKind::Gnss, pgn::SPEED) => "ground",
        (DeviceKind::Weather, pgn::TEMPERATURE | pgn::TEMPERATURE_EXT) => "air",
        (DeviceKind::Dst, pgn::ENV_OBSOLETE | pgn::ENV_PARAMETERS) => "sea",
        _ => VARIANTS.iter().find(|(q, _)| *q == p).map(|(_, v)| v[0]).unwrap_or(""),
    }
}

/// How long a device waits after claiming before it sends data (ISO 11783-5).
pub const CLAIM_WAIT_S: f64 = 0.25;
pub const TX_QUEUE: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DevState {
    Off,
    Claiming { until: f64 },
    Online,
    /// Lost its address contest and cannot pick another: source 254.
    CannotClaim,
    BusOff { until: f64 },
    Brownout { until: f64 },
    Unplugged { until: f64 },
}

impl DevState {
    pub fn key(&self) -> &'static str {
        match self {
            DevState::Off => "off",
            DevState::Claiming { .. } => "claiming",
            DevState::Online => "online",
            DevState::CannotClaim => "cannot_claim",
            DevState::BusOff { .. } => "bus_off",
            DevState::Brownout { .. } => "brownout",
            DevState::Unplugged { .. } => "unplugged",
        }
    }
    /// On the bus and listening.
    pub fn on_bus(&self) -> bool {
        matches!(self, DevState::Claiming { .. } | DevState::Online | DevState::CannotClaim)
    }
    /// Drawing current.
    pub fn powered(&self) -> bool {
        !matches!(self, DevState::Off | DevState::Brownout { .. } | DevState::Unplugged { .. })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    Device(usize),
    /// A TCP client of the gateway; the frame is in the gateway's queue.
    Client(u32),
    /// The web UI's own interface to the bus.
    Console,
    Replay,
}

#[derive(Clone, Debug)]
pub struct TxFrame {
    pub frame: Frame,
    pub ready_t: f64,
    pub origin: Origin,
}

pub struct Device {
    pub cfg: DeviceCfg,
    pub index: usize,
    pub state: DevState,
    pub address: u8,
    pub tx: VecDeque<TxFrame>,
    pub tec: u32,
    pub rec: u32,
    pub frames_tx: u64,
    pub frames_rx: u64,
    pub dropped: u64,
    pub error_frames: u64,
    pub voltage: f64,
    pub boots: u32,
    pub address_changes: u32,
    /// Last angle order heard on PGN 127245 (rudder drive only): time, rad, source.
    pub last_order: Option<(f64, f64, u8)>,
    due: Vec<f64>,
    seq: u8,
    sid: u8,
    hb_due: f64,
    hb_seq: u8,
    stuck: HashMap<(u32, String), Vec<u8>>,
    wind_lag: (f64, f64),
    stw_lag: f64,
    pos_err: (Ou, Ou),
    rng: Rng,
    seen: HashMap<u8, u64>,
    pending: VecDeque<(f64, u8, u32)>,
    asked: HashSet<(u8, u64)>,
    babble_acc: f64,
    next_flake_check: f64,
    /// Data PGNs someone asked for with an ISO request.
    pending_data: Vec<u32>,
    /// The address it last claimed successfully. Real devices keep this in
    /// non-volatile memory and claim it again at power-up.
    pub stored_address: Option<u8>,
}

impl Device {
    pub fn new(cfg: DeviceCfg, index: usize, seed: u64) -> Self {
        let mut d = Device {
            address: cfg.preferred_address,
            rng: Rng::new(seed ^ (cfg.name.unique_number as u64) << 8 ^ index as u64),
            cfg,
            index,
            state: DevState::Off,
            tx: VecDeque::new(),
            tec: 0,
            rec: 0,
            frames_tx: 0,
            frames_rx: 0,
            dropped: 0,
            error_frames: 0,
            voltage: 0.0,
            boots: 0,
            address_changes: 0,
            last_order: None,
            due: Vec::new(),
            seq: 0,
            sid: 0,
            hb_due: 0.0,
            hb_seq: 0,
            stuck: HashMap::new(),
            wind_lag: (0.0, 0.0),
            stw_lag: 0.0,
            pos_err: (Ou::default(), Ou::default()),
            seen: HashMap::new(),
            pending: VecDeque::new(),
            asked: HashSet::new(),
            babble_acc: 0.0,
            next_flake_check: 0.0,
            pending_data: Vec::new(),
            stored_address: None,
        };
        d.seq = (d.rng.next_u64() % 8) as u8;
        d
    }

    pub fn name(&self) -> Name {
        self.cfg.name.to_name()
    }

    /// Power up (or restart) and claim an address.
    pub fn boot(&mut self, t: f64) {
        self.tx.clear();
        self.tec = 0;
        self.rec = 0;
        self.boots += 1;
        self.address = self.stored_address.unwrap_or(self.cfg.preferred_address);
        self.seen.clear();
        self.asked.clear();
        self.pending.clear();
        self.stuck.clear();
        self.claim(t);
        self.due = self.cfg.pgns.iter().map(|p| t + CLAIM_WAIT_S + self.rng.range(0.0, p.interval_ms.max(1) as f64 / 1000.0)).collect();
        self.hb_due = t + CLAIM_WAIT_S + self.rng.range(0.5, 2.0);
        self.next_flake_check = t + 1.0;
    }

    pub fn power_off(&mut self) {
        self.state = DevState::Off;
        self.tx.clear();
    }

    fn claim(&mut self, t: f64) {
        self.tx.clear();
        let data = self.name().to_bytes();
        self.send(t, pgn::ADDRESS_CLAIM, 6, BROADCAST, &data);
        self.state = if self.address == NULL_ADDRESS { DevState::CannotClaim } else { DevState::Claiming { until: t + CLAIM_WAIT_S } };
    }

    pub fn send(&mut self, t: f64, pgn: u32, priority: u8, dst: u8, payload: &[u8]) {
        let id = CanId::new(priority, pgn, self.address, dst);
        self.seq = (self.seq + 1) % 8;
        for f in frames_for(id, payload, self.seq) {
            self.push(TxFrame {
                frame: f,
                ready_t: t,
                origin: Origin::Device(self.index),
            });
        }
    }

    pub fn push(&mut self, f: TxFrame) {
        if self.tx.len() >= TX_QUEUE {
            self.tx.pop_front();
            self.dropped += 1;
        }
        self.tx.push_back(f);
    }

    fn next_sid(&mut self) -> f64 {
        self.sid = (self.sid + 1) % 253;
        self.sid as f64
    }

    /// Advance timers and queue anything due.
    pub fn tick(&mut self, t: f64, world: &World, mute_data: bool) {
        match self.state {
            DevState::Claiming { until } if t >= until => {
                self.state = DevState::Online;
                self.stored_address = Some(self.address);
                if self.cfg.kind == DeviceKind::Display {
                    // Ask everyone who they are, as plotters do at start-up.
                    let req = Builder::new(pgn::ISO_REQUEST).bin("pgn", pgn::ADDRESS_CLAIM as u64).encode();
                    self.send(t, pgn::ISO_REQUEST, 6, BROADCAST, &req);
                }
            }
            DevState::BusOff { until } | DevState::Brownout { until } | DevState::Unplugged { until } if t >= until => {
                self.boot(t);
                return;
            }
            _ => {}
        }
        if !self.state.on_bus() {
            return;
        }
        let f = self.cfg.faults.clone();
        if f.intermittent && t >= self.next_flake_check {
            self.next_flake_check = t + 1.0;
            if self.rng.chance(0.1) {
                let off = self.rng.range(1.0, 6.0);
                self.state = DevState::Unplugged { until: t + off };
                self.tx.clear();
                return;
            }
        }
        if f.babble_hz > 0.0 {
            self.babble_acc += f.babble_hz * 0.001;
            while self.babble_acc >= 1.0 {
                self.babble_acc -= 1.0;
                let id = CanId::new(0, pgn::PROPRIETARY_SINGLE, self.address, BROADCAST).encode();
                let b = (self.rng.next_u64() & 0xFF) as u8;
                self.push(TxFrame {
                    frame: Frame::new(id, &[0xD0, 0x87, b, 0, 0, 0, 0, 0]),
                    ready_t: t,
                    origin: Origin::Device(self.index),
                });
            }
        }
        if self.state != DevState::Online {
            return;
        }
        // Requests a display makes of devices it has just seen.
        while let Some(&(at, dst, p)) = self.pending.front() {
            if at > t {
                break;
            }
            self.pending.pop_front();
            let req = Builder::new(pgn::ISO_REQUEST).bin("pgn", p as u64).encode();
            self.send(t, pgn::ISO_REQUEST, 6, dst, &req);
        }
        if t >= self.hb_due {
            self.hb_due = t + 60.0;
            self.heartbeat(t);
        }
        if f.silent || mute_data {
            self.pending_data.clear();
            return;
        }
        for p in std::mem::take(&mut self.pending_data) {
            let v = self.cfg.pgns.iter().find(|c| c.pgn == p).map(|c| c.variant.clone()).unwrap_or_default();
            self.send_data(t, p, &v, world);
        }
        if self.due.len() != self.cfg.pgns.len() {
            self.due = self.cfg.pgns.iter().map(|_| t).collect();
        }
        for i in 0..self.cfg.pgns.len() {
            let PgnCfg { pgn, enabled, interval_ms, .. } = self.cfg.pgns[i].clone();
            if t < self.due[i] {
                continue;
            }
            let interval = interval_ms.max(10) as f64 / 1000.0 * f.rate_mult.max(0.05);
            let jitter = if f.jitter_ms > 0.0 { self.rng.range(0.0, f.jitter_ms / 1000.0) } else { 0.0 };
            self.due[i] = (self.due[i] + interval).max(t) + jitter;
            if enabled {
                let v = self.cfg.pgns[i].variant.clone();
                self.send_data(t, pgn, &v, world);
            }
        }
    }

    fn heartbeat(&mut self, t: f64) {
        let state = |c: u32| if c > 255 { 2 } else if c > 127 { 1 } else { 0 };
        let data = Builder::new(pgn::HEARTBEAT)
            .num("dataTransmitOffset", 60.0)
            .num("sequenceCounter", self.hb_seq as f64)
            .lookup("controller1State", state(self.tec.max(self.rec)))
            .lookup("controller2State", 3)
            .lookup("equipmentStatus", 0)
            .encode();
        self.hb_seq = (self.hb_seq + 1) % 253;
        self.send(t, pgn::HEARTBEAT, 7, BROADCAST, &data);
    }

    pub fn send_data(&mut self, t: f64, p: u32, variant: &str, world: &World) {
        let var = if variant.is_empty() { default_variant(self.cfg.kind, p).to_string() } else { variant.to_string() };
        let payload = if self.cfg.faults.stuck {
            if let Some(d) = self.stuck.get(&(p, var.clone())) {
                d.clone()
            } else {
                let Some(b) = self.measure(p, &var, world) else { return };
                let d = b.encode();
                self.stuck.insert((p, var), d.clone());
                d
            }
        } else {
            let Some(mut b) = self.measure(p, &var, world) else { return };
            if self.cfg.faults.na_data {
                b = b.blank_numbers(&["instance", "sid", "date", "time"]);
                if p == pgn::GNSS_POSITION {
                    b = b.lookup("method", 0).num("numberOfSvs", 0.0);
                }
            }
            b.encode()
        };
        let prio = pgn::def_of(p).map(|d| d.priority).unwrap_or(6);
        self.send(t, p, prio, BROADCAST, &payload);
    }

    fn noise(&mut self, sd: f64) -> f64 {
        sd * self.cfg.faults.noise_mult * self.rng.gauss()
    }


    /// Build one message from the world as this device sees it. Any device
    /// can send any PGN in `SIMULATED`; the kind and the PGN's variant
    /// decide the details. A processor's data is modelled as a sensor's
    /// with noise, not computed from the other devices' messages.
    fn measure(&mut self, p: u32, var: &str, w: &World) -> Option<Builder> {
        use pgn::*;
        let s = &w.s;
        let off = self.cfg.faults.angle_offset_deg.to_radians();
        let scale = 1.0 + self.cfg.faults.scale_pct / 100.0;
        let utc = w.utc();
        let date = (utc / 86400.0).floor();
        let time = utc - date * 86400.0;
        let inst = self.cfg.instance as f64;
        let air = w.env.air_temp_c + 273.15;
        let sea = w.env.water_temp_c + 273.15;
        let b = match p {
            POSITION_RAPID | GNSS_POSITION => {
                let en = self.pos_err.0.step(0.1, 20.0, 1.5 * self.cfg.faults.noise_mult, &mut self.rng);
                let ee = self.pos_err.1.step(0.1, 20.0, 1.5 * self.cfg.faults.noise_mult, &mut self.rng);
                let (lat, lon) = w.lat_lon();
                let lat = lat + (en / 6_371_000.0).to_degrees();
                let lon = lon + (ee / (6_371_000.0 * lat.to_radians().cos())).to_degrees();
                if p == POSITION_RAPID {
                    Builder::new(p).num("latitude", lat).num("longitude", lon)
                } else {
                    Builder::new(p)
                        .num("sid", self.next_sid())
                        .num("date", date)
                        .num("time", time)
                        .num("latitude", lat)
                        .num("longitude", lon)
                        .num("altitude", 0.0)
                        .lookup("gnssType", 2)
                        .lookup("method", 2)
                        .lookup("integrity", 0)
                        .num("numberOfSvs", 14.0)
                        .num("hdop", 0.8)
                        .num("pdop", 1.4)
                        .num("geoidalSeparation", 47.0)
                        .num("referenceStations", 0.0)
                }
            }
            COG_SOG => {
                let cog_sd = (0.03 / s.sog.max(0.2)).min(0.5);
                let cog = wrap_2pi(s.cog + off + self.noise(cog_sd));
                let sog = (s.sog * scale + self.noise(0.03)).max(0.0);
                Builder::new(p).num("sid", self.next_sid()).lookup("cogReference", 0).num("cog", cog).num("sog", sog)
            }
            SYSTEM_TIME => Builder::new(p).num("sid", self.next_sid()).lookup("source", 0).num("date", date).num("time", time),
            TIME_DATE => Builder::new(p).num("date", date).num("time", time).num("localOffset", 0.0),
            GNSS_DOPS => Builder::new(p).num("sid", self.next_sid()).lookup("desiredMode", 3).lookup("actualMode", 2).num("hdop", 0.8).num("vdop", 1.2).num("tdop", 0.9),
            MAGNETIC_VARIATION => Builder::new(p)
                .num("sid", self.next_sid())
                .lookup("source", 9)
                .num("ageOfService", date)
                .num("variation", w.env.variation_deg.to_radians()),
            HEADING => {
                let variation = w.env.variation_deg.to_radians();
                if var == "true" {
                    // GNSS compasses measure true heading directly.
                    let sd: f64 = if self.cfg.kind == DeviceKind::Gnss { 0.15 } else { 0.4 };
                    let h = wrap_2pi(s.heading + off + self.noise(sd.to_radians()));
                    Builder::new(p).num("sid", self.next_sid()).num("heading", h).num("variation", variation).lookup("reference", 0)
                } else {
                    // Magnetic: true minus variation, plus a small heeling error.
                    let mag = s.heading - variation + 0.15 * s.heel;
                    let h = wrap_2pi(mag + off + self.noise(0.4f64.to_radians()));
                    Builder::new(p).num("sid", self.next_sid()).num("heading", h).lookup("reference", 1)
                }
            }
            RATE_OF_TURN => Builder::new(p).num("sid", self.next_sid()).num("rate", s.yaw_rate + self.noise(0.1f64.to_radians())),
            ATTITUDE => Builder::new(p)
                .num("sid", self.next_sid())
                .num("pitch", s.pitch + self.noise(0.1f64.to_radians()))
                .num("roll", s.heel + self.noise(0.1f64.to_radians())),
            // Sign not stated by canboat; sent positive up. Check against a real unit.
            HEAVE => Builder::new(p).num("sid", self.next_sid()).num("heave", -s.heave + self.noise(0.02)).num("delay", 0.0).lookup("delaySource", 2),
            WIND => {
                // Damped apparent wind (about 0.8 s here), then true wind
                // from it where asked.
                let a = 0.1 / (0.8 + 0.1);
                let (x, y) = (s.aws * s.awa.cos(), s.aws * s.awa.sin());
                self.wind_lag.0 += a * (x - self.wind_lag.0);
                self.wind_lag.1 += a * (y - self.wind_lag.1);
                let aws = (self.wind_lag.0.hypot(self.wind_lag.1) * scale + self.noise(0.15)).max(0.0);
                let awa = wrap_pi(self.wind_lag.1.atan2(self.wind_lag.0) + off + self.noise(1.5f64.to_radians()));
                let (speed, angle, reference) = match var {
                    "true_boat" | "true_water" => {
                        let (tx, ty) = (aws * awa.cos() - s.bsp, aws * awa.sin());
                        (tx.hypot(ty), ty.atan2(tx), if var == "true_boat" { 3 } else { 4 })
                    }
                    "true_north" => (s.tws + self.noise(0.15), s.twd + self.noise(1.5f64.to_radians()), 0),
                    _ => (aws, awa, 2),
                };
                Builder::new(p).num("sid", self.next_sid()).num("windSpeed", speed.max(0.0)).num("windAngle", wrap_2pi(angle)).lookup("reference", reference)
            }
            SPEED => {
                if var == "ground" {
                    Builder::new(p).num("sid", self.next_sid()).num("speedGroundReferenced", (s.sog * scale + self.noise(0.03)).max(0.0))
                } else {
                    self.stw_lag += (s.bsp - self.stw_lag) * 0.4;
                    let stw = (self.stw_lag * scale + self.noise(0.04)).max(0.0);
                    Builder::new(p).num("sid", self.next_sid()).num("speedWaterReferenced", stw).lookup("speedWaterReferencedType", 0)
                }
            }
            DEPTH => {
                // Transducer 0.5 m below the waterline; loses the bottom in
                // very shallow or very deep water.
                let below = s.depth * scale - 0.5 + self.noise(0.03);
                let b = Builder::new(p).num("sid", self.next_sid()).num("offset", 0.5);
                if (0.3..200.0).contains(&below) {
                    b.num("depth", below)
                } else {
                    b
                }
            }
            DISTANCE_LOG => Builder::new(p).num("date", date).num("time", time).num("log", 2_290_000.0 + s.log_m * scale).num("tripLog", s.log_m * scale),
            TEMPERATURE | TEMPERATURE_EXT => {
                let (src, k) = if var == "air" { (1, air) } else { (0, sea) };
                let field = if p == TEMPERATURE { "actualTemperature" } else { "temperature" };
                Builder::new(p).num("sid", self.next_sid()).num("instance", inst).lookup("source", src).num(field, k + self.noise(0.05))
            }
            ENV_OBSOLETE => {
                let b = Builder::new(p).num("sid", self.next_sid()).num("waterTemperature", sea + self.noise(0.05));
                if var == "sea" {
                    b
                } else {
                    b.num("outsideAmbientAirTemperature", air + self.noise(0.1)).num("atmosphericPressure", w.env.pressure_hpa * 100.0)
                }
            }
            ENV_PARAMETERS => {
                let b = Builder::new(p).num("sid", self.next_sid());
                if var == "sea" {
                    b.lookup("temperatureSource", 0).num("temperature", sea + self.noise(0.05))
                } else {
                    b.lookup("temperatureSource", 1)
                        .lookup("humiditySource", 1)
                        .num("temperature", air + self.noise(0.1))
                        .num("humidity", w.env.humidity_pct)
                        .num("atmosphericPressure", w.env.pressure_hpa * 100.0)
                }
            }
            HUMIDITY => Builder::new(p).num("sid", self.next_sid()).num("instance", inst).lookup("source", 1).num("actualHumidity", w.env.humidity_pct),
            PRESSURE => Builder::new(p).num("sid", self.next_sid()).num("instance", inst).lookup("source", 0).num("pressure", w.env.pressure_hpa * 100.0 + self.noise(5.0)),
            DIRECTION_DATA => Builder::new(p)
                .lookup("dataMode", 0)
                .lookup("cogReference", 0)
                .num("sid", self.next_sid())
                .num("cog", wrap_2pi(s.cog + self.noise(0.01)))
                .num("sog", s.sog)
                .num("heading", wrap_2pi(s.heading + off + self.noise(0.4f64.to_radians())))
                .num("speedThroughWater", s.bsp)
                .num("set", w.env.current_set_deg.to_radians())
                .num("drift", w.env.current_kn * crate::world::KN),
            RUDDER => Builder::new(p).num("instance", inst).num("position", s.rudder + off + self.noise(0.05f64.to_radians())),
            BATTERY => {
                let amps = 2.5 + s.ram_current;
                Builder::new(p)
                    .num("instance", inst)
                    .num("voltage", 12.75 - 0.03 * amps + self.noise(0.01))
                    .num("current", amps + self.noise(0.05))
                    .num("temperature", 293.15)
                    .num("sid", self.next_sid())
            }
            _ => return None,
        };
        Some(b)
    }

    fn product_info(&self) -> Vec<u8> {
        let p = &self.cfg.product;
        Builder::new(pgn::PRODUCT_INFO)
            .num("nmea2000Version", 2.1)
            .num("productCode", p.product_code as f64)
            .text("modelId", &p.model_id)
            .text("softwareVersionCode", &p.software_version)
            .text("modelVersion", &p.model_version)
            .text("modelSerialCode", &p.serial)
            .num("certificationLevel", p.certification_level as f64)
            .num("loadEquivalency", self.cfg.len as f64)
            .encode()
    }

    fn tx_pgns(&self) -> Vec<u32> {
        let mut v = vec![pgn::ISO_ACK, pgn::ISO_REQUEST, pgn::ADDRESS_CLAIM, pgn::PGN_LIST, pgn::HEARTBEAT, pgn::PRODUCT_INFO];
        v.extend(self.cfg.pgns.iter().map(|p| p.pgn));
        v
    }

    fn pick_new_address(&self) -> u8 {
        for k in 1..=252u16 {
            let a = ((self.address as u16 + k) % 252) as u8;
            if !self.seen.contains_key(&a) {
                return a;
            }
        }
        NULL_ADDRESS
    }

    /// A frame from someone else on the bus.
    pub fn rx(&mut self, t: f64, f: &Frame) {
        if !self.state.on_bus() {
            return;
        }
        self.frames_rx += 1;
        let id = f.can_id();
        match id.pgn {
            pgn::ADDRESS_CLAIM => {
                let Some(theirs) = Name::from_bytes(f.bytes()) else { return };
                let theirs = theirs.to_u64();
                self.seen.retain(|_, n| *n != theirs);
                if id.source != NULL_ADDRESS {
                    self.seen.insert(id.source, theirs);
                }
                if self.cfg.kind == DeviceKind::Display && self.state == DevState::Online && id.source != NULL_ADDRESS && self.asked.insert((id.source, theirs)) {
                    let at = t + 0.05 * (self.pending.len() as f64 + 1.0);
                    self.pending.push_back((at, id.source, pgn::PRODUCT_INFO));
                    self.pending.push_back((at + 0.02, id.source, pgn::PGN_LIST));
                }
                if id.source == self.address && self.address != NULL_ADDRESS {
                    let mine = self.name().to_u64();
                    if mine < theirs {
                        // Defend the address; an online device stays online.
                        let was = self.state;
                        let data = self.name().to_bytes();
                        self.send(t, pgn::ADDRESS_CLAIM, 6, BROADCAST, &data);
                        self.state = was;
                    } else if mine != theirs {
                        self.address = if self.cfg.name.arbitrary_address_capable { self.pick_new_address() } else { NULL_ADDRESS };
                        self.address_changes += 1;
                        self.claim(t);
                    }
                }
            }
            pgn::ISO_REQUEST if id.destination == self.address || id.destination == BROADCAST => {
                let Some(req) = n2k::bits::get(f.bytes(), 0, 24) else { return };
                let req = req as u32;
                let to = id.source;
                match req {
                    pgn::ADDRESS_CLAIM => {
                        let data = self.name().to_bytes();
                        self.send(t, pgn::ADDRESS_CLAIM, 6, BROADCAST, &data);
                    }
                    _ if self.state == DevState::CannotClaim => {}
                    pgn::PRODUCT_INFO => {
                        let d = self.product_info();
                        self.send(t, pgn::PRODUCT_INFO, 6, BROADCAST, &d);
                    }
                    pgn::PGN_LIST => {
                        let d = Builder::new(pgn::PGN_LIST).lookup("functionCode", 0).list("pgns", &self.tx_pgns()).encode();
                        self.send(t, pgn::PGN_LIST, 6, BROADCAST, &d);
                        let rx = if self.cfg.rx_pgns.is_empty() { self.cfg.kind.receive_pgns() } else { self.cfg.rx_pgns.clone() };
                        let d = Builder::new(pgn::PGN_LIST).lookup("functionCode", 1).list("pgns", &rx).encode();
                        self.send(t, pgn::PGN_LIST, 6, BROADCAST, &d);
                    }
                    pgn::HEARTBEAT => self.heartbeat(t),
                    p if self.cfg.pgns.iter().any(|c| c.pgn == p) => {}
                    p if id.destination == self.address => {
                        let d = Builder::new(pgn::ISO_ACK).lookup("control", 1).num("groupFunction", 255.0).bin("pgn", p as u64).encode();
                        self.send(t, pgn::ISO_ACK, 6, to, &d);
                    }
                    _ => {}
                }
                // Data PGNs on request need the world; the simulator sends them.
                if self.cfg.pgns.iter().any(|c| c.pgn == req) && self.state == DevState::Online {
                    self.pending_data.push(req);
                }
            }
            // Only the simulator's own drive (a generic rudder device) takes
            // orders; library rudder sensors just report.
            pgn::RUDDER if self.cfg.kind == DeviceKind::Rudder && self.cfg.library.is_empty() && id.source != self.address => {
                let m = n2k::decode(pgn::RUDDER, f.bytes());
                let inst = m.num("instance").unwrap_or(0.0) as u8;
                if inst == self.cfg.instance {
                    if let Some(a) = m.num("angleOrder") {
                        self.last_order = Some((t, a, id.source));
                    }
                }
            }
            _ => {}
        }
    }
}
