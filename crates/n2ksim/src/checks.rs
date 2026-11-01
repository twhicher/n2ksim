//! Troubleshooting checks, run once a second.
//!
//! Each check says where its evidence comes from:
//! - `bus`: visible to any analyser on a real network
//! - `design`: from the network layout (cable, terminators, power), as an
//!   installer would check it on paper or with a meter
//! - `sim`: only the simulator knows (transceiver error counters, queues)

use crate::analyser::key_field;
use crate::device::DevState;
use crate::network::DeviceKind;
use crate::power::MIN_VOLTS;
use crate::sim::Sim;
use crate::world::{wrap_pi, HelmMode, KN};
use n2k::pgn;
use serde::Serialize;

#[derive(Clone, Debug, Serialize)]
pub struct Check {
    pub level: &'static str,
    pub basis: &'static str,
    pub title: String,
    pub detail: String,
    pub devices: Vec<String>,
    pub srcs: Vec<u8>,
}

fn c(level: &'static str, basis: &'static str, title: impl Into<String>, detail: impl Into<String>) -> Check {
    Check {
        level,
        basis,
        title: title.into(),
        detail: detail.into(),
        devices: vec![],
        srcs: vec![],
    }
}

/// Which simulated device is at a bus address, for linking a bus finding
/// to the diagram. A real analyser would show only the address and NAME.
fn dev_at(sim: &Sim, src: u8) -> Vec<String> {
    sim.devices.iter().filter(|d| d.address == src && d.state.on_bus()).map(|d| d.cfg.id.clone()).collect()
}

fn src_label(sim: &Sim, src: u8) -> String {
    let a = &sim.analyser;
    let s = a.srcs.get(&src);
    let model = s.and_then(|s| s.product.as_ref()).map(|p| p.model.clone()).or_else(|| s.and_then(|s| s.name).and_then(|n| a.models.get(&n).cloned()));
    match model {
        Some(m) if !m.is_empty() => format!("{m} ({src})"),
        _ => format!("address {src}"),
    }
}

fn name_label(sim: &Sim, n: u64) -> String {
    sim.analyser.models.get(&n).cloned().unwrap_or_else(|| format!("NAME {n:016X}"))
}

pub fn run(sim: &Sim) -> Vec<Check> {
    let mut out = Vec::new();
    let now = sim.t;
    let bb = &sim.net.backbone;

    // Design: wiring and power.
    let terms = bb.term_start as u8 + bb.term_end as u8;
    if terms != 2 {
        out.push(c(
            "error",
            "design",
            format!("Backbone has {terms} terminator{}; it needs exactly 2", if terms == 1 { "" } else { "s" }),
            "Without a 120 Ω terminator at each end, signals reflect and frames are corrupted. Expect error frames, retransmissions and devices dropping off. With the network unpowered, the resistance between CAN-H and CAN-L should read about 60 Ω.",
        ));
    }
    if !bb.powered {
        out.push(c("error", "design", "Network has no power", "Nothing on the bus can run. Check the power tee's fuse and supply."));
    } else if bb.power_tap_m < 0.0 || bb.power_tap_m > bb.length_m {
        out.push(c("error", "design", "Power tee is not on the backbone", "Move the power tap within the backbone length."));
    }
    if bb.length_m > bb.cable.max_backbone_m() {
        out.push(c(
            "error",
            "design",
            format!("Backbone {:.0} m is over the {:.0} m limit for this cable", bb.length_m, bb.cable.max_backbone_m()),
            "Limits from the Actisense NMEA 2000 guide, terminator to terminator. Garmin's guide says 100 m between any two points.",
        ));
    }
    let total_drop: f64 = sim.devices.iter().filter(|d| d.cfg.enabled).map(|d| d.cfg.drop_m).sum();
    if total_drop > 78.0 {
        out.push(c("error", "design", format!("Drop cables total {total_drop:.0} m, over 78 m"), "Cumulative drop length limit (Garmin and Actisense guides)."));
    }
    let amps: f64 = sim.devices.iter().filter(|d| d.cfg.enabled).map(|d| d.cfg.len as f64 * crate::power::LEN_AMPS).sum();
    if amps > bb.cable.max_current_a() {
        out.push(c("error", "design", format!("Network draws {amps:.1} A, over the cable's {:.0} A", bb.cable.max_current_a()), "Add a second power insertion or use heavier cable."));
    }
    for d in sim.devices.iter().filter(|d| d.cfg.enabled) {
        if d.cfg.drop_m > 6.0 {
            let mut k = c("warn", "design", format!("{}: drop cable {:.1} m is over 6 m", d.cfg.label, d.cfg.drop_m), "Extend the backbone to the device instead of using a long drop.");
            k.devices.push(d.cfg.id.clone());
            out.push(k);
        }
    }
    let low: Vec<_> = sim.devices.iter().filter(|d| d.cfg.enabled && bb.powered && d.voltage > 0.0 && d.voltage < MIN_VOLTS + 1.0).collect();
    if !low.is_empty() {
        let worst = low.iter().min_by(|a, b| a.voltage.total_cmp(&b.voltage)).expect("not empty");
        let resetting: Vec<&str> = low.iter().filter(|d| matches!(d.state, DevState::Brownout { .. })).map(|d| d.cfg.label.as_str()).collect();
        let lvl = if worst.voltage < MIN_VOLTS || !resetting.is_empty() { "error" } else { "warn" };
        let title = if low.len() == 1 {
            format!("{}: {:.2} V at its connector", worst.cfg.label, worst.voltage)
        } else {
            format!("{} devices below {:.0} V, lowest {:.2} V ({})", low.len(), MIN_VOLTS + 1.0, worst.voltage, worst.cfg.label)
        };
        let mut k = c(
            lvl,
            "design",
            title,
            format!(
                "Devices need at least {MIN_VOLTS:.0} V. Voltage falls along the backbone with distance from the power tee and with the load beyond it. Move the power tee towards the middle of the load, use heavier cable, shorten the run, or check the supply.{}",
                if resetting.is_empty() { String::new() } else { format!(" Resetting now: {}. A device that is off draws nothing, so the voltage recovers, it restarts, and the cycle repeats.", resetting.join(", ")) }
            ),
        );
        k.devices = low.iter().map(|d| d.cfg.id.clone()).collect();
        out.push(k);
    }

    // Bus: load and errors.
    let load = sim.bus_load();
    if load > 0.7 {
        out.push(c("error", "bus", format!("Bus load {:.0}%", load * 100.0), "Low-priority messages are waiting or being lost. Look for a device flooding the bus (one source with a very high frame rate)."));
    } else if load > 0.4 {
        out.push(c("warn", "bus", format!("Bus load {:.0}%", load * 100.0), "Rule of thumb: keep NMEA 2000 load below about 40% so bursts have room."));
    }
    let recent_errors: f64 = sim.history.iter().rev().take(5).map(|h| h.eps).sum::<f64>() / 5f64.min(sim.history.len().max(1) as f64);
    if recent_errors > 0.0 {
        out.push(c(
            if recent_errors > 5.0 { "error" } else { "warn" },
            "bus",
            format!("{recent_errors:.1} error frames per second"),
            "Corrupted frames are retransmitted, so data still arrives but the bus works harder. Usual causes: missing or extra terminators, wet or loose connectors, a faulty transceiver, a backbone that is too long.",
        ));
    }
    let mut per_src: std::collections::BTreeMap<u8, f64> = Default::default();
    for x in sim.analyser.stats.values() {
        *per_src.entry(x.src).or_default() += x.rate(now);
    }
    for (src, n) in per_src {
        if n > 500.0 {
            let mut k = c("error", "bus", format!("{} sends {n:.0} messages a second", src_label(sim, src)), "Far more than any sensor needs. A device stuck sending like this is called a babbling idiot. Unplug it and see whether the load falls.");
            k.srcs.push(src);
            k.devices = dev_at(sim, src);
            out.push(k);
        }
    }

    // Bus: addressing.
    let (conflicts, gave_up) = sim.analyser.conflicts(now);
    for (addr, names) in conflicts {
        let mut k = c(
            "warn",
            "bus",
            format!("Address {addr} claimed by {} devices in the last minute", names.len()),
            "Address claiming resolved it: the lower NAME keeps the address and the other moves. Anything that picked its data source by address (some autopilots and instruments) will now read the wrong device or nothing. Choose sources by NAME.",
        );
        k.srcs.push(addr);
        out.push(k);
    }
    let mut moves: Vec<(u64, f64, Vec<u8>)> = Vec::new();
    for &(t, n, from, to) in &sim.analyser.moves {
        match moves.iter_mut().find(|m| m.0 == n) {
            Some(m) => {
                m.1 = t;
                m.2.push(to);
            }
            None => moves.push((n, t, vec![from, to])),
        }
    }
    for (n, t, path) in moves {
        let hops = path.len() - 1;
        let route = path.iter().map(|a| a.to_string()).collect::<Vec<_>>().join(" → ");
        let mut k = c(
            if now - t < 60.0 { "warn" } else { "info" },
            "bus",
            format!("{} moved address{}: {route}", name_label(sim, n), if hops > 1 { format!(" {hops} times") } else { String::new() }),
            format!(
                "Last move {:.0} s ago. It lost its address to another device and picked a new one, so its data now comes from a different address. Anything that chose this source by address is reading the wrong device or nothing; choose sources by NAME.{}",
                now - t,
                if hops > 1 { " Moving more than once usually means devices are restarting (check power) or several share a fixed address." } else { "" }
            ),
        );
        k.srcs = path.clone();
        k.devices = dev_at(sim, *path.last().expect("two entries"));
        out.push(k);
    }
    let mut restarts: std::collections::BTreeMap<(u64, u8), u32> = Default::default();
    for &(t, n, a) in &sim.analyser.restarts {
        if now - t < 60.0 {
            *restarts.entry((n, a)).or_default() += 1;
        }
    }
    for ((name, a), n) in restarts {
        let who = sim.analyser.models.get(&name).map(|m| format!("{m} ({a})")).unwrap_or_else(|| src_label(sim, a));
        let mut k = c(
            "warn",
            "bus",
            format!("{who} restarted {n} time{} in the last minute", if n == 1 { "" } else { "s" }),
            "It announced its address again without being asked, which a device does when it powers up. Look for low voltage at the device, a loose connector, or a crash.",
        );
        k.srcs.push(a);
        k.devices = dev_at(sim, a);
        out.push(k);
    }
    for n in gave_up {
        let mut k = c("error", "bus", "A device could not claim an address", format!("NAME {n:016X} sent its claim from address 254: it lost a contest and cannot pick another address. It will send no data. Give it a free fixed address."));
        k.devices = sim.devices.iter().filter(|d| d.name().to_u64() == n).map(|d| d.cfg.id.clone()).collect();
        out.push(k);
    }
    for (&a, s) in &sim.analyser.srcs {
        if s.name.is_none() && now - s.last_t < 5.0 && s.frames > 20 && now - s.first_t > 3.0 && a != 254 && a != crate::server::CONSOLE_ADDRESS {
            let mut k = c("info", "bus", format!("Address {a} sends without a claim seen"), "No address claim from this source since the analyser started listening. Send an ISO request for PGN 60928 to find out who it is.");
            k.srcs.push(a);
            k.devices = dev_at(sim, a);
            out.push(k);
        }
    }
    let mut gaps: std::collections::BTreeMap<u8, (u32, f64)> = Default::default();
    for &(t, a, g) in &sim.analyser.gaps {
        if now - t < 60.0 {
            let e = gaps.entry(a).or_default();
            e.0 += 1;
            e.1 = e.1.max(g);
        }
    }
    for (a, (n, longest)) in gaps {
        let mut k = c(
            "warn",
            "bus",
            format!("{} went quiet {n} time{} in the last minute", src_label(sim, a), if n == 1 { "" } else { "s" }),
            format!("Longest silence {longest:.1} s, then it came back. Usually a loose or corroded connector, a device resetting on low voltage, or a cable chafing."),
        );
        k.srcs.push(a);
        k.devices = dev_at(sim, a);
        out.push(k);
    }
    if sim.analyser.fast_errors_log.iter().any(|e| now - e.0 < 10.0) {
        out.push(c("warn", "bus", "Fast-packet messages arriving incomplete", "Frames of multi-frame messages are missing or out of order: lost frames under high load, or two senders reusing a sequence number."));
    }

    // Bus: data health per source.
    let mut stopped: std::collections::BTreeMap<(u8, String), (Vec<&str>, f64)> = Default::default();
    for s in sim.analyser.stats.values() {
        let Some(def) = pgn::def_of(s.pgn) else { continue };
        if def.interval_ms == 0 || s.count < 3 {
            continue;
        }
        let expect = 1000.0 / def.interval_ms as f64;
        let age = now - s.last_t;
        let link = |mut k: Check| {
            k.srcs.push(s.src);
            k.devices = dev_at(sim, s.src);
            k
        };
        let who = match &s.model {
            Some(m) if !m.is_empty() => format!("{m} ({})", s.src),
            _ => src_label(sim, s.src),
        };
        if age > (3.0 * def.interval_ms as f64 / 1000.0).max(3.0) {
            if age < 120.0 {
                let e = stopped.entry((s.src, who.clone())).or_insert((Vec::new(), 0.0));
                e.0.push(def.name);
                e.1 = f64::max(e.1, age);
            }
            continue;
        }
        let r = s.rate(now);
        if def.interval_ms <= 5000 && now - s.first_t > 6.0 && r < 0.5 * expect {
            out.push(link(c("warn", "bus", format!("{} from {} at {r:.1} Hz", def.name, who), format!("canboat lists {expect:.0} Hz as usual for this PGN. Slow data makes steering laggy."))));
        }
        if key_field(s.pgn).is_some() && s.key.is_none() && s.na_count > 3 {
            out.push(link(c("warn", "bus", format!("{} from {} says 'not available'", def.name, who), "The device is alive but has no valid reading: no GNSS fix, lost bottom, sensor fault.")));
        }
        let jumpy = match s.pgn {
            pgn::HEADING => 3.0,
            pgn::WIND => 7.0,
            pgn::RUDDER => 2.0,
            _ => f64::INFINITY,
        };
        if s.count > 50 && s.jump.to_degrees() > jumpy {
            out.push(link(c("warn", "bus", format!("{} from {} is jumpy: {:.1}° between readings", def.name, who, s.jump.to_degrees()), "Successive readings differ by more than the boat can move. A noisy sensor, a damaged vane or cups, or interference.")));
        }
        if key_field(s.pgn).is_some() && s.key.is_some() && now - s.key_changed_t > 10.0 && s.pgn != pgn::POSITION_RAPID {
            out.push(link(c("warn", "bus", format!("{} from {} has not changed for {:.0} s", def.name, who, now - s.key_changed_t), "A real sensor's reading always moves a little. A frozen value suggests a stuck sensor or firmware repeating its last message.")));
        }
    }

    for ((src, who), (names, age)) in stopped {
        let mut k = c(
            "error",
            "bus",
            format!("{who} stopped sending: {}", names.join(" · ")),
            format!("Silent for {age:.0} s after sending normally. The device may be off, hung, unplugged or resetting, or have moved to another address."),
        );
        k.srcs.push(src);
        k.devices = dev_at(sim, src);
        out.push(k);
    }

    // Bus: sources that disagree.
    let latest = |p: u32, f: &dyn Fn(&n2k::Message) -> bool| -> Vec<(u8, n2k::Message)> {
        sim.analyser.stats.values().filter(|s| s.pgn == p && now - s.last_t < 3.0 && f(&s.msg)).map(|s| (s.src, s.msg.clone())).collect()
    };
    let headings = latest(pgn::HEADING, &|_| true);
    let var = sim.world.env.variation_deg.to_radians();
    let trues: Vec<(u8, f64)> = headings
        .iter()
        .filter_map(|(a, m)| {
            let h = m.num("heading")?;
            match m.lookup("reference") {
                Some(0) => Some((*a, h)),
                Some(1) => Some((*a, h + m.num("variation").unwrap_or(var))),
                _ => None,
            }
        })
        .collect();
    for i in 0..trues.len() {
        for j in i + 1..trues.len() {
            let d = wrap_pi(trues[i].1 - trues[j].1).to_degrees().abs();
            if d > 8.0 {
                let mut k = c("warn", "bus", format!("Heading sources disagree by {d:.0}°"), format!("{} and {} (magnetic corrected by variation). One may need calibration or an offset, or sits near something magnetic.", src_label(sim, trues[i].0), src_label(sim, trues[j].0)));
                k.srcs = vec![trues[i].0, trues[j].0];
                out.push(k);
            }
        }
    }
    if headings.len() > 1 {
        out.push(c("info", "bus", format!("{} sources of heading", headings.len()), "Displays and autopilots each pick one. Make sure they agree on which, by NAME."));
    }
    let winds = latest(pgn::WIND, &|m| m.lookup("reference") == Some(2));
    if winds.len() > 1 {
        out.push(c("info", "bus", format!("{} sources of apparent wind", winds.len()), "Two wind sensors, or a wind processor repeating the masthead data."));
    }
    let stw = latest(pgn::SPEED, &|_| true).into_iter().find_map(|(_, m)| m.num("speedWaterReferenced"));
    let sog = latest(pgn::COG_SOG, &|_| true).into_iter().find_map(|(_, m)| m.num("sog"));
    if let (Some(a), Some(b)) = (stw, sog) {
        if (a - b).abs() > 2.0 * KN {
            out.push(c("info", "bus", format!("Speed through water {:.1} kn, over ground {:.1} kn", a / KN, b / KN), "A difference this big is either a strong stream or a speed log that needs calibrating or cleaning."));
        }
    }

    // Simulator only: controller state and queues.
    for d in &sim.devices {
        let link = |mut k: Check| {
            k.devices.push(d.cfg.id.clone());
            k
        };
        match d.state {
            DevState::BusOff { .. } => out.push(link(c("error", "sim", format!("{}: bus off", d.cfg.label), "Its CAN controller saw too many of its own frames fail (transmit error count over 255) and has stopped transmitting. It will restart."))),
            DevState::Unplugged { .. } => out.push(link(c("error", "sim", format!("{}: disconnected", d.cfg.label), "The connector is intermittent. On a real boat, look for green corrosion or a loose coupling nut."))),
            _ if d.tec > 127 => out.push(link(c("warn", "sim", format!("{}: error passive (TEC {})", d.cfg.label, d.tec), "Many of its frames are failing. Close to going bus off."))),
            _ => {}
        }
        if d.dropped > 0 && d.state.on_bus() && d.tx.len() >= crate::device::TX_QUEUE - 1 {
            out.push(link(c("warn", "sim", format!("{}: transmit queue full, dropping messages", d.cfg.label), "It cannot win arbitration often enough. The bus is too busy, usually because of a higher-priority flood.")));
        }
    }
    if sim.world.helm.mode == HelmMode::External {
        let drive = sim.devices.iter().find(|d| d.cfg.kind == DeviceKind::Rudder && d.cfg.library.is_empty() && d.cfg.enabled);
        if sim.world.external_timed_out {
            let mut k = c(
                "warn",
                "sim",
                "Rudder drive has no angle orders",
                format!(
                    "Steering is set to external, but no PGN 127245 angle order has arrived for {:.1} s. The drive holds its last order. {} gateway client(s) connected.",
                    sim.world.helm.order_timeout_s, sim.clients
                ),
            );
            if let Some(d) = drive {
                k.devices.push(d.cfg.id.clone());
            }
            out.push(k);
        }
    }
    out.sort_by_key(|k| match k.level {
        "error" => 0,
        "warn" => 1,
        _ => 2,
    });
    out
}
