//! Ready-made faults to practise on, and sailing conditions.

use crate::network::{default_network, template, DeviceKind, Faults, Network};
use crate::sim::Sim;
use crate::world::{EnvCfg, HelmMode};

pub struct Drill {
    pub key: &'static str,
    pub title: &'static str,
    pub what: &'static str,
}

pub const DRILLS: &[Drill] = &[
    Drill { key: "missing_terminator", title: "Missing terminator", what: "The masthead terminator has gone. Watch error frames and retransmissions." },
    Drill { key: "address_conflict", title: "Address conflict", what: "A new sounder with a fixed address takes the heading sensor's address. The heading sensor moves; anything bound to its old address loses heading." },
    Drill { key: "wind_stuck", title: "Wind sensor stuck", what: "The masthead unit keeps sending its last reading." },
    Drill { key: "gnss_no_fix", title: "GNSS lost its fix", what: "The receiver is alive but sends 'not available' for position, course and speed." },
    Drill { key: "gnss_hung", title: "GNSS hung", what: "The receiver keeps its address but sends nothing." },
    Drill { key: "babbler", title: "Babbling device", what: "The battery monitor floods the bus with priority-0 frames." },
    Drill { key: "brownout", title: "Low voltage", what: "Supply at 9.3 V through thin cable fed from the stern: devices far from the power tee reset." },
    Drill { key: "compass_offset", title: "Heading sensor offset", what: "The heading sensor reads 12° high, so it disagrees with the GNSS compass." },
    Drill { key: "slow_wind", title: "Slow wind data", what: "The wind sensor sends at 2.5 Hz instead of 10 Hz." },
    Drill { key: "noisy_wind", title: "Noisy wind", what: "Wind angle noise six times normal, as from a damaged cup or vane." },
    Drill { key: "loose_connector", title: "Loose connector", what: "The depth, speed and temperature unit drops off the bus for a few seconds at random." },
    Drill { key: "bad_transceiver", title: "Faulty transceiver", what: "The heading sensor corrupts 15% of its own frames." },
];

/// The device a drill should break: by its generic id if present, else the
/// first plugged-in device that sends the data the drill is about, so drills
/// work on library networks too.
fn target<'a>(net: &'a mut Network, role: &str) -> Result<&'a mut crate::network::DeviceCfg, String> {
    use n2k::pgn::*;
    let sends = |d: &crate::network::DeviceCfg, p: u32| d.enabled && d.pgns.iter().any(|c| c.pgn == p && c.enabled);
    let i = net.devices.iter().position(|d| d.id == role).or_else(|| {
        net.devices.iter().position(|d| match role {
            "wind" => sends(d, WIND),
            "gnss" => sends(d, POSITION_RAPID),
            "compass" => sends(d, HEADING) && d.kind != DeviceKind::Gnss,
            "dst" => sends(d, DEPTH) || sends(d, SPEED),
            "battery" => d.kind == DeviceKind::Battery,
            _ => false,
        })
    });
    // Any device can babble; prefer a battery monitor, else anything that is
    // not the drive or the gateway.
    let i = i.or_else(|| (role == "battery").then(|| net.devices.iter().position(|d| d.enabled && d.id != "drive" && d.kind != DeviceKind::Gateway)).flatten());
    let what = match role {
        "wind" => "a wind sensor",
        "gnss" => "a GNSS receiver",
        "compass" => "a heading sensor",
        "dst" => "a depth or speed sensor",
        _ => "a battery monitor",
    };
    i.map(move |i| &mut net.devices[i]).ok_or_else(|| format!("this drill needs {what} on the network"))
}

/// Apply a drill. Returns the label of the device it broke, if one.
pub fn drill(sim: &mut Sim, key: &str) -> Result<Option<String>, String> {
    let mut net: Network = sim.net.clone();
    let mut hit = None;
    let mut save_backbone = false;
    match key {
        "reset" => {
            net = default_network();
            sim.pre_drill_backbone = None;
        }
        "clear_faults" => {
            for d in &mut net.devices {
                d.faults = Faults::default();
                d.enabled = true;
            }
            if let Some(b) = sim.pre_drill_backbone.take() {
                net.backbone = b;
            }
            net.devices.retain(|d| d.id != "dst2");
        }
        "missing_terminator" => {
            save_backbone = true;
            net.backbone.term_end = false;
        }
        "brownout" => {
            save_backbone = true;
            net.backbone.cable = crate::network::Cable::Micro;
            net.backbone.power_tap_m = 0.0;
            net.backbone.supply_v = 9.3;
        }
        "address_conflict" => {
            let id = target(&mut net, "compass")?.id.clone();
            let d = sim.devices.iter().find(|d| d.cfg.id == id).ok_or("heading sensor not running")?;
            let (addr, label, tee) = (d.address, d.cfg.label.clone(), d.cfg.tee_m);
            net.devices.retain(|d| d.id != "dst2");
            let mut d = template(DeviceKind::Dst, "dst2", "New sounder (fixed address)", 9);
            d.preferred_address = addr;
            d.name.arbitrary_address_capable = false;
            d.tee_m = (tee + 1.5).min(net.backbone.length_m);
            d.pgns.retain(|p| p.pgn == n2k::pgn::DEPTH);
            net.devices.push(d);
            hit = Some(label);
        }
        _ => {
            let (role, f): (&str, fn(&mut Faults)) = match key {
                "wind_stuck" => ("wind", |f| f.stuck = true),
                "gnss_no_fix" => ("gnss", |f| f.na_data = true),
                "gnss_hung" => ("gnss", |f| f.silent = true),
                "babbler" => ("battery", |f| f.babble_hz = 3000.0),
                "compass_offset" => ("compass", |f| f.angle_offset_deg = 12.0),
                "slow_wind" => ("wind", |f| f.rate_mult = 4.0),
                "noisy_wind" => ("wind", |f| f.noise_mult = 6.0),
                "loose_connector" => ("dst", |f| f.intermittent = true),
                "bad_transceiver" => ("compass", |f| f.tx_error_pct = 15.0),
                _ => return Err("unknown drill".into()),
            };
            let d = target(&mut net, role)?;
            f(&mut d.faults);
            hit = Some(d.label.clone());
        }
    }
    if save_backbone && sim.pre_drill_backbone.is_none() {
        sim.pre_drill_backbone = Some(sim.net.backbone.clone());
    }
    sim.set_network(net);
    Ok(hit)
}

pub struct Preset {
    pub key: &'static str,
    pub title: &'static str,
}

pub const PRESETS: &[Preset] = &[
    Preset { key: "upwind", title: "Upwind, 14 kn, chop" },
    Preset { key: "reach", title: "Reach, 18 kn, gusty" },
    Preset { key: "downwind", title: "Downwind, 22 kn, big following sea" },
    Preset { key: "light", title: "Light airs, flat" },
    Preset { key: "shoaling", title: "Shoaling ahead" },
];

pub fn preset(sim: &mut Sim, key: &str) -> Result<(), String> {
    let base = EnvCfg { seed: sim.world.env.seed, start_lat: sim.world.env.start_lat, start_lon: sim.world.env.start_lon, ..EnvCfg::default() };
    let (env, course) = match key {
        "upwind" => (EnvCfg { tws_kn: 14.0, twd_deg: 225.0, wave_height_m: 0.8, wave_period_s: 4.0, wave_dir_deg: 225.0, ..base }, 180.0),
        "reach" => (EnvCfg { tws_kn: 18.0, twd_deg: 270.0, gust_pct: 18.0, shift_deg: 8.0, wave_height_m: 1.2, wave_period_s: 5.0, wave_dir_deg: 260.0, ..base }, 170.0),
        "downwind" => (EnvCfg { tws_kn: 22.0, twd_deg: 250.0, gust_pct: 15.0, wave_height_m: 2.2, wave_period_s: 7.0, wave_dir_deg: 250.0, ..base }, 85.0),
        "light" => (EnvCfg { tws_kn: 5.0, twd_deg: 200.0, gust_pct: 20.0, shift_deg: 10.0, wave_height_m: 0.1, current_kn: 0.3, ..base }, 120.0),
        "shoaling" => (EnvCfg { tws_kn: 12.0, twd_deg: 270.0, depth_m: 9.0, shoal_m_per_km: 6.0, shoal_dir_deg: 0.0, wave_height_m: 0.4, ..base }, 0.0),
        _ => return Err("unknown preset".into()),
    };
    sim.world.env = env;
    sim.world.helm.course_deg = course;
    if sim.world.helm.mode == HelmMode::Manual {
        sim.world.helm.mode = HelmMode::Hold;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world::HelmCfg;

    #[test]
    fn every_drill_applies_and_shows_up_in_the_checks() {
        for d in DRILLS {
            let mut s = Sim::new(default_network(), EnvCfg::default(), HelmCfg::default());
            s.run_for(3.0);
            drill(&mut s, d.key).unwrap();
            s.run_for(30.0);
            let flagged = s.checks.iter().any(|c| c.level != "info");
            assert!(flagged, "drill {} raised nothing: {:?}", d.key, s.checks);
        }
    }
}

pub struct NetworkPreset {
    pub key: &'static str,
    pub title: &'static str,
    pub what: &'static str,
}

pub const NETWORKS: &[NetworkPreset] = &[
    NetworkPreset { key: "generic", title: "Generic race boat", what: "The default network of simulated devices." },
    NetworkPreset {
        key: "bg",
        title: "B&G race boat",
        what: "H5000 system with Precision-9, ZG100, WS310, Airmar DST810, Triton2, RF25N. B&G publishes no H5000 PGN lists, so the H5000 units send no data here.",
    },
    NetworkPreset {
        key: "nke",
        title: "NKE race boat",
        what: "NKE's sensors and pilot are on its own Topline bus; the Box N2K puts their data on NMEA 2000 beside a chart plotter.",
    },
    NetworkPreset {
        key: "raymarine",
        title: "Raymarine boat",
        what: "Evolution EV-1 and ACU-200, iTC-5 with wind, speed and depth transducers, two i70s, a generic GNSS. Powered from the ACU.",
    },
];

fn lib(key: &str, id: &str, addr: u8, tee: f64, drop: f64, unique: u32) -> Result<crate::network::DeviceCfg, String> {
    let e = crate::library::find(key).ok_or_else(|| format!("library has no {key}"))?;
    let mut d = e.to_device(id, unique);
    d.preferred_address = addr;
    d.tee_m = tee;
    d.drop_m = drop;
    Ok(d)
}

fn generic(kind: DeviceKind, id: &str, label: &str, addr: u8, tee: f64, drop: f64, unique: u32) -> crate::network::DeviceCfg {
    let mut d = template(kind, id, label, unique);
    d.preferred_address = addr;
    d.tee_m = tee;
    d.drop_m = drop;
    d
}

/// The simulator's own rudder drive, which takes angle orders from the
/// autopilot. Every preset has one so external steering works.
fn drive(tee: f64) -> crate::network::DeviceCfg {
    generic(DeviceKind::Rudder, "drive", "Simulated rudder drive", 6, tee, 1.0, 601_001)
}

pub fn network_preset(key: &str) -> Result<Network, String> {
    use crate::network::Backbone;
    Ok(match key {
        "generic" => default_network(),
        "bg" => Network {
            name: "B&G race boat".into(),
            // The WS310 interface sits at the mast base, so the backbone
            // stops there.
            backbone: Backbone { length_m: 12.0, power_tap_m: 6.0, ..Default::default() },
            devices: vec![
                drive(0.6),
                lib("navico_rf25n", "rf25n", 12, 0.9, 1.0, 610_001)?,
                lib("bg_h5000_pilot_computer", "h5000_pilot", 13, 1.5, 1.0, 610_002)?,
                lib("bg_zg100", "zg100", 2, 2.5, 2.0, 610_003)?,
                lib("bg_h5000_graphic", "h5000_graphic", 14, 3.2, 1.0, 610_004)?,
                lib("bg_h5000_pilot_controller", "h5000_controller", 15, 3.5, 1.0, 610_005)?,
                lib("maretron_dcm100", "dcm100", 7, 5.0, 0.5, 610_006)?,
                lib("bg_h5000_cpu", "h5000_cpu", 16, 6.2, 0.5, 610_007)?,
                lib("yd_ydwg02", "gateway", 8, 6.8, 0.5, 610_008)?,
                lib("bg_precision9", "precision9", 3, 8.0, 1.0, 610_009)?,
                lib("airmar_dst810", "dst810", 4, 9.5, 1.5, 610_010)?,
                lib("bg_triton2", "triton2", 17, 11.0, 1.0, 610_011)?,
                lib("bg_ws310", "ws310", 5, 11.6, 0.5, 610_012)?,
            ],
        },
        "nke" => Network {
            name: "NKE race boat".into(),
            backbone: Backbone { length_m: 8.0, power_tap_m: 4.0, ..Default::default() },
            devices: vec![
                drive(0.6),
                generic(DeviceKind::Display, "plotter", "Chart plotter", 1, 5.5, 1.0, 620_001),
                lib("nke_box_n2k", "box_n2k", 20, 6.2, 0.5, 620_002)?,
                lib("yd_ydwg02", "gateway", 8, 6.8, 0.5, 620_003)?,
            ],
        },
        "raymarine" => {
            let mut itc5 = lib("raymarine_itc5", "itc5", 4, 9.0, 1.0, 630_004)?;
            // No rudder reference or fluxgate on the iTC-5: the ACU and the
            // EV-1 provide those.
            for p in &mut itc5.pgns {
                if p.pgn == n2k::pgn::RUDDER || p.pgn == n2k::pgn::HEADING {
                    p.enabled = false;
                }
            }
            Network {
                name: "Raymarine boat".into(),
                // Raymarine says the ACU-200 powers the SeaTalkng bus.
                backbone: Backbone { length_m: 12.0, power_tap_m: 1.0, ..Default::default() },
                devices: vec![
                    drive(0.6),
                    lib("raymarine_acu200", "acu200", 11, 1.0, 1.0, 630_001)?,
                    generic(DeviceKind::Gnss, "gnss", "GNSS (generic)", 2, 2.5, 2.0, 630_002),
                    lib("raymarine_i70s", "i70s_port", 14, 3.0, 1.0, 630_003)?,
                    lib("raymarine_i70s", "i70s_stbd", 15, 3.4, 1.0, 630_005)?,
                    generic(DeviceKind::Battery, "battery", "Battery monitor (generic)", 7, 5.0, 0.5, 630_006),
                    lib("yd_ydwg02", "gateway", 8, 6.8, 0.5, 630_007)?,
                    lib("raymarine_ev1", "ev1", 3, 8.0, 1.0, 630_008)?,
                    itc5,
                ],
            }
        }
        _ => return Err("unknown network".into()),
    })
}

#[cfg(test)]
mod network_tests {
    use super::*;
    use crate::device::DevState;
    use crate::world::HelmCfg;

    #[test]
    fn every_network_preset_comes_up_cleanly() {
        for n in NETWORKS {
            let net = network_preset(n.key).unwrap();
            let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
            s.run_for(8.0);
            for d in &s.devices {
                assert_eq!(d.state, DevState::Online, "{}: {}", n.key, d.cfg.id);
                assert_eq!(d.address, d.cfg.preferred_address, "{}: {} moved", n.key, d.cfg.id);
            }
            assert!(s.bus_load() < 0.4, "{}: load {}", n.key, s.bus_load());
            let errors: Vec<_> = s.checks.iter().filter(|c| c.level == "error").map(|c| c.title.clone()).collect();
            assert!(errors.is_empty(), "{}: {:?}", n.key, errors);
            // External steering reaches the simulator's drive.
            s.world.helm.mode = crate::world::HelmMode::External;
            let order = n2k::Builder::new(n2k::pgn::RUDDER).num("instance", 0.0).num("angleOrder", 0.1).encode();
            let id = n2k::CanId::new(2, n2k::pgn::RUDDER, 50, 255).encode();
            for _ in 0..20 {
                s.inbox.push((1, n2k::Frame::new(id, &order)));
                s.run_for(0.1);
            }
            assert!((s.world.s.rudder - 0.1).abs() < 0.01, "{}: rudder {}", n.key, s.world.s.rudder);
        }
    }

    #[test]
    fn drills_find_a_target_on_every_network() {
        for n in NETWORKS {
            for d in DRILLS {
                let mut s = Sim::new(network_preset(n.key).unwrap(), EnvCfg::default(), HelmCfg::default());
                s.run_for(2.0);
                let before = s.net.backbone.clone();
                drill(&mut s, d.key).unwrap_or_else(|e| panic!("{} on {}: {e}", d.key, n.key));
                drill(&mut s, "clear_faults").unwrap();
                assert_eq!(s.net.backbone, before, "{} on {}: backbone not restored", d.key, n.key);
            }
        }
    }
}
