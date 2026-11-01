//! The network as configured: backbone, power and devices. Saved as JSON.
//!
//! Generic devices use class and function codes from canboat's lookups.
//! Their manufacturer code (2000) and product details are placeholders: code
//! 2000 is not in canboat's manufacturer list, which may not be complete.
//! Real products come from the instrument library (`library.rs`).

use n2k::pgn;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum DeviceKind {
    /// GNSS receiver, dual antenna: position, COG and SOG, time, true heading.
    Gnss,
    /// Heading and motion sensor: magnetic heading, rate of turn, attitude.
    Compass,
    /// Masthead wind sensor: apparent wind.
    Wind,
    /// Depth, speed and temperature transducer.
    Dst,
    /// Rudder drive: reports rudder angle, takes angle orders (PGN 127245).
    Rudder,
    /// Battery monitor.
    Battery,
    /// Chart plotter or display: asks every device who it is.
    Display,
    /// Gateway to a computer: carries frames from TCP clients onto the bus.
    Gateway,
    /// Instrument or autopilot processor that publishes many kinds of data.
    Processor,
    /// Weather station: wind, air temperature, pressure, often position too.
    Weather,
}

impl DeviceKind {
    pub const ALL: [DeviceKind; 10] = [
        DeviceKind::Gnss,
        DeviceKind::Compass,
        DeviceKind::Wind,
        DeviceKind::Dst,
        DeviceKind::Rudder,
        DeviceKind::Battery,
        DeviceKind::Display,
        DeviceKind::Gateway,
        DeviceKind::Processor,
        DeviceKind::Weather,
    ];

    pub fn key(self) -> &'static str {
        match self {
            DeviceKind::Gnss => "gnss",
            DeviceKind::Compass => "compass",
            DeviceKind::Wind => "wind",
            DeviceKind::Dst => "dst",
            DeviceKind::Rudder => "rudder",
            DeviceKind::Battery => "battery",
            DeviceKind::Display => "display",
            DeviceKind::Gateway => "gateway",
            DeviceKind::Processor => "processor",
            DeviceKind::Weather => "weather",
        }
    }

    pub fn title(self) -> &'static str {
        match self {
            DeviceKind::Gnss => "GNSS compass",
            DeviceKind::Compass => "Heading sensor",
            DeviceKind::Wind => "Masthead wind",
            DeviceKind::Dst => "Depth, speed, temp",
            DeviceKind::Rudder => "Rudder drive",
            DeviceKind::Battery => "Battery monitor",
            DeviceKind::Display => "Display",
            DeviceKind::Gateway => "PC gateway",
            DeviceKind::Processor => "Instrument processor",
            DeviceKind::Weather => "Weather station",
        }
    }

    /// Data PGNs this kind can send, with default intervals in ms.
    pub fn default_pgns(self) -> Vec<(u32, u32)> {
        use pgn::*;
        match self {
            DeviceKind::Gnss => vec![(POSITION_RAPID, 100), (COG_SOG, 250), (GNSS_POSITION, 1000), (SYSTEM_TIME, 1000), (HEADING, 100)],
            DeviceKind::Compass => vec![(HEADING, 100), (RATE_OF_TURN, 100), (ATTITUDE, 100)],
            DeviceKind::Wind => vec![(WIND, 100)],
            DeviceKind::Dst => vec![(SPEED, 1000), (DEPTH, 1000), (TEMPERATURE, 2000), (DISTANCE_LOG, 1000)],
            DeviceKind::Rudder => vec![(RUDDER, 100)],
            DeviceKind::Battery => vec![(BATTERY, 1500)],
            DeviceKind::Display => vec![],
            DeviceKind::Gateway => vec![],
            DeviceKind::Processor => vec![(WIND, 100), (SPEED, 1000), (HEADING, 100), (ATTITUDE, 100)],
            DeviceKind::Weather => vec![(WIND, 100), (ENV_PARAMETERS, 500), (PRESSURE, 2000)],
        }
    }

    /// PGNs this kind listens to, for its receive list (126464).
    pub fn receive_pgns(self) -> Vec<u32> {
        use pgn::*;
        let mut v = vec![ISO_REQUEST, ADDRESS_CLAIM];
        match self {
            DeviceKind::Rudder => v.push(RUDDER),
            DeviceKind::Display => v.extend([HEADING, RATE_OF_TURN, ATTITUDE, POSITION_RAPID, COG_SOG, GNSS_POSITION, SPEED, DEPTH, WIND, TEMPERATURE, BATTERY, RUDDER, DISTANCE_LOG, PRODUCT_INFO, PGN_LIST]),
            _ => {}
        }
        v
    }

    /// (device class, device function) for a generic device of this kind,
    /// chosen from canboat's DEVICE_CLASS and DEVICE_FUNCTION lookups
    /// (v9.1.0). Library instruments use their own where published.
    pub fn class_function(self) -> (u8, u8) {
        match self {
            DeviceKind::Gnss => (60, 145),      // Navigation: Ownship Position (GNSS)
            DeviceKind::Compass => (60, 140),   // Navigation: Ownship Attitude
            DeviceKind::Wind => (85, 130),      // External Environment: Atmospheric
            DeviceKind::Dst => (60, 136),       // Navigation: Bottom Depth/Speed/Temperature
            DeviceKind::Rudder => (40, 155),    // Steering and Control surfaces: Rudder
            DeviceKind::Battery => (35, 170),   // Electrical Generation: Battery
            DeviceKind::Display => (120, 130),  // Display: Display
            DeviceKind::Gateway => (25, 130),   // Internetwork device: PC Gateway
            DeviceKind::Processor => (60, 175), // Navigation: Integrated Navigation System
            DeviceKind::Weather => (80, 180),   // Instrumentation: Weather Instruments
        }
    }

    fn default_len(self) -> u8 {
        match self {
            DeviceKind::Gnss => 2,
            DeviceKind::Display => 1,
            _ => 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Cable {
    Micro,
    Mid,
    Heavy,
}

impl Cable {
    /// Resistance of one power conductor, ohms per metre. From standard
    /// copper wire-gauge tables for the gauges Actisense's network guide
    /// gives (Micro 22 AWG, Mid 18 AWG, Heavy 15 AWG). An approximation:
    /// no cable maker's figure was found.
    pub fn ohm_per_m(self) -> f64 {
        match self {
            Cable::Micro => 0.0530,
            Cable::Mid => 0.0210,
            Cable::Heavy => 0.0104,
        }
    }

    /// Terminator-to-terminator limit, metres (Actisense network guide).
    pub fn max_backbone_m(self) -> f64 {
        match self {
            Cable::Micro => 100.0,
            Cable::Mid | Cable::Heavy => 250.0,
        }
    }

    /// Current limit, amps (Actisense network guide).
    pub fn max_current_a(self) -> f64 {
        match self {
            Cable::Micro => 3.0,
            Cable::Mid => 4.0,
            Cable::Heavy => 8.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Backbone {
    pub length_m: f64,
    pub cable: Cable,
    /// Terminator at the start (0 m) and at the end of the backbone.
    pub term_start: bool,
    pub term_end: bool,
    /// Where the power tee joins the backbone.
    pub power_tap_m: f64,
    pub supply_v: f64,
    pub powered: bool,
    /// Bit rate. NMEA 2000 is 250 kbit/s.
    pub bitrate: f64,
}

impl Default for Backbone {
    fn default() -> Self {
        Backbone {
            length_m: 22.0,
            cable: Cable::Mid,
            term_start: true,
            term_end: true,
            power_tap_m: 6.0,
            supply_v: 12.6,
            powered: true,
            bitrate: 250_000.0,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct PgnCfg {
    pub pgn: u32,
    pub enabled: bool,
    pub interval_ms: u32,
    /// Which flavour of the data, where a PGN has several. Empty for the
    /// kind's default. See `device::VARIANTS`.
    pub variant: String,
}

impl Default for PgnCfg {
    fn default() -> Self {
        PgnCfg {
            pgn: 0,
            enabled: true,
            interval_ms: 1000,
            variant: String::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
#[serde(default)]
pub struct NameCfg {
    pub unique_number: u32,
    pub manufacturer_code: u16,
    pub device_function: u8,
    pub device_class: u8,
    pub device_instance: u8,
    pub system_instance: u8,
    pub industry_group: u8,
    pub arbitrary_address_capable: bool,
}

impl NameCfg {
    pub fn to_name(&self) -> n2k::Name {
        n2k::Name {
            unique_number: self.unique_number & 0x1F_FFFF,
            manufacturer_code: self.manufacturer_code & 0x7FF,
            device_instance: self.device_instance,
            device_function: self.device_function,
            device_class: self.device_class & 0x7F,
            system_instance: self.system_instance & 0xF,
            industry_group: self.industry_group & 7,
            arbitrary_address_capable: self.arbitrary_address_capable,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
#[serde(default)]
pub struct ProductCfg {
    pub model_id: String,
    pub software_version: String,
    pub model_version: String,
    pub serial: String,
    pub product_code: u16,
    pub certification_level: u8,
}

/// Faults a device can have. Defaults are a healthy device.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Faults {
    /// Stops sending data but keeps its address (hung firmware).
    pub silent: bool,
    /// Keeps sending the same values.
    pub stuck: bool,
    /// Sends "not available" in its data fields (for GNSS: no fix).
    pub na_data: bool,
    /// Added to every angle it reports, degrees.
    pub angle_offset_deg: f64,
    /// Added to every speed and depth it reports, percent of the value.
    pub scale_pct: f64,
    /// Multiplies the sensor noise.
    pub noise_mult: f64,
    /// Multiplies every transmit interval.
    pub rate_mult: f64,
    /// Random extra delay on each transmission, ms.
    pub jitter_ms: f64,
    /// Floods the bus with priority 0 frames at this rate (babbling idiot).
    pub babble_hz: f64,
    /// Share of this device's frames its transceiver corrupts, percent.
    pub tx_error_pct: f64,
    /// Loose connector: drops off the bus for a few seconds at random.
    pub intermittent: bool,
}

impl Default for Faults {
    fn default() -> Self {
        Faults {
            silent: false,
            stuck: false,
            na_data: false,
            angle_offset_deg: 0.0,
            scale_pct: 0.0,
            noise_mult: 1.0,
            rate_mult: 1.0,
            jitter_ms: 0.0,
            babble_hz: 0.0,
            tx_error_pct: 0.0,
            intermittent: false,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct DeviceCfg {
    pub id: String,
    pub kind: DeviceKind,
    pub label: String,
    /// Plugged in and switched on.
    pub enabled: bool,
    /// Tee position along the backbone, metres from the start.
    pub tee_m: f64,
    pub drop_m: f64,
    /// Load equivalency number: 1 LEN is 50 mA.
    pub len: u8,
    pub preferred_address: u8,
    /// Data instance (rudder, battery, temperature).
    pub instance: u8,
    pub name: NameCfg,
    pub product: ProductCfg,
    pub pgns: Vec<PgnCfg>,
    /// PGNs it lists as received in 126464. Empty for the kind's default.
    pub rx_pgns: Vec<u32>,
    /// Instrument library key, empty for a generic device.
    pub library: String,
    pub faults: Faults,
}

impl Default for DeviceCfg {
    fn default() -> Self {
        template(DeviceKind::Display, "device", "Device", 0)
    }
}

/// A healthy device of `kind` with default PGNs.
pub fn template(kind: DeviceKind, id: &str, label: &str, unique: u32) -> DeviceCfg {
    let (class, function) = kind.class_function();
    DeviceCfg {
        id: id.to_string(),
        kind,
        label: label.to_string(),
        enabled: true,
        tee_m: 5.0,
        drop_m: 1.0,
        len: kind.default_len(),
        preferred_address: 0,
        instance: 0,
        name: NameCfg {
            unique_number: unique,
            manufacturer_code: 2000,
            device_function: function,
            device_class: class,
            device_instance: 0,
            system_instance: 0,
            industry_group: 4,
            arbitrary_address_capable: true,
        },
        product: ProductCfg {
            model_id: format!("Sim {}", kind.title()),
            software_version: "1.0.0".into(),
            model_version: "A".into(),
            serial: format!("SIM{unique:06}"),
            product_code: 1000 + kind as u16,
            certification_level: 1,
        },
        pgns: kind
            .default_pgns()
            .into_iter()
            .map(|(pgn, interval_ms)| PgnCfg {
                pgn,
                enabled: true,
                interval_ms,
                variant: crate::device::default_variant(kind, pgn).to_string(),
            })
            .collect(),
        rx_pgns: Vec::new(),
        library: String::new(),
        faults: Faults::default(),
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Default)]
#[serde(default)]
pub struct Network {
    /// Shown in the drawing's title block.
    pub name: String,
    pub backbone: Backbone,
    pub devices: Vec<DeviceCfg>,
}

/// A 34 ft race boat's network: positions measured from the stern along the
/// backbone, which runs forward to the mast and up it to the masthead.
pub fn default_network() -> Network {
    let mk = |kind, id: &str, label: &str, unique, addr, tee, drop| {
        let mut d = template(kind, id, label, unique);
        d.preferred_address = addr;
        d.tee_m = tee;
        d.drop_m = drop;
        d
    };
    let mut gateway = mk(DeviceKind::Gateway, "gateway", "PC gateway (autopilot)", 108_001, 8, 6.8, 0.5);
    gateway.len = 1;
    Network {
        name: "Generic race boat".into(),
        backbone: Backbone::default(),
        devices: vec![
            mk(DeviceKind::Rudder, "rudder", "Rudder drive", 101_001, 6, 0.8, 1.0),
            mk(DeviceKind::Gnss, "gnss", "GNSS compass", 102_001, 2, 2.5, 2.0),
            mk(DeviceKind::Battery, "battery", "Battery monitor", 103_001, 7, 5.0, 0.5),
            mk(DeviceKind::Display, "display", "Chart plotter", 104_001, 1, 6.2, 1.0),
            gateway,
            mk(DeviceKind::Compass, "compass", "Heading sensor", 105_001, 3, 8.0, 1.0),
            mk(DeviceKind::Dst, "dst", "Depth, speed, temp", 106_001, 4, 9.5, 1.5),
            mk(DeviceKind::Wind, "wind", "Masthead wind", 107_001, 5, 21.8, 0.3),
        ],
    }
}
