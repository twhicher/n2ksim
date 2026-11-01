//! PGN field definitions and generic encode and decode.
//!
//! Each definition lists its fields in wire order. Numbers are stored raw
//! times `res`; values are SI (radians, m/s, metres, kelvin, pascals) except
//! latitude and longitude, which are degrees as on the wire.
//!
//! "Not available": all ones for unsigned fields and lookups, the largest
//! positive value for signed fields. All-ones minus one ("out of range") is
//! decoded as not available too.

use crate::bits::{get, mask, set};

pub const ISO_ACK: u32 = 59392;
pub const ISO_REQUEST: u32 = 59904;
pub const ADDRESS_CLAIM: u32 = 60928;
pub const PROPRIETARY_SINGLE: u32 = 61184;
pub const PGN_LIST: u32 = 126464;
pub const SYSTEM_TIME: u32 = 126992;
pub const HEARTBEAT: u32 = 126993;
pub const PRODUCT_INFO: u32 = 126996;
pub const RUDDER: u32 = 127245;
pub const HEADING: u32 = 127250;
pub const RATE_OF_TURN: u32 = 127251;
pub const ATTITUDE: u32 = 127257;
pub const BATTERY: u32 = 127508;
pub const SPEED: u32 = 128259;
pub const DEPTH: u32 = 128267;
pub const DISTANCE_LOG: u32 = 128275;
pub const POSITION_RAPID: u32 = 129025;
pub const COG_SOG: u32 = 129026;
pub const GNSS_POSITION: u32 = 129029;
pub const XTE: u32 = 129283;
pub const NAVIGATION_DATA: u32 = 129284;
pub const WIND: u32 = 130306;
pub const TEMPERATURE: u32 = 130312;
pub const PRESSURE: u32 = 130314;
pub const HEAVE: u32 = 127252;
pub const MAGNETIC_VARIATION: u32 = 127258;
pub const TIME_DATE: u32 = 129033;
pub const GNSS_DOPS: u32 = 129539;
pub const ENV_OBSOLETE: u32 = 130310;
pub const ENV_PARAMETERS: u32 = 130311;
pub const HUMIDITY: u32 = 130313;
pub const TEMPERATURE_EXT: u32 = 130316;
pub const DIRECTION_DATA: u32 = 130577;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Num {
        signed: bool,
        res: f64,
        unit: &'static str,
    },
    Lookup(&'static [(u32, &'static str)]),
    Reserved,
    /// Fixed-length text, padded with 0xFF.
    Str,
    /// Raw bits with no scaling.
    Bin,
    /// 126464 only: 24-bit PGNs to the end of the message.
    PgnList,
}

#[derive(Clone, Copy, Debug)]
pub struct Field {
    pub id: &'static str,
    pub name: &'static str,
    pub bits: u16,
    pub kind: Kind,
}

#[derive(Debug)]
pub struct PgnDef {
    pub pgn: u32,
    pub name: &'static str,
    pub fast: bool,
    pub priority: u8,
    /// Usual interval in milliseconds, 0 if sent on request or on change.
    pub interval_ms: u32,
    /// Pad single-frame messages to 8 bytes (false only for ISO request).
    pub pad8: bool,
    pub fields: &'static [Field],
}

const fn un(id: &'static str, name: &'static str, bits: u16, res: f64, unit: &'static str) -> Field {
    Field {
        id,
        name,
        bits,
        kind: Kind::Num {
            signed: false,
            res,
            unit,
        },
    }
}
const fn sn(id: &'static str, name: &'static str, bits: u16, res: f64, unit: &'static str) -> Field {
    Field {
        id,
        name,
        bits,
        kind: Kind::Num {
            signed: true,
            res,
            unit,
        },
    }
}
const fn lk(id: &'static str, name: &'static str, bits: u16, t: &'static [(u32, &'static str)]) -> Field {
    Field {
        id,
        name,
        bits,
        kind: Kind::Lookup(t),
    }
}
const fn rs(bits: u16) -> Field {
    Field {
        id: "reserved",
        name: "Reserved",
        bits,
        kind: Kind::Reserved,
    }
}
const fn st(id: &'static str, name: &'static str, bytes: u16) -> Field {
    Field {
        id,
        name,
        bits: bytes * 8,
        kind: Kind::Str,
    }
}
const fn bn(id: &'static str, name: &'static str, bits: u16) -> Field {
    Field {
        id,
        name,
        bits,
        kind: Kind::Bin,
    }
}
const fn sid() -> Field {
    un("sid", "SID", 8, 1.0, "")
}

const fn def(
    pgn: u32,
    name: &'static str,
    fast: bool,
    priority: u8,
    interval_ms: u32,
    fields: &'static [Field],
) -> PgnDef {
    PgnDef {
        pgn,
        name,
        fast,
        priority,
        interval_ms,
        pad8: true,
        fields,
    }
}

const RAD: &str = "rad";
const DIRECTION_REF: &[(u32, &str)] = &[(0, "True"), (1, "Magnetic"), (2, "Error")];
const WIND_REF: &[(u32, &str)] = &[
    (0, "True (ground referenced to North)"),
    (1, "Magnetic (ground referenced to Magnetic North)"),
    (2, "Apparent"),
    (3, "True (boat referenced)"),
    (4, "True (water referenced)"),
];
const GNSS_TYPE: &[(u32, &str)] = &[
    (0, "GPS"),
    (1, "GLONASS"),
    (2, "GPS+GLONASS"),
    (3, "GPS+SBAS/WAAS"),
    (4, "GPS+SBAS/WAAS+GLONASS"),
    (5, "Chayka"),
    (6, "integrated"),
    (7, "surveyed"),
    (8, "Galileo"),
];
const GNSS_METHOD: &[(u32, &str)] = &[
    (0, "no GNSS"),
    (1, "GNSS fix"),
    (2, "DGNSS fix"),
    (3, "Precise GNSS"),
    (4, "RTK Fixed Integer"),
    (5, "RTK float"),
    (6, "Estimated (DR) mode"),
    (7, "Manual Input"),
    (8, "Simulate mode"),
];
const TEMPERATURE_SOURCE: &[(u32, &str)] = &[
    (0, "Sea Temperature"),
    (1, "Outside Temperature"),
    (2, "Inside Temperature"),
    (3, "Engine Room Temperature"),
    (4, "Main Cabin Temperature"),
    (5, "Live Well Temperature"),
    (6, "Bait Well Temperature"),
    (7, "Refrigeration Temperature"),
    (8, "Heating System Temperature"),
    (9, "Dew Point Temperature"),
    (10, "Apparent Wind Chill Temperature"),
    (11, "Theoretical Wind Chill Temperature"),
    (12, "Heat Index Temperature"),
    (13, "Freezer Temperature"),
    (14, "Exhaust Gas Temperature"),
    (15, "Shaft Seal Temperature"),
];
const HUMIDITY_SOURCE: &[(u32, &str)] = &[(0, "Inside"), (1, "Outside")];
const GNSS_MODE: &[(u32, &str)] = &[(0, "1D"), (1, "2D"), (2, "3D"), (3, "Auto")];
const RESIDUAL_MODE: &[(u32, &str)] = &[(0, "Autonomous"), (1, "Differential enhanced"), (2, "Estimated"), (3, "Simulator"), (4, "Manual")];
const CONTROLLER_STATE: &[(u32, &str)] = &[(0, "Error Active"), (1, "Error Passive"), (2, "Bus Off")];

pub static DEFS: &[PgnDef] = &[
    PgnDef {
        pgn: ISO_ACK,
        name: "ISO Acknowledgement",
        fast: false,
        priority: 6,
        interval_ms: 0,
        pad8: true,
        fields: &[
            lk("control", "Control", 8, &[(0, "ACK"), (1, "NAK"), (2, "Access Denied"), (3, "Address Busy")]),
            un("groupFunction", "Group Function", 8, 1.0, ""),
            rs(24),
            bn("pgn", "PGN", 24),
        ],
    },
    PgnDef {
        pgn: ISO_REQUEST,
        name: "ISO Request",
        fast: false,
        priority: 6,
        interval_ms: 0,
        pad8: false,
        fields: &[bn("pgn", "PGN", 24)],
    },
    def(
        ADDRESS_CLAIM,
        "ISO Address Claim",
        false,
        6,
        0,
        &[
            bn("uniqueNumber", "Unique Number", 21),
            bn("manufacturerCode", "Manufacturer Code", 11),
            bn("deviceInstanceLower", "Device Instance Lower", 3),
            bn("deviceInstanceUpper", "Device Instance Upper", 5),
            bn("deviceFunction", "Device Function", 8),
            bn("spare", "Spare", 1),
            bn("deviceClass", "Device Class", 7),
            bn("systemInstance", "System Instance", 4),
            lk("industryGroup", "Industry Group", 3, &[(0, "Global"), (1, "Highway"), (2, "Agriculture"), (3, "Construction"), (4, "Marine Industry"), (5, "Industrial")]),
            bn("arbitraryAddressCapable", "Arbitrary Address Capable", 1),
        ],
    ),
    def(
        PROPRIETARY_SINGLE,
        "Proprietary single-frame addressed",
        false,
        6,
        0,
        &[bn("manufacturerCode", "Manufacturer Code", 11), rs(2), lk("industryCode", "Industry Code", 3, &[(4, "Marine Industry")]), bn("data", "Data", 48)],
    ),
    def(
        PGN_LIST,
        "PGN List (Transmit and Receive)",
        true,
        6,
        0,
        &[
            lk("functionCode", "Function Code", 8, &[(0, "Transmit PGN list"), (1, "Receive PGN list")]),
            Field {
                id: "pgns",
                name: "PGNs",
                bits: 0,
                kind: Kind::PgnList,
            },
        ],
    ),
    def(
        SYSTEM_TIME,
        "System Time",
        false,
        3,
        1000,
        &[
            sid(),
            lk("source", "Source", 4, &[(0, "GPS"), (1, "GLONASS"), (2, "Radio Station"), (3, "Local Cesium clock"), (4, "Local Rubidium clock"), (5, "Local Crystal clock")]),
            rs(4),
            un("date", "Date", 16, 1.0, "d"),
            un("time", "Time", 32, 0.0001, "s"),
        ],
    ),
    def(
        HEARTBEAT,
        "Heartbeat",
        false,
        7,
        60000,
        &[
            un("dataTransmitOffset", "Data transmit offset", 16, 0.001, "s"),
            un("sequenceCounter", "Sequence Counter", 8, 1.0, ""),
            lk("controller1State", "Controller 1 State", 2, CONTROLLER_STATE),
            lk("controller2State", "Controller 2 State", 2, CONTROLLER_STATE),
            lk("equipmentStatus", "Equipment Status", 2, &[(0, "Operational"), (1, "Fault")]),
            rs(34),
        ],
    ),
    def(
        PRODUCT_INFO,
        "Product Information",
        true,
        6,
        0,
        &[
            un("nmea2000Version", "NMEA 2000 Version", 16, 0.001, ""),
            un("productCode", "Product Code", 16, 1.0, ""),
            st("modelId", "Model ID", 32),
            st("softwareVersionCode", "Software Version Code", 32),
            st("modelVersion", "Model Version", 32),
            st("modelSerialCode", "Model Serial Code", 32),
            un("certificationLevel", "Certification Level", 8, 1.0, ""),
            un("loadEquivalency", "Load Equivalency", 8, 1.0, "LEN"),
        ],
    ),
    def(
        RUDDER,
        "Rudder",
        false,
        2,
        100,
        &[
            un("instance", "Instance", 8, 1.0, ""),
            lk("directionOrder", "Direction Order", 3, &[(0, "No Order"), (1, "Move to starboard"), (2, "Move to port")]),
            rs(5),
            sn("angleOrder", "Angle Order", 16, 0.0001, RAD),
            sn("position", "Position", 16, 0.0001, RAD),
            rs(16),
        ],
    ),
    def(
        HEADING,
        "Vessel Heading",
        false,
        2,
        100,
        &[
            sid(),
            un("heading", "Heading", 16, 0.0001, RAD),
            sn("deviation", "Deviation", 16, 0.0001, RAD),
            sn("variation", "Variation", 16, 0.0001, RAD),
            lk("reference", "Reference", 2, DIRECTION_REF),
            rs(6),
        ],
    ),
    def(
        RATE_OF_TURN,
        "Rate of Turn",
        false,
        2,
        100,
        &[sid(), sn("rate", "Rate", 32, 3.125e-8, "rad/s"), rs(24)],
    ),
    def(
        ATTITUDE,
        "Attitude",
        false,
        3,
        1000,
        &[
            sid(),
            sn("yaw", "Yaw", 16, 0.0001, RAD),
            sn("pitch", "Pitch", 16, 0.0001, RAD),
            sn("roll", "Roll", 16, 0.0001, RAD),
            rs(8),
        ],
    ),
    def(
        BATTERY,
        "Battery Status",
        false,
        6,
        1500,
        &[
            un("instance", "Instance", 8, 1.0, ""),
            sn("voltage", "Voltage", 16, 0.01, "V"),
            sn("current", "Current", 16, 0.1, "A"),
            un("temperature", "Temperature", 16, 0.01, "K"),
            sid(),
        ],
    ),
    def(
        SPEED,
        "Speed",
        false,
        2,
        1000,
        &[
            sid(),
            un("speedWaterReferenced", "Speed Water Referenced", 16, 0.01, "m/s"),
            un("speedGroundReferenced", "Speed Ground Referenced", 16, 0.01, "m/s"),
            lk("speedWaterReferencedType", "Speed Water Referenced Type", 8, &[(0, "Paddle wheel"), (1, "Pitot tube"), (2, "Doppler"), (3, "Correlation (ultra sound)"), (4, "Electro Magnetic")]),
            un("speedDirection", "Speed Direction", 4, 1.0, ""),
            rs(12),
        ],
    ),
    def(
        DEPTH,
        "Water Depth",
        false,
        3,
        1000,
        &[
            sid(),
            un("depth", "Depth", 32, 0.01, "m"),
            sn("offset", "Offset", 16, 0.001, "m"),
            un("range", "Range", 8, 10.0, "m"),
        ],
    ),
    def(
        DISTANCE_LOG,
        "Distance Log",
        true,
        6,
        1000,
        &[
            un("date", "Date", 16, 1.0, "d"),
            un("time", "Time", 32, 0.0001, "s"),
            un("log", "Log", 32, 1.0, "m"),
            un("tripLog", "Trip Log", 32, 1.0, "m"),
        ],
    ),
    def(
        POSITION_RAPID,
        "Position, Rapid Update",
        false,
        2,
        100,
        &[sn("latitude", "Latitude", 32, 1e-7, "deg"), sn("longitude", "Longitude", 32, 1e-7, "deg")],
    ),
    def(
        COG_SOG,
        "COG & SOG, Rapid Update",
        false,
        2,
        250,
        &[
            sid(),
            lk("cogReference", "COG Reference", 2, DIRECTION_REF),
            rs(6),
            un("cog", "COG", 16, 0.0001, RAD),
            un("sog", "SOG", 16, 0.01, "m/s"),
            rs(16),
        ],
    ),
    def(
        GNSS_POSITION,
        "GNSS Position Data",
        true,
        3,
        1000,
        &[
            sid(),
            un("date", "Date", 16, 1.0, "d"),
            un("time", "Time", 32, 0.0001, "s"),
            sn("latitude", "Latitude", 64, 1e-16, "deg"),
            sn("longitude", "Longitude", 64, 1e-16, "deg"),
            sn("altitude", "Altitude", 64, 1e-6, "m"),
            lk("gnssType", "GNSS type", 4, GNSS_TYPE),
            lk("method", "Method", 4, GNSS_METHOD),
            lk("integrity", "Integrity", 2, &[(0, "No integrity checking"), (1, "Safe"), (2, "Caution")]),
            rs(6),
            un("numberOfSvs", "Number of SVs", 8, 1.0, ""),
            sn("hdop", "HDOP", 16, 0.01, ""),
            sn("pdop", "PDOP", 16, 0.01, ""),
            sn("geoidalSeparation", "Geoidal Separation", 32, 0.01, "m"),
            un("referenceStations", "Reference Stations", 8, 1.0, ""),
        ],
    ),
    def(
        XTE,
        "Cross Track Error",
        false,
        3,
        1000,
        &[
            sid(),
            lk("xteMode", "XTE mode", 4, &[(0, "Autonomous"), (1, "Differential enhanced"), (2, "Estimated"), (3, "Simulator"), (4, "Manual")]),
            rs(2),
            lk("navigationTerminated", "Navigation Terminated", 2, &[(0, "No"), (1, "Yes")]),
            sn("xte", "XTE", 32, 0.01, "m"),
            rs(16),
        ],
    ),
    def(
        NAVIGATION_DATA,
        "Navigation Data",
        true,
        3,
        1000,
        &[
            sid(),
            un("distanceToWaypoint", "Distance to Waypoint", 32, 0.01, "m"),
            lk("courseBearingReference", "Course/Bearing reference", 2, DIRECTION_REF),
            lk("perpendicularCrossed", "Perpendicular Crossed", 2, &[(0, "No"), (1, "Yes")]),
            lk("arrivalCircleEntered", "Arrival Circle Entered", 2, &[(0, "No"), (1, "Yes")]),
            lk("calculationType", "Calculation Type", 2, &[(0, "Great Circle"), (1, "Rhumb Line")]),
            un("etaTime", "ETA Time", 32, 0.0001, "s"),
            un("etaDate", "ETA Date", 16, 1.0, "d"),
            un("bearingOriginToDestination", "Bearing, Origin to Destination Waypoint", 16, 0.0001, RAD),
            un("bearingPositionToDestination", "Bearing, Position to Destination Waypoint", 16, 0.0001, RAD),
            un("originWaypointNumber", "Origin Waypoint Number", 32, 1.0, ""),
            un("destinationWaypointNumber", "Destination Waypoint Number", 32, 1.0, ""),
            sn("destinationLatitude", "Destination Latitude", 32, 1e-7, "deg"),
            sn("destinationLongitude", "Destination Longitude", 32, 1e-7, "deg"),
            sn("waypointClosingVelocity", "Waypoint Closing Velocity", 16, 0.01, "m/s"),
        ],
    ),
    def(
        WIND,
        "Wind Data",
        false,
        2,
        100,
        &[
            sid(),
            un("windSpeed", "Wind Speed", 16, 0.01, "m/s"),
            un("windAngle", "Wind Angle", 16, 0.0001, RAD),
            lk("reference", "Reference", 3, WIND_REF),
            rs(5),
            rs(16),
        ],
    ),
    def(
        TEMPERATURE,
        "Temperature",
        false,
        5,
        2000,
        &[
            sid(),
            un("instance", "Instance", 8, 1.0, ""),
            lk("source", "Source", 8, &[(0, "Sea Temperature"), (1, "Outside Temperature"), (2, "Inside Temperature"), (3, "Engine Room Temperature"), (4, "Main Cabin Temperature")]),
            un("actualTemperature", "Actual Temperature", 16, 0.01, "K"),
            un("setTemperature", "Set Temperature", 16, 0.01, "K"),
            rs(8),
        ],
    ),
    def(
        PRESSURE,
        "Actual Pressure",
        false,
        5,
        2000,
        &[
            sid(),
            un("instance", "Instance", 8, 1.0, ""),
            lk("source", "Source", 8, &[(0, "Atmospheric"), (1, "Water"), (2, "Steam"), (3, "Compressed Air"), (4, "Hydraulic")]),
            sn("pressure", "Pressure", 32, 0.1, "Pa"),
            rs(8),
        ],
    ),
    def(
        HEAVE,
        "Heave",
        false,
        3,
        0,
        &[
            sid(),
            sn("heave", "Heave", 16, 0.01, "m"),
            un("delay", "Delay", 16, 0.01, "s"),
            lk("delaySource", "Delay Source", 4, &[(0, "User defined"), (1, "Minimum defined"), (2, "Factory default"), (3, "Maximum supported")]),
            rs(4),
            rs(16),
        ],
    ),
    def(
        MAGNETIC_VARIATION,
        "Magnetic Variation",
        false,
        7,
        1000,
        &[
            sid(),
            lk("source", "Source", 4, &[(0, "Manual"), (1, "Automatic Chart"), (2, "Automatic Table"), (3, "Automatic Calculation"), (4, "WMM 2000"), (5, "WMM 2005"), (6, "WMM 2010"), (7, "WMM 2015"), (8, "WMM 2020"), (9, "WMM 2025")]),
            rs(4),
            un("ageOfService", "Age of service", 16, 1.0, "d"),
            sn("variation", "Variation", 16, 0.0001, RAD),
            rs(16),
        ],
    ),
    def(
        TIME_DATE,
        "Time & Date",
        false,
        3,
        1000,
        &[un("date", "Date", 16, 1.0, "d"), un("time", "Time", 32, 0.0001, "s"), sn("localOffset", "Local Offset", 16, 60.0, "s")],
    ),
    def(
        GNSS_DOPS,
        "GNSS DOPs",
        false,
        6,
        1000,
        &[
            sid(),
            lk("desiredMode", "Desired Mode", 3, GNSS_MODE),
            lk("actualMode", "Actual Mode", 3, GNSS_MODE),
            rs(2),
            sn("hdop", "HDOP", 16, 0.01, ""),
            sn("vdop", "VDOP", 16, 0.01, ""),
            sn("tdop", "TDOP", 16, 0.01, ""),
        ],
    ),
    def(
        ENV_OBSOLETE,
        "Environmental Parameters (obsolete)",
        false,
        5,
        500,
        &[
            sid(),
            un("waterTemperature", "Water Temperature", 16, 0.01, "K"),
            un("outsideAmbientAirTemperature", "Outside Ambient Air Temperature", 16, 0.01, "K"),
            un("atmosphericPressure", "Atmospheric Pressure", 16, 100.0, "Pa"),
            rs(8),
        ],
    ),
    def(
        ENV_PARAMETERS,
        "Environmental Parameters",
        false,
        5,
        500,
        &[
            sid(),
            lk("temperatureSource", "Temperature Source", 6, TEMPERATURE_SOURCE),
            lk("humiditySource", "Humidity Source", 2, HUMIDITY_SOURCE),
            un("temperature", "Temperature", 16, 0.01, "K"),
            sn("humidity", "Humidity", 16, 0.004, "%"),
            un("atmosphericPressure", "Atmospheric Pressure", 16, 100.0, "Pa"),
        ],
    ),
    def(
        HUMIDITY,
        "Humidity",
        false,
        5,
        2000,
        &[
            sid(),
            un("instance", "Instance", 8, 1.0, ""),
            lk("source", "Source", 8, HUMIDITY_SOURCE),
            sn("actualHumidity", "Actual Humidity", 16, 0.004, "%"),
            sn("setHumidity", "Set Humidity", 16, 0.004, "%"),
            rs(8),
        ],
    ),
    def(
        TEMPERATURE_EXT,
        "Temperature Extended Range",
        false,
        5,
        2000,
        &[
            sid(),
            un("instance", "Instance", 8, 1.0, ""),
            lk("source", "Source", 8, TEMPERATURE_SOURCE),
            un("temperature", "Temperature", 24, 0.001, "K"),
            un("setTemperature", "Set Temperature", 16, 0.1, "K"),
        ],
    ),
    def(
        DIRECTION_DATA,
        "Direction Data",
        true,
        3,
        1000,
        &[
            lk("dataMode", "Data Mode", 4, RESIDUAL_MODE),
            lk("cogReference", "COG Reference", 2, DIRECTION_REF),
            rs(2),
            sid(),
            un("cog", "COG", 16, 0.0001, RAD),
            un("sog", "SOG", 16, 0.01, "m/s"),
            un("heading", "Heading", 16, 0.0001, RAD),
            un("speedThroughWater", "Speed through Water", 16, 0.01, "m/s"),
            un("set", "Set", 16, 0.0001, RAD),
            un("drift", "Drift", 16, 0.01, "m/s"),
        ],
    ),
];

pub fn def_of(pgn: u32) -> Option<&'static PgnDef> {
    DEFS.iter().find(|d| d.pgn == pgn)
}

/// Fast-packet PGNs we know about, including ones we have no field
/// definitions for. The proprietary range 130816 to 131071 is fast packet.
pub fn is_fast(pgn: u32) -> bool {
    if let Some(d) = def_of(pgn) {
        return d.fast;
    }
    matches!(
        pgn,
        126208 | 126720 | 126983 | 126984 | 126985 | 126986 | 126987 | 126988 | 126998 | 127233
            | 127237 | 127489 | 127496 | 127497 | 127498 | 127503 | 127504 | 127506 | 127510
            | 128520 | 129038 | 129039 | 129040 | 129044 | 129045 | 129285 | 129540 | 129794
            | 129809 | 129810 | 130074 | 130577
    ) || (130816..=131071).contains(&pgn)
}

pub fn pgn_name(pgn: u32) -> &'static str {
    def_of(pgn).map(|d| d.name).unwrap_or(match pgn {
        126208 => "NMEA Group Function",
        126720 => "Proprietary fast-packet addressed",
        127237 => "Heading/Track control",
        127488 => "Engine Parameters, Rapid Update",
        129540 => "GNSS Sats in View",
        p if (65280..=65535).contains(&p) => "Proprietary single-frame",
        p if (130816..=131071).contains(&p) => "Proprietary fast-packet",
        _ => "Unknown",
    })
}

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Num(Option<f64>),
    Lookup(Option<u32>),
    Str(String),
    Bin(Option<u64>),
    List(Vec<u32>),
    Reserved,
}

/// A decoded message. `def` is `None` for PGNs we have no definition for.
#[derive(Clone, Debug)]
pub struct Message {
    pub pgn: u32,
    pub def: Option<&'static PgnDef>,
    pub values: Vec<(&'static Field, Value)>,
}

impl Message {
    pub fn get(&self, id: &str) -> Option<&Value> {
        self.values.iter().find(|(f, _)| f.id == id).map(|(_, v)| v)
    }
    pub fn num(&self, id: &str) -> Option<f64> {
        match self.get(id)? {
            Value::Num(v) => *v,
            Value::Bin(v) => v.map(|x| x as f64),
            Value::Lookup(v) => v.map(|x| x as f64),
            _ => None,
        }
    }
    pub fn lookup(&self, id: &str) -> Option<u32> {
        match self.get(id)? {
            Value::Lookup(v) => *v,
            Value::Bin(v) => v.map(|x| x as u32),
            Value::Num(v) => v.map(|x| x as u32),
            _ => None,
        }
    }
    pub fn text(&self, id: &str) -> Option<&str> {
        match self.get(id)? {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn list(&self) -> &[u32] {
        match self.get("pgns") {
            Some(Value::List(l)) => l,
            _ => &[],
        }
    }
}

fn signed_na(bits: usize) -> u64 {
    mask(bits - 1)
}

fn sign_extend(raw: u64, bits: usize) -> i128 {
    if bits < 64 && raw & (1u64 << (bits - 1)) != 0 {
        raw as i128 - (1i128 << bits)
    } else if bits == 64 {
        raw as i64 as i128
    } else {
        raw as i128
    }
}

/// Decode a whole message (already reassembled if fast packet).
pub fn decode(pgn: u32, data: &[u8]) -> Message {
    let def = def_of(pgn);
    let mut values = Vec::new();
    if let Some(d) = def {
        let mut pos = 0usize;
        for f in d.fields {
            let n = f.bits as usize;
            if let Kind::PgnList = f.kind {
                let mut l = Vec::new();
                let mut b = pos / 8;
                while b + 3 <= data.len() {
                    let p = data[b] as u32 | (data[b + 1] as u32) << 8 | (data[b + 2] as u32) << 16;
                    if p != 0xFF_FFFF {
                        l.push(p);
                    }
                    b += 3;
                }
                values.push((f, Value::List(l)));
                break;
            }
            if let Kind::Str = f.kind {
                let start = pos / 8;
                if start + n / 8 > data.len() {
                    break;
                }
                let bytes = &data[start..start + n / 8];
                let end = bytes.iter().position(|&c| c == 0xFF || c == 0 || c == b'@').unwrap_or(bytes.len());
                values.push((f, Value::Str(String::from_utf8_lossy(&bytes[..end]).trim_end().to_string())));
                pos += n;
                continue;
            }
            let Some(raw) = get(data, pos, n) else { break };
            pos += n;
            let v = match f.kind {
                Kind::Num { signed, res, .. } => {
                    if signed {
                        let na = signed_na(n);
                        if n >= 4 && (raw == na || raw == na - 1) {
                            Value::Num(None)
                        } else {
                            Value::Num(Some(sign_extend(raw, n) as f64 * res))
                        }
                    } else if n >= 4 && raw >= mask(n) - 1 {
                        Value::Num(None)
                    } else {
                        Value::Num(Some(raw as f64 * res))
                    }
                }
                Kind::Lookup(_) => Value::Lookup(if n >= 2 && raw == mask(n) { None } else { Some(raw as u32) }),
                Kind::Bin => Value::Bin(if n >= 8 && raw == mask(n) { None } else { Some(raw) }),
                Kind::Reserved => Value::Reserved,
                Kind::Str | Kind::PgnList => unreachable!(),
            };
            values.push((f, v));
        }
    }
    Message { pgn, def, values }
}

/// Builds a message field by field. Fields not set are "not available";
/// reserved bits are ones.
#[derive(Clone)]
pub struct Builder {
    def: &'static PgnDef,
    vals: Vec<Value>,
}

impl Builder {
    pub fn new(pgn: u32) -> Self {
        let def = def_of(pgn).unwrap_or_else(|| panic!("no definition for PGN {pgn}"));
        let vals = def
            .fields
            .iter()
            .map(|f| match f.kind {
                Kind::Num { .. } => Value::Num(None),
                Kind::Lookup(_) => Value::Lookup(None),
                Kind::Reserved => Value::Reserved,
                Kind::Str => Value::Str(String::new()),
                Kind::Bin => Value::Bin(None),
                Kind::PgnList => Value::List(Vec::new()),
            })
            .collect();
        Builder { def, vals }
    }

    pub fn pgn(&self) -> u32 {
        self.def.pgn
    }

    pub fn priority(&self) -> u8 {
        self.def.priority
    }

    fn slot(&mut self, id: &str) -> &mut Value {
        let i = self
            .def
            .fields
            .iter()
            .position(|f| f.id == id)
            .unwrap_or_else(|| panic!("PGN {} has no field {id}", self.def.pgn));
        &mut self.vals[i]
    }

    pub fn num(mut self, id: &str, v: f64) -> Self {
        *self.slot(id) = Value::Num(if v.is_finite() { Some(v) } else { None });
        self
    }
    pub fn opt(mut self, id: &str, v: Option<f64>) -> Self {
        *self.slot(id) = Value::Num(v.filter(|x| x.is_finite()));
        self
    }
    pub fn lookup(mut self, id: &str, v: u32) -> Self {
        *self.slot(id) = Value::Lookup(Some(v));
        self
    }
    pub fn bin(mut self, id: &str, v: u64) -> Self {
        *self.slot(id) = Value::Bin(Some(v));
        self
    }
    pub fn text(mut self, id: &str, s: &str) -> Self {
        *self.slot(id) = Value::Str(s.to_string());
        self
    }
    pub fn list(mut self, id: &str, l: &[u32]) -> Self {
        *self.slot(id) = Value::List(l.to_vec());
        self
    }

    /// Set every numeric field to "not available", keeping lookups and
    /// identifiers. Used for the "sends no data" fault.
    pub fn blank_numbers(mut self, keep: &[&str]) -> Self {
        for (f, v) in self.def.fields.iter().zip(self.vals.iter_mut()) {
            if matches!(f.kind, Kind::Num { .. }) && !keep.contains(&f.id) {
                *v = Value::Num(None);
            }
        }
        self
    }

    pub fn encode(&self) -> Vec<u8> {
        let fixed_bits: usize = self.def.fields.iter().map(|f| f.bits as usize).sum();
        let mut nbytes = fixed_bits.div_ceil(8);
        if let Some(Value::List(l)) = self.vals.iter().find(|v| matches!(v, Value::List(_))) {
            nbytes += 3 * l.len();
        }
        if !self.def.fast && self.def.pad8 {
            nbytes = nbytes.max(8);
        }
        let mut buf = vec![0xFFu8; nbytes];
        let mut pos = 0usize;
        for (f, v) in self.def.fields.iter().zip(self.vals.iter()) {
            let n = f.bits as usize;
            match (f.kind, v) {
                (Kind::Num { signed, res, .. }, Value::Num(x)) => {
                    set(&mut buf, pos, n, encode_num(*x, n, signed, res));
                }
                (Kind::Lookup(_), Value::Lookup(x)) => set(&mut buf, pos, n, x.map(|x| x as u64 & mask(n)).unwrap_or(mask(n))),
                (Kind::Bin, Value::Bin(x)) => set(&mut buf, pos, n, x.map(|x| x & mask(n)).unwrap_or(mask(n))),
                (Kind::Str, Value::Str(s)) => {
                    let start = pos / 8;
                    for (i, b) in s.bytes().take(n / 8).enumerate() {
                        buf[start + i] = b;
                    }
                }
                (Kind::PgnList, Value::List(l)) => {
                    let mut b = pos / 8;
                    for p in l {
                        buf[b] = *p as u8;
                        buf[b + 1] = (*p >> 8) as u8;
                        buf[b + 2] = (*p >> 16) as u8;
                        b += 3;
                    }
                }
                _ => {}
            }
            pos += n;
        }
        buf
    }
}

fn encode_num(v: Option<f64>, n: usize, signed: bool, res: f64) -> u64 {
    match v {
        None => {
            if signed {
                signed_na(n)
            } else {
                mask(n)
            }
        }
        Some(x) => {
            let r = (x / res).round();
            if signed {
                let half = 1i128 << (n - 1);
                let r = (r as i128).clamp(-half, half - 3);
                (r as i64 as u64) & mask(n)
            } else {
                let hi = if n >= 4 { mask(n) - 2 } else { mask(n) };
                (r.max(0.0) as u64).min(hi)
            }
        }
    }
}

/// One-line human reading of a value, with the units sailors use:
/// degrees for angles, knots for speeds, Celsius, hectopascals.
pub fn human(f: &Field, v: &Value) -> String {
    match (f.kind, v) {
        (_, Value::Num(None)) | (_, Value::Lookup(None)) | (_, Value::Bin(None)) => "n/a".into(),
        (Kind::Num { unit, .. }, Value::Num(Some(x))) => match unit {
            "rad" => format!("{:.1}°", x.to_degrees()),
            "rad/s" => format!("{:.2}°/s", x.to_degrees()),
            "m/s" => format!("{:.2} kn", x * 3600.0 / 1852.0),
            "K" => format!("{:.1} °C", x - 273.15),
            "Pa" => format!("{:.1} hPa", x / 100.0),
            "deg" => format!("{x:.6}°"),
            "d" => format!("{x:.0} d"),
            "s" if f.id == "time" || f.id == "etaTime" => {
                let s = *x as u64;
                format!("{:02}:{:02}:{:06.3}", s / 3600, (s / 60) % 60, x % 60.0)
            }
            "" => trim_float(*x),
            u => format!("{} {u}", trim_float(*x)),
        },
        (Kind::Lookup(t), Value::Lookup(Some(x))) => t
            .iter()
            .find(|(k, _)| k == x)
            .map(|(_, s)| s.to_string())
            .unwrap_or_else(|| x.to_string()),
        (_, Value::Bin(Some(x))) => x.to_string(),
        (_, Value::Str(s)) => format!("\"{s}\""),
        (_, Value::List(l)) => l.iter().map(|p| p.to_string()).collect::<Vec<_>>().join(", "),
        _ => String::new(),
    }
}

fn trim_float(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e15 {
        format!("{x:.0}")
    } else {
        let s = format!("{x:.4}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

/// `name=value` pairs for every non-reserved field, for logs.
pub fn summary(m: &Message) -> String {
    m.values
        .iter()
        .filter(|(f, _)| !matches!(f.kind, Kind::Reserved) && f.id != "sid" && f.id != "spare")
        .map(|(f, v)| format!("{}={}", f.id, human(f, v)))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_definition_is_whole_bytes() {
        for d in DEFS {
            let bits: usize = d.fields.iter().map(|f| f.bits as usize).sum();
            assert_eq!(bits % 8, 0, "PGN {} is {bits} bits", d.pgn);
            if !d.fast {
                assert!(bits <= 64, "PGN {} too long for one frame", d.pgn);
            }
        }
        let gnss: usize = def_of(GNSS_POSITION).unwrap().fields.iter().map(|f| f.bits as usize).sum();
        assert_eq!(gnss / 8, 43);
        let prod: usize = def_of(PRODUCT_INFO).unwrap().fields.iter().map(|f| f.bits as usize).sum();
        assert_eq!(prod / 8, 134);
    }

    #[test]
    fn heading_round_trip() {
        let b = Builder::new(HEADING).num("sid", 7.0).num("heading", 1.2345).num("variation", -0.02).lookup("reference", 1);
        let data = b.encode();
        assert_eq!(data.len(), 8);
        let m = decode(HEADING, &data);
        assert!((m.num("heading").unwrap() - 1.2345).abs() < 1e-4);
        assert!((m.num("variation").unwrap() + 0.02).abs() < 1e-4);
        assert_eq!(m.num("deviation"), None, "unset means not available");
        assert_eq!(m.lookup("reference"), Some(1));
        assert_eq!(data[7] & 0xFC, 0xFC, "reserved bits are ones");
    }

    #[test]
    fn signed_and_wide_fields() {
        let d = Builder::new(RATE_OF_TURN).num("rate", -0.05).encode();
        assert!((decode(RATE_OF_TURN, &d).num("rate").unwrap() + 0.05).abs() < 1e-6);
        let d = Builder::new(GNSS_POSITION).num("latitude", 50.123456789).num("longitude", -1.987654321).encode();
        assert_eq!(d.len(), 43);
        let m = decode(GNSS_POSITION, &d);
        assert!((m.num("latitude").unwrap() - 50.123456789).abs() < 1e-9);
        assert!((m.num("longitude").unwrap() + 1.987654321).abs() < 1e-9);
    }

    #[test]
    fn strings_and_pgn_lists() {
        let d = Builder::new(PRODUCT_INFO).text("modelId", "Sim GNSS").num("loadEquivalency", 2.0).encode();
        assert_eq!(d.len(), 134);
        let m = decode(PRODUCT_INFO, &d);
        assert_eq!(m.text("modelId"), Some("Sim GNSS"));
        let d = Builder::new(PGN_LIST).lookup("functionCode", 0).list("pgns", &[129025, 129026]).encode();
        assert_eq!(d.len(), 7);
        assert_eq!(decode(PGN_LIST, &d).list(), &[129025, 129026]);
    }

    #[test]
    fn added_pgns_round_trip() {
        let d = Builder::new(TEMPERATURE_EXT).num("temperature", 288.65).lookup("source", 1).encode();
        assert!((decode(TEMPERATURE_EXT, &d).num("temperature").unwrap() - 288.65).abs() < 1e-3);
        let d = Builder::new(ENV_PARAMETERS).lookup("temperatureSource", 1).num("humidity", 72.5).num("atmosphericPressure", 101_500.0).encode();
        let m = decode(ENV_PARAMETERS, &d);
        assert_eq!(m.lookup("temperatureSource"), Some(1));
        assert!((m.num("humidity").unwrap() - 72.5).abs() < 0.01);
        assert_eq!(m.num("atmosphericPressure"), Some(101_500.0));
        let d = Builder::new(DIRECTION_DATA).num("heading", 1.0).num("set", 0.5).num("drift", 0.4).encode();
        assert_eq!(d.len(), 14);
        assert!((decode(DIRECTION_DATA, &d).num("drift").unwrap() - 0.4).abs() < 1e-9);
        let d = Builder::new(MAGNETIC_VARIATION).num("variation", -0.0175).encode();
        assert!((decode(MAGNETIC_VARIATION, &d).num("variation").unwrap() + 0.0175).abs() < 1e-4);
    }

    #[test]
    fn iso_request_is_three_bytes() {
        let d = Builder::new(ISO_REQUEST).bin("pgn", 126996).encode();
        assert_eq!(d, vec![0x14, 0xF0, 0x01]);
    }

    #[test]
    fn clamps_instead_of_wrapping() {
        let d = Builder::new(SPEED).num("speedWaterReferenced", 1e6).num("speedGroundReferenced", -3.0).encode();
        let m = decode(SPEED, &d);
        assert!(m.num("speedWaterReferenced").unwrap() > 600.0);
        assert_eq!(m.num("speedGroundReferenced"), Some(0.0));
    }
}
