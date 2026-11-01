//! The instrument library: real products described from their makers'
//! documentation, in `library/instruments.json`.
//!
//! Every figure in an entry comes with a source, and anything not found is
//! left out (null) and listed in `gaps`. Where an entry gives no transmit
//! interval, the device uses canboat's usual interval for that PGN and the
//! UI says so. Manufacturer codes come from canboat's MANUFACTURER_CODE
//! lookup (v9.1.0).
//!
//! What a library device sends is the real unit's PGNs that the simulator
//! can generate; the rest are listed as not simulated.

use crate::device::{default_variant, MANAGEMENT, SIMULATED};
use crate::network::{template, DeviceCfg, DeviceKind, PgnCfg};
use n2k::pgn;
use serde::{Deserialize, Serialize};
use std::sync::OnceLock;

const LIBRARY: &str = include_str!("../library/instruments.json");

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct LibPgn {
    pub pgn: u32,
    pub interval_ms: Option<u32>,
    pub variant: Option<String>,
    pub note: Option<String>,
    /// Not in the maker's documentation: inferred, and the note says why.
    pub inferred: bool,
    /// Off when the device is added (for PGNs a unit may or may not send).
    pub off: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct Setting {
    pub name: String,
    pub default: Option<String>,
    pub effect: Option<String>,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
#[serde(default)]
pub struct Source {
    pub what: String,
    pub url: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct Entry {
    pub key: String,
    pub make: String,
    pub model: String,
    pub category: String,
    pub kind: DeviceKind,
    pub summary: String,
    pub native_n2k: Option<bool>,
    pub via: Option<String>,
    pub manufacturer_code: Option<u16>,
    pub device_class: Option<u8>,
    pub device_function: Option<u8>,
    pub len: Option<u8>,
    pub model_id: Option<String>,
    pub tx: Vec<LibPgn>,
    pub rx: Vec<u32>,
    pub settings: Vec<Setting>,
    pub notes: String,
    pub gaps: Vec<String>,
    pub sources: Vec<Source>,
    /// Physical size and look for the schematic, with its own sources.
    pub appearance: Option<serde_json::Value>,
}

impl Default for Entry {
    fn default() -> Self {
        Entry {
            key: String::new(),
            make: String::new(),
            model: String::new(),
            category: String::new(),
            kind: DeviceKind::Display,
            summary: String::new(),
            native_n2k: None,
            via: None,
            manufacturer_code: None,
            device_class: None,
            device_function: None,
            len: None,
            model_id: None,
            tx: Vec::new(),
            rx: Vec::new(),
            settings: Vec::new(),
            notes: String::new(),
            gaps: Vec::new(),
            sources: Vec::new(),
            appearance: None,
        }
    }
}

impl Entry {
    /// The real unit's data PGNs the simulator can generate, and those it
    /// cannot. Network management PGNs are in neither.
    pub fn split_tx(&self) -> (Vec<&LibPgn>, Vec<&LibPgn>) {
        self.tx.iter().filter(|p| !MANAGEMENT.contains(&p.pgn)).partition(|p| SIMULATED.contains(&p.pgn))
    }

    /// A device configured as this product.
    pub fn to_device(&self, id: &str, unique: u32) -> DeviceCfg {
        let mut d = template(self.kind, id, &format!("{} {}", self.make, self.model), unique);
        d.library = self.key.clone();
        if let Some(c) = self.manufacturer_code {
            d.name.manufacturer_code = c;
        }
        let (class, function) = self.kind.class_function();
        d.name.device_class = self.device_class.unwrap_or(class);
        d.name.device_function = self.device_function.unwrap_or(function);
        if let Some(l) = self.len {
            d.len = l;
        }
        let model_id = self.model_id.clone().unwrap_or_else(|| self.model.clone());
        d.product.model_id = model_id.chars().take(32).collect();
        d.product.software_version = "simulated".into();
        let (sim, _) = self.split_tx();
        d.pgns = sim
            .iter()
            .map(|p| PgnCfg {
                pgn: p.pgn,
                enabled: !p.off,
                interval_ms: p.interval_ms.unwrap_or_else(|| pgn::def_of(p.pgn).map(|d| d.interval_ms).filter(|&i| i > 0).unwrap_or(1000)),
                variant: p.variant.clone().unwrap_or_else(|| default_variant(self.kind, p.pgn).to_string()),
            })
            .collect();
        d.rx_pgns = self.rx.clone();
        d
    }
}

pub fn entries() -> &'static [Entry] {
    static E: OnceLock<Vec<Entry>> = OnceLock::new();
    E.get_or_init(|| serde_json::from_str(LIBRARY).expect("library/instruments.json is valid"))
}

pub fn find(key: &str) -> Option<&'static Entry> {
    entries().iter().find(|e| e.key == key)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::network::default_network;
    use crate::sim::Sim;
    use crate::world::{EnvCfg, HelmCfg};

    #[test]
    fn entries_are_sourced_and_consistent() {
        let mut keys = std::collections::HashSet::new();
        for e in entries() {
            assert!(keys.insert(e.key.clone()), "duplicate key {}", e.key);
            assert!(!e.sources.is_empty(), "{} has no sources", e.key);
            for s in &e.sources {
                assert!(s.url.starts_with("https://") || s.url.starts_with("http://"), "{}: bad url {}", e.key, s.url);
            }
            if let Some(a) = &e.appearance {
                let srcs = a.get("sources").and_then(|v| v.as_array()).map(|v| v.len()).unwrap_or(0);
                assert!(srcs > 0, "{}: appearance has no sources", e.key);
            }
            for p in &e.tx {
                if let Some(v) = &p.variant {
                    let ok = crate::device::VARIANTS.iter().any(|(q, vs)| *q == p.pgn && vs.contains(&v.as_str()));
                    assert!(ok, "{}: unknown variant {v} for {}", e.key, p.pgn);
                }
            }
        }
    }

    #[test]
    fn every_entry_runs_on_the_bus() {
        for (i, e) in entries().iter().enumerate() {
            let mut net = default_network();
            let mut d = e.to_device("lib", 300_000 + i as u32);
            d.preferred_address = 20;
            net.devices.push(d);
            let mut s = Sim::new(net, EnvCfg::default(), HelmCfg::default());
            s.run_for(5.0);
            let d = s.devices.iter().find(|d| d.cfg.id == "lib").unwrap();
            assert!(d.state.on_bus(), "{} is {:?}", e.key, d.state);
            for p in d.cfg.pgns.iter().filter(|p| p.enabled) {
                let seen = s.analyser.stats.contains_key(&(p.pgn, d.address));
                assert!(seen, "{} never sent {} (bus load {:.0}%, {} dropped)", e.key, p.pgn, s.bus_load() * 100.0, d.dropped);
            }
        }
    }
}
