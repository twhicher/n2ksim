//! What an analyser plugged into the bus sees: it knows only the frames.
//! It reassembles fast packets, decodes, and keeps per-source and per-PGN
//! statistics. Everything the UI shows as "on the bus" comes from here.

use n2k::canid::NULL_ADDRESS;
use n2k::fast::{Assembler, Push};
use n2k::pgn::{self, Message};
use n2k::{CanId, Frame, Name};
use std::collections::{BTreeMap, VecDeque};

#[derive(Clone, Debug, Default)]
pub struct Product {
    pub model: String,
    pub software: String,
    pub model_version: String,
    pub serial: String,
    pub code: u32,
    pub len: Option<u32>,
}

#[derive(Clone, Debug, Default)]
pub struct SrcInfo {
    pub name: Option<u64>,
    pub product: Option<Product>,
    pub tx_list: Option<Vec<u32>>,
    pub rx_list: Option<Vec<u32>>,
    pub first_t: f64,
    pub last_t: f64,
    pub frames: u64,
    pub claims: u32,
    pub heartbeat_t: Option<f64>,
    /// Average time between frames from this source.
    pub interval: f64,
    /// Sends data on a schedule, so silence means something.
    pub periodic: bool,
}

#[derive(Clone, Debug)]
pub struct Stat {
    pub pgn: u32,
    pub src: u8,
    pub count: u64,
    pub first_t: f64,
    pub last_t: f64,
    times: VecDeque<f64>,
    pub msg: Message,
    pub summary: String,
    /// Messages whose key field was "not available".
    pub na_count: u64,
    pub key: Option<f64>,
    pub key_changed_t: f64,
    /// Average size of the change between successive readings of the key
    /// field (radians for angles): high means a noisy sensor.
    pub jump: f64,
    /// Model ID of the sender when it was last heard, so a stale entry
    /// still names the device that sent it after its address changes hands.
    pub model: Option<String>,
}

impl Stat {
    /// Messages per second over the last five seconds.
    pub fn rate(&self, now: f64) -> f64 {
        let n = self.times.iter().filter(|&&x| now - x <= 5.0).count();
        let span = (now - self.first_t).clamp(0.5, 5.0);
        n as f64 / span
    }
}

/// The field whose value says whether a source is alive and sane.
pub fn key_field(p: u32) -> Option<&'static str> {
    Some(match p {
        pgn::HEADING => "heading",
        pgn::WIND => "windAngle",
        pgn::SPEED => "speedWaterReferenced",
        pgn::DEPTH => "depth",
        pgn::RUDDER => "position",
        pgn::COG_SOG => "sog",
        pgn::POSITION_RAPID => "latitude",
        pgn::GNSS_POSITION => "latitude",
        pgn::RATE_OF_TURN => "rate",
        pgn::ATTITUDE => "roll",
        _ => return None,
    })
}

#[derive(Clone, Debug)]
pub struct MsgRec {
    pub seq: u64,
    pub t: f64,
    pub id: CanId,
    pub text: String,
}

#[derive(Default)]
pub struct Analyser {
    asm: Assembler,
    pub srcs: BTreeMap<u8, SrcInfo>,
    pub stats: BTreeMap<(u32, u8), Stat>,
    /// (time, address, NAME) of every claim, last 60 s.
    pub claims: VecDeque<(f64, u8, u64)>,
    pub msgs: VecDeque<MsgRec>,
    pub msg_seq: u64,
    pub fast_errors_log: VecDeque<(f64, u8, u32)>,
    /// (time it came back, address, seconds it was silent), last 120 s.
    pub gaps: VecDeque<(f64, u8, f64)>,
    /// (time, NAME, old address, new address): a device that moved.
    pub moves: VecDeque<(f64, u64, u8, u8)>,
    /// (time, NAME, address): a claim nobody asked for from a NAME already
    /// known at that address, which is what a device does when it restarts.
    pub restarts: VecDeque<(f64, u64, u8)>,
    addr_of: std::collections::HashMap<u64, u8>,
    /// Model ID by NAME, so a device keeps its name after it moves.
    pub models: std::collections::HashMap<u64, String>,
    claim_request_t: f64,
}

const MSG_RING: usize = 20_000;

impl Analyser {
    pub fn fast_errors(&self) -> u64 {
        self.asm.errors
    }

    pub fn ingest(&mut self, t: f64, f: &Frame) -> Option<(CanId, Message)> {
        let id = f.can_id();
        let src = self.srcs.entry(id.source).or_insert_with(|| SrcInfo {
            first_t: t,
            ..Default::default()
        });
        let gap = t - src.last_t;
        if src.frames == 1 {
            src.interval = gap;
        } else if src.periodic && src.frames > 20 && gap > (5.0 * src.interval).max(2.0) {
            self.gaps.push_back((t, id.source, gap));
        } else if src.frames > 1 {
            src.interval += 0.05 * (gap - src.interval);
        }
        while self.gaps.front().is_some_and(|g| t - g.0 > 120.0) {
            self.gaps.pop_front();
        }
        src.frames += 1;
        src.last_t = t;
        if pgn::def_of(id.pgn).is_some_and(|d| d.interval_ms > 0 && d.interval_ms <= 5000) {
            src.periodic = true;
        }
        let payload = if pgn::is_fast(id.pgn) {
            match self.asm.push(id.source, id.pgn, f.bytes()) {
                Push::Done(b) => b,
                Push::More => return None,
                Push::Error(_) => {
                    self.fast_errors_log.push_back((t, id.source, id.pgn));
                    while self.fast_errors_log.front().is_some_and(|e| t - e.0 > 30.0) {
                        self.fast_errors_log.pop_front();
                    }
                    return None;
                }
            }
        } else {
            f.bytes().to_vec()
        };
        let m = n2k::decode(id.pgn, &payload);
        self.learn(t, &id, &m, &payload);
        let text = if m.def.is_some() { pgn::summary(&m) } else { payload.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ") };
        let key = key_field(id.pgn).and_then(|k| m.num(k));
        let stat = self.stats.entry((id.pgn, id.source)).or_insert_with(|| Stat {
            pgn: id.pgn,
            src: id.source,
            count: 0,
            first_t: t,
            last_t: t,
            times: VecDeque::new(),
            msg: m.clone(),
            summary: String::new(),
            na_count: 0,
            key,
            key_changed_t: t,
            jump: 0.0,
            model: None,
        });
        if let Some(s) = self.srcs.get(&id.source) {
            if let Some(m) = s.product.as_ref().map(|p| p.model.clone()).or_else(|| s.name.and_then(|n| self.models.get(&n).cloned())) {
                stat.model = Some(m);
            }
        }
        stat.count += 1;
        stat.last_t = t;
        stat.times.push_back(t);
        while stat.times.front().is_some_and(|&x| t - x > 5.0) {
            stat.times.pop_front();
        }
        if key_field(id.pgn).is_some() && key.is_none() {
            stat.na_count += 1;
        }
        if key != stat.key {
            stat.key_changed_t = t;
        }
        if let (Some(a), Some(b)) = (key, stat.key) {
            if matches!(id.pgn, pgn::HEADING | pgn::WIND | pgn::RUDDER) {
                let d = crate::world::wrap_pi(a - b).abs();
                stat.jump += 0.05 * (d - stat.jump);
            }
        }
        stat.key = key;
        stat.msg = m.clone();
        stat.summary = text.clone();
        self.msg_seq += 1;
        self.msgs.push_back(MsgRec { seq: self.msg_seq, t, id, text });
        if self.msgs.len() > MSG_RING {
            self.msgs.pop_front();
        }
        Some((id, m))
    }

    fn learn(&mut self, t: f64, id: &CanId, m: &Message, payload: &[u8]) {
        let s = self.srcs.get_mut(&id.source).expect("inserted above");
        match id.pgn {
            pgn::ADDRESS_CLAIM => {
                if let Some(n) = Name::from_bytes(payload) {
                    let v = n.to_u64();
                    if s.name.is_some_and(|old| old != v) {
                        // A different device at this address now.
                        s.product = None;
                        s.tx_list = None;
                        s.rx_list = None;
                    }
                    s.name = Some(v);
                    s.claims += 1;
                    self.claims.push_back((t, id.source, v));
                    while self.claims.front().is_some_and(|c| t - c.0 > 60.0) {
                        self.claims.pop_front();
                    }
                    // The same NAME at a new address: it moved, and the old
                    // entry is stale.
                    let prompted = t - self.claim_request_t < 1.5;
                    match self.addr_of.insert(v, id.source) {
                        Some(old) if old != id.source && id.source != NULL_ADDRESS => {
                            self.moves.push_back((t, v, old, id.source));
                            if let Some(o) = self.srcs.get_mut(&old) {
                                if o.name == Some(v) {
                                    o.name = None;
                                    o.product = None;
                                }
                            }
                        }
                        Some(_) if !prompted => self.restarts.push_back((t, v, id.source)),
                        _ => {}
                    }
                    while self.restarts.front().is_some_and(|m| t - m.0 > 120.0) {
                        self.restarts.pop_front();
                    }
                    while self.moves.front().is_some_and(|m| t - m.0 > 300.0) {
                        self.moves.pop_front();
                    }
                }
            }
            pgn::PRODUCT_INFO => {
                if let (Some(n), Some(m)) = (s.name, m.text("modelId")) {
                    self.models.insert(n, m.to_string());
                }
                s.product = Some(Product {
                    model: m.text("modelId").unwrap_or("").to_string(),
                    software: m.text("softwareVersionCode").unwrap_or("").to_string(),
                    model_version: m.text("modelVersion").unwrap_or("").to_string(),
                    serial: m.text("modelSerialCode").unwrap_or("").to_string(),
                    code: m.num("productCode").unwrap_or(0.0) as u32,
                    len: m.num("loadEquivalency").map(|x| x as u32),
                })
            }
            pgn::PGN_LIST => match m.lookup("functionCode") {
                Some(0) => s.tx_list = Some(m.list().to_vec()),
                Some(1) => s.rx_list = Some(m.list().to_vec()),
                _ => {}
            },
            pgn::HEARTBEAT => s.heartbeat_t = Some(t),
            pgn::ISO_REQUEST if m.num("pgn") == Some(pgn::ADDRESS_CLAIM as f64) => self.claim_request_t = t,
            _ => {}
        }
    }

    /// Addresses claimed by more than one NAME in the last 60 s, and
    /// devices that gave up (claimed from 254).
    pub fn conflicts(&self, now: f64) -> (Vec<(u8, Vec<u64>)>, Vec<u64>) {
        let mut by_addr: BTreeMap<u8, Vec<u64>> = BTreeMap::new();
        let mut null = Vec::new();
        for &(t, a, n) in &self.claims {
            if now - t > 60.0 {
                continue;
            }
            if a == NULL_ADDRESS {
                if !null.contains(&n) {
                    null.push(n);
                }
                continue;
            }
            let v = by_addr.entry(a).or_default();
            if !v.contains(&n) {
                v.push(n);
            }
        }
        (by_addr.into_iter().filter(|(_, v)| v.len() > 1).collect(), null)
    }

    pub fn reset(&mut self) {
        *self = Analyser::default();
    }
}
