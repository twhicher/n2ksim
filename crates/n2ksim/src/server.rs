//! A small HTTP server (standard library only): the page, a JSON API, and a
//! server-sent event stream with state four times a second plus every frame
//! and decoded message.
//!
//! Locally it listens on 127.0.0.1, runs one simulation and refuses changes
//! whose Origin is not its own page. Hosted (`--hosted`), it gives each
//! visitor their own simulation (by cookie), checks changes come from the
//! same site, can require a password, and limits uploads, connections and
//! the number of simulations.

use crate::device::{DevState, Origin};
use crate::network::{template, DeviceKind, Network};
use crate::scenarios;
use crate::sim::Sim;
use crate::world::{EnvCfg, HelmCfg};
use n2k::format::{candump_line, parse_any, ydraw_line};
use n2k::pgn::{self, human, Kind, Value};
use n2k::{frames_for, CanId, Frame, Name};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value as J};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const PAGE: &str = include_str!("../static/index.html");
const MAX_BODY: usize = 32 * 1024 * 1024;
const MAX_BODY_HOSTED: usize = 4 * 1024 * 1024;
const MAX_CONNECTIONS: usize = 200;
const MAX_STREAMS_PER_VISITOR: usize = 4;
const COOKIE: &str = "n2ksim";
/// The web UI's own interface claims no address; it uses this one.
pub const CONSOLE_ADDRESS: u8 = 252;

/// Settings for running on a public server.
pub struct Hosted {
    /// Required with HTTP Basic auth if set (any user name).
    pub password: Option<String>,
    /// Every new visitor starts from this.
    pub template: Saved,
    pub max_visitors: usize,
    pub idle: Duration,
}

pub struct Visitor {
    pub sim: Arc<Mutex<Sim>>,
    pub last_seen: Instant,
    pub streams: usize,
}

pub struct Ctx {
    /// The one simulation when running locally.
    pub local: Option<Arc<Mutex<Sim>>>,
    /// One simulation per visitor when hosted.
    pub visitors: Mutex<HashMap<String, Visitor>>,
    pub hosted: Option<Hosted>,
    pub port: u16,
    pub gateway_addr: String,
    pub config_path: PathBuf,
    pub connections: AtomicUsize,
}

impl Ctx {
    pub fn local(sim: Arc<Mutex<Sim>>, port: u16, gateway_addr: String, config_path: PathBuf) -> Self {
        Ctx { local: Some(sim), visitors: Mutex::new(HashMap::new()), hosted: None, port, gateway_addr, config_path, connections: AtomicUsize::new(0) }
    }

    pub fn hosted(h: Hosted, port: u16) -> Self {
        Ctx { local: None, visitors: Mutex::new(HashMap::new()), hosted: Some(h), port, gateway_addr: "off".into(), config_path: PathBuf::new(), connections: AtomicUsize::new(0) }
    }

    /// Every running simulation, for the clock thread.
    pub fn all_sims(&self) -> Vec<Arc<Mutex<Sim>>> {
        match &self.local {
            Some(s) => vec![s.clone()],
            None => self.visitors.lock().unwrap().values().map(|v| v.sim.clone()).collect(),
        }
    }

    /// Drop visitors nobody has looked at for a while.
    pub fn evict_idle(&self) {
        if let Some(h) = &self.hosted {
            let mut v = self.visitors.lock().unwrap();
            v.retain(|_, x| x.streams > 0 || x.last_seen.elapsed() < h.idle);
        }
    }

    fn touch(&self, id: &str) {
        if let Some(v) = self.visitors.lock().unwrap().get_mut(id) {
            v.last_seen = Instant::now();
        }
    }

    /// The simulation for this request, and a new cookie id if one was made.
    fn sim_for(&self, cookie: Option<&str>) -> Result<(Arc<Mutex<Sim>>, String, bool), &'static str> {
        if let Some(s) = &self.local {
            return Ok((s.clone(), String::new(), false));
        }
        let h = self.hosted.as_ref().expect("hosted when not local");
        let mut map = self.visitors.lock().unwrap();
        if let Some(id) = cookie {
            if let Some(v) = map.get_mut(id) {
                v.last_seen = Instant::now();
                return Ok((v.sim.clone(), id.to_string(), false));
            }
        }
        if map.len() >= h.max_visitors {
            // Make room by dropping the longest-idle visitor without a page open.
            let oldest = map.iter().filter(|(_, v)| v.streams == 0 && v.last_seen.elapsed() > Duration::from_secs(60)).min_by_key(|(_, v)| v.last_seen).map(|(k, _)| k.clone());
            match oldest {
                Some(k) => {
                    map.remove(&k);
                }
                None => return Err("The simulator is full at the moment. Try again in a few minutes."),
            }
        }
        let t = &h.template;
        let sim = Arc::new(Mutex::new(Sim::new(t.network.clone(), t.env.clone(), t.helm.clone())));
        let id = random_id();
        map.insert(id.clone(), Visitor { sim: sim.clone(), last_seen: Instant::now(), streams: 0 });
        Ok((sim, id, true))
    }
}

/// 128 random bits as hex, from the operating system.
fn random_id() -> String {
    let mut b = [0u8; 16];
    let ok = std::fs::File::open("/dev/urandom").and_then(|mut f| f.read_exact(&mut b)).is_ok();
    if !ok {
        // No /dev/urandom: fall back to the clock, which is guessable, so
        // only reached on unusual systems.
        let n = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos()).unwrap_or(0);
        b = n.to_le_bytes();
    }
    b.iter().map(|x| format!("{x:02x}")).collect()
}

fn base64_decode(s: &str) -> Option<Vec<u8>> {
    let val = |c: u8| -> Option<u32> {
        Some(match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return None,
        } as u32)
    };
    let s = s.trim().trim_end_matches('=').as_bytes();
    let mut out = Vec::with_capacity(s.len() * 3 / 4);
    for chunk in s.chunks(4) {
        let mut n = 0u32;
        for (i, &c) in chunk.iter().enumerate() {
            n |= val(c)? << (18 - 6 * i);
        }
        let bytes = n.to_be_bytes();
        out.extend_from_slice(&bytes[1..chunk.len()]);
    }
    Some(out)
}

/// Compare without stopping at the first difference.
fn same(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

struct ConnGuard<'a>(&'a AtomicUsize);
impl Drop for ConnGuard<'_> {
    fn drop(&mut self) {
        self.0.fetch_sub(1, Ordering::Relaxed);
    }
}

#[derive(Serialize, Deserialize, Default)]
#[serde(default)]
pub struct Saved {
    pub network: Network,
    pub env: EnvCfg,
    pub helm: HelmCfg,
}

pub fn serve(bind: &str, ctx: Arc<Ctx>) -> std::io::Result<()> {
    let listener = TcpListener::bind(bind)?;
    for stream in listener.incoming().flatten() {
        let ctx = ctx.clone();
        std::thread::spawn(move || handle(stream, &ctx));
    }
    Ok(())
}

fn respond(s: &mut TcpStream, status: &str, ctype: &str, extra: &str, body: &[u8]) {
    let csp = if ctype.starts_with("text/html") {
        "Content-Security-Policy: default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'\r\n"
    } else {
        ""
    };
    let head = format!(
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {}\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nX-Frame-Options: DENY\r\nReferrer-Policy: no-referrer\r\n{csp}{extra}Connection: close\r\n\r\n",
        body.len()
    );
    let _ = s.write_all(head.as_bytes());
    let _ = s.write_all(body);
}

fn err(s: &mut TcpStream, msg: &str) {
    respond(s, "400 Bad Request", "application/json", "", json!({"ok": false, "error": msg}).to_string().as_bytes());
}

fn handle(mut stream: TcpStream, ctx: &Ctx) {
    let open = ctx.connections.fetch_add(1, Ordering::Relaxed) + 1;
    let _guard = ConnGuard(&ctx.connections);
    if open > MAX_CONNECTIONS {
        respond(&mut stream, "503 Service Unavailable", "text/plain", "", b"too many connections");
        return;
    }
    let _ = stream.set_read_timeout(Some(Duration::from_secs(10)));
    let Ok(clone) = stream.try_clone() else { return };
    let mut reader = BufReader::new(clone);
    let mut line = String::new();
    if reader.read_line(&mut line).is_err() {
        return;
    }
    let mut parts = line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let target = parts.next().unwrap_or("/").to_string();
    let mut len = 0usize;
    let (mut origin, mut host, mut cookie, mut auth) = (None, None, None, None);
    loop {
        let mut h = String::new();
        if reader.read_line(&mut h).is_err() || h == "\r\n" || h.is_empty() {
            break;
        }
        let Some((name, value)) = h.split_once(':') else { continue };
        let value = value.trim().to_string();
        match name.trim().to_ascii_lowercase().as_str() {
            "content-length" => len = value.parse().unwrap_or(0),
            "origin" => origin = Some(value.to_ascii_lowercase()),
            "host" => host = Some(value.to_ascii_lowercase()),
            "authorization" => auth = Some(value),
            "cookie" => {
                cookie = value.split(';').filter_map(|c| c.trim().split_once('=')).find(|(k, _)| *k == COOKIE).map(|(_, v)| v.to_string()).filter(|v| v.len() == 32 && v.bytes().all(|b| b.is_ascii_hexdigit()));
            }
            _ => {}
        }
    }

    if let Some(pw) = ctx.hosted.as_ref().and_then(|h| h.password.as_ref()) {
        let given = auth.as_deref().and_then(|a| a.strip_prefix("Basic ")).and_then(base64_decode).and_then(|b| String::from_utf8(b).ok());
        let pass_ok = given.as_deref().and_then(|g| g.split_once(':')).is_some_and(|(_, p)| same(p.as_bytes(), pw.as_bytes()));
        if !pass_ok {
            respond(&mut stream, "401 Unauthorized", "text/plain", "WWW-Authenticate: Basic realm=\"n2ksim\", charset=\"UTF-8\"\r\n", b"password needed");
            return;
        }
    }

    let (path, query) = target.split_once('?').unwrap_or((&target, ""));
    if method == "GET" && (path == "/" || path == "/index.html") && ctx.hosted.is_none() {
        respond(&mut stream, "200 OK", "text/html; charset=utf-8", "", PAGE.as_bytes());
        return;
    }
    let (sim, id, fresh) = match ctx.sim_for(cookie.as_deref()) {
        Ok(x) => x,
        Err(e) => {
            respond(&mut stream, "503 Service Unavailable", "text/plain; charset=utf-8", "", e.as_bytes());
            return;
        }
    };
    let set_cookie = if fresh { format!("Set-Cookie: {COOKIE}={id}; Path=/; HttpOnly; SameSite=Lax; Secure; Max-Age=86400\r\n") } else { String::new() };

    if method == "GET" {
        match path {
            "/" | "/index.html" => respond(&mut stream, "200 OK", "text/html; charset=utf-8", &set_cookie, PAGE.as_bytes()),
            "/api/state" => {
                let v = snapshot(&sim.lock().unwrap(), ctx, true);
                respond(&mut stream, "200 OK", "application/json", &set_cookie, v.to_string().as_bytes());
            }
            "/api/events" => events(stream, ctx, &sim, &id, &set_cookie),
            "/api/capture" => capture(&mut stream, &sim, query),
            "/api/export" => {
                let body = {
                    let s = sim.lock().unwrap();
                    serde_json::to_string_pretty(&Saved { network: s.net.clone(), env: s.world.env.clone(), helm: s.world.helm.clone() }).unwrap_or_default()
                };
                respond(&mut stream, "200 OK", "application/json", "Content-Disposition: attachment; filename=\"n2ksim-network.json\"\r\n", body.as_bytes());
            }
            _ => respond(&mut stream, "404 Not Found", "text/plain", "", b"not found"),
        }
        return;
    }
    if method != "POST" {
        respond(&mut stream, "405 Method Not Allowed", "text/plain", "", b"");
        return;
    }
    // Changes must come from this site's own page. Locally the allowed
    // origins are fixed, which also defeats DNS rebinding; hosted, the page's
    // origin must match the host it was served from.
    let same_site = match &ctx.hosted {
        None => {
            let allowed = [format!("http://127.0.0.1:{}", ctx.port), format!("http://localhost:{}", ctx.port)];
            origin.as_ref().is_none_or(|o| allowed.contains(o))
        }
        Some(_) => match (&origin, &host) {
            (Some(o), Some(h)) => *o == format!("https://{h}") || *o == format!("http://{h}"),
            (None, _) => true,
            _ => false,
        },
    };
    if !same_site {
        respond(&mut stream, "403 Forbidden", "text/plain", "", b"cross-origin request refused");
        return;
    }
    if len > if ctx.hosted.is_some() { MAX_BODY_HOSTED } else { MAX_BODY } {
        return err(&mut stream, "That upload is too large.");
    }
    let mut body = vec![0u8; len];
    if reader.read_exact(&mut body).is_err() {
        return err(&mut stream, "short body");
    }
    let body: J = match serde_json::from_slice(&body) {
        Ok(v) => v,
        Err(_) if len == 0 => J::Null,
        Err(_) => return err(&mut stream, "body is not JSON"),
    };
    let result = {
        let mut s = sim.lock().unwrap();
        post(&mut s, ctx, path, &body)
    };
    match result {
        Ok(v) => respond(&mut stream, "200 OK", "application/json", &set_cookie, v.to_string().as_bytes()),
        Err(e) => err(&mut stream, &e),
    }
}

fn merge(a: &mut J, b: &J) {
    match (a, b) {
        (J::Object(a), J::Object(b)) => {
            for (k, v) in b {
                merge(a.entry(k.clone()).or_insert(J::Null), v);
            }
        }
        (a, b) => *a = b.clone(),
    }
}

fn patched<T: Serialize + for<'de> Deserialize<'de>>(cur: &T, patch: &J) -> Result<T, String> {
    let mut v = serde_json::to_value(cur).map_err(|e| e.to_string())?;
    merge(&mut v, patch);
    serde_json::from_value(v).map_err(|e| format!("invalid value: {e}"))
}

fn hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let s: String = s.chars().filter(|c| !c.is_whitespace() && *c != ',' && *c != ':').collect();
    if !s.len().is_multiple_of(2) {
        return Err("data must be whole bytes in hex".into());
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|_| "data is not hex".to_string())).collect()
}

fn post(sim: &mut Sim, ctx: &Ctx, path: &str, b: &J) -> Result<J, String> {
    let s = |k: &str| b.get(k).and_then(|v| v.as_str()).unwrap_or("").to_string();
    match path {
        "/api/control" => match s("action").as_str() {
            "pause" => sim.paused = true,
            "resume" => sim.paused = false,
            "speed" => sim.speed = b.get("speed").and_then(|v| v.as_f64()).unwrap_or(1.0).clamp(0.1, 50.0),
            "reset_world" => {
                sim.world.reset();
            }
            "power_cycle" => sim.power_cycle(),
            "clear_analyser" => sim.analyser.reset(),
            "clear_capture" => sim.ring.clear(),
            "stop_replay" => sim.replay = None,
            "save" if ctx.hosted.is_some() => return Err("Saving to the server is off in the hosted version; use Download instead.".into()),
            "save" => {
                let saved = Saved { network: sim.net.clone(), env: sim.world.env.clone(), helm: sim.world.helm.clone() };
                let text = serde_json::to_string_pretty(&saved).map_err(|e| e.to_string())?;
                if let Some(dir) = ctx.config_path.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                std::fs::write(&ctx.config_path, text).map_err(|e| {
                    eprintln!("save failed: {e}");
                    "could not save the configuration file".to_string()
                })?;
                return Ok(json!({"ok": true, "saved": ctx.config_path.display().to_string()}));
            }
            "request" => {
                let p = b.get("pgn").and_then(|v| v.as_u64()).ok_or("pgn missing")? as u32;
                let dst = b.get("dst").and_then(|v| v.as_u64()).unwrap_or(255) as u8;
                let data = n2k::Builder::new(pgn::ISO_REQUEST).bin("pgn", p as u64).encode();
                sim.inject(Frame::new(CanId::new(6, pgn::ISO_REQUEST, CONSOLE_ADDRESS, dst).encode(), &data));
            }
            _ => return Err("unknown action".into()),
        },
        "/api/backbone" => {
            let mut net = sim.net.clone();
            net.backbone = patched(&net.backbone, b)?;
            sim.set_network(net);
        }
        "/api/device" => {
            let id = s("id");
            let mut net = sim.net.clone();
            let i = net.devices.iter().position(|d| d.id == id).ok_or("no such device")?;
            let mut cfg = patched(&net.devices[i], b.get("patch").unwrap_or(&J::Null))?;
            cfg.id = id;
            net.devices[i] = cfg;
            sim.set_network(net);
        }
        "/api/device/add" => {
            let mut net = sim.net.clone();
            let lib = b.get("library").and_then(|v| v.as_str()).filter(|k| !k.is_empty());
            let entry = match lib {
                Some(k) => Some(crate::library::find(k).ok_or("no such library instrument")?),
                None => None,
            };
            let kind: DeviceKind = match entry {
                Some(e) => e.kind,
                None => serde_json::from_value(b.get("kind").cloned().unwrap_or(J::Null)).map_err(|_| "unknown device kind")?,
            };
            let base = entry.map(|e| e.key.clone()).unwrap_or_else(|| kind.key().to_string());
            let mut n = 2;
            let mut id = base.clone();
            while net.devices.iter().any(|d| d.id == id) {
                id = format!("{base}{n}");
                n += 1;
            }
            let used: Vec<u8> = sim.devices.iter().map(|d| d.address).chain(net.devices.iter().map(|d| d.preferred_address)).collect();
            let unique = 200_000 + (sim.frame_seq % 100_000) as u32 + n;
            let mut d = match entry {
                Some(e) => e.to_device(&id, unique),
                None => template(kind, &id, kind.title(), unique),
            };
            d.preferred_address = (10..250).find(|a| !used.contains(a)).unwrap_or(250);
            d.tee_m = b.get("tee_m").and_then(|v| v.as_f64()).unwrap_or(net.backbone.length_m / 2.0);
            net.devices.push(d);
            sim.set_network(net);
            return Ok(json!({"ok": true, "id": id}));
        }
        "/api/device/remove" => {
            let id = s("id");
            let mut net = sim.net.clone();
            net.devices.retain(|d| d.id != id);
            sim.set_network(net);
        }
        "/api/device/action" => {
            let id = s("id");
            let t = sim.t;
            let d = sim.devices.iter_mut().find(|d| d.cfg.id == id).ok_or("no such device")?;
            match s("action").as_str() {
                "reboot" if d.state.powered() => d.boot(t),
                "reboot" => {}
                "unplug" => {
                    d.state = DevState::Unplugged { until: t + b.get("secs").and_then(|v| v.as_f64()).unwrap_or(5.0) };
                    d.tx.clear();
                }
                _ => return Err("unknown action".into()),
            }
        }
        "/api/world" => {
            if let Some(e) = b.get("env") {
                sim.world.env = patched(&sim.world.env, e)?;
            }
            if let Some(h) = b.get("helm") {
                sim.world.helm = patched(&sim.world.helm, h)?;
            }
        }
        "/api/drill" => {
            let hit = scenarios::drill(sim, &s("key"))?;
            return Ok(json!({"ok": true, "device": hit}));
        }
        "/api/preset" => scenarios::preset(sim, &s("key"))?,
        "/api/network" => {
            let net = scenarios::network_preset(&s("key"))?;
            sim.pre_drill_backbone = None;
            sim.set_network(net);
            sim.analyser.reset();
        }
        "/api/inject" => {
            let data = hex_bytes(&s("data"))?;
            if let Some(id) = b.get("id").and_then(|v| v.as_str()).filter(|x| !x.is_empty()) {
                let id = u32::from_str_radix(id.trim_start_matches("0x"), 16).map_err(|_| "id is not hex")?;
                if data.len() > 8 || id > 0x1FFF_FFFF {
                    return Err("a raw frame has a 29-bit id and at most 8 bytes".into());
                }
                sim.inject(Frame::new(id, &data));
            } else {
                let g = |k: &str, d: u64| b.get(k).and_then(|v| v.as_u64()).unwrap_or(d);
                let p = g("pgn", 0) as u32;
                if p == 0 || p > 0x3FFFF {
                    return Err("pgn missing or too large".into());
                }
                if data.len() > 8 && !pgn::is_fast(p) {
                    return Err("more than 8 bytes needs a fast-packet PGN".into());
                }
                if data.len() > n2k::fast::MAX_LEN {
                    return Err("at most 223 bytes".into());
                }
                let id = CanId::new(g("priority", 6) as u8, p, g("src", CONSOLE_ADDRESS as u64) as u8, g("dst", 255) as u8);
                for f in frames_for(id, &data, (sim.frame_seq % 8) as u8) {
                    sim.inject(f);
                }
            }
        }
        "/api/replay" => {
            let text = s("text");
            let mut frames: Vec<(f64, Frame)> = Vec::new();
            let mut t0: Option<f64> = None;
            let mut last = 0.0;
            let mut bad = 0usize;
            for line in text.lines() {
                let Some(lf) = parse_any(line) else {
                    if !line.trim().is_empty() {
                        bad += 1;
                    }
                    continue;
                };
                let t = match lf.t {
                    Some(t) => {
                        let base = *t0.get_or_insert(t);
                        let mut r = t - base;
                        if r < last - 43_200.0 {
                            r += 86_400.0; // past midnight in a RAW log
                        }
                        r.max(last)
                    }
                    None => last + 0.001,
                };
                last = t;
                frames.push((t, lf.frame));
                if frames.len() > 2_000_000 {
                    return Err("log too long (over two million frames)".into());
                }
            }
            if frames.is_empty() {
                return Err("no frames found: expected candump or Yacht Devices RAW lines".into());
            }
            let n = frames.len();
            let dur = last;
            sim.start_replay(s("name"), frames, b.get("mute").and_then(|v| v.as_bool()).unwrap_or(true));
            return Ok(json!({"ok": true, "frames": n, "seconds": dur, "skipped_lines": bad}));
        }
        "/api/load" => {
            let saved: Saved = serde_json::from_value(b.clone()).map_err(|e| format!("not a saved network: {e}"))?;
            sim.world.env = saved.env;
            sim.world.helm = saved.helm;
            sim.set_network(saved.network);
        }
        _ => return Err("unknown endpoint".into()),
    }
    Ok(json!({"ok": true}))
}

fn capture(stream: &mut TcpStream, sim: &Arc<Mutex<Sim>>, query: &str) {
    let ydraw = query.contains("format=ydraw");
    let body = {
        let s = sim.lock().unwrap();
        let utc0 = s.world.env.start_utc;
        let mut out = String::with_capacity(s.ring.len() * 48);
        for r in s.ring.iter().filter(|r| !r.error) {
            let utc = utc0 + r.t;
            if ydraw {
                out.push_str(&ydraw_line(utc.rem_euclid(86400.0), 'R', &r.frame));
            } else {
                out.push_str(&candump_line(utc, "can0", &r.frame));
            }
            out.push('\n');
        }
        out
    };
    let name = if ydraw { "n2ksim-capture.ydraw.log" } else { "n2ksim-capture.candump.log" };
    respond(stream, "200 OK", "text/plain; charset=utf-8", &format!("Content-Disposition: attachment; filename=\"{name}\"\r\n"), body.as_bytes());
}

/// Server-sent events until the page goes away.
/// Counts a visitor's open pages, so their simulation is kept while any is open.
struct StreamGuard<'a> {
    ctx: &'a Ctx,
    id: String,
}
impl Drop for StreamGuard<'_> {
    fn drop(&mut self) {
        if let Some(v) = self.ctx.visitors.lock().unwrap().get_mut(&self.id) {
            v.streams = v.streams.saturating_sub(1);
            v.last_seen = Instant::now();
        }
    }
}

fn events(mut stream: TcpStream, ctx: &Ctx, sim: &Arc<Mutex<Sim>>, id: &str, set_cookie: &str) {
    let _guard = if ctx.hosted.is_some() {
        let mut map = ctx.visitors.lock().unwrap();
        let Some(v) = map.get_mut(id) else { return };
        if v.streams >= MAX_STREAMS_PER_VISITOR {
            drop(map);
            respond(&mut stream, "429 Too Many Requests", "text/plain", "", b"too many open pages");
            return;
        }
        v.streams += 1;
        Some(StreamGuard { ctx, id: id.to_string() })
    } else {
        None
    };
    let head = format!("HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n{set_cookie}Connection: keep-alive\r\n\r\n");
    if stream.write_all(head.as_bytes()).is_err() {
        return;
    }
    let (mut fseq, mut mseq) = {
        let s = sim.lock().unwrap();
        (s.frame_seq.saturating_sub(300), s.analyser.msg_seq.saturating_sub(300))
    };
    let mut full = true;
    loop {
        if !id.is_empty() {
            ctx.touch(id);
        }
        let payload = {
            let s = sim.lock().unwrap();
            let mut v = snapshot(&s, ctx, full);
            full = false;
            const CAP: usize = 600;
            let start = s.ring.partition_point(|r| r.seq <= fseq);
            let new = s.ring.len() - start;
            let skip = new.saturating_sub(CAP);
            let frames: Vec<J> = s
                .ring
                .range(start + skip..)
                .map(|r| {
                    let flags = r.error as u8 | match r.origin {
                        Origin::Client(_) => 2,
                        Origin::Console => 4,
                        Origin::Replay => 8,
                        Origin::Device(_) => 0,
                    };
                    json!([r.seq, r.t, format!("{:08X}", r.frame.id), r.frame.bytes().iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" "), flags])
                })
                .collect();
            if let Some(r) = s.ring.back() {
                fseq = r.seq;
            }
            let a = &s.analyser;
            let mstart = a.msgs.partition_point(|m| m.seq <= mseq);
            let mnew = a.msgs.len() - mstart;
            let mskip = mnew.saturating_sub(CAP);
            let msgs: Vec<J> = a.msgs.range(mstart + mskip..).map(|m| json!([m.seq, m.t, m.id.pgn, m.id.source, m.id.destination, m.id.priority, m.text])).collect();
            if let Some(m) = a.msgs.back() {
                mseq = m.seq;
            }
            v["frames"] = J::Array(frames);
            v["frames_skipped"] = json!(skip);
            v["msgs"] = J::Array(msgs);
            v["msgs_skipped"] = json!(mskip);
            v.to_string()
        };
        if stream.write_all(format!("data: {payload}\n\n").as_bytes()).is_err() {
            return;
        }
        std::thread::sleep(Duration::from_millis(250));
    }
}

fn name_json(n: u64) -> J {
    let n = Name::from_u64(n);
    json!({
        "hex": format!("{:016X}", n.to_u64()),
        "unique_number": n.unique_number,
        "manufacturer_code": n.manufacturer_code,
        "device_instance": n.device_instance,
        "device_function": n.device_function,
        "device_class": n.device_class,
        "system_instance": n.system_instance,
        "industry_group": n.industry_group,
        "arbitrary_address_capable": n.arbitrary_address_capable,
    })
}

fn r3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

pub fn snapshot(s: &Sim, ctx: &Ctx, full: bool) -> J {
    let now = s.t;
    let w = &s.world;
    let (lat, lon) = w.lat_lon();
    let st = &w.s;
    let deg = |x: f64| r3(x.to_degrees());
    let kn = |x: f64| r3(x / crate::world::KN);
    let devices: Vec<J> = s
        .devices
        .iter()
        .map(|d| {
            json!({
                "cfg": d.cfg,
                "state": d.state.key(),
                "address": d.address,
                "voltage": r3(d.voltage),
                "tec": d.tec, "rec": d.rec,
                "frames_tx": d.frames_tx, "frames_rx": d.frames_rx,
                "dropped": d.dropped, "error_frames": d.error_frames,
                "boots": d.boots, "address_changes": d.address_changes,
                "queue": d.tx.len(),
                "name_hex": format!("{:016X}", d.name().to_u64()),
                "order": d.last_order.map(|(t, a, src)| json!({"age": r3(now - t), "deg": deg(a), "src": src})),
            })
        })
        .collect();
    let sources: Vec<J> = s
        .analyser
        .srcs
        .iter()
        .map(|(a, i)| {
            json!({
                "src": a,
                "name": i.name.map(name_json),
                "product": i.product.as_ref().map(|p| json!({"model": p.model, "software": p.software, "model_version": p.model_version, "serial": p.serial, "code": p.code, "len": p.len})),
                "tx_list": i.tx_list, "rx_list": i.rx_list,
                "frames": i.frames, "age": r3(now - i.last_t), "claims": i.claims, "periodic": i.periodic,
                "heartbeat_age": i.heartbeat_t.map(|t| r3(now - t)),
            })
        })
        .collect();
    let pgns: Vec<J> = s
        .analyser
        .stats
        .values()
        .map(|x| {
            let mut v = serde_json::Map::new();
            let mut h = Vec::new();
            for (f, val) in &x.msg.values {
                if matches!(f.kind, Kind::Reserved) {
                    continue;
                }
                let num = match val {
                    Value::Num(n) => n.map(|n| json!(n)).unwrap_or(J::Null),
                    Value::Lookup(n) => n.map(|n| json!(n)).unwrap_or(J::Null),
                    Value::Bin(n) => n.map(|n| json!(n)).unwrap_or(J::Null),
                    Value::Str(t) => json!(t),
                    Value::List(l) => json!(l),
                    Value::Reserved => J::Null,
                };
                v.insert(f.id.to_string(), num);
                h.push(json!([f.name, human(f, val)]));
            }
            let def = pgn::def_of(x.pgn);
            json!({
                "pgn": x.pgn, "src": x.src, "name": pgn::pgn_name(x.pgn),
                "count": x.count, "hz": r3(x.rate(now)), "age": r3(now - x.last_t),
                "expected_hz": def.and_then(|d| (d.interval_ms > 0).then(|| 1000.0 / d.interval_ms as f64)),
                "fast": pgn::is_fast(x.pgn),
                "v": v, "h": h, "na": x.na_count,
            })
        })
        .collect();
    let mut out = json!({
        "t": r3(now),
        "utc": w.utc(),
        "paused": s.paused,
        "speed": s.speed,
        "bus": {
            "load": r3(s.bus_load()),
            "frames_total": s.frames_total,
            "errors_total": s.errors_total,
            "wiring_error_p": r3(s.wiring_error_prob()),
            "fast_errors": s.analyser.fast_errors(),
            "capture_frames": s.ring.len(),
        },
        "history": s.history.iter().rev().take(120).rev().map(|h| json!([r3(h.t), r3(h.load), r3(h.fps), r3(h.eps)])).collect::<Vec<_>>(),
        "backbone": s.net.backbone,
        "network_name": s.net.name,
        "devices": devices,
        "sources": sources,
        "pgns": pgns,
        "checks": s.checks,
        "truth": {
            "lat": lat, "lon": lon,
            "heading": deg(st.heading), "yaw_rate": deg(st.yaw_rate),
            "heel": deg(st.heel), "pitch": deg(st.pitch),
            "bsp": kn(st.bsp), "sog": kn(st.sog), "cog": deg(st.cog), "leeway": deg(st.leeway),
            "tws": kn(st.tws), "twd": deg(st.twd), "twa": deg(st.twa),
            "aws": kn(st.aws), "awa": deg(st.awa),
            "depth": r3(st.depth), "rudder": deg(st.rudder), "rudder_order": deg(st.rudder_order),
            "log_nm": r3(st.log_m / 1852.0), "heave": r3(st.heave), "ram_current": r3(st.ram_current),
            "external_timed_out": w.external_timed_out,
        },
        "env": w.env,
        "helm": w.helm,
        "gateway": {
            "addr": ctx.gateway_addr,
            "clients": s.clients,
            "up": s.gateway_up(),
            "dropped": s.client_dropped,
        },
        "replay": s.replay.as_ref().map(|r| json!({"name": r.name, "done": r.idx, "total": r.frames.len(), "mute": r.mute})),
    });
    if full {
        out["kinds"] = DeviceKind::ALL.iter().map(|k| json!({"key": k.key(), "title": k.title(), "pgns": k.default_pgns()})).collect();
        out["drills"] = scenarios::DRILLS.iter().map(|d| json!({"key": d.key, "title": d.title, "what": d.what})).collect();
        out["presets"] = scenarios::PRESETS.iter().map(|p| json!({"key": p.key, "title": p.title})).collect();
        out["pgn_defs"] = pgn::DEFS.iter().map(|d| json!({"pgn": d.pgn, "name": d.name, "fast": d.fast, "priority": d.priority, "interval_ms": d.interval_ms})).collect();
        out["config_path"] = json!(ctx.config_path.display().to_string());
        out["hosted"] = json!(ctx.hosted.is_some());
        out["library"] = crate::library::entries()
            .iter()
            .map(|e| {
                let (sim, not) = e.split_tx();
                let mut v = serde_json::to_value(e).unwrap_or(J::Null);
                v["simulated"] = json!(sim.iter().map(|p| p.pgn).collect::<Vec<_>>());
                v["not_simulated"] = json!(not.iter().map(|p| p.pgn).collect::<Vec<_>>());
                v
            })
            .collect();
        out["networks"] = scenarios::NETWORKS.iter().map(|n| json!({"key": n.key, "title": n.title, "what": n.what})).collect();
        out["variants"] = crate::device::VARIANTS.iter().map(|(p, v)| json!([p, v])).collect();
    }
    out
}
