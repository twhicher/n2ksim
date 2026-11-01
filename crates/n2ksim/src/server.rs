//! A small HTTP server (standard library only): the page, a JSON API, and a
//! server-sent event stream with state four times a second plus every frame
//! and decoded message.
//!
//! Listens on 127.0.0.1 by default and refuses changes whose Origin is not
//! its own page.

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
use std::sync::{Arc, Mutex};
use std::time::Duration;

const PAGE: &str = include_str!("../static/index.html");
const MAX_BODY: usize = 32 * 1024 * 1024;
/// The web UI's own interface claims no address; it uses this one.
pub const CONSOLE_ADDRESS: u8 = 252;

pub struct Ctx {
    pub sim: Arc<Mutex<Sim>>,
    pub port: u16,
    pub gateway_addr: String,
    pub config_path: PathBuf,
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

fn ok(s: &mut TcpStream, v: J) {
    respond(s, "200 OK", "application/json", "", v.to_string().as_bytes());
}

fn err(s: &mut TcpStream, msg: &str) {
    respond(s, "400 Bad Request", "application/json", "", json!({"ok": false, "error": msg}).to_string().as_bytes());
}

fn handle(mut stream: TcpStream, ctx: &Ctx) {
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
    let mut origin = None;
    loop {
        let mut h = String::new();
        if reader.read_line(&mut h).is_err() || h == "\r\n" || h.is_empty() {
            break;
        }
        let lower = h.to_ascii_lowercase();
        if let Some(v) = lower.strip_prefix("content-length:") {
            len = v.trim().parse().unwrap_or(0);
        } else if let Some(v) = lower.strip_prefix("origin:") {
            origin = Some(v.trim().to_string());
        }
    }
    let (path, query) = target.split_once('?').unwrap_or((&target, ""));
    if method == "GET" {
        match path {
            "/" | "/index.html" => respond(&mut stream, "200 OK", "text/html; charset=utf-8", "", PAGE.as_bytes()),
            "/api/state" => {
                let v = snapshot(&ctx.sim.lock().unwrap(), ctx, true);
                ok(&mut stream, v)
            }
            "/api/events" => events(stream, ctx),
            "/api/capture" => capture(&mut stream, ctx, query),
            _ => respond(&mut stream, "404 Not Found", "text/plain", "", b"not found"),
        }
        return;
    }
    if method != "POST" {
        respond(&mut stream, "405 Method Not Allowed", "text/plain", "", b"");
        return;
    }
    let allowed = [format!("http://127.0.0.1:{}", ctx.port), format!("http://localhost:{}", ctx.port)];
    if origin.as_ref().is_some_and(|o| !allowed.contains(o)) {
        respond(&mut stream, "403 Forbidden", "text/plain", "", b"cross-origin request refused");
        return;
    }
    if len > MAX_BODY {
        return err(&mut stream, "body too large");
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
        let mut sim = ctx.sim.lock().unwrap();
        post(&mut sim, ctx, path, &body)
    };
    match result {
        Ok(v) => ok(&mut stream, v),
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

fn capture(stream: &mut TcpStream, ctx: &Ctx, query: &str) {
    let ydraw = query.contains("format=ydraw");
    let body = {
        let s = ctx.sim.lock().unwrap();
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
fn events(mut stream: TcpStream, ctx: &Ctx) {
    let head = "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\nConnection: keep-alive\r\n\r\n";
    if stream.write_all(head.as_bytes()).is_err() {
        return;
    }
    let (mut fseq, mut mseq) = {
        let s = ctx.sim.lock().unwrap();
        (s.frame_seq.saturating_sub(300), s.analyser.msg_seq.saturating_sub(300))
    };
    let mut full = true;
    loop {
        let payload = {
            let s = ctx.sim.lock().unwrap();
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
