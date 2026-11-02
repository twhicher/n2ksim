//! n2ksim's page served by another program, under `/network/`, with that
//! program's boat in charge of wind, sea and steering. A stand-in for a
//! host such as an autopilot simulator; the boat here is n2ksim's own,
//! wrapped so the page treats it as the host's.
//!
//! `cargo run --release -p n2ksim --example mounted -- 8095`
//! then open http://127.0.0.1:8095 and go below deck.

use n2ksim::network::default_network;
use n2ksim::server::{answer, read_head, Ctx, Mount};
use n2ksim::sim::{Sim, DT};
use n2ksim::world::{EnvCfg, HelmCfg, Physics, SimpleBoat, State};
use std::io::{BufReader, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const HOST: &str = "Example host";

struct HostBoat(SimpleBoat);

impl Physics for HostBoat {
    fn reset(&mut self, env: &EnvCfg, helm: &HelmCfg, s: &mut State) {
        self.0.reset(env, helm, s)
    }
    fn step(&mut self, dt: f64, t: f64, env: &EnvCfg, order: f64, s: &mut State) {
        self.0.step(dt, t, env, order, s)
    }
    fn owner(&self) -> Option<&str> {
        Some(HOST)
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

const DECK: &str = r#"<!doctype html><meta charset="utf-8"><title>Example host</title>
<body style="font:16px system-ui;margin:40px">
<h1>Example host</h1>
<p>This stands in for the program that owns the boat and runs the clock.</p>
<p><a href="network/">Below deck</a>: the NMEA 2000 network.</p>"#;

fn main() {
    let port: u16 = std::env::args().nth(1).and_then(|p| p.parse().ok()).unwrap_or(8095);
    let sim = Sim::with_physics(default_network(), EnvCfg::default(), HelmCfg::default(), Box::new(HostBoat(SimpleBoat::default())));
    let sim = Arc::new(Mutex::new(sim));
    let mount = Mount { host: HOST.into(), back_label: "On deck".into(), back_href: "../".into(), public: false };
    let ctx = Arc::new(Ctx::mounted(sim.clone(), port, mount));

    // The host's clock, in real time.
    std::thread::spawn(move || {
        let mut last = Instant::now();
        let mut owed = 0.0;
        loop {
            std::thread::sleep(Duration::from_millis(5));
            owed = (owed + last.elapsed().as_secs_f64()).min(0.25);
            last = Instant::now();
            let mut s = sim.lock().unwrap();
            while owed >= DT {
                s.step();
                owed -= DT;
            }
        }
    });

    let listener = TcpListener::bind(("127.0.0.1", port)).expect("port free");
    println!("example host on http://127.0.0.1:{port}, n2ksim below deck at /network/");
    for stream in listener.incoming().flatten() {
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let Ok(clone) = stream.try_clone() else { return };
            let mut reader = BufReader::new(clone);
            let Some(mut head) = read_head(&mut reader) else { return };
            let mut stream = stream;
            match head.target.strip_prefix("/network") {
                Some(rest) if rest.starts_with('/') => {
                    head.target = rest.to_string();
                    answer(stream, reader, &ctx, head);
                }
                // The page's paths are relative, so it needs the slash.
                Some("") => {
                    let _ = stream.write_all(b"HTTP/1.1 308 Permanent Redirect\r\nLocation: /network/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                }
                _ => {
                    let head = format!("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n", DECK.len());
                    let _ = stream.write_all(head.as_bytes());
                    let _ = stream.write_all(DECK.as_bytes());
                }
            }
        });
    }
}
