//! `cargo run --release -p n2ksim` then open http://127.0.0.1:8090
//!
//! Options:
//!   --port N             web UI port (8090, or $PORT when hosted)
//!   --bind ADDR          address to listen on (127.0.0.1; 0.0.0.0 when hosted)
//!   --gateway HOST:PORT  Yacht Devices RAW over TCP (127.0.0.1:1457), or "off"
//!   --config PATH        saved network to load and save (config/network.json)
//!   --speed X            start at X times real time
//!   --hosted             public server: a simulation per visitor, no gateway,
//!                        no saving to disk. Also set by N2KSIM_HOSTED=1.
//!                        N2KSIM_PASSWORD, if set, is required to open it.

use n2ksim::server::{self, Ctx, Hosted, Saved};
use n2ksim::sim::{Sim, DT};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

fn main() {
    let hosted = std::env::var("N2KSIM_HOSTED").is_ok_and(|v| v == "1" || v == "true") || std::env::args().any(|a| a == "--hosted");
    let mut port: u16 = std::env::var("PORT").ok().and_then(|p| p.parse().ok()).filter(|_| hosted).unwrap_or(8090);
    let mut bind = if hosted { "0.0.0.0" } else { "127.0.0.1" }.to_string();
    let mut gateway = if hosted { "off" } else { "127.0.0.1:1457" }.to_string();
    let mut config = PathBuf::from("config/network.json");
    let mut speed = 1.0;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| die(&format!("{a} needs a value")));
        match a.as_str() {
            "--port" => port = val().parse().unwrap_or_else(|_| die("bad --port")),
            "--bind" => bind = val(),
            "--gateway" => gateway = val(),
            "--config" => config = PathBuf::from(val()),
            "--speed" => speed = val().parse().unwrap_or_else(|_| die("bad --speed")),
            "--hosted" => {}
            "-h" | "--help" => {
                println!("n2ksim [--port 8090] [--bind 127.0.0.1] [--gateway 127.0.0.1:1457|off] [--config config/network.json] [--speed 1] [--hosted]");
                return;
            }
            _ => die(&format!("unknown option {a}")),
        }
    }

    let saved: Saved = match std::fs::read_to_string(&config) {
        Ok(text) => serde_json::from_str(&text).unwrap_or_else(|e| die(&format!("{}: {e}", config.display()))),
        Err(_) => Saved { network: n2ksim::network::default_network(), ..Default::default() },
    };
    let addr = format!("{bind}:{port}");
    let ctx = if hosted {
        let password = std::env::var("N2KSIM_PASSWORD").ok().filter(|p| !p.is_empty());
        println!("n2ksim: hosted on http://{addr}, a simulation per visitor{}", if password.is_some() { ", password required" } else { "" });
        Arc::new(Ctx::hosted(Hosted { password, template: saved, max_visitors: 25, idle: Duration::from_secs(30 * 60) }, port))
    } else {
        let mut sim = Sim::new(saved.network, saved.env, saved.helm);
        sim.speed = speed;
        let sim = Arc::new(Mutex::new(sim));
        println!("n2ksim: web UI on http://{addr}");
        if gateway != "off" {
            let (s, g) = (sim.clone(), gateway.clone());
            std::thread::spawn(move || n2ksim::gateway::serve(g, s));
            println!("n2ksim: Yacht Devices RAW gateway on tcp://{gateway}");
        }
        Arc::new(Ctx::local(sim, port, if gateway == "off" { "off".into() } else { gateway }, config))
    };
    {
        let ctx = ctx.clone();
        std::thread::spawn(move || run(ctx));
    }
    if let Err(e) = server::serve(&addr, ctx) {
        die(&format!("cannot listen on {addr}: {e}"));
    }
}

/// Keep each simulation in step with the wall clock times its speed.
fn run(ctx: Arc<Ctx>) {
    let mut last = Instant::now();
    let mut last_evict = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(5));
        let now = Instant::now();
        let wall = (now - last).as_secs_f64();
        last = now;
        for sim in ctx.all_sims() {
            let mut s = sim.lock().unwrap();
            if s.paused {
                s.owed = 0.0;
                continue;
            }
            // Never try to catch up more than a quarter second at once.
            s.owed = (s.owed + wall * s.speed).min(0.25 * s.speed.max(1.0));
            while s.owed >= DT {
                s.step();
                s.owed -= DT;
            }
        }
        if last_evict.elapsed() > Duration::from_secs(10) {
            last_evict = Instant::now();
            ctx.evict_idle();
        }
    }
}

fn die(msg: &str) -> ! {
    eprintln!("n2ksim: {msg}");
    std::process::exit(2)
}
