//! `cargo run --release -p n2ksim` then open http://127.0.0.1:8090
//!
//! Options:
//!   --port N             web UI port (8090)
//!   --gateway HOST:PORT  Yacht Devices RAW over TCP (127.0.0.1:1457)
//!   --config PATH        saved network to load and save (config/network.json)
//!   --speed X            start at X times real time

use n2ksim::server::{self, Ctx, Saved};
use n2ksim::sim::{Sim, DT};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

fn main() {
    let mut port: u16 = 8090;
    let mut gateway = "127.0.0.1:1457".to_string();
    let mut config = PathBuf::from("config/network.json");
    let mut speed = 1.0;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        let mut val = || args.next().unwrap_or_else(|| die(&format!("{a} needs a value")));
        match a.as_str() {
            "--port" => port = val().parse().unwrap_or_else(|_| die("bad --port")),
            "--gateway" => gateway = val(),
            "--config" => config = PathBuf::from(val()),
            "--speed" => speed = val().parse().unwrap_or_else(|_| die("bad --speed")),
            "-h" | "--help" => {
                println!("n2ksim [--port 8090] [--gateway 127.0.0.1:1457] [--config config/network.json] [--speed 1]");
                return;
            }
            _ => die(&format!("unknown option {a}")),
        }
    }

    let saved: Saved = match std::fs::read_to_string(&config) {
        Ok(text) => serde_json::from_str(&text).unwrap_or_else(|e| die(&format!("{}: {e}", config.display()))),
        Err(_) => Saved { network: n2ksim::network::default_network(), ..Default::default() },
    };
    let mut sim = Sim::new(saved.network, saved.env, saved.helm);
    sim.speed = speed;
    let sim = Arc::new(Mutex::new(sim));

    {
        let sim = sim.clone();
        std::thread::spawn(move || run(sim));
    }
    {
        let sim = sim.clone();
        let addr = gateway.clone();
        std::thread::spawn(move || n2ksim::gateway::serve(addr, sim));
    }
    let bind = format!("127.0.0.1:{port}");
    println!("n2ksim: web UI on http://{bind}");
    println!("n2ksim: Yacht Devices RAW gateway on tcp://{gateway}");
    let ctx = Arc::new(Ctx { sim, port, gateway_addr: gateway, config_path: config });
    if let Err(e) = server::serve(&bind, ctx) {
        die(&format!("cannot listen on {bind}: {e}"));
    }
}

/// Keep simulation time in step with the wall clock times the speed.
fn run(sim: Arc<Mutex<Sim>>) {
    let mut last = Instant::now();
    let mut owed = 0.0f64;
    loop {
        std::thread::sleep(Duration::from_millis(5));
        let now = Instant::now();
        let wall = (now - last).as_secs_f64();
        last = now;
        let mut s = sim.lock().unwrap();
        if s.paused {
            owed = 0.0;
            continue;
        }
        // Never try to catch up more than a quarter second at once.
        owed = (owed + wall * s.speed).min(0.25 * s.speed.max(1.0));
        while owed >= DT {
            s.step();
            owed -= DT;
        }
    }
}

fn die(msg: &str) -> ! {
    eprintln!("n2ksim: {msg}");
    std::process::exit(2)
}
