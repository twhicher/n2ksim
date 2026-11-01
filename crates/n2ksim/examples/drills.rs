//! Run each troubleshooting drill for 30 s and print what the checks say.
//! `cargo run --release -p n2ksim --example drills`

use n2ksim::network::default_network;
use n2ksim::scenarios::{drill, DRILLS};
use n2ksim::sim::Sim;
use n2ksim::world::{EnvCfg, HelmCfg};
use std::time::Instant;

fn main() {
    for d in DRILLS {
        let mut s = Sim::new(default_network(), EnvCfg::default(), HelmCfg::default());
        s.run_for(3.0);
        drill(&mut s, d.key).unwrap();
        let t0 = Instant::now();
        s.run_for(30.0);
        println!("\n== {} ({:.0}x real time, load {:.0}%, {} frames, {} errors)", d.title, 30.0 / t0.elapsed().as_secs_f64(), s.bus_load() * 100.0, s.frames_total, s.errors_total);
        for c in &s.checks {
            println!("  [{} {}] {}", c.level, c.basis, c.title);
        }
    }
}

