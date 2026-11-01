//! A minimal heading-hold pilot over a Yacht Devices RAW gateway, to show
//! the autopilot's side of the bus end to end. Standard library only.
//!
//! ```text
//! cargo run --release -p n2ksim                      # in one terminal
//! cargo run -p n2k --example ydraw_pilot -- 127.0.0.1:1457 160
//! ```
//!
//! Set steering to "Autopilot (external)" in the simulator's Sailing tab.
//!
//! It does what a well-behaved device should:
//! - claims an address and moves if it loses a contest, then asks every
//!   device to announce itself so it knows who is where
//! - answers ISO requests for its address claim
//! - picks its heading source by NAME, so it follows the sensor if the
//!   sensor's address changes (try the "Address conflict" drill)
//! - sends PGN 127245 with an angle order ten times a second

use n2k::canid::BROADCAST;
use n2k::fast::{Assembler, Push};
use n2k::format::{parse_ydraw, ydraw_line};
use n2k::pgn::{self, Builder};
use n2k::{frames_for, CanId, Name};
use std::collections::HashMap;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

#[derive(Default)]
struct Bus {
    address: u8,
    /// Address to NAME, from claims.
    names: HashMap<u8, u64>,
    /// The NAME of the device we take heading from, once chosen.
    heading_name: Option<u64>,
    heading: Option<(Instant, f64)>,
    rate: Option<f64>,
    reclaim: bool,
}

fn my_name() -> Name {
    Name {
        unique_number: 4242,
        manufacturer_code: 2000,
        device_function: 150,
        device_class: 40,
        industry_group: 4,
        arbitrary_address_capable: true,
        ..Default::default()
    }
}

fn send(w: &mut TcpStream, prio: u8, p: u32, src: u8, dst: u8, data: &[u8]) -> std::io::Result<()> {
    for f in frames_for(CanId::new(prio, p, src, dst), data, 0) {
        // A client sends the line without time and direction.
        let line = ydraw_line(0.0, 'X', &f);
        let bare = line.splitn(3, ' ').nth(2).unwrap_or("");
        w.write_all(format!("{bare}\r\n").as_bytes())?;
    }
    Ok(())
}

fn wrap_pi(a: f64) -> f64 {
    let t = std::f64::consts::TAU;
    let x = (a + std::f64::consts::PI).rem_euclid(t) - std::f64::consts::PI;
    if x <= -std::f64::consts::PI {
        x + t
    } else {
        x
    }
}

fn main() -> std::io::Result<()> {
    let mut args = std::env::args().skip(1);
    let addr = args.next().unwrap_or_else(|| "127.0.0.1:1457".into());
    let target: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(160.0);
    let stream = TcpStream::connect(&addr)?;
    stream.set_nodelay(true)?;
    let mut w = stream.try_clone()?;
    let bus = Arc::new(Mutex::new(Bus { address: 50, ..Default::default() }));
    let me = my_name().to_u64();
    send(&mut w, 6, pgn::ADDRESS_CLAIM, 50, BROADCAST, &my_name().to_bytes())?;
    // Joining a running bus: ask everyone to claim again so we learn who
    // is at which address.
    let req = Builder::new(pgn::ISO_REQUEST).bin("pgn", pgn::ADDRESS_CLAIM as u64).encode();
    send(&mut w, 6, pgn::ISO_REQUEST, 50, BROADCAST, &req)?;
    println!("connected to {addr}, claiming address 50, holding {target}°T");

    let rb = bus.clone();
    std::thread::spawn(move || {
        let mut asm = Assembler::default();
        for line in BufReader::new(stream).lines() {
            let Ok(line) = line else { break };
            let Some(lf) = parse_ydraw(&line) else { continue };
            if lf.dir == Some('T') {
                continue; // our own frame echoed back
            }
            let id = lf.frame.can_id();
            let payload = if pgn::is_fast(id.pgn) {
                match asm.push(id.source, id.pgn, lf.frame.bytes()) {
                    Push::Done(b) => b,
                    _ => continue,
                }
            } else {
                lf.frame.bytes().to_vec()
            };
            let mut b = rb.lock().unwrap();
            match id.pgn {
                pgn::ADDRESS_CLAIM => {
                    let Some(n) = Name::from_bytes(&payload) else { continue };
                    let n = n.to_u64();
                    b.names.retain(|_, v| *v != n);
                    b.names.insert(id.source, n);
                    if id.source == b.address && n != me {
                        if me < n {
                            b.reclaim = true; // defend
                        } else {
                            let taken: Vec<u8> = b.names.keys().copied().collect();
                            b.address = (b.address + 1..252).find(|a| !taken.contains(a)).unwrap_or(254);
                            b.reclaim = true;
                            println!("lost address contest, moving to {}", b.address);
                        }
                    }
                }
                pgn::ISO_REQUEST => {
                    let req = n2k::decode(pgn::ISO_REQUEST, &payload).num("pgn");
                    if req == Some(pgn::ADDRESS_CLAIM as f64) && (id.destination == b.address || id.destination == BROADCAST) {
                        b.reclaim = true;
                    }
                }
                pgn::HEADING => {
                    let m = n2k::decode(pgn::HEADING, &payload);
                    if m.lookup("reference") != Some(0) {
                        continue; // true heading only, for this example
                    }
                    let Some(&name) = b.names.get(&id.source) else { continue };
                    if b.heading_name.is_none() {
                        b.heading_name = Some(name);
                        println!("taking true heading from NAME {name:016X} at address {}", id.source);
                    }
                    if b.heading_name == Some(name) {
                        if let Some(h) = m.num("heading") {
                            b.heading = Some((Instant::now(), h));
                        }
                    }
                }
                pgn::RATE_OF_TURN => b.rate = n2k::decode(pgn::RATE_OF_TURN, &payload).num("rate"),
                _ => {}
            }
        }
        println!("gateway closed the connection");
        std::process::exit(0);
    });

    let target = target.to_radians();
    let mut last_print = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(100));
        let (src, heading, rate, reclaim) = {
            let mut b = bus.lock().unwrap();
            let r = b.reclaim;
            b.reclaim = false;
            (b.address, b.heading, b.rate.unwrap_or(0.0), r)
        };
        if reclaim {
            send(&mut w, 6, pgn::ADDRESS_CLAIM, src, BROADCAST, &my_name().to_bytes())?;
        }
        let Some((at, h)) = heading else { continue };
        if at.elapsed() > Duration::from_secs(2) {
            if last_print.elapsed() > Duration::from_secs(2) {
                println!("heading is stale; sending no orders (the drive will hold and alarm)");
                last_print = Instant::now();
            }
            continue;
        }
        let err = wrap_pi(target - h);
        let order = (1.0 * err - 3.0 * rate).clamp(-0.5, 0.5);
        let data = Builder::new(pgn::RUDDER).num("instance", 0.0).lookup("directionOrder", 0).num("angleOrder", order).encode();
        send(&mut w, 2, pgn::RUDDER, src, BROADCAST, &data)?;
        if last_print.elapsed() > Duration::from_secs(2) {
            println!("heading {:6.1}°  error {:+5.1}°  order {:+5.1}°", h.to_degrees(), err.to_degrees(), order.to_degrees());
            last_print = Instant::now();
        }
    }
}
