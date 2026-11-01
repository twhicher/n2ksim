//! A TCP server that behaves like a Yacht Devices gateway with a server set
//! to RAW: every frame on the bus goes to every client as
//! `hh:mm:ss.ddd R 09FD0223 01 02 ...`; a client sends a frame by writing
//! `09FD0223 01 02 ...` and gets it echoed back with `T`. Lines that do not
//! parse are ignored, as the real gateway does.
//!
//! Frames reach clients only while the gateway device is on the bus, and
//! clients' frames go out through its transmit queue, so a gateway with
//! faults (unplugged, low voltage, bus off) behaves like one.

use crate::device::Origin;
use crate::sim::Sim;
use n2k::format::{parse_ydraw, ydraw_line};
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

static NEXT_CLIENT: AtomicU32 = AtomicU32::new(1);

pub fn serve(addr: String, sim: Arc<Mutex<Sim>>) {
    let listener = match TcpListener::bind(&addr) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("gateway: cannot listen on {addr}: {e}");
            return;
        }
    };
    for stream in listener.incoming().flatten() {
        let sim = sim.clone();
        std::thread::spawn(move || client(stream, sim));
    }
}

fn client(stream: TcpStream, sim: Arc<Mutex<Sim>>) {
    let id = NEXT_CLIENT.fetch_add(1, Ordering::Relaxed);
    let _ = stream.set_nodelay(true);
    let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
    eprintln!("gateway: client {id} connected from {peer}");
    let mut cursor = {
        let mut s = sim.lock().unwrap();
        s.clients += 1;
        s.frame_seq
    };
    let reader_stream = match stream.try_clone() {
        Ok(s) => s,
        Err(_) => return,
    };
    let rsim = sim.clone();
    let reader = std::thread::spawn(move || {
        let r = BufReader::new(reader_stream);
        for line in r.lines() {
            let Ok(line) = line else { break };
            if let Some(lf) = parse_ydraw(&line) {
                if lf.t.is_none() && lf.dir.is_none() {
                    rsim.lock().unwrap().inbox.push((id, lf.frame));
                }
            }
        }
    });
    let mut w = stream;
    loop {
        std::thread::sleep(Duration::from_millis(20));
        if reader.is_finished() {
            break;
        }
        let mut out = String::new();
        {
            let s = sim.lock().unwrap();
            let utc0 = s.world.env.start_utc;
            let start = s.ring.partition_point(|r| r.seq <= cursor);
            for r in s.ring.range(start..) {
                cursor = r.seq;
                if r.error || !r.gw {
                    continue;
                }
                let dir = if r.origin == Origin::Client(id) { 'T' } else { 'R' };
                out.push_str(&ydraw_line((utc0 + r.t).rem_euclid(86400.0), dir, &r.frame));
                out.push_str("\r\n");
            }
        }
        if !out.is_empty() && w.write_all(out.as_bytes()).is_err() {
            break;
        }
    }
    let _ = w.shutdown(std::net::Shutdown::Both);
    sim.lock().unwrap().clients -= 1;
    eprintln!("gateway: client {id} disconnected");
}
