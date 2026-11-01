# n2ksim

A simulated NMEA 2000 network for developing the autopilot in `../autopilot`. It runs a boat, a set of instruments on a CAN bus, and a gateway the autopilot connects to over TCP, with a web page to build the network, watch it, break it and find out what broke.

Rust, standard library plus `serde` and `serde_json`. Nothing here is data for a real boat or a real product.

## Run it

```
cargo run --release -p n2ksim
```

Open http://127.0.0.1:8090. The gateway listens on `tcp://127.0.0.1:1457`.

Options: `--port 8090`, `--gateway 127.0.0.1:1457` (use `0.0.0.0:1457` to reach it from another machine), `--config config/network.json` (loaded at start, written by **Save**), `--speed 5`.

## Hosting it

`--hosted` (or `N2KSIM_HOSTED=1`) runs it as a public demo: it listens on `0.0.0.0:$PORT`, gives each visitor their own simulation (kept by a cookie, cleared after 30 minutes idle, at most 25 at once), turns the gateway off, and offers Download instead of saving to disk. Changes are accepted only from the site's own pages, uploads are limited to 4 MB, and if `N2KSIM_PASSWORD` is set the page asks for it (any user name).

The `Dockerfile` builds that. On Railway:

```
railway init --name n2ksim
railway add --service n2ksim
railway up --service n2ksim --ci
railway domain --service n2ksim
railway variables --service n2ksim --set N2KSIM_PASSWORD=...   # optional
```

## What is on the bus

A 22 m Mid backbone from the stern up the mast, powered at the nav station, with eight devices:

| Device | Address | Sends |
|---|---|---|
| GNSS compass (dual antenna) | 2 | 129025 position 10 Hz, 129026 COG/SOG 4 Hz, 129029 GNSS 1 Hz, 126992 time 1 Hz, 127250 true heading 10 Hz |
| Heading sensor | 3 | 127250 magnetic heading, 127251 rate of turn, 127257 attitude, all 10 Hz |
| Depth, speed, temperature | 4 | 128259 speed, 128267 depth, 128275 log, 1 Hz; 130312 sea temperature 0.5 Hz |
| Masthead wind | 5 | 130306 apparent wind 10 Hz |
| Rudder drive | 6 | 127245 rudder position 10 Hz; takes angle orders |
| Battery monitor | 7 | 127508 1.5 s |
| Chart plotter | 1 | asks every device who it is |
| PC gateway | 8 | carries TCP clients' frames onto the bus |

Every device claims its address (60928) and defends it, remembers the address it last held across restarts, answers ISO requests (59904) for its claim, product information (126996), PGN lists (126464) and heartbeat (126993), and sends a heartbeat every 60 s. Frames are arbitrated by identifier, timed to the bit at 250 kbit/s including stuff bits, and carry fast-packet framing where the PGN needs it.

## The page

A black-and-white schematic in a drawing's title block. The title block across the top shows the network, the clock, speed, a moving-coil bus-load meter, frame and error rates, and the gateway.

- **Network**: the backbone drawn as a cable with its tees and terminators, data pulses running along it at the frame rate. Mast and deck kit hangs above the cable, the rest below, every device drawn to one scale from its maker's dimensions (generic devices use illustrative sizes and say so). Displays show live data, each device has an activity light, and the rudder ram's rod moves with the rudder. Drag a device or the power tee and it snaps to a free connector; click a terminator to remove or fit it. Select a device to edit its address, NAME, PGNs, rates and faults, and to see its error counters and its library details with sources. Below: the bus as an analyser sees it, and the instrument library.
- **Instruments**: what a display would show, decoded from frames on the bus, with a source picker on each, a track, and bus against truth.
- **Traffic**: load and frame-rate charts, every PGN by source, a live log, sending frames, capture and replay.
- **Sailing**: conditions, presets, steering and how to connect the autopilot.
- **Test** (the tab on the right edge): findings with where their evidence comes from, and drills that each break one thing.

## Instrument library

`crates/n2ksim/library/instruments.json` describes 29 real products from their makers' documentation: what each sends and receives, how often, its LEN and settings, with the source of every figure. Anything not found is left out and listed under "gaps and conflicts", never guessed, as are figures where sources disagree; PGNs we inferred rather than read are marked. Manufacturer codes and class and function codes come from canboat's lookups.

| Make | In the library |
|---|---|
| B&G | WS310 and WS320 wind (each through its interface), ZG100, Precision-9, H5000 CPU, Pilot Computer, Graphic Display and Pilot Controller, Triton2, Nemesis |
| NKE | Box N2K. NKE's sensors, processors and pilots are on its own Topline bus, not NMEA 2000; the Box N2K puts their data on the bus |
| Raymarine | EV-1, ACU-200, iTC-5, i70s |
| Others | Airmar DST810, 220WX and H2183; Furuno SCX-20; Garmin GPS 19x and GNX 20; Calypso ULP; LCJ CV7-C with WindyPlug; Navico RF25N; Maretron DCM100; Victron Lynx Shunt VE.Can; Yacht Devices YDWG-02 and YDEN-02; Actisense NGX-1 |

Add any of them from **Add device**; the inspector shows the maker's PGN list with each one's status (simulated, not simulated, network management), settings, gaps and sources. A library device sends the real unit's PGNs that the simulator can generate, at the documented interval or canboat's usual one. PGNs the simulator cannot generate yet (mostly proprietary ones, AIS, GNSS satellites in view) are listed, not sent.

**Load network** replaces the network with a ready-made one: generic, a B&G race boat, an NKE race boat, or a Raymarine boat. Each keeps the simulator's own rudder drive so the autopilot can still steer; library rudder sensors (RF25N, ACU-200) only report.

What the documentation does not give, and so the library does not either: B&G publishes no PGN lists for the H5000 units or the Nemesis, so they send only network management here; almost no product documents its transmit intervals, its NMEA 2000 class and function, or the Model ID it reports.

## Connecting the autopilot

The gateway speaks the Yacht Devices RAW text format (YDWG-02 manual, appendix E). Frames arrive as:

```
10:00:01.234 R 09FD0205 03 4C 06 1B 21 FA FF FF
```

A client sends a frame by writing the identifier and bytes, and gets it echoed back with `T`:

```
09F10D32 00 F8 69 03 FF 7F FF FF
```

That line is PGN 127245 from address 50 with an angle order of 0.0873 rad (+5°). With steering set to **Autopilot (external)**, the rudder drive follows angle orders for instance 0 from any other address, moves the rudder at 6°/s, and holds the last order and raises a finding when none has arrived for the timeout (1 s by default).

`crates/n2k/examples/ydraw_pilot.rs` is a small working client: it claims an address, asks everyone to announce themselves, takes heading from a source chosen by NAME, and steers to a course.

```
cargo run --release -p n2ksim
cargo run -p n2k --example ydraw_pilot -- 127.0.0.1:1457 160
```

A real YDWG-02 or YDEN-02 needs one of its servers switched to RAW; the factory default is NMEA 0183 on TCP 1456, and the manual documents no default RAW port. Port 1457 here is our choice.

The `n2k` crate (no dependencies) has the codec the autopilot needs on the boat's computer: identifiers, NAME, fast packet, field definitions, encode and decode, and both log formats. The natural next step in the autopilot is a small bridge that turns these PGNs into its `SensorMsg`.

## Drills

Each drill breaks one thing so you can see how it looks on the bus and test how the autopilot copes: missing terminator, address conflict, wind sensor stuck, GNSS without a fix, GNSS hung, a babbling device, low voltage, heading sensor offset, slow wind data, noisy wind, loose connector, faulty transceiver. **Clear faults** undoes them; **Reset network** goes back to the default.

`cargo run --release -p n2ksim --example drills` runs each one for 30 s and prints the findings.

## Test

```
cargo test --release
cargo clippy --release --all-targets
```

## Where the numbers come from

- **PGN layouts**: canboat v9.1.0 `docs/canboat.json`, checked 1 November 2026, including resolutions and usual intervals. Fast-packet fill bytes of 0xFF are common practice; canboat does not state them.
- **Physical layer**: drops at most 6 m each and 78 m in total, 1 LEN = 50 mA, devices run on 9 to 16 V (Garmin NMEA 2000 network guide); backbone limits of 100 m for Micro and 250 m for Mid and Heavy, terminator to terminator, and current limits of 3, 4 and 8 A (Actisense, *Complete Guide to NMEA 2000*, revision 4, 2021).
- **Cable resistance**: no manufacturer figure was found, so voltage drop uses copper wire-gauge values for the gauges Actisense gives (22, 18 and 15 AWG). Treat voltages as approximate.
- **Not from any source** and illustrative only: the error rates caused by missing terminators and long backbones, the 40% bus-load rule of thumb, sensor noise and lag, the boat model and its polar, device class and function codes, manufacturer code 2000, and every product name.

## What it does not model

- The boat is a simple model (first-order yaw, surge towards a placeholder polar, roll as a damped oscillator, three-component waves), not the autopilot's four-dof model. For steering performance use the autopilot's own harness; use this for the bus.
- No ISO transport protocol (60160, 60416), no group function (126208) handling, no proprietary PGNs beyond the babbler's junk frames.
- One backbone with one power tee. No NMEA 0183, no SignalK, no SocketCAN or USB-CAN adapter yet; on Linux a `vcan` bridge would be the obvious addition.
- Error frames are a probability per frame, not a signal-level simulation.
- Actisense binary log formats are not read; convert them with canboat first.
- Library devices model each product's documented bus output, not its internals: a processor's data is the simulator's truth with sensor noise, not computed from the other devices' messages.
- Library devices claim real manufacturer codes. That is fine on the simulated bus; anything that ever transmits onto a real boat's network should not impersonate another maker's product.
