//! Text formats for CAN frames.
//!
//! - Yacht Devices RAW (YDWG-02 manual, appendix E):
//!   `hh:mm:ss.ddd R 19F51323 01 02 03 04 05 06 07 08`. `R` is from the
//!   network, `T` is a frame the app sent. To send, a client writes the same
//!   line without the time and direction.
//! - candump log (`candump -L`): `(1700000000.123456) can0 19F51323#0102030405060708`.
//!   The plain candump layout `can0  19F51323   [8]  01 02 ...` is read too.
//!
//! canboat's `analyzer` reads both.

use crate::Frame;

fn hex_bytes(data: &[u8], sep: &str) -> String {
    data.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(sep)
}

/// `secs_of_day` is UTC seconds since midnight.
pub fn ydraw_line(secs_of_day: f64, dir: char, f: &Frame) -> String {
    let ms_total = (secs_of_day.rem_euclid(86400.0) * 1000.0) as u64;
    let (h, m, s, ms) = (ms_total / 3_600_000, (ms_total / 60_000) % 60, (ms_total / 1000) % 60, ms_total % 1000);
    format!("{h:02}:{m:02}:{s:02}.{ms:03} {dir} {:08X} {}", f.id, hex_bytes(f.bytes(), " "))
}

pub fn candump_line(unix_secs: f64, iface: &str, f: &Frame) -> String {
    format!("({unix_secs:.6}) {iface} {:08X}#{}", f.id, hex_bytes(f.bytes(), ""))
}

/// A frame read from a log line, with its time if the line had one:
/// seconds since midnight for RAW, Unix seconds for candump.
#[derive(Clone, Debug, PartialEq)]
pub struct LogFrame {
    pub t: Option<f64>,
    pub dir: Option<char>,
    pub frame: Frame,
}

fn parse_id(s: &str) -> Option<u32> {
    let id = u32::from_str_radix(s, 16).ok()?;
    (id <= 0x1FFF_FFFF).then_some(id)
}

fn parse_hex_tokens<'a>(toks: impl Iterator<Item = &'a str>) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    for t in toks {
        if out.len() == 8 {
            return None;
        }
        out.push(u8::from_str_radix(t, 16).ok()?);
    }
    Some(out)
}

pub fn parse_ydraw(line: &str) -> Option<LogFrame> {
    let mut toks = line.split_whitespace().peekable();
    let mut t = None;
    if let Some(first) = toks.peek() {
        if first.contains(':') {
            let p: Vec<f64> = first.split(':').map(|x| x.parse().ok()).collect::<Option<_>>()?;
            if p.len() != 3 {
                return None;
            }
            t = Some(p[0] * 3600.0 + p[1] * 60.0 + p[2]);
            toks.next();
        }
    }
    let mut dir = None;
    if let Some(&d) = toks.peek() {
        if d == "R" || d == "T" {
            dir = d.chars().next();
            toks.next();
        }
    }
    let id = parse_id(toks.next()?)?;
    let data = parse_hex_tokens(toks)?;
    Some(LogFrame { t, dir, frame: Frame::new(id, &data) })
}

pub fn parse_candump(line: &str) -> Option<LogFrame> {
    let line = line.trim();
    if let Some(rest) = line.strip_prefix('(') {
        let (ts, rest) = rest.split_once(')')?;
        let t: f64 = ts.parse().ok()?;
        let mut toks = rest.split_whitespace();
        let _iface = toks.next()?;
        let (id, data) = toks.next()?.split_once('#')?;
        if data.len() % 2 != 0 || data.len() > 16 {
            return None;
        }
        let bytes = (0..data.len()).step_by(2).map(|i| u8::from_str_radix(&data[i..i + 2], 16).ok()).collect::<Option<Vec<u8>>>()?;
        return Some(LogFrame { t: Some(t), dir: None, frame: Frame::new(parse_id(id)?, &bytes) });
    }
    // can0  19F51323   [8]  01 02 03 04 05 06 07 08
    let mut toks = line.split_whitespace();
    let _iface = toks.next()?;
    let id = parse_id(toks.next()?)?;
    let n = toks.next()?.strip_prefix('[')?.strip_suffix(']')?.parse::<usize>().ok()?;
    let data = parse_hex_tokens(toks)?;
    (data.len() == n).then(|| LogFrame { t: None, dir: None, frame: Frame::new(id, &data) })
}

/// Try candump first (it starts with `(` or an interface name), then RAW.
pub fn parse_any(line: &str) -> Option<LogFrame> {
    let l = line.trim();
    if l.is_empty() || l.starts_with('#') {
        return None;
    }
    parse_candump(l).or_else(|| parse_ydraw(l))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ydraw_round_trip() {
        let f = Frame::new(0x09FD_0223, &[1, 2, 3, 4, 5, 6, 7, 8]);
        let l = ydraw_line(13.0 * 3600.0 + 4.0 * 60.0 + 5.678, 'R', &f);
        assert_eq!(l, "13:04:05.678 R 09FD0223 01 02 03 04 05 06 07 08");
        let p = parse_ydraw(&l).unwrap();
        assert_eq!(p.frame, f);
        assert_eq!(p.dir, Some('R'));
        assert!((p.t.unwrap() - 47045.678).abs() < 1e-6);
        // What a client sends.
        let s = parse_ydraw("18EAFF10 14 F0 01").unwrap();
        assert_eq!(s.t, None);
        assert_eq!(s.frame.bytes(), &[0x14, 0xF0, 0x01]);
    }

    #[test]
    fn candump_both_layouts() {
        let f = Frame::new(0x09F8_0123, &[0xAA, 0xBB]);
        let l = candump_line(1_700_000_000.25, "can0", &f);
        assert_eq!(l, "(1700000000.250000) can0 09F80123#AABB");
        assert_eq!(parse_any(&l).unwrap().frame, f);
        let p = parse_any("  can0  09F80123   [2]  AA BB").unwrap();
        assert_eq!(p.frame, f);
        assert!(parse_any("garbage line").is_none());
    }
}
