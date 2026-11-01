//! Fast packet: messages up to 223 bytes over up to 32 frames.
//!
//! First frame: `seq << 5 | 0`, total length, 6 data bytes.
//! Later frames: `seq << 5 | index`, 7 data bytes. Unused bytes are 0xFF
//! (common practice; canboat does not state the fill for fast packet).

use std::collections::HashMap;

pub const MAX_LEN: usize = 223;

pub fn fragment(payload: &[u8], seq: u8) -> Vec<[u8; 8]> {
    assert!(payload.len() <= MAX_LEN, "fast packet longer than 223 bytes");
    let seq = (seq & 7) << 5;
    let mut out = Vec::new();
    let mut f = [0xFF; 8];
    f[0] = seq;
    f[1] = payload.len() as u8;
    let first = payload.len().min(6);
    f[2..2 + first].copy_from_slice(&payload[..first]);
    out.push(f);
    let mut i = first;
    let mut idx = 1u8;
    while i < payload.len() {
        let mut f = [0xFF; 8];
        f[0] = seq | idx;
        let n = (payload.len() - i).min(7);
        f[1..1 + n].copy_from_slice(&payload[i..i + n]);
        out.push(f);
        i += n;
        idx += 1;
    }
    out
}

/// Number of frames a payload of `len` bytes takes.
pub fn frame_count(len: usize) -> usize {
    if len <= 6 {
        1
    } else {
        1 + (len - 6).div_ceil(7)
    }
}

struct Partial {
    seq: u8,
    total: usize,
    next: u8,
    buf: Vec<u8>,
}

#[derive(Debug, PartialEq)]
pub enum Push {
    /// Waiting for more frames.
    More,
    /// The whole message.
    Done(Vec<u8>),
    /// A frame arrived out of order or without its first frame; the partial
    /// message (if any) was dropped.
    Error(&'static str),
}

/// Reassembles fast-packet messages per (source, PGN).
#[derive(Default)]
pub struct Assembler {
    partial: HashMap<(u8, u32), Partial>,
    pub errors: u64,
}

impl Assembler {
    pub fn push(&mut self, source: u8, pgn: u32, data: &[u8]) -> Push {
        if data.len() < 2 {
            self.errors += 1;
            return Push::Error("short frame");
        }
        let seq = data[0] >> 5;
        let idx = data[0] & 0x1F;
        let key = (source, pgn);
        if idx == 0 {
            let replaced = self.partial.remove(&key).is_some();
            let total = data[1] as usize;
            let n = total.min(6).min(data.len() - 2);
            let buf = data[2..2 + n].to_vec();
            if replaced {
                self.errors += 1;
            }
            if buf.len() >= total {
                return Push::Done(buf);
            }
            self.partial.insert(
                key,
                Partial {
                    seq,
                    total,
                    next: 1,
                    buf,
                },
            );
            return if replaced {
                Push::Error("new message before the last one finished")
            } else {
                Push::More
            };
        }
        let Some(p) = self.partial.get_mut(&key) else {
            self.errors += 1;
            return Push::Error("continuation frame without a first frame");
        };
        if p.seq != seq || p.next != idx {
            self.partial.remove(&key);
            self.errors += 1;
            return Push::Error("frame missing or out of order");
        }
        let n = (p.total - p.buf.len()).min(7).min(data.len() - 1);
        p.buf.extend_from_slice(&data[1..1 + n]);
        p.next += 1;
        if p.buf.len() >= p.total {
            let p = self.partial.remove(&key).expect("present");
            Push::Done(p.buf)
        } else {
            Push::More
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fragment_and_reassemble() {
        let payload: Vec<u8> = (0..43).collect();
        let frames = fragment(&payload, 5);
        assert_eq!(frames.len(), frame_count(43));
        assert_eq!(frames[0][0], 5 << 5);
        assert_eq!(frames[0][1], 43);
        assert_eq!(frames[1][0], (5 << 5) | 1);
        let mut a = Assembler::default();
        let mut out = None;
        for f in &frames {
            if let Push::Done(b) = a.push(9, 129029, f) {
                out = Some(b);
            }
        }
        assert_eq!(out, Some(payload));
        assert_eq!(a.errors, 0);
    }

    #[test]
    fn missing_frame_is_an_error() {
        let payload: Vec<u8> = (0..30).collect();
        let frames = fragment(&payload, 1);
        let mut a = Assembler::default();
        assert_eq!(a.push(1, 126996, &frames[0]), Push::More);
        assert!(matches!(a.push(1, 126996, &frames[2]), Push::Error(_)));
        assert_eq!(a.errors, 1);
    }
}
