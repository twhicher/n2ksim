//! NMEA 2000 over CAN, with no dependencies.
//!
//! - [`canid`]: the 29-bit identifier (priority, PGN, source, destination)
//! - [`name`]: the 64-bit ISO NAME carried by address claims
//! - [`fast`]: fast-packet fragmenting and reassembly
//! - [`pgn`]: field definitions for the PGNs we use, generic encode and decode
//! - [`format`]: Yacht Devices RAW and candump text lines
//!
//! Field layouts follow canboat's published definitions (canboat v9.1.0,
//! `docs/canboat.json`), transcribed from that file on 1 November 2026. Values are SI and
//! radians, as on the wire.

pub mod bits;
pub mod canid;
pub mod fast;
pub mod format;
pub mod name;
pub mod pgn;

pub use canid::CanId;
pub use name::Name;
pub use pgn::{decode, Builder, Message, Value};

/// One CAN frame as seen on the bus. `len` is the data length (0 to 8).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub len: u8,
    pub data: [u8; 8],
}

impl Frame {
    pub fn new(id: u32, bytes: &[u8]) -> Self {
        let mut data = [0xFF; 8];
        let len = bytes.len().min(8);
        data[..len].copy_from_slice(&bytes[..len]);
        Frame {
            id,
            len: len as u8,
            data,
        }
    }

    pub fn bytes(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn can_id(&self) -> CanId {
        CanId::decode(self.id)
    }
}

/// Split a message into CAN frames: one frame if it fits and the PGN is not
/// fast packet, otherwise fast-packet frames using sequence counter `seq`.
pub fn frames_for(id: CanId, payload: &[u8], seq: u8) -> Vec<Frame> {
    let raw = id.encode();
    if !pgn::is_fast(id.pgn) && payload.len() <= 8 {
        vec![Frame::new(raw, payload)]
    } else {
        fast::fragment(payload, seq)
            .into_iter()
            .map(|d| Frame::new(raw, &d))
            .collect()
    }
}
