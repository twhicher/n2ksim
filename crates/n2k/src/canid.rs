//! The 29-bit CAN identifier.
//!
//! ```text
//! bits 28..26  priority (0 highest)
//! bits 25..24  extended data page, data page
//! bits 23..16  PDU format (PF)
//! bits 15..8   PDU specific (PS): destination if PF < 240, else group extension
//! bits  7..0   source address
//! ```

/// Address 255: everyone.
pub const BROADCAST: u8 = 255;
/// Address 254: a device that could not claim an address.
pub const NULL_ADDRESS: u8 = 254;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CanId {
    pub priority: u8,
    pub pgn: u32,
    pub source: u8,
    /// 255 for broadcast and for every PDU2 (PF >= 240) PGN.
    pub destination: u8,
}

/// PDU1 PGNs are addressed: the PS byte carries the destination.
pub fn is_pdu1(pgn: u32) -> bool {
    ((pgn >> 8) & 0xFF) < 240
}

impl CanId {
    pub fn new(priority: u8, pgn: u32, source: u8, destination: u8) -> Self {
        CanId {
            priority,
            pgn,
            source,
            destination,
        }
    }

    pub fn encode(&self) -> u32 {
        let pgn = self.pgn & 0x3FFFF;
        let pgn_bits = if is_pdu1(pgn) {
            (pgn & 0x3FF00) | self.destination as u32
        } else {
            pgn
        };
        ((self.priority as u32 & 7) << 26) | (pgn_bits << 8) | self.source as u32
    }

    pub fn decode(id: u32) -> Self {
        let priority = ((id >> 26) & 7) as u8;
        let dp = (id >> 24) & 3;
        let pf = (id >> 16) & 0xFF;
        let ps = (id >> 8) & 0xFF;
        let source = (id & 0xFF) as u8;
        if pf < 240 {
            CanId {
                priority,
                pgn: (dp << 16) | (pf << 8),
                source,
                destination: ps as u8,
            }
        } else {
            CanId {
                priority,
                pgn: (dp << 16) | (pf << 8) | ps,
                source,
                destination: BROADCAST,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pdu2_wind() {
        // Wind data, priority 2, from address 0x23.
        let id = CanId::new(2, 130306, 0x23, 255);
        assert_eq!(id.encode(), 0x09FD_0223);
        assert_eq!(CanId::decode(0x09FD_0223), id);
    }

    #[test]
    fn pdu1_iso_request_carries_destination() {
        let id = CanId::new(6, 59904, 0x10, 0x23);
        let raw = id.encode();
        assert_eq!(raw, 0x18EA_2310);
        let back = CanId::decode(raw);
        assert_eq!(back.pgn, 59904);
        assert_eq!(back.destination, 0x23);
        assert_eq!(back.source, 0x10);
    }

    #[test]
    fn address_claim_to_broadcast() {
        let raw = CanId::new(6, 60928, 7, 255).encode();
        assert_eq!(raw, 0x18EE_FF07);
    }
}
