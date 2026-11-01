//! The 64-bit ISO NAME sent in PGN 60928 (address claim).
//!
//! Lower NAME wins an address contest. Bit 48 is a spare bit and is 0.

use crate::bits::{get, mask, set};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Name {
    pub unique_number: u32,     // 21 bits
    pub manufacturer_code: u16, // 11 bits
    pub device_instance: u8,    // 8 bits, sent as lower 3 then upper 5
    pub device_function: u8,
    pub device_class: u8,    // 7 bits
    pub system_instance: u8, // 4 bits
    pub industry_group: u8,  // 3 bits, 4 = marine
    pub arbitrary_address_capable: bool,
}

impl Name {
    pub fn to_bytes(&self) -> [u8; 8] {
        let mut d = [0u8; 8];
        set(&mut d, 0, 21, self.unique_number as u64 & mask(21));
        set(&mut d, 21, 11, self.manufacturer_code as u64 & mask(11));
        set(&mut d, 32, 3, self.device_instance as u64 & 7);
        set(&mut d, 35, 5, (self.device_instance as u64 >> 3) & 31);
        set(&mut d, 40, 8, self.device_function as u64);
        set(&mut d, 48, 1, 0);
        set(&mut d, 49, 7, self.device_class as u64 & mask(7));
        set(&mut d, 56, 4, self.system_instance as u64 & 15);
        set(&mut d, 60, 3, self.industry_group as u64 & 7);
        set(&mut d, 63, 1, self.arbitrary_address_capable as u64);
        d
    }

    pub fn from_bytes(d: &[u8]) -> Option<Self> {
        if d.len() < 8 {
            return None;
        }
        let g = |s, n| get(d, s, n).unwrap_or(0);
        Some(Name {
            unique_number: g(0, 21) as u32,
            manufacturer_code: g(21, 11) as u16,
            device_instance: (g(32, 3) | (g(35, 5) << 3)) as u8,
            device_function: g(40, 8) as u8,
            device_class: g(49, 7) as u8,
            system_instance: g(56, 4) as u8,
            industry_group: g(60, 3) as u8,
            arbitrary_address_capable: g(63, 1) == 1,
        })
    }

    pub fn to_u64(&self) -> u64 {
        u64::from_le_bytes(self.to_bytes())
    }

    pub fn from_u64(v: u64) -> Self {
        Self::from_bytes(&v.to_le_bytes()).expect("eight bytes")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_and_ordering() {
        let a = Name {
            unique_number: 123_456,
            manufacturer_code: 2000,
            device_instance: 0b1010_1101,
            device_function: 145,
            device_class: 60,
            system_instance: 3,
            industry_group: 4,
            arbitrary_address_capable: true,
        };
        assert_eq!(Name::from_bytes(&a.to_bytes()), Some(a));
        assert_eq!(Name::from_u64(a.to_u64()), a);
        // The arbitrary-address bit is the most significant, so a device
        // that can move loses to an otherwise identical fixed one.
        let fixed = Name {
            arbitrary_address_capable: false,
            ..a
        };
        assert!(fixed.to_u64() < a.to_u64());
        assert_eq!(a.to_bytes()[6] & 1, 0, "spare bit 48 is zero");
    }
}
