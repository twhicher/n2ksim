//! Little-endian bit fields, least significant bit first, as NMEA 2000 packs them.

/// Read `n` bits (up to 64) starting at bit `start`. `None` if past the end.
pub fn get(data: &[u8], start: usize, n: usize) -> Option<u64> {
    if n == 0 || n > 64 || start + n > data.len() * 8 {
        return None;
    }
    let mut v = 0u64;
    for i in 0..n {
        let b = start + i;
        if (data[b / 8] >> (b % 8)) & 1 == 1 {
            v |= 1u64 << i;
        }
    }
    Some(v)
}

/// Write the low `n` bits of `v` starting at bit `start`.
pub fn set(data: &mut [u8], start: usize, n: usize, v: u64) {
    for i in 0..n {
        let b = start + i;
        if (v >> i) & 1 == 1 {
            data[b / 8] |= 1 << (b % 8);
        } else {
            data[b / 8] &= !(1 << (b % 8));
        }
    }
}

/// All ones in the low `n` bits.
pub fn mask(n: usize) -> u64 {
    if n >= 64 {
        u64::MAX
    } else {
        (1u64 << n) - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip_across_byte_boundaries() {
        let mut d = [0u8; 8];
        set(&mut d, 3, 13, 0x1ABC);
        assert_eq!(get(&d, 3, 13), Some(0x1ABC));
        set(&mut d, 16, 32, 0xDEAD_BEEF);
        assert_eq!(get(&d, 16, 32), Some(0xDEAD_BEEF));
        assert_eq!(d[2], 0xEF, "little endian");
        assert_eq!(get(&d, 60, 8), None);
    }
}
