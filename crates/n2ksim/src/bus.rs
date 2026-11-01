//! CAN frame timing: how many bits an extended data frame takes on the wire,
//! including stuff bits, so bus load is right rather than estimated.

use n2k::Frame;

fn crc15(bits: &[bool]) -> u16 {
    let mut crc: u16 = 0;
    for &b in bits {
        let next = b ^ ((crc >> 14) & 1 == 1);
        crc = (crc << 1) & 0x7FFF;
        if next {
            crc ^= 0x4599;
        }
    }
    crc
}

fn push(bits: &mut Vec<bool>, v: u64, n: usize) {
    for i in (0..n).rev() {
        bits.push((v >> i) & 1 == 1);
    }
}

/// Bits on the wire for an extended (29-bit) data frame, from start of frame
/// through interframe space.
pub fn frame_bits(f: &Frame) -> u32 {
    let mut bits = Vec::with_capacity(160);
    bits.push(false); // SOF
    push(&mut bits, (f.id >> 18) as u64 & 0x7FF, 11);
    bits.push(true); // SRR
    bits.push(true); // IDE
    push(&mut bits, f.id as u64 & 0x3FFFF, 18);
    bits.push(false); // RTR
    bits.push(false); // r1
    bits.push(false); // r0
    push(&mut bits, f.len as u64, 4);
    for b in f.bytes() {
        push(&mut bits, *b as u64, 8);
    }
    let crc = crc15(&bits);
    push(&mut bits, crc as u64, 15);
    // Stuff bits: after five equal bits the sender inserts the opposite one,
    // which then counts towards the next run.
    let mut stuffed = 0u32;
    let mut run = 0u32;
    let mut last = None;
    for &b in &bits {
        if Some(b) == last {
            run += 1;
        } else {
            run = 1;
            last = Some(b);
        }
        if run == 5 {
            stuffed += 1;
            last = Some(!b);
            run = 1;
        }
    }
    // CRC delimiter, ACK slot and delimiter, end of frame, interframe space.
    bits.len() as u32 + stuffed + 1 + 2 + 7 + 3
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eight_byte_frame_is_in_the_expected_range() {
        // 131 bits without stuffing; at most about 25 stuff bits.
        let f = Frame::new(0x09FD_0223, &[0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]);
        let n = frame_bits(&f);
        assert!((131..=160).contains(&n), "{n}");
        let zeros = Frame::new(0, &[0; 8]);
        assert!(frame_bits(&zeros) > frame_bits(&f), "long runs stuff more");
        assert!(frame_bits(&Frame::new(0x18EA_FF10, &[1, 2, 3])) < n);
    }
}
