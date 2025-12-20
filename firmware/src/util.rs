/// Sign extend a 21-bit value to 32-bit signed integer.
///
/// The BMM350 magnetometer uses 21-bit signed values stored in 24-bit registers.
/// Bit 20 is the sign bit (value 0x100000 = 2^20 = 1,048,576).
pub fn sign_extend_21bit(value: u32) -> i32 {
    // Mask to 21 bits
    let value = value & 0x1FFFFF;
    if value & 0x100000 != 0 {
        // Negative number: sign bit (bit 20) is set
        // Extend by setting bits 21-31
        (value | 0xFFE00000) as i32
    } else {
        value as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sign_extend_21bit() {
        // Zero
        assert_eq!(sign_extend_21bit(0), 0);

        // Max positive (2^20 - 1 = 1,048,575)
        assert_eq!(sign_extend_21bit(0x0FFFFF), 1048575);

        // -1 in 21-bit two's complement
        assert_eq!(sign_extend_21bit(0x1FFFFF), -1);

        // Min negative (-2^20 = -1,048,576)
        assert_eq!(sign_extend_21bit(0x100000), -1048576);

        // Some negative value
        assert_eq!(sign_extend_21bit(0x1FFFFE), -2);
    }
}
