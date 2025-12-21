//! Utility functions for sign extension

/// Sign extend an 8-bit value to 32-bit signed integer.
#[inline]
pub fn sign_extend_8bit(value: u8) -> i32 {
    value as i8 as i32
}

/// Sign extend a 12-bit value to 32-bit signed integer.
#[inline]
pub fn sign_extend_12bit(value: u16) -> i32 {
    (((value as i16) << 4) >> 4) as i32
}

/// Sign extend a 16-bit value to 32-bit signed integer.
#[inline]
pub fn sign_extend_16bit(value: u16) -> i32 {
    value as i16 as i32
}

/// Sign extend a 24-bit value to 32-bit signed integer.
#[inline]
pub fn sign_extend_24bit(value: u32) -> i32 {
    ((value as i32) << 8) >> 8
}
