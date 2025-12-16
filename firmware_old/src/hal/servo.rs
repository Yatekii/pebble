// Servo control driver using LEDC PWM
// Standard servos use 50Hz PWM with 1-2ms pulse width for 0-180 degrees
//
// Due to esp-hal's LEDC lifetime requirements, this module provides helper
// functions rather than a self-contained struct. The timer must outlive the
// channel, so both must be kept in the same scope (typically main).

use esp_hal::ledc::channel::{self, ChannelHW};
use esp_hal::ledc::LowSpeed;

/// Convert servo angle (0-180 degrees) to PWM duty value
///
/// With 14-bit resolution (16384 steps) at 50Hz (20ms period):
/// - 0° = 5% duty (1ms pulse) = 819
/// - 90° = 7.5% duty (1.5ms pulse) = 1229
/// - 180° = 10% duty (2ms pulse) = 1638
pub fn angle_to_duty(degrees: u8) -> u32 {
    let degrees = degrees.min(180) as u32;
    let min_duty: u32 = 819; // 5% for 1ms pulse
    let max_duty: u32 = 1638; // 10% for 2ms pulse

    // Linear interpolation between min and max duty
    min_duty + (degrees * (max_duty - min_duty)) / 180
}

/// Set servo position using a pre-configured LEDC channel
pub fn set_angle(channel: &channel::Channel<'_, LowSpeed>, degrees: u8) {
    channel.set_duty_hw(angle_to_duty(degrees));
}

/// Set servo to minimum position (0 degrees)
pub fn set_min(channel: &channel::Channel<'_, LowSpeed>) {
    set_angle(channel, 0);
}

/// Set servo to center position (90 degrees)
pub fn set_center(channel: &channel::Channel<'_, LowSpeed>) {
    set_angle(channel, 90);
}

/// Set servo to maximum position (180 degrees)
pub fn set_max(channel: &channel::Channel<'_, LowSpeed>) {
    set_angle(channel, 180);
}
