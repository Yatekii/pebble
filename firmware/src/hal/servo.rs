//! Servo motor driver using PWM
//!
//! Controls a 180-degree servo via GPIO1 PWM signal.
//! Standard servo timing: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
//! PWM frequency: 50Hz (20ms period)

use esp_hal::gpio::DriveMode;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{Ledc, LowSpeed, channel, timer};
use esp_hal::time::Rate;

/// Servo pulse width range in microseconds
const PULSE_MIN_US: u32 = 500; // 0 degrees (0.5ms for wider range)
const PULSE_MAX_US: u32 = 2500; // 180 degrees (2.5ms for wider range)

/// PWM period in microseconds (50Hz = 20ms)
const PERIOD_US: u32 = 20_000;

/// PWM resolution (14 bits gives good precision)
const DUTY_RESOLUTION: timer::config::Duty = timer::config::Duty::Duty14Bit;

/// Servo driver - wraps a LEDC channel
pub struct Servo<'a> {
    channel: channel::Channel<'a, LowSpeed>,
    current_angle: u8,
}

impl Servo<'_> {
    /// Set servo angle (0-180 degrees)
    pub fn set_angle(&mut self, angle: u8) -> Result<(), channel::Error> {
        let angle = angle.min(180);
        self.current_angle = angle;

        // Calculate pulse width for this angle
        let pulse_range = PULSE_MAX_US - PULSE_MIN_US;
        let pulse_us = PULSE_MIN_US + (pulse_range * angle as u32) / 180;

        // Convert pulse width to duty cycle percentage (0-100)
        // set_duty takes a percentage, so we need to convert our absolute duty to percentage
        let duty_pct = ((pulse_us * 100) / PERIOD_US) as u8;

        self.channel.set_duty(duty_pct)?;
        Ok(())
    }

    /// Get current angle
    pub fn angle(&self) -> u8 {
        self.current_angle
    }
}

/// Initialize the LEDC timer for servo use.
/// The timer must be stored in a StaticCell and passed to init_with_timer.
pub fn init_timer<'a>(ledc: &'a Ledc<'a>) -> timer::Timer<'a, LowSpeed> {
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: DUTY_RESOLUTION,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .expect("Failed to configure LEDC timer");
    timer
}

/// Initialize the servo channel using a pre-configured timer.
/// The timer must have a static lifetime (e.g., stored in a StaticCell).
pub fn init_channel<'a>(
    ledc: &'a Ledc<'a>,
    pin: esp_hal::peripherals::GPIO1<'a>,
    timer: &'a timer::Timer<'a, LowSpeed>,
) -> Result<Servo<'a>, channel::Error> {
    let mut channel = ledc.channel(channel::Number::Channel0, pin);
    channel.configure(channel::config::Config {
        timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })?;

    // Set initial angle to center (90 degrees)
    let pulse_us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) / 2;
    let duty_pct = ((pulse_us * 100) / PERIOD_US) as u8;
    channel.set_duty(duty_pct)?;

    Ok(Servo {
        channel,
        current_angle: 90,
    })
}
