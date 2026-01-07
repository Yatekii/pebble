//! Pebble Reverse Geocache Firmware
//!
//! This firmware implements a reverse geocache puzzle box with multiple puzzles
//! that use an IMU, GPS, LEDs, and servo motor.
//!
//! # Architecture
//!
//! The firmware is organized into layers:
//!
//! 1. **HAL Layer** ([`hal`]): Hardware abstraction for sensors and actuators
//!    - IMU (BMI270 accelerometer/gyroscope)
//!    - Magnetometer (BMM350)
//!    - GPS (MAX-M8Q)
//!    - LEDs (WS2812B ring)
//!    - Servo motor
//!    - Flash storage
//!
//! 2. **Filter Layer** ([`filter`]): Sensor fusion and state computation
//!    - AHRS (Madgwick filter for orientation)
//!    - Compass heading calculation
//!    - Unified device state
//!
//! 3. **Control Layer** ([`puzzle`]): Puzzle logic and game state machine
//!    - State machine for puzzle progression
//!    - Individual puzzle implementations
//!    - Event/action system for layer communication
//!
//! 4. **Communications Layer** ([`comms`]): External connectivity
//!    - BLE GATT server for sensor data and control
//!
//! # Tasks
//!
//! The firmware uses Embassy async runtime with these main tasks:
//! - IMU task: Sensor fusion and orientation updates
//! - GPS task: Position tracking
//! - LED task: Compass display and animations
//! - Servo task: Lock mechanism control
//! - BLE task: External communication

#![no_std]

pub mod comms;
pub mod filter;
pub mod hal;
pub mod math;
mod nvs_stubs;
pub mod puzzle;
pub mod state;
pub mod tasks;
pub mod util;
