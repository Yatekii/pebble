//! Hardware Abstraction Layer (HAL).
//!
//! This module provides low-level access to all hardware peripherals:
//!
//! - [`gps`]: MAX-M8Q GPS module via UART
//! - [`imu`]: BMI270 IMU and BMM350 magnetometer via I2C
//! - [`led`]: WS2812B LED ring via RMT
//! - [`servo`]: PWM servo motor via LEDC
//! - [`storage`]: Flash storage for persistent state
//! - [`peripherals`]: Shared peripheral resources (I2C bus)

pub mod gps;
pub mod imu;
pub mod led;
pub mod peripherals;
pub mod servo;
pub mod storage;
