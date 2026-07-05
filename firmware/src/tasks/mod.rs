//! Async tasks for the firmware.
//!
//! Each task runs concurrently using Embassy and handles a specific aspect
//! of the puzzlebox functionality.
//!
//! # Task Categories
//!
//! ## Sensor Tasks
//! - [`imu`]: Reads IMU/magnetometer, runs AHRS filter, broadcasts sensor data
//! - [`gps`]: Polls GPS UART, broadcasts position data
//!
//! ## Actuator Tasks
//! - [`led`]: Receives LED commands, updates NeoPixel ring
//! - [`servo`]: Receives servo commands, controls servo motor
//!
//! ## Communication Tasks
//! - [`ble`]: Handles BLE GATT writes for LED control

pub mod ble;
pub mod gps;
pub mod imu;
pub mod led;
pub mod puzzle;
pub mod servo;
