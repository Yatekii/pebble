//! Async tasks for the firmware.
//!
//! Each task runs concurrently using Embassy and handles a specific aspect
//! of the puzzlebox functionality.

pub mod ble;
pub mod gps;
pub mod imu;
pub mod led;
pub mod servo;
