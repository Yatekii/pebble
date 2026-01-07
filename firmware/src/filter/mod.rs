//! Filter layer: sensor fusion and device state computation.
//!
//! This layer sits between the HAL (raw sensor access) and the control layer
//! (puzzle logic). It processes raw sensor data through filters to produce
//! stable, usable device state.
//!
//! # Components
//!
//! - [`ahrs`]: Madgwick AHRS filter for orientation estimation
//! - [`heading`]: Compass heading calculation from orientation and magnetometer
//! - [`device_state`]: Unified device state combining all sensor outputs
//! - [`watches`]: Inter-layer communication channels

pub mod ahrs;
pub mod device_state;
pub mod heading;
pub mod watches;
