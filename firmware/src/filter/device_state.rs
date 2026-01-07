//! Unified device state combining all sensor outputs.
//!
//! This module defines the [`DeviceState`] struct that represents the
//! current state of the device as computed from all sensor inputs.

use super::ahrs::Orientation;

/// GPS position data.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsPosition {
    /// Latitude in degrees.
    pub latitude: f64,
    /// Longitude in degrees.
    pub longitude: f64,
    /// Altitude in meters.
    pub altitude: f32,
    /// Number of satellites in view.
    pub satellites: u8,
    /// True if position fix is valid.
    pub has_fix: bool,
}

/// Unified device state from all sensors.
///
/// This struct is the primary output of the filter layer, combining
/// orientation from AHRS, compass heading, and GPS position into a
/// single coherent state representation.
#[derive(Debug, Clone, Copy, Default)]
pub struct DeviceState {
    /// Current device orientation (roll, pitch, yaw).
    pub orientation: Orientation,
    /// Tilt-compensated compass heading in degrees (0-359).
    pub heading: u16,
    /// GPS position, if available.
    pub position: Option<GpsPosition>,
    /// Timestamp in milliseconds since boot.
    pub timestamp_ms: u64,
}

impl DeviceState {
    /// Create a new device state with orientation and heading.
    pub fn new(orientation: Orientation, heading: u16) -> Self {
        Self {
            orientation,
            heading,
            position: None,
            timestamp_ms: 0,
        }
    }

    /// Update the GPS position.
    pub fn with_position(mut self, position: GpsPosition) -> Self {
        self.position = Some(position);
        self
    }

    /// Update the timestamp.
    pub fn with_timestamp(mut self, timestamp_ms: u64) -> Self {
        self.timestamp_ms = timestamp_ms;
        self
    }
}
