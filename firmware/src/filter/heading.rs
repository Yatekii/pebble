//! Compass heading calculation from orientation and magnetometer data.
//!
//! This module provides functions to compute a tilt-compensated compass heading
//! from the AHRS orientation output.

use super::ahrs::Orientation;

/// Compute compass heading from AHRS orientation.
///
/// The AHRS yaw angle is used directly as the tilt-compensated heading.
/// This is more accurate than raw magnetometer heading because the AHRS
/// filter accounts for device tilt.
///
/// # Arguments
///
/// * `orientation` - Current orientation from AHRS filter
///
/// # Returns
///
/// Heading in degrees (0-359), where 0 is North.
pub fn compute_heading(orientation: &Orientation) -> u16 {
    let mut heading = orientation.yaw;
    if heading < 0.0 {
        heading += 360.0;
    }
    heading as u16
}

/// Hard iron calibration state for magnetometer.
///
/// Tracks min/max values on each axis to compute hard iron offsets.
/// Hard iron distortion is caused by permanent magnets or magnetized
/// materials near the sensor.
pub struct MagCalibration {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
}

impl MagCalibration {
    /// Create a new calibration state with no data.
    pub fn new() -> Self {
        Self {
            x_min: f32::MAX,
            x_max: f32::MIN,
            y_min: f32::MAX,
            y_max: f32::MIN,
            z_min: f32::MAX,
            z_max: f32::MIN,
        }
    }

    /// Update calibration with a new magnetometer reading.
    ///
    /// Call this with every magnetometer reading to build up
    /// calibration data. For best results, rotate the device
    /// through all orientations.
    pub fn update(&mut self, x: f32, y: f32, z: f32) {
        if x < self.x_min {
            self.x_min = x;
        }
        if x > self.x_max {
            self.x_max = x;
        }
        if y < self.y_min {
            self.y_min = y;
        }
        if y > self.y_max {
            self.y_max = y;
        }
        if z < self.z_min {
            self.z_min = z;
        }
        if z > self.z_max {
            self.z_max = z;
        }
    }

    /// Apply hard iron calibration to a magnetometer reading.
    ///
    /// Subtracts the computed offsets from each axis to center
    /// the readings around zero.
    ///
    /// # Returns
    ///
    /// Calibrated (x, y, z) values.
    pub fn apply(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        let x_offset = (self.x_min + self.x_max) / 2.0;
        let y_offset = (self.y_min + self.y_max) / 2.0;
        let z_offset = (self.z_min + self.z_max) / 2.0;

        (x - x_offset, y - y_offset, z - z_offset)
    }

    /// Get the current X and Y axis offsets for logging.
    pub fn offsets(&self) -> (f32, f32) {
        (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
        )
    }
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self::new()
    }
}
