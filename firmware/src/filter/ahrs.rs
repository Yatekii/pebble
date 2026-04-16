//! AHRS (Attitude and Heading Reference System) using Madgwick filter.
//!
//! Provides sensor fusion for IMU data to get stable orientation estimates.
//! The Madgwick filter combines accelerometer, gyroscope, and magnetometer
//! readings to produce a quaternion orientation that is then converted to
//! Euler angles.
//!
//! # Example
//!
//! ```ignore
//! let mut ahrs = AhrsFilter::new();
//!
//! // In sensor loop:
//! let orientation = ahrs.update_marg(accel, gyro, mag);
//! println!("Roll: {}, Pitch: {}, Yaw: {}", orientation.roll, orientation.pitch, orientation.yaw);
//! ```

use ahrs::{Ahrs as AhrsTrait, Madgwick};
use defmt::error;
use nalgebra::Vector3;
use uom::si::f32::{Acceleration, AngularVelocity, MagneticFluxDensity};

use crate::math::quaternion_to_euler;

/// Sample period in seconds (100Hz IMU update rate).
const SAMPLE_PERIOD: f32 = 0.01;

/// Madgwick filter gain parameter.
///
/// Higher values = faster convergence but more noise.
/// Lower values = smoother but slower response.
/// Typical range: 0.01 - 0.5.
/// Using 0.8 to strongly trust magnetometer for heading correction.
const BETA: f32 = 0.8;

/// Orientation output from AHRS.
///
/// Contains Euler angles representing the device orientation in 3D space.
#[derive(Debug, Clone, Copy, Default)]
pub struct Orientation {
    /// Roll angle in degrees (-180 to 180).
    /// Rotation around the forward axis.
    pub roll: f32,
    /// Pitch angle in degrees (-90 to 90).
    /// Rotation around the lateral axis.
    pub pitch: f32,
    /// Yaw angle in degrees (-180 to 180).
    /// Rotation around the vertical axis.
    pub yaw: f32,
}

impl Orientation {
    /// Get the compass heading in degrees (0-359).
    ///
    /// This normalizes the yaw angle to the 0-360 range where 0 is North.
    pub fn heading(&self) -> u16 {
        let mut heading = self.yaw;
        if heading < 0.0 {
            heading += 360.0;
        }
        heading as u16
    }
}

/// AHRS filter wrapper using Madgwick algorithm.
///
/// The Madgwick filter is a computationally efficient orientation filter
/// that fuses 9-DOF MARG (Magnetic, Angular Rate, Gravity) sensor data.
pub struct AhrsFilter {
    filter: Madgwick<f32>,
}

impl AhrsFilter {
    /// Create a new AHRS filter with default parameters.
    pub fn new() -> Self {
        Self {
            filter: Madgwick::new(SAMPLE_PERIOD, BETA),
        }
    }

    /// Update the filter with 9-DOF MARG data (accelerometer + gyroscope + magnetometer).
    ///
    /// # Arguments
    ///
    /// * `acceleration` - Accelerometer readings (x, y, z)
    /// * `angular_velocity` - Gyroscope readings (x, y, z)
    /// * `magnetometer` - Magnetometer readings (x, y, z)
    ///
    /// # Returns
    ///
    /// The current orientation estimate as Euler angles.
    pub fn update_marg(
        &mut self,
        acceleration: Vector3<Acceleration>,
        angular_velocity: Vector3<AngularVelocity>,
        magnetometer: Vector3<MagneticFluxDensity>,
    ) -> Orientation {
        let acceleration = acceleration.map(|a| a.value);
        let angular_velocity = angular_velocity.map(|a| a.value);
        let magnetometer = magnetometer.map(|m| m.value);

        // Update filter (ignore error, filter still updates internal state)
        if let Err(error) = self
            .filter
            .update(&angular_velocity, &acceleration, &magnetometer)
        {
            error!("Updating AHRS had an issue: {:?}", error);
        }

        self.orientation()
    }

    /// Get the current orientation as Euler angles.
    ///
    /// Returns the most recent orientation estimate without updating the filter.
    pub fn orientation(&self) -> Orientation {
        let quat = self.filter.quat;

        // Convert quaternion to Euler angles (roll, pitch, yaw)
        let (roll, pitch, yaw) = quaternion_to_euler(quat.i, quat.j, quat.k, quat.w);

        Orientation {
            roll: roll.to_degrees(),
            pitch: pitch.to_degrees(),
            yaw: yaw.to_degrees(),
        }
    }
}

impl Default for AhrsFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// Gyroscope bias calibration state.
///
/// Computes the average gyroscope reading over a warmup period while
/// the device is stationary. This bias is then subtracted from all
/// subsequent readings to eliminate drift.
pub struct GyroCalibration {
    x_sum: f32,
    y_sum: f32,
    z_sum: f32,
    sample_count: u32,
    x_bias: f32,
    y_bias: f32,
    z_bias: f32,
    calibrated: bool,
}

/// Number of samples to collect for gyroscope bias calibration.
/// At 100Hz, 400 samples = 4 seconds of warmup.
/// More samples = more accurate bias estimate, but longer warmup time.
const GYRO_CALIBRATION_SAMPLES: u32 = 400;

impl GyroCalibration {
    /// Create a new gyroscope calibration state.
    pub fn new() -> Self {
        Self {
            x_sum: 0.0,
            y_sum: 0.0,
            z_sum: 0.0,
            sample_count: 0,
            x_bias: 0.0,
            y_bias: 0.0,
            z_bias: 0.0,
            calibrated: false,
        }
    }

    /// Returns true if calibration is complete.
    pub fn is_ready(&self) -> bool {
        self.calibrated
    }

    /// Update calibration with a new gyroscope reading.
    ///
    /// During the warmup period, readings are accumulated to compute
    /// the average bias. The device should be stationary during this time.
    pub fn update(&mut self, x: f32, y: f32, z: f32) {
        if self.calibrated {
            return;
        }

        self.x_sum += x;
        self.y_sum += y;
        self.z_sum += z;
        self.sample_count += 1;

        if self.sample_count >= GYRO_CALIBRATION_SAMPLES {
            let n = self.sample_count as f32;
            self.x_bias = self.x_sum / n;
            self.y_bias = self.y_sum / n;
            self.z_bias = self.z_sum / n;
            self.calibrated = true;
        }
    }

    /// Apply bias correction to a gyroscope reading.
    ///
    /// Subtracts the computed bias from each axis.
    ///
    /// # Returns
    ///
    /// Bias-corrected (x, y, z) values.
    pub fn apply(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        (x - self.x_bias, y - self.y_bias, z - self.z_bias)
    }

    /// Get the current bias values for logging (in mrad/s as i32).
    pub fn bias(&self) -> (i32, i32, i32) {
        (
            (self.x_bias * 1000.0) as i32,
            (self.y_bias * 1000.0) as i32,
            (self.z_bias * 1000.0) as i32,
        )
    }
}

impl Default for GyroCalibration {
    fn default() -> Self {
        Self::new()
    }
}

/// Hard iron calibration state for magnetometer.
///
/// Tracks min/max values on each axis to compute hard iron offsets.
/// Hard iron distortion is caused by permanent magnets or magnetized
/// materials near the sensor.
///
/// Note: Currently unused because proper hard iron calibration requires
/// rotating the device through all orientations, which we don't have
/// during static warmup. Kept for potential future use.
#[allow(dead_code)]
pub struct MagCalibration {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
    sample_count: u32,
}

/// Minimum samples required before calibration can be applied.
/// This ensures we have enough data spread to avoid near-zero calibrated values.
const MIN_CALIBRATION_SAMPLES: u32 = 50;

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
            sample_count: 0,
        }
    }

    /// Returns true if enough samples have been collected for calibration.
    pub fn is_ready(&self) -> bool {
        self.sample_count >= MIN_CALIBRATION_SAMPLES
    }

    /// Update calibration with a new magnetometer reading.
    ///
    /// Call this with every magnetometer reading to build up
    /// calibration data. For best results, rotate the device
    /// through all orientations.
    pub fn update(&mut self, x: f32, y: f32, z: f32) {
        self.x_min = self.x_min.min(x);
        self.x_max = self.x_max.max(x);
        self.y_min = self.y_min.min(y);
        self.y_max = self.y_max.max(y);
        self.z_min = self.z_min.min(z);
        self.z_max = self.z_max.max(z);
        self.sample_count = self.sample_count.saturating_add(1);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heading_positive() {
        let orientation = Orientation {
            roll: 0.0,
            pitch: 0.0,
            yaw: 45.0,
        };
        assert_eq!(orientation.heading(), 45);
    }

    #[test]
    fn test_heading_negative() {
        let orientation = Orientation {
            roll: 0.0,
            pitch: 0.0,
            yaw: -45.0,
        };
        assert_eq!(orientation.heading(), 315);
    }

    #[test]
    fn test_heading_zero() {
        let orientation = Orientation {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        };
        assert_eq!(orientation.heading(), 0);
    }

    #[test]
    fn test_mag_calibration_update() {
        let mut cal = MagCalibration::new();
        cal.update(10.0, 20.0, 30.0);
        cal.update(-10.0, -20.0, -30.0);

        let (x_off, y_off) = cal.offsets();
        assert_eq!(x_off, 0.0);
        assert_eq!(y_off, 0.0);
    }

    #[test]
    fn test_mag_calibration_apply() {
        let mut cal = MagCalibration::new();
        cal.update(100.0, 200.0, 300.0);
        cal.update(-100.0, -200.0, -300.0);

        let (x, y, z) = cal.apply(50.0, 100.0, 150.0);
        assert_eq!(x, 50.0);
        assert_eq!(y, 100.0);
        assert_eq!(z, 150.0);
    }
}
