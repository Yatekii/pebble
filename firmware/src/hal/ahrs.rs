//! AHRS (Attitude and Heading Reference System) using Madgwick filter
//!
//! Provides sensor fusion for IMU data to get stable orientation estimates.

use ahrs::{Ahrs as AhrsTrait, Madgwick};
use defmt::error;
use nalgebra::Vector3;
use uom::si::f32::{Acceleration, AngularVelocity, Length, MagneticFluxDensity};

/// Sample period in seconds (assuming 10Hz IMU update rate)
const SAMPLE_PERIOD: f32 = 0.1;

/// Madgwick filter gain parameter
/// Higher values = faster convergence but more noise
/// Lower values = smoother but slower response
/// Typical range: 0.01 - 0.5
const BETA: f32 = 0.1;

/// Orientation output from AHRS
#[derive(Debug, Clone, Copy, Default)]
pub struct Orientation {
    /// Roll angle in degrees (-180 to 180)
    pub roll: f32,
    /// Pitch angle in degrees (-90 to 90)
    pub pitch: f32,
    /// Yaw angle in degrees (0 to 360)
    pub yaw: f32,
}

/// AHRS filter wrapper using Madgwick algorithm
pub struct AhrsFilter {
    filter: Madgwick<f32>,
}

impl AhrsFilter {
    /// Create a new AHRS filter with default parameters
    pub fn new() -> Self {
        Self {
            filter: Madgwick::new(SAMPLE_PERIOD, BETA),
        }
    }

    /// Create a new AHRS filter with custom sample period and beta gain
    pub fn with_params(sample_period: f32, beta: f32) -> Self {
        Self {
            filter: Madgwick::new(sample_period, beta),
        }
    }

    /// Update the filter with 6-DOF IMU data (accelerometer + gyroscope)
    ///
    /// # Arguments
    /// * `accel` - Accelerometer readings in g (x, y, z)
    /// * `gyro` - Gyroscope readings in rad/s (x, y, z)
    ///
    /// # Returns
    /// The current orientation estimate
    pub fn update_imu(&mut self, accel: (f32, f32, f32), gyro: (f32, f32, f32)) -> Orientation {
        let accel_vec = Vector3::new(accel.0, accel.1, accel.2);
        let gyro_vec = Vector3::new(gyro.0, gyro.1, gyro.2);

        // Update filter (ignore error, filter still updates internal state)
        let _ = self.filter.update_imu(&gyro_vec, &accel_vec);

        self.orientation()
    }

    /// Update the filter with 9-DOF MARG data (accelerometer + gyroscope + magnetometer)
    ///
    /// # Arguments
    /// * `accel` - Accelerometer readings in g (x, y, z)
    /// * `gyro` - Gyroscope readings in rad/s (x, y, z)
    /// * `mag` - Magnetometer readings in any consistent unit (x, y, z)
    ///
    /// # Returns
    /// The current orientation estimate
    #[allow(dead_code)]
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

    /// Get the current orientation as Euler angles
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

    /// Get the raw quaternion (w, x, y, z)
    #[allow(dead_code)]
    pub fn quaternion(&self) -> (f32, f32, f32, f32) {
        let q = self.filter.quat;
        (q.w, q.i, q.j, q.k)
    }
}

impl Default for AhrsFilter {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
fn quaternion_to_euler(x: f32, y: f32, z: f32, w: f32) -> (f32, f32, f32) {
    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if libm::fabsf(sinp) >= 1.0 {
        libm::copysignf(core::f32::consts::FRAC_PI_2, sinp) // Use 90 degrees if out of range
    } else {
        libm::asinf(sinp)
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);

    (roll, pitch, yaw)
}
