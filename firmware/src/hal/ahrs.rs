//! AHRS (Attitude and Heading Reference System) using Madgwick filter
//!
//! Provides sensor fusion for IMU data to get stable orientation estimates.

use ahrs::{Ahrs as AhrsTrait, Madgwick};
use defmt::error;
use nalgebra::Vector3;
use uom::si::f32::{Acceleration, AngularVelocity, MagneticFluxDensity};

use crate::math::quaternion_to_euler;

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

    /// Update the filter with 9-DOF MARG data (accelerometer + gyroscope + magnetometer)
    ///
    /// # Arguments
    /// * `accel` - Accelerometer readings in g (x, y, z)
    /// * `gyro` - Gyroscope readings in rad/s (x, y, z)
    /// * `mag` - Magnetometer readings in any consistent unit (x, y, z)
    ///
    /// # Returns
    /// The current orientation estimate
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
}
