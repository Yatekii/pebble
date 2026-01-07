//! Mathematical utilities for sensor data processing.
//!
//! This module provides coordinate transformations, centripetal acceleration
//! correction, and quaternion-to-Euler angle conversion.
//!
//! # Coordinate Systems
//!
//! The PCB has sensors mounted in different orientations:
//! - BMI270 (IMU): Rotated 90° CCW and Z points down
//! - BMM350 (Magnetometer): Z points down (no XY rotation)
//!
//! These transformations convert sensor readings to a consistent world frame
//! where +X is forward, +Y is left, and +Z is up.

use nalgebra::Vector3;
use uom::si::{
    acceleration::meter_per_second_squared,
    f32::{Acceleration, AngularVelocity},
};

/// Transform BMI270 coordinates to world frame.
/// BMI270 is rotated 90° CCW on PCB and rotated 180° around X-axis (Z points down).
/// 180° rotation around X-axis: (x, y, z) -> (x, -y, -z)
/// Then 90° CCW in XY plane: (x, y, z) -> (y, -x, z)
/// Combined: (x, y, z) -> (-y, -x, -z)
pub fn transform_bmi270(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-v.y, -v.x, -v.z)
}

/// Transform BMM350 coordinates to world frame.
/// BMM350 is rotated 180° around X-axis (Z points down, not rotated in XY plane).
/// 180° rotation around X-axis: (x, y, z) -> (x, -y, -z)
pub fn transform_bmm350(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.x, -v.y, -v.z)
}

/// Correct accelerometer for sensor offset from center of rotation.
///
/// The IMU is 45mm to the right (+X) of the PCB center.
/// Formula: a_corrected = a_measured - ω × (ω × r)
pub fn correct_centripetal(
    acceleration: Vector3<Acceleration>,
    angular_velocity: Vector3<AngularVelocity>,
    offset: Vector3<f32>,
) -> Vector3<Acceleration> {
    (acceleration.map(|a| a.value)
        - angular_velocity
            .map(|a| a.value)
            .cross(&angular_velocity.map(|a| a.value).cross(&offset)))
    .map(Acceleration::new::<meter_per_second_squared>)
}

/// Convert quaternion to Euler angles (roll, pitch, yaw) in radians
pub fn quaternion_to_euler(x: f32, y: f32, z: f32, w: f32) -> (f32, f32, f32) {
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
