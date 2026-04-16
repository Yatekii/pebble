//! Mathematical utilities for sensor data processing.
//!
//! This module provides coordinate transformations, centripetal acceleration
//! correction, and quaternion-to-Euler angle conversion.
//!
//! # Board Frame Convention
//!
//! All sensor readings are transformed into a consistent board frame:
//! - **+X**: toward LED 72
//! - **+Y**: toward LED 18 (90° CCW from LED 72, completing the right-hand rule with Z)
//! - **+Z**: up (out of the PCB surface)
//!
//! When the board is flat, the accelerometer specific-force reads (0, 0, +g).
//!
//! # Sensor Orientations (looking at PCB from above)
//!
//! **BMM350**: chip +x toward pin-A row, chip +y toward LED 18 side (away from LED 72),
//! chip +z into the PCB (downward). LED 72 is at BMM350 −y.
//!
//! **BMI270**: pin 1 is at top-right on PCB, so chip +x points in the BMM350 +y direction
//! (away from LED 72). Chip +z into the PCB (downward).

use nalgebra::{Matrix3, Vector3};
use uom::si::{
    acceleration::meter_per_second_squared,
    f32::{Acceleration, AngularVelocity},
};

/// Rotation matrix: BMM350 sensor frame → board frame.
///
/// Derivation:
/// - Board +X = BMM350 −y  (LED 72 is at BMM350 −y)
/// - Board +Z = BMM350 +z  (empirically verified: BMM350 OTP-compensated output reports
///   +z upward, i.e. the reported sensor frame is effectively left-handed after Bosch's
///   internal axis remapping, so board +Z = +sensor_z despite chip +z appearing to go
///   into the PCB by physical inspection)
/// - Board +Y = Board_Z × Board_X = BMM350 −x  (right-hand rule with above)
#[rustfmt::skip]
pub const BMM350_TO_BOARD: Matrix3<f32> = Matrix3::new(
     0.0, -1.0,  0.0,   // board X = −sensor_y
    -1.0,  0.0,  0.0,   // board Y = −sensor_x
     0.0,  0.0,  1.0,   // board Z = +sensor_z  (empirically confirmed, see comment above)
);

/// Rotation matrix: BMI270 sensor frame → board frame.
///
/// Derivation:
/// - BMI270 +x = board +X  (chip +x points toward LED 72, verified empirically)
/// - Board +Z = BMI270 −z  (chip z into PCB, board z up)
/// - Board +Y = Board_Z × Board_X = BMI270 −y  (right-hand rule)
///
/// This is a 180° rotation around the X-axis.
#[rustfmt::skip]
pub const BMI270_TO_BOARD: Matrix3<f32> = Matrix3::new(
     1.0,  0.0,  0.0,   // board X = +sensor_x
     0.0, -1.0,  0.0,   // board Y = −sensor_y
     0.0,  0.0, -1.0,   // board Z = −sensor_z
);

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
