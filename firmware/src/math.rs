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
/// - Board +Z = BMM350 −z  (forced by right-hand consistency: with the verified X/Y rows,
///   Board_X × Board_Y = (0,0,−1), so board +Z = −sensor_z. The earlier "+sensor_z" note
///   was taken flat, where Z never enters the horizontal heading `atan2` and so could not
///   reveal the sign. A +sensor_z row makes this a reflection (det = −1), which is
///   inconsistent with the accelerometer's proper-rotation board frame and corrupts
///   tilt-compensated heading. See `tilt_compensated_heading`.)
/// - Board +Y = Board_Z × Board_X = BMM350 −x  (right-hand rule with above)
#[rustfmt::skip]
pub const BMM350_TO_BOARD: Matrix3<f32> = Matrix3::new(
     0.0, -1.0,  0.0,   // board X = −sensor_y
    -1.0,  0.0,  0.0,   // board Y = −sensor_x
     0.0,  0.0, -1.0,   // board Z = −sensor_z  (right-hand rule → det = +1)
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

/// Tilt-compensated magnetic heading from board-frame accelerometer and magnetometer.
///
/// Both inputs must be in the board frame (+X toward LED 72, +Y toward LED 18, +Z up).
/// Returns `(heading_rad, roll_deg, pitch_deg)`; heading is CW from board +X in the range
/// returned by `atan2` (caller wraps to 0..360). Roll/pitch are exposed for telemetry.
///
/// The board's tilt is estimated from gravity, then the magnetometer is de-rotated back to
/// the level world frame before taking the horizontal `atan2` — so the heading is invariant
/// to how the box is held. De-rotation is `R = Ry(pitch)·Rx(roll)`; a mismatched matrix or a
/// board frame that disagrees with gravity's handedness (see `BMM350_TO_BOARD`) reintroduces
/// tilt error even though flat still "works" (flat makes R the identity).
pub fn tilt_compensated_heading(acc: Vector3<f32>, mag_board: Vector3<f32>) -> (f32, f32, f32) {
    // Roll (around board X = LED-72 axis) and pitch (around board Y = LED-18 axis).
    // Board Z is UP so specific force = +g on Z when flat → atan2(0, +g) = 0.
    let roll = libm::atan2f(acc.y, acc.z);
    let pitch = libm::atan2f(-acc.x, libm::sqrtf(acc.y * acc.y + acc.z * acc.z));

    // Near ±90° pitch, acc.z → 0 and roll becomes numerically indeterminate (gimbal lock).
    // Clamp pitch to ±80° and zero roll there so the ill-conditioned roll estimate does not
    // corrupt the heading.
    const PITCH_LIMIT: f32 = 80.0 * core::f32::consts::PI / 180.0;
    let (pitch, roll) = if pitch.abs() < PITCH_LIMIT {
        (pitch, roll)
    } else {
        (pitch.clamp(-PITCH_LIMIT, PITCH_LIMIT), 0.0_f32)
    };

    let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
    let (sp, cp) = (libm::sinf(pitch), libm::cosf(pitch));

    // R = Ry(pitch)·Rx(roll): de-rotates board-frame field back to the level world frame.
    #[rustfmt::skip]
    let r_tilt = Matrix3::new(
        cp,  sp * sr,  sp * cr,
       0.0,       cr,      -sr,
       -sp,  cp * sr,  cp * cr,
    );
    let mag_h = r_tilt * mag_board;

    (
        libm::atan2f(mag_h.y, mag_h.x),
        roll.to_degrees(),
        pitch.to_degrees(),
    )
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

#[cfg(test)]
mod tests {
    use super::*;

    fn rx(a: f32) -> Matrix3<f32> {
        let (s, c) = (libm::sinf(a), libm::cosf(a));
        #[rustfmt::skip]
        let m = Matrix3::new(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
        m
    }
    fn ry(a: f32) -> Matrix3<f32> {
        let (s, c) = (libm::sinf(a), libm::cosf(a));
        #[rustfmt::skip]
        let m = Matrix3::new(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
        m
    }
    fn rz(a: f32) -> Matrix3<f32> {
        let (s, c) = (libm::sinf(a), libm::cosf(a));
        #[rustfmt::skip]
        let m = Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
        m
    }

    fn deg(r: f32) -> f32 {
        r * 180.0 / core::f32::consts::PI
    }
    fn rad(d: f32) -> f32 {
        d * core::f32::consts::PI / 180.0
    }

    /// Synthetic board-frame (acc, mag) for a box held at (roll, pitch, yaw), in an
    /// Earth field inclined 60° below horizontal. Board→world = Rz·Ry·Rx.
    fn synth(roll: f32, pitch: f32, yaw: f32) -> (Vector3<f32>, Vector3<f32>) {
        let incl = rad(60.0);
        let b_world = Vector3::new(libm::cosf(incl), 0.0, -libm::sinf(incl));
        let r = rz(yaw) * ry(pitch) * rx(roll);
        let rt = r.transpose();
        // Accelerometer measures specific force = +g up (world +Z), in board frame.
        let acc = rt * Vector3::new(0.0, 0.0, 9.81);
        let mag = rt * b_world;
        (acc, mag)
    }

    fn ang_diff(a: f32, b: f32) -> f32 {
        let mut d = (a - b) % 360.0;
        if d > 180.0 {
            d -= 360.0;
        }
        if d < -180.0 {
            d += 360.0;
        }
        libm::fabsf(d)
    }

    #[test]
    fn transform_is_a_rotation() {
        // A reflection (det = −1) puts the magnetometer in a frame that disagrees with
        // gravity's handedness and corrupts tilt compensation. This is bug #2.
        assert!(BMM350_TO_BOARD.determinant() > 0.0);
        assert!(BMI270_TO_BOARD.determinant() > 0.0);
    }

    #[test]
    fn heading_is_tilt_invariant() {
        for &yaw in &[0.0f32, 30.0, 90.0, 150.0, 210.0, 300.0] {
            let (a0, m0) = synth(0.0, 0.0, rad(yaw));
            let (h0, _, _) = tilt_compensated_heading(a0, m0);
            let flat = deg(h0);
            for &roll in &[-40.0f32, -20.0, 20.0, 40.0] {
                for &pitch in &[-40.0f32, -20.0, 20.0, 40.0] {
                    let (a, m) = synth(rad(roll), rad(pitch), rad(yaw));
                    let (h, _, _) = tilt_compensated_heading(a, m);
                    let err = ang_diff(deg(h), flat);
                    assert!(err < 1.0, "yaw={yaw} roll={roll} pitch={pitch} err={err}");
                }
            }
        }
    }
}
