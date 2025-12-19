use nalgebra::Vector3;
use uom::si::{
    acceleration::meter_per_second_squared,
    f32::{Acceleration, AngularVelocity},
};

/// Rotate coordinates 90° CCW to compensate for BMI270 PCB orientation.
pub fn rotate_90ccw(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(v.y, -v.x, v.z)
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
