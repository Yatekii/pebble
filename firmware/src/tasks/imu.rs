//! IMU task: reads sensor data, applies AHRS filter, and broadcasts results.

use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use nalgebra::Vector3;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::microtesla;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::filter::ahrs::{AhrsFilter, GyroCalibration};
use crate::hal::imu::bmi270::SharedBmi270;
use crate::hal::imu::bmm350::SharedBmm350;
use crate::math::{correct_centripetal, transform_bmm350};
use crate::state::{COMPASS_HEADING, SENSOR_DATA, SensorData};

/// IMU offset from center of rotation in meters (45mm).
const IMU_OFFSET: Vector3<f32> = Vector3::new(0.045, 0.0, 0.0);

/// Sample period in milliseconds (50Hz = 20ms).
const SAMPLE_PERIOD_MS: u64 = 20;

/// Run the IMU task.
///
/// Reads IMU and magnetometer data at 50Hz, applies AHRS filter,
/// and broadcasts sensor data and compass heading.
pub async fn run<I2C, E>(imu: &SharedBmi270<'_, I2C>, magnetometer: Option<&SharedBmm350<'_, I2C>>)
where
    I2C: I2c<Error = E>,
    E: defmt::Format,
{
    let mut ahrs = AhrsFilter::new();
    let mut sample_count: u32 = 0;
    let mut gyro_cal = GyroCalibration::new();

    let Some(magnetometer) = magnetometer else {
        return;
    };

    loop {
        // 50Hz sample rate for responsive compass
        Timer::after(Duration::from_millis(SAMPLE_PERIOD_MS)).await;

        let imu_result = imu.read();
        let mag_result = magnetometer.read();

        let mut data = SensorData::default();

        // Process magnetometer first to get calibrated values for AHRS
        let Ok(mag_data) = &mag_result else {
            continue;
        };

        // Get values in microtesla for processing
        let x_ut = mag_data.x.get::<microtesla>();
        let y_ut = mag_data.y.get::<microtesla>();
        let z_ut = mag_data.z.get::<microtesla>();

        // Store raw values for BLE (in microtesla * 100 as i32 for precision)
        data.mag = MagBleData {
            x: (x_ut * 100.0) as i32,
            y: (y_ut * 100.0) as i32,
            z: (z_ut * 100.0) as i32,
        };

        // Need IMU data for AHRS - skip if not available
        let Ok(imu_data) = &imu_result else {
            continue;
        };

        // Convert to physical units and rotate for PCB orientation
        let acceleration_corrected = imu_data.acceleration();
        let angular_velocity_raw = imu_data.angular_velocity();

        // Update gyro calibration during warmup (device should be stationary)
        if !gyro_cal.is_ready() {
            gyro_cal.update(
                angular_velocity_raw.x.value,
                angular_velocity_raw.y.value,
                angular_velocity_raw.z.value,
            );
        }

        // Apply gyroscope bias correction
        let (gx, gy, gz) = gyro_cal.apply(
            angular_velocity_raw.x.value,
            angular_velocity_raw.y.value,
            angular_velocity_raw.z.value,
        );
        let angular_velocity_corrected = Vector3::new(
            uom::si::f32::AngularVelocity::new::<uom::si::angular_velocity::radian_per_second>(gx),
            uom::si::f32::AngularVelocity::new::<uom::si::angular_velocity::radian_per_second>(gy),
            uom::si::f32::AngularVelocity::new::<uom::si::angular_velocity::radian_per_second>(gz),
        );

        // Correct for centripetal acceleration
        let acceleration = correct_centripetal(
            acceleration_corrected,
            angular_velocity_corrected,
            IMU_OFFSET,
        );

        // Only run AHRS once gyro calibration is ready
        let orientation = if gyro_cal.is_ready() {
            // Transform magnetometer to world frame (Z points down on PCB, so flip it)
            // Skip hard iron calibration - it requires rotation through all orientations
            // which we don't have during static warmup
            let mag_transformed = transform_bmm350(Vector3::new(x_ut, y_ut, z_ut));

            // Create magnetic field vector for AHRS
            let mag_calibrated = mag_transformed.map(|v| MagneticFluxDensity::new::<microtesla>(v));

            // Update AHRS filter - this fuses accel, gyro, and mag for stable orientation
            ahrs.update_marg(acceleration, angular_velocity_corrected, mag_calibrated)
        } else {
            // During calibration warmup, return default orientation
            crate::filter::ahrs::Orientation::default()
        };

        let heading = orientation.heading();
        COMPASS_HEADING.sender().send(heading);

        // Log periodically (every 50 samples = 1 second at 50Hz)
        sample_count += 1;
        if sample_count % 50 == 0 {
            info!(
                "AHRS: heading={} roll={} pitch={}",
                heading, orientation.roll as i32, orientation.pitch as i32
            );
        }

        let accelerometer = imu_data.raw_acceleration();
        let angular_velocity = imu_data.raw_angular_velocity();
        data.acc = AccBleData {
            x: accelerometer.x,
            y: accelerometer.y,
            z: accelerometer.z,
        };
        data.gyro = GyroBleData {
            x: angular_velocity.x,
            y: angular_velocity.y,
            z: angular_velocity.z,
        };
        data.orientation = AhrsBleData {
            roll: orientation.roll,
            pitch: orientation.pitch,
            yaw: orientation.yaw,
        };
        data.valid = true;

        SENSOR_DATA.sender().send(data);
    }
}
