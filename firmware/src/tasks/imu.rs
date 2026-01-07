//! IMU task: reads sensor data, applies AHRS filter, and broadcasts results.

use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use nalgebra::Vector3;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::microtesla;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::hal::ahrs::AhrsFilter;
use crate::hal::imu::bmi270::SharedBmi270;
use crate::hal::imu::bmm350::SharedBmm350;
use crate::math::{correct_centripetal, transform_bmm350};
use crate::state::{COMPASS_HEADING, SENSOR_DATA, SensorData};

/// IMU offset from center of rotation in meters (45mm).
const IMU_OFFSET: Vector3<f32> = Vector3::new(0.045, 0.0, 0.0);

/// Sample period in milliseconds (50Hz = 20ms).
const SAMPLE_PERIOD_MS: u64 = 20;

/// Hard iron calibration state for magnetometer.
struct MagCalibration {
    x_min: f32,
    x_max: f32,
    y_min: f32,
    y_max: f32,
    z_min: f32,
    z_max: f32,
}

impl MagCalibration {
    fn new() -> Self {
        Self {
            x_min: f32::MAX,
            x_max: f32::MIN,
            y_min: f32::MAX,
            y_max: f32::MIN,
            z_min: f32::MAX,
            z_max: f32::MIN,
        }
    }

    fn update(&mut self, x: f32, y: f32, z: f32) {
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

    fn apply(&self, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
        let x_offset = (self.x_min + self.x_max) / 2.0;
        let y_offset = (self.y_min + self.y_max) / 2.0;
        let z_offset = (self.z_min + self.z_max) / 2.0;

        (x - x_offset, y - y_offset, z - z_offset)
    }

    fn offsets(&self) -> (f32, f32) {
        (
            (self.x_min + self.x_max) / 2.0,
            (self.y_min + self.y_max) / 2.0,
        )
    }
}

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
    let mut mag_cal = MagCalibration::new();

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

        // Update and apply hard iron calibration
        mag_cal.update(x_ut, y_ut, z_ut);
        let (x_cal, y_cal, z_cal) = mag_cal.apply(x_ut, y_ut, z_ut);

        // Transform to world frame (Z points down on PCB, so flip it)
        let mag_transformed = transform_bmm350(Vector3::new(x_cal, y_cal, z_cal));

        // Create calibrated magnetic field vector for AHRS
        let mag_calibrated = mag_transformed.map(|v| MagneticFluxDensity::new::<microtesla>(v));

        // Need IMU data for AHRS - skip if not available
        let Ok(imu_data) = &imu_result else {
            continue;
        };

        // Convert to physical units and rotate for PCB orientation
        let acceleration_corrected = imu_data.acceleration();
        let angular_velocity_corrected = imu_data.angular_velocity();

        // Correct for centripetal acceleration
        let acceleration = correct_centripetal(
            acceleration_corrected,
            angular_velocity_corrected,
            IMU_OFFSET,
        );

        // Update AHRS filter - this fuses accel, gyro, and mag for stable orientation
        let orientation =
            ahrs.update_marg(acceleration, angular_velocity_corrected, mag_calibrated);

        // Use AHRS yaw as compass heading (tilt-compensated and filtered)
        let mut heading = orientation.yaw;
        if heading < 0.0 {
            heading += 360.0;
        }

        COMPASS_HEADING.sender().send(heading as u16);

        // Log periodically (every 50 samples = 1 second at 50Hz)
        sample_count += 1;
        if sample_count % 50 == 0 {
            let (x_off, y_off) = mag_cal.offsets();
            info!(
                "AHRS: yaw/heading={} roll={} pitch={} (mag_cal: x_off={} y_off={})",
                heading as i32,
                orientation.roll as i32,
                orientation.pitch as i32,
                x_off as i32,
                y_off as i32
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
