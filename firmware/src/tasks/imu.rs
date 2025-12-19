//! IMU task: reads sensor data, applies AHRS filter, and broadcasts results.

use defmt::{debug, info};
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c as I2cTrait;
use nalgebra::Vector3;
use uom::si::f32::{Length, MagneticFluxDensity};
use uom::si::magnetic_flux_density::tesla;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::hal::ahrs::AhrsFilter;
use crate::hal::imu::{SharedBmi270, SharedBmm350};
use crate::math::correct_centripetal;
use crate::state::{COMPASS_HEADING, SENSOR_DATA, SensorData};

/// IMU offset from center of rotation in meters (45mm).
const IMU_OFFSET: Vector3<f32> = Vector3::new(0.045, 0.0, 0.0);

/// Hard iron calibration state for magnetometer.
struct MagCalibration {
    x_min: i32,
    x_max: i32,
    y_min: i32,
    y_max: i32,
    z_min: i32,
    z_max: i32,
}

impl MagCalibration {
    fn new() -> Self {
        Self {
            x_min: i32::MAX,
            x_max: i32::MIN,
            y_min: i32::MAX,
            y_max: i32::MIN,
            z_min: i32::MAX,
            z_max: i32::MIN,
        }
    }

    fn update(&mut self, x: i32, y: i32, z: i32) {
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

    fn apply(&self, x: i32, y: i32, z: i32) -> (f32, f32, f32) {
        let x_offset = (self.x_min + self.x_max) / 2;
        let y_offset = (self.y_min + self.y_max) / 2;
        let z_offset = (self.z_min + self.z_max) / 2;

        (
            (x - x_offset) as f32,
            (y - y_offset) as f32,
            (z - z_offset) as f32,
        )
    }

    fn offsets(&self) -> (i32, i32) {
        ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)
    }
}

/// Run the IMU task.
///
/// Reads IMU and magnetometer data at 10Hz, applies AHRS filter,
/// and broadcasts sensor data and compass heading.
pub async fn run<I2C, E>(imu: &SharedBmi270<'_, I2C>, magnetometer: Option<&SharedBmm350<'_, I2C>>)
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    let mut ahrs = AhrsFilter::new();
    let mut sample_count: u32 = 0;
    let mut mag_cal = MagCalibration::new();

    let Some(magnetometer) = magnetometer else {
        return;
    };

    loop {
        // We wait at the start so we can always continue and still yield a bit and not hog all the CPU.
        Timer::after(Duration::from_millis(100)).await;

        let imu_result = imu.read();

        let mag_result = magnetometer.read();

        let mut data = SensorData::default();

        // Process magnetometer first to get calibrated values for AHRS
        let Ok(mag_data) = &mag_result else {
            // Without a result there is no reason to continue.
            continue;
        };

        data.mag = MagBleData {
            x: mag_data.x,
            y: mag_data.y,
            z: mag_data.z,
        };

        // Update and apply hard iron calibration
        mag_cal.update(mag_data.x, mag_data.y, mag_data.z);
        let (x, y, z) = mag_cal.apply(mag_data.x, mag_data.y, mag_data.z);
        let mag_calibrated = Vector3::new(x, y, z).map(MagneticFluxDensity::new::<tesla>);

        // Calculate compass heading from magnetometer X and Y
        let heading_rad = libm::atan2f(y, x);
        let mut heading_deg = heading_rad * 180.0 / core::f32::consts::PI;
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }

        let (x_off, y_off) = mag_cal.offsets();
        info!(
            "MAG: x={} y={} heading={} (cal: x_off={} y_off={})",
            mag_data.x, mag_data.y, heading_deg as i32, x_off, y_off
        );
        COMPASS_HEADING.sender().send(heading_deg as u16);

        if let Ok(imu_data) = &imu_result {
            // Convert to physical units and rotate for PCB orientation
            let acceleration_corrected = imu_data.acceleration();
            let angular_valocity_corrected = imu_data.angular_velocity();

            // Correct for centripetal acceleration
            let acceleration = correct_centripetal(
                acceleration_corrected,
                angular_valocity_corrected,
                IMU_OFFSET,
            );

            // Update AHRS filter
            let orientation =
                ahrs.update_marg(acceleration, angular_valocity_corrected, mag_calibrated);

            // Log orientation periodically (every 50 samples = 5 seconds)
            sample_count += 1;
            if sample_count % 50 == 0 {
                debug!(
                    "AHRS: roll={} pitch={} yaw={}",
                    orientation.roll as i32, orientation.pitch as i32, orientation.yaw as i32
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
        }

        SENSOR_DATA.sender().send(data);
    }
}
