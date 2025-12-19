//! IMU task: reads sensor data, applies AHRS filter, and broadcasts results.

use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c as I2cTrait;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::hal::ahrs::AhrsFilter;
use crate::hal::imu::{SharedBmi270, SharedBmm350};
use crate::state::{COMPASS_HEADING, SENSOR_DATA, SensorData};

/// IMU offset from center of rotation in meters (45mm).
const IMU_OFFSET_X: f32 = 0.045;

/// Gravity in m/s² per g.
const G: f32 = 9.81;

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

/// Correct accelerometer for sensor offset from center of rotation.
///
/// The IMU is 45mm to the right (+X) of the PCB center.
/// Formula: a_corrected = a_measured - ω × (ω × r)
fn correct_centripetal(accel: (f32, f32, f32), gyro: (f32, f32, f32)) -> (f32, f32, f32) {
    let (wx, wy, wz) = gyro;
    let a_cent_x = -IMU_OFFSET_X * (wy * wy + wz * wz) / G;
    let a_cent_y = IMU_OFFSET_X * wx * wy / G;
    let a_cent_z = IMU_OFFSET_X * wx * wz / G;

    (accel.0 - a_cent_x, accel.1 - a_cent_y, accel.2 - a_cent_z)
}

/// Rotate coordinates 90° CCW to compensate for BMI270 PCB orientation.
fn rotate_90ccw(v: (f32, f32, f32)) -> (f32, f32, f32) {
    (v.1, -v.0, v.2)
}

/// Run the IMU task.
///
/// Reads IMU and magnetometer data at 10Hz, applies AHRS filter,
/// and broadcasts sensor data and compass heading.
pub async fn run<I2C, E>(imu: &SharedBmi270<'_, I2C>, mag: Option<&SharedBmm350<'_, I2C>>) -> !
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    let mut ahrs = AhrsFilter::new();
    let mut sample_count: u32 = 0;
    let mut mag_cal = MagCalibration::new();

    loop {
        let imu_result = imu.read();
        let mag_result = mag.map(|m| m.read());

        let mut data = SensorData::default();

        // Process magnetometer first to get calibrated values for AHRS
        let mut mag_calibrated: Option<(f32, f32, f32)> = None;
        if let Some(Ok(mag_data)) = &mag_result {
            data.mag = MagBleData {
                x: mag_data.x,
                y: mag_data.y,
                z: mag_data.z,
            };

            // Update and apply hard iron calibration
            mag_cal.update(mag_data.x, mag_data.y, mag_data.z);
            let (x, y, z) = mag_cal.apply(mag_data.x, mag_data.y, mag_data.z);
            mag_calibrated = Some((x, y, z));

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
        }

        if let Ok(imu_data) = &imu_result {
            // Convert to physical units and rotate for PCB orientation
            let accel_raw = imu_data.accel_g();
            let gyro_raw = imu_data.gyro_rads();

            let accel_rotated = rotate_90ccw(accel_raw);
            let gyro = rotate_90ccw(gyro_raw);

            // Correct for centripetal acceleration
            let accel = correct_centripetal(accel_rotated, gyro);

            // Update AHRS filter
            let orientation = if let Some(mag) = mag_calibrated {
                ahrs.update_marg(accel, gyro, mag)
            } else {
                ahrs.update_imu(accel, gyro)
            };

            // Log orientation periodically (every 50 samples = 5 seconds)
            sample_count += 1;
            if sample_count % 50 == 0 {
                info!(
                    "AHRS: roll={} pitch={} yaw={}",
                    orientation.roll as i32, orientation.pitch as i32, orientation.yaw as i32
                );
            }

            data.acc = AccBleData {
                x: imu_data.acc_x,
                y: imu_data.acc_y,
                z: imu_data.acc_z,
            };
            data.gyro = GyroBleData {
                x: imu_data.gyr_x,
                y: imu_data.gyr_y,
                z: imu_data.gyr_z,
            };
            data.orientation = AhrsBleData {
                roll: orientation.roll,
                pitch: orientation.pitch,
                yaw: orientation.yaw,
            };
            data.valid = true;
        }

        SENSOR_DATA.sender().send(data);
        Timer::after(Duration::from_millis(100)).await;
    }
}
