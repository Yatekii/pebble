//! IMU task: reads sensor data, computes tilt-compensated heading, and broadcasts results.

use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use nalgebra::Vector3;
use uom::si::magnetic_flux_density::microtesla;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::hal::imu::bmi270::SharedBmi270;
use crate::hal::imu::bmm350::{RAW_LSB_TO_UT_XY, RAW_LSB_TO_UT_Z, SharedBmm350};
use crate::math::transform_bmm350;
use crate::state::{COMPASS_HEADING, SENSOR_DATA, SensorData};

/// Sample period in milliseconds (100Hz = 10ms).
const SAMPLE_PERIOD_MS: u64 = 10;

/// Expected Earth-field magnitude after calibration (µT).
const MAG_FIELD_CENTER: f32 = 48.5;
/// Half-width of the acceptance window around `MAG_FIELD_CENTER` (µT).
const MAG_FIELD_TOLERANCE: f32 = 5.0;

/// When built with `--features mag-calibrate` (done automatically by the
/// `mag-calibrate` tool), streams raw magnetometer values over RTT so the
/// host tool can fit an ellipsoid and print calibration constants.
const MAG_CALIBRATING: bool = cfg!(feature = "mag-calibrate");

/// Hard iron offset (µT). Updated by calibrate_mag.py.
const MAG_HI_OFFSET: [f32; 3] = [-8.7998, 50.6150, -17.1461];

/// Soft iron correction matrix.
/// Identity = no correction (before first calibration run).
const MAG_SOFT_IRON: [[f32; 3]; 3] = [
    [0.98750320, 0.00032133, -0.00179177],
    [0.00032133, 0.99343141, -0.00748462],
    [-0.00179177, -0.00748462, 1.01976655],
];

/// Run the IMU task.
///
/// Reads IMU and magnetometer data at 100Hz, computes tilt-compensated
/// compass heading, and broadcasts sensor data.
pub async fn run<I2C, E>(imu: &SharedBmi270<'_, I2C>, magnetometer: Option<&SharedBmm350<'_, I2C>>)
where
    I2C: I2c<Error = E>,
    E: defmt::Format,
{
    let Some(magnetometer) = magnetometer else {
        return;
    };

    let calib_tick: u32 = 0;

    loop {
        Timer::after(Duration::from_millis(SAMPLE_PERIOD_MS)).await;

        // In calibration mode: stream raw (no OTP compensation) values and skip
        // heading computation.  Using read_raw() + fixed datasheet scale factors
        // avoids the temperature-dependent OTP correction, which shrinks the
        // apparent field magnitude as the chip heats up — corrupting the calibration.
        if MAG_CALIBRATING {
            let Ok(raw) = magnetometer.read_raw() else {
                continue;
            };
            let Ok(comp) = magnetometer.read() else {
                continue;
            };

            // Raw: fixed datasheet LSB→µT, no per-chip OTP correction
            let x_ut = raw.x as f32 * RAW_LSB_TO_UT_XY;
            let y_ut = raw.y as f32 * RAW_LSB_TO_UT_XY;
            let z_ut = raw.z as f32 * RAW_LSB_TO_UT_Z;
            let rx = (x_ut * 100.0) as i32;
            let ry = (y_ut * 100.0) as i32;
            let rz = (z_ut * 100.0) as i32;
            // Comp: full OTP + temperature compensation + cross-axis
            let cx = (comp.x.get::<microtesla>() * 100.0) as i32;
            let cy = (comp.y.get::<microtesla>() * 100.0) as i32;
            let cz = (comp.z.get::<microtesla>() * 100.0) as i32;
            // Firmware-calibrated: apply persisted MAG_HI_OFFSET + MAG_SOFT_IRON.
            // Use OTP-compensated values (comp) here — constants were calibrated
            // from comp samples in the tool, and the normal heading path also uses
            // comp values.  Using raw here would cause a ~10-200 µT offset in row 3.
            let fw_cx = comp.x.get::<microtesla>();
            let fw_cy = comp.y.get::<microtesla>();
            let fw_cz = comp.z.get::<microtesla>();
            let bx = fw_cx - MAG_HI_OFFSET[0];
            let by = fw_cy - MAG_HI_OFFSET[1];
            let bz = fw_cz - MAG_HI_OFFSET[2];
            let fw_x =
                MAG_SOFT_IRON[0][0] * bx + MAG_SOFT_IRON[0][1] * by + MAG_SOFT_IRON[0][2] * bz;
            let fw_y =
                MAG_SOFT_IRON[1][0] * bx + MAG_SOFT_IRON[1][1] * by + MAG_SOFT_IRON[1][2] * bz;
            let fw_z =
                MAG_SOFT_IRON[2][0] * bx + MAG_SOFT_IRON[2][1] * by + MAG_SOFT_IRON[2][2] * bz;

            info!(
                "MAG_CAL {=i32} {=i32} {=i32} {=i32} {=i32} {=i32} {=i32} {=i32} {=i32}",
                rx,
                ry,
                rz,
                cx,
                cy,
                cz,
                (fw_x * 100.0) as i32,
                (fw_y * 100.0) as i32,
                (fw_z * 100.0) as i32,
            );
            let _ = calib_tick;
            continue;
        }

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

        // Apply hard iron offset then soft iron correction matrix
        let bx = x_ut - MAG_HI_OFFSET[0];
        let by = y_ut - MAG_HI_OFFSET[1];
        let bz = z_ut - MAG_HI_OFFSET[2];
        let x_cal = MAG_SOFT_IRON[0][0] * bx + MAG_SOFT_IRON[0][1] * by + MAG_SOFT_IRON[0][2] * bz;
        let y_cal = MAG_SOFT_IRON[1][0] * bx + MAG_SOFT_IRON[1][1] * by + MAG_SOFT_IRON[1][2] * bz;
        let z_cal = MAG_SOFT_IRON[2][0] * bx + MAG_SOFT_IRON[2][1] * by + MAG_SOFT_IRON[2][2] * bz;

        // Reject readings whose calibrated magnitude is outside the expected Earth-field range.
        let cal_mag = libm::sqrtf(x_cal * x_cal + y_cal * y_cal + z_cal * z_cal);
        if libm::fabsf(cal_mag - MAG_FIELD_CENTER) > MAG_FIELD_TOLERANCE {
            continue;
        }

        // Transform magnetometer to board frame.
        // After transform: x=1 points toward LED 72 (board +X), y=1 points toward LED 18.
        let mag_transformed = transform_bmm350(Vector3::new(x_cal, y_cal, z_cal));

        // Flat-plane compass heading from horizontal components only (no IMU/tilt compensation).
        // atan2(-y, x): heading is CW from north, 0° when LED 72 faces north.
        let heading_rad = libm::atan2f(-mag_transformed.y, mag_transformed.x);
        let mut heading_deg = heading_rad.to_degrees();
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }
        let heading = heading_deg as u16;
        COMPASS_HEADING.sender().send(heading);

        info!(
            "heading={} mag=({},{},{})",
            heading, x_cal as i32, y_cal as i32, z_cal as i32
        );

        // IMU data is only used for BLE telemetry — not required for compass heading.
        let Ok(imu_data) = &imu_result else {
            continue;
        };

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
            roll: 0.0,
            pitch: 0.0,
            yaw: heading_deg,
        };
        data.valid = true;

        SENSOR_DATA.sender().send(data);
    }
}
