//! IMU task: reads sensor data, computes tilt-compensated heading, and broadcasts results.

use defmt::info;
use embassy_time::{Duration, Timer};
use embedded_hal::i2c::I2c;
use nalgebra::Vector3;
use uom::si::magnetic_flux_density::microtesla;

use crate::comms::ble::{AccBleData, AhrsBleData, GyroBleData, MagBleData};
use crate::filter::ahrs::{AhrsFilter, GyroCalibration};
use crate::hal::imu::bmi270::SharedBmi270;
use crate::hal::imu::bmm350::{RAW_LSB_TO_UT_XY, RAW_LSB_TO_UT_Z, SharedBmm350};
use crate::math::{BMI270_TO_BOARD, BMM350_TO_BOARD, tilt_compensated_heading};
use crate::state::{
    COMPASS_HEADING, DOUBLE_TAP_EVENT, IMMEDIATE_TAP_EVENT, SENSOR_DATA, SensorData, TAP_EVENT,
};

/// Sample period in milliseconds (100Hz = 10ms).
const SAMPLE_PERIOD_MS: u64 = 10;

/// BMI270 gyroscope scale at ±2000 dps range: 16.384 LSB per dps.
const GYRO_LSB_TO_DPS: f32 = 1.0 / 16.384;
/// Degrees → radians (Madgwick expects gyro in rad/s).
const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
/// Heading offset (degrees) applied to the fused yaw so 0° aligns with the LED
/// ring's north (board +X, LED 72). Tune on hardware; negate the fused yaw first
/// if rotation runs the wrong way.
const AHRS_HEADING_OFFSET_DEG: f32 = 0.0;

/// EMA low-pass filter coefficient for the accelerometer used in tilt compensation.
/// α = 1 − exp(−dt/τ) ≈ dt/τ.  With dt = 10 ms and τ = 50 ms → α ≈ 0.2.
/// In Switzerland Bv ≈ 43 µT > Bh ≈ 20 µT, so B_board.x is negative when tilted
/// up until enough pitch compensation kicks in.  With α=0.02 (τ=500ms) that
/// transition takes ~330 ms and shows heading=180° until it converges.
/// α=0.2 (τ=50ms) converges in <250ms, fast enough for hand-held use.
const ACC_LPF_ALPHA: f32 = 0.2;

/// Expected Earth-field magnitude after calibration (µT).
/// Centered to span indoor-distorted (~20 µT, steel/rebar shielding) through
/// outdoor Earth field (~50-65 µT). Tighten once calibration is verified in place.
const MAG_FIELD_CENTER: f32 = 40.0;
/// Half-width of the acceptance window around `MAG_FIELD_CENTER` (µT).
const MAG_FIELD_TOLERANCE: f32 = 30.0;

/// When built with `--features mag-calibrate` (done automatically by the
/// `mag-calibrate` tool), streams raw magnetometer values over RTT so the
/// host tool can fit an ellipsoid and print calibration constants.
const MAG_CALIBRATING: bool = cfg!(feature = "mag-calibrate");

/// Hard iron offset (µT). Updated by calibrate_mag.py.
const MAG_HI_OFFSET: [f32; 3] = [-8.7998, 50.6150, -17.1461];

/// Soft iron correction matrix.
/// Identity = no correction (before first calibration run).
const MAG_SOFT_IRON: [[f32; 3]; 3] = [
    [0.987_503_2, 0.00032133, -0.00179177],
    [0.00032133, 0.993_431_4, -0.00748462],
    [-0.00179177, -0.00748462, 1.019_766_6],
];

// =============================================================================
// Tap / double-tap detection
// =============================================================================

/// Squared angular-rate threshold for tap detection (raw gyro LSB²).
///
/// Detection runs on the gyro (see the tap dispatch in `run`): a tap jolts the
/// box rotationally, showing as a clean gyro spike, while resting noise is only
/// a few dozen LSB. 300 LSB ÷ 16.384 ≈ 18 dps — comfortably above rest, below a
/// tap peak (~500-850 LSB observed). Sustained rotation (handling) is rejected
/// by `TAP_PEAK_MAX_TICKS`, not this threshold. Raise if handling false-triggers.
const TAP_THRESHOLD_SQ: u32 = 300 * 300;

/// Maximum spike duration for a valid tap impulse (samples at 100 Hz).
///
/// A genuine knock resolves within ~100 ms (10 ticks). Sustained motion
/// stays above the threshold longer and is rejected as not a tap.
const TAP_PEAK_MAX_TICKS: u8 = 10;

/// Inter-tap window for double-tap detection (samples at 100 Hz).
///
/// After the first tap resolves, wait up to 300 ms (30 ticks) for a second
/// spike. If none arrives the event is classified as a single tap.
const TAP_BETWEEN_TICKS: u8 = 30;

/// Debounce period after any confirmed tap event (samples at 100 Hz).
///
/// 50 samples × 10 ms = 500 ms.
const TAP_COOLDOWN_TICKS: u8 = 50;

/// Outcome returned by [`TapDetector::update`] each sample.
#[derive(Clone, Copy, PartialEq, Eq)]
enum TapResult {
    None,
    /// A tap spike just resolved — fired immediately, before single/double is known.
    ImmediateTap,
    Single,
    Double,
}

/// Software tap / double-tap detector.
///
/// State machine:
/// ```text
/// Idle ──[spike]──► Peak1 ──[resolved quickly]──► BetweenTaps ──[spike]──► Peak2 ──[resolved]──► Double + Cooldown
///                         └─[too long]──► Idle               └─[timeout]──► Single + Cooldown  └─[too long]──► Single + Cooldown
/// ```
struct TapDetector {
    state: TapState,
    /// Incremented on each confirmed single tap.
    single_count: u32,
    /// Incremented on each confirmed double tap.
    double_count: u32,
}

enum TapState {
    Idle,
    /// First spike in progress; `ticks` counts samples spent above threshold.
    Peak1 { ticks: u8 },
    /// First tap confirmed; waiting to see if a second spike arrives.
    BetweenTaps { remaining: u8 },
    /// Second spike in progress.
    Peak2 { ticks: u8 },
    /// Waiting out the post-event debounce window.
    Cooldown { remaining: u8 },
}

impl TapDetector {
    fn new() -> Self {
        Self {
            state: TapState::Idle,
            single_count: 0,
            double_count: 0,
        }
    }

    /// Feed one raw-LSB accelerometer sample. Returns the event (if any).
    fn update(&mut self, x: i16, y: i16, z: i16) -> TapResult {
        let sq = (x as i32 * x as i32) as u32
            + (y as i32 * y as i32) as u32
            + (z as i32 * z as i32) as u32;
        let above = sq > TAP_THRESHOLD_SQ;

        match self.state {
            TapState::Idle => {
                if above {
                    self.state = TapState::Peak1 { ticks: 0 };
                }
                TapResult::None
            }

            TapState::Peak1 { ref mut ticks } => {
                if !above {
                    // Spike resolved quickly → first tap confirmed; open the inter-tap window.
                    // Fire ImmediateTap now so the LED can respond without waiting for
                    // the BetweenTaps window to expire (which takes up to 300 ms).
                    self.state = TapState::BetweenTaps { remaining: TAP_BETWEEN_TICKS };
                    TapResult::ImmediateTap
                } else {
                    *ticks += 1;
                    if *ticks >= TAP_PEAK_MAX_TICKS {
                        self.state = TapState::Idle;
                    }
                    TapResult::None
                }
            }

            TapState::BetweenTaps { ref mut remaining } => {
                if above {
                    // Second spike started → move to Peak2.
                    self.state = TapState::Peak2 { ticks: 0 };
                    TapResult::None
                } else {
                    *remaining -= 1;
                    if *remaining == 0 {
                        // Window expired with no second tap → single tap.
                        self.single_count = self.single_count.wrapping_add(1);
                        self.state = TapState::Cooldown { remaining: TAP_COOLDOWN_TICKS };
                        TapResult::Single
                    } else {
                        TapResult::None
                    }
                }
            }

            TapState::Peak2 { ref mut ticks } => {
                if !above {
                    // Second spike resolved → double tap confirmed.
                    self.double_count = self.double_count.wrapping_add(1);
                    self.state = TapState::Cooldown { remaining: TAP_COOLDOWN_TICKS };
                    TapResult::Double
                } else {
                    *ticks += 1;
                    if *ticks >= TAP_PEAK_MAX_TICKS {
                        // Second spike lasted too long → count only the first tap.
                        self.single_count = self.single_count.wrapping_add(1);
                        self.state = TapState::Cooldown { remaining: TAP_COOLDOWN_TICKS };
                        TapResult::Single
                    } else {
                        TapResult::None
                    }
                }
            }

            TapState::Cooldown { ref mut remaining } => {
                *remaining -= 1;
                if *remaining == 0 {
                    self.state = TapState::Idle;
                }
                TapResult::None
            }
        }
    }
}

// =============================================================================

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

    // Accelerometer EMA low-pass filter state.
    // Initialised to a flat-board reading: +1g on board Z (4096 LSB at 8g range).
    let mut acc_lpf = Vector3::new(0.0f32, 0.0, 4096.0);

    let mut tap = TapDetector::new();

    // Madgwick sensor fusion for a smooth, gyro-stabilised, drift-corrected
    // orientation. Gyro bias is measured during a stationary warmup; until it's
    // ready we fall back to the instantaneous tilt-compensated heading.
    let mut ahrs = AhrsFilter::new();
    let mut gyro_cal = GyroCalibration::new();

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

        // Always update the accelerometer LPF regardless of magnetometer health.
        // Transform raw counts to board frame; when flat acc ≈ (0, 0, +g_counts).
        // The EMA (τ ≈ 500 ms) rejects vibration noise from the tilt estimate.
        if let Ok(ref imu_data) = imu_result {
            let raw_acc = imu_data.raw_acceleration();
            let acc_raw = BMI270_TO_BOARD
                * Vector3::new(raw_acc.x as f32, raw_acc.y as f32, raw_acc.z as f32);
            acc_lpf += ACC_LPF_ALPHA * (acc_raw - acc_lpf);

            // Tap detection runs on the gyro, not the accel: at 100 Hz ODR the
            // linear tap impulse is filtered down to ~1.3 g, but the rotational
            // jolt of tapping the box shows as a large, clean gyro spike. Raw
            // magnitude is frame-invariant so we skip the board-frame transform.
            let raw_gyr = imu_data.raw_angular_velocity();
            // TEMP tap tuning: log gyro spikes so we can confirm the threshold.
            let dbg_sq = (raw_gyr.x as i32 * raw_gyr.x as i32) as u32
                + (raw_gyr.y as i32 * raw_gyr.y as i32) as u32
                + (raw_gyr.z as i32 * raw_gyr.z as i32) as u32;
            if dbg_sq > 150 * 150 {
                defmt::info!("gyro spike: ~{=u32} LSB (~{=u32} dps)", dbg_sq.isqrt(), dbg_sq.isqrt() * 1000 / 16384);
            }
            match tap.update(raw_gyr.x, raw_gyr.y, raw_gyr.z) {
                TapResult::ImmediateTap => {
                    // Spike resolved — fire immediately for low-latency LED feedback.
                    // single_count is not yet incremented here; the count is just a
                    // monotone token so the LED receiver can detect the change.
                    IMMEDIATE_TAP_EVENT
                        .sender()
                        .send(tap.single_count.wrapping_add(tap.double_count));
                }
                TapResult::Single => {
                    defmt::info!("tap: single (count={})", tap.single_count);
                    TAP_EVENT.sender().send(tap.single_count);
                }
                TapResult::Double => {
                    defmt::info!("tap: double (count={})", tap.double_count);
                    // Peak2 resolving is itself an immediate tap (second flash of double).
                    IMMEDIATE_TAP_EVENT
                        .sender()
                        .send(tap.single_count.wrapping_add(tap.double_count));
                    DOUBLE_TAP_EVENT.sender().send(tap.double_count);
                }
                TapResult::None => {}
            }
        }

        // Magnetometer processing – a failed read or out-of-range magnitude skips
        // only the heading computation, not the acc update above.
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

        // Transform calibrated magnetometer to board frame.
        // Board frame: +X toward LED 72, +Y toward LED 18, +Z up.
        let mag_board = BMM350_TO_BOARD * Vector3::new(x_cal, y_cal, z_cal);

        // Orientation: fuse gyro + accel + mag with Madgwick once gyro bias is
        // calibrated; before that, and with no IMU, fall back to the
        // instantaneous tilt-compensated heading.
        let (roll_deg, pitch_deg, mut heading_deg) = if let Ok(imu_data) = &imu_result {
            let raw_gyro = imu_data.raw_angular_velocity();
            let gyro_rads = (BMI270_TO_BOARD
                * Vector3::new(raw_gyro.x as f32, raw_gyro.y as f32, raw_gyro.z as f32))
                * (GYRO_LSB_TO_DPS * DEG_TO_RAD);

            if !gyro_cal.is_ready() {
                gyro_cal.update(gyro_rads.x, gyro_rads.y, gyro_rads.z);
            }

            if gyro_cal.is_ready() {
                let (gx, gy, gz) = gyro_cal.apply(gyro_rads.x, gyro_rads.y, gyro_rads.z);
                // accel/mag normalized internally, so board-frame LPF accel and
                // calibrated mag can be passed as-is.
                let o = ahrs.update(Vector3::new(gx, gy, gz), acc_lpf, mag_board);
                (o.roll, o.pitch, o.yaw + AHRS_HEADING_OFFSET_DEG)
            } else {
                let (h, r, p) = tilt_compensated_heading(acc_lpf, mag_board);
                (r, p, h.to_degrees())
            }
        } else {
            // Flat-plane fallback, heading CW from board +X (LED 72).
            (0.0_f32, 0.0_f32, libm::atan2f(mag_board.y, mag_board.x).to_degrees())
        };

        heading_deg = libm::fmodf(heading_deg, 360.0);
        if heading_deg < 0.0 {
            heading_deg += 360.0;
        }
        let heading = heading_deg as u16;
        COMPASS_HEADING.sender().send(heading);

        if let Ok(imu_data) = &imu_result {
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
                roll: roll_deg,
                pitch: pitch_deg,
                yaw: heading_deg,
            };
            data.valid = true;
            SENSOR_DATA.sender().send(data);
        }
    }
}
