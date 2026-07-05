//! IMU driver wrapper for BMI270 and BMM350
//!
//! BMI270 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
//! BMM350 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf

pub mod bmi270;
pub mod bmm350;
pub mod constants;

use esp_hal::Blocking;
use esp_hal::i2c::master::{Config as I2cConfig, I2c, SoftwareTimeout};
use esp_hal::time::{Duration, Rate};

use crate::hal::peripherals::i2c::{Error, SharedI2c};
/// Initialize IMU and magnetometer on shared I2C bus
///
/// Returns a SharedI2c wrapper, SharedImu, and SharedBmm350.
/// The SharedI2c must be stored in a static or kept alive for the lifetime of the drivers.
pub fn init_shared(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO2<'static>,
    scl: esp_hal::peripherals::GPIO3<'static>,
) -> Result<SharedI2c<I2c<'static, Blocking>>, Error<esp_hal::i2c::master::Error>> {
    // Software timeout so a wedged bus returns Err instead of spinning forever in the
    // blocking wait (which freezes the whole executor). A register read is ~0.3 ms at
    // 400 kHz; callers treat a failed read as a skipped sample.
    // ponytail: if lockups recur, add 9-clock bus recovery on the Err path.
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default()
            .with_frequency(Rate::from_khz(400))
            .with_software_timeout(SoftwareTimeout::Transaction(Duration::from_millis(10))),
    )
    .map_err(|_| Error::I2cCreate)?
    .with_sda(sda)
    .with_scl(scl);

    Ok(SharedI2c::new(i2c))
}
