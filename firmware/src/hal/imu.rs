//! IMU driver wrapper for BMI270 and BMM350
//!
//! BMI270 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
//! BMM350 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf

pub mod bmi270;
pub mod bmm350;
pub mod constants;

use embedded_hal::delay::DelayNs;
use esp_hal::Blocking;
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Flex, OutputConfig, Pin, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c, SoftwareTimeout};
use esp_hal::time::{Duration, Rate};

use crate::hal::peripherals::i2c::{Error, SharedI2c};

/// Reset a wedged I2C slave before the peripheral takes over the pins.
///
/// A reflash/reset resets the MCU but *not* the BMI270/BMM350, so a sensor left
/// mid-transaction stays stuck and every transfer then times out — only a full
/// power cycle would clear it, impossible once the battery is sealed in the box.
/// Observed on hardware: the lines idle high but the slave's state machine is
/// still mid-transaction, so a level gate sees nothing wrong. The fix is to
/// clock SCL (up to 9 pulses, freeing SDA if it *is* held) and then issue a
/// STOP, which resets the slave. Harmless on a healthy bus.
fn recover_bus(sda: impl Pin, scl: impl Pin) {
    let od = OutputConfig::default()
        .with_drive_mode(DriveMode::OpenDrain)
        .with_pull(Pull::Up);
    let mut sda = Flex::new(sda);
    let mut scl = Flex::new(scl);
    for pin in [&mut sda, &mut scl] {
        pin.apply_output_config(&od);
        pin.set_high(); // release (open-drain high = high-Z)
        pin.set_output_enable(true);
        pin.set_input_enable(true);
    }

    let mut delay = Delay::new();
    delay.delay_us(50); // let the released lines settle before sampling
    defmt::info!(
        "bus recovery: entry sda={} scl={}",
        sda.is_high(),
        scl.is_high()
    );

    // Always clock a full 9 pulses (no early break): a slave interrupted mid-byte
    // must be clocked through its remaining bits + ACK before it releases SDA, and
    // a level gate can stop short of that. SDA stays released (high) so the slave
    // sees a NACK and lets go. ~100 kHz recovery clock (5 µs half-period).
    for _ in 0..9 {
        scl.set_low();
        delay.delay_us(5);
        scl.set_high();
        delay.delay_us(5);
    }
    // STOP condition: SDA low while SCL high, then release SDA.
    sda.set_low();
    delay.delay_us(5);
    scl.set_high();
    delay.delay_us(5);
    sda.set_high();
    delay.delay_us(5);

    defmt::info!("bus recovery: exit sda={} scl={}", sda.is_high(), scl.is_high());
}

/// Initialize IMU and magnetometer on shared I2C bus
///
/// Returns a SharedI2c wrapper, SharedImu, and SharedBmm350.
/// The SharedI2c must be stored in a static or kept alive for the lifetime of the drivers.
pub fn init_shared(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    mut sda: esp_hal::peripherals::GPIO2<'static>,
    mut scl: esp_hal::peripherals::GPIO3<'static>,
) -> Result<SharedI2c<I2c<'static, Blocking>>, Error<esp_hal::i2c::master::Error>> {
    // Unwedge any slave stuck holding SDA low before the peripheral grabs the pins.
    recover_bus(sda.reborrow(), scl.reborrow());

    // Software timeout so a wedged bus returns Err instead of spinning forever in the
    // blocking wait (which freezes the whole executor). A register read is ~0.3 ms at
    // 400 kHz; callers treat a failed read as a skipped sample.
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
