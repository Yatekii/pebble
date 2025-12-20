//! BMM350 3-axis magnetometer driver
//!
//! The BMM350 provides magnetic field measurements in 21-bit signed format.
//! Raw values are converted to microtesla (µT) using scaling factors from
//! the Bosch sensor API.

use embedded_hal::i2c::I2c;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::microtesla;

use crate::hal::imu::constants::{
    BMM350_CHIP_ID, BMM350_I2C_ADDR, BMM350_SOFT_RESET, bmm350_avg, bmm350_odr, bmm350_pmu,
    bmm350_reg,
};
use crate::hal::peripherals::i2c::{Error, SharedI2c, SharedI2cDevice};
use crate::util::sign_extend_21bit;

/// Sensitivity coefficient for LSB to microtesla conversion.
///
/// BMM350 has a measurement range of ±2000 µT with 21-bit signed resolution.
/// 21-bit signed = 20 bits for magnitude = 2^20 steps
/// 1 LSB = 2000 µT / 2^20 = 0.0019073486 µT/LSB
const LSB_TO_UT: f32 = 2000.0 / 1_048_576.0;

/// Shared BMM350 magnetometer driver (uses shared I2C bus)
pub struct SharedBmm350<'a, I2C> {
    i2c: SharedI2cDevice<'a, BMM350_I2C_ADDR, I2C>,
}

impl<'a, I2C, E> SharedBmm350<'a, I2C>
where
    I2C: I2c<Error = E>,
    E: defmt::Format,
{
    fn write_reg(&self, register: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c.write_reg(register, value)
    }

    fn read_reg(&self, register: u8) -> Result<u8, Error<E>> {
        // BMM350 uses 2 dummy bytes protocol - first 2 bytes after register address are dummy
        let mut buf = [0u8; 3];
        self.i2c.read_regs(register, &mut buf)?;
        // buf[0], buf[1] are dummy, buf[2] is actual data
        Ok(buf[2])
    }

    fn read_regs(&self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        // BMM350 uses 2 dummy bytes protocol
        // We need to read len+2 bytes and skip the first 2
        let len = buf.len();
        let mut tmp = [0u8; 18]; // Max we'll ever need (9 bytes data + 2 dummy)

        self.i2c.read_regs(register, &mut tmp[..len + 2])?;
        // Skip 2 dummy bytes (tmp[0], tmp[1]), copy rest to buf
        buf.copy_from_slice(&tmp[2..len + 2]);
        Ok(())
    }

    /// Wait for PMU command to complete
    fn wait_pmu_cmd<D: embedded_hal::delay::DelayNs>(
        &self,
        expected_cmd: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        for _ in 0..20 {
            delay.delay_ms(2);
            let status = self.read_reg(bmm350_reg::PMU_CMD_STATUS_0)?;
            let busy = (status & 0x01) != 0;
            let cmd_value = (status >> 5) & 0x07;
            if !busy && cmd_value == expected_cmd {
                return Ok(());
            }
        }
        defmt::warn!("BMM350 PMU command timeout");
        Ok(())
    }

    /// Create and initialize a new shared BMM350 driver
    pub fn new<D: embedded_hal::delay::DelayNs>(
        i2c: &'a SharedI2c<I2C>,
        delay: &mut D,
    ) -> Result<Self, Error<E>> {
        let i2c = i2c.device_with_address::<BMM350_I2C_ADDR>();
        let mag = Self { i2c };

        // Wait for BMM350 power-up (1ms from datasheet)
        delay.delay_ms(5);

        // The BMM350 starts in suspend mode - do a soft reset to wake it up
        mag.write_reg(bmm350_reg::CMD, BMM350_SOFT_RESET)?;
        delay.delay_ms(30); // 24ms from datasheet

        // Verify chip ID
        let chip_id = mag.read_reg(bmm350_reg::CHIP_ID)?;
        if chip_id != BMM350_CHIP_ID {
            defmt::warn!(
                "BMM350 chip ID mismatch: {:#x} (expected {:#x})",
                chip_id,
                BMM350_CHIP_ID
            );
            return Err(Error::InvalidChipId(chip_id));
        }

        // Power off OTP (auto-loaded on reset)
        mag.write_reg(bmm350_reg::OTP_CMD_REG, 0x80)?;
        delay.delay_ms(5);

        // Magnetic reset sequence (required for proper operation)
        // 1. Bit Reset
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::BIT_RESET)?;
        delay.delay_ms(14);
        mag.wait_pmu_cmd(bmm350_pmu::BIT_RESET, delay)?;

        // 2. Flux Guide Reset
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::FLUX_GUIDE_RESET)?;
        delay.delay_ms(18);
        mag.wait_pmu_cmd(bmm350_pmu::FLUX_GUIDE_RESET, delay)?;

        // Enable all axes (X, Y, Z)
        mag.write_reg(bmm350_reg::PMU_CMD_AXIS_EN, 0x07)?;
        delay.delay_ms(1);

        // Set ODR to 100Hz with 4x averaging
        let odr_avg = bmm350_odr::ODR_100HZ | bmm350_avg::AVG_4;
        mag.write_reg(bmm350_reg::PMU_CMD_AGGR_SET, odr_avg)?;
        delay.delay_ms(1);

        // Apply ODR/AVG settings
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::UPDATE_OAE)?;
        delay.delay_ms(2);
        mag.wait_pmu_cmd(bmm350_pmu::UPDATE_OAE, delay)?;

        // Set normal mode for continuous measurement
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::NORMAL)?;
        delay.delay_ms(38);
        mag.wait_pmu_cmd(bmm350_pmu::NORMAL, delay)?;

        defmt::info!("BMM350 initialized (chip_id={:#x})", chip_id);
        Ok(mag)
    }

    /// Read raw magnetometer data (21-bit signed values)
    pub fn read_raw(&self) -> Result<MagDataRaw, Error<E>> {
        let mut buf = [0u8; 9];
        self.read_regs(bmm350_reg::MAG_X_XLSB, &mut buf)?;

        // Each axis is 3 bytes (24-bit register), but only 21 bits are used
        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let raw_z = (buf[6] as u32) | ((buf[7] as u32) << 8) | ((buf[8] as u32) << 16);

        Ok(MagDataRaw {
            x: sign_extend_21bit(raw_x),
            y: sign_extend_21bit(raw_y),
            z: sign_extend_21bit(raw_z),
        })
    }

    /// Read magnetometer data in microtesla (µT)
    ///
    /// Returns uncompensated magnetic field values. For best accuracy,
    /// temperature compensation should be applied using OTP calibration data.
    pub fn read(&self) -> Result<MagData, Error<E>> {
        let raw = self.read_raw()?;
        Ok(MagData {
            x: MagneticFluxDensity::new::<microtesla>(raw.x as f32 * LSB_TO_UT),
            y: MagneticFluxDensity::new::<microtesla>(raw.y as f32 * LSB_TO_UT),
            z: MagneticFluxDensity::new::<microtesla>(raw.z as f32 * LSB_TO_UT),
        })
    }
}

/// Raw magnetometer data (21-bit signed values in LSB)
#[derive(Debug, Clone, Copy, Default)]
pub struct MagDataRaw {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Magnetometer data in physical units (microtesla)
#[derive(Debug, Clone, Copy)]
pub struct MagData {
    pub x: MagneticFluxDensity,
    pub y: MagneticFluxDensity,
    pub z: MagneticFluxDensity,
}
