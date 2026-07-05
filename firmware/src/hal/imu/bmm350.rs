//! BMM350 3-axis magnetometer driver with temperature compensation
//!
//! The BMM350 provides magnetic field measurements in 24-bit signed format.
//! This driver implements the full compensation algorithm from the Bosch
//! BMM350 Sensor API, including:
//! - Per-chip OTP calibration data
//! - Temperature compensation (TCO and TCS)
//! - Cross-axis compensation

use embedded_hal::i2c::I2c;
use uom::si::f32::MagneticFluxDensity;
use uom::si::magnetic_flux_density::microtesla;

use crate::hal::imu::constants::{
    BMM350_CHIP_ID, BMM350_I2C_ADDR, BMM350_SOFT_RESET, bmm350_avg, bmm350_odr, bmm350_otp,
    bmm350_pmu, bmm350_reg,
};
use crate::hal::peripherals::i2c::{Error, SharedI2c, SharedI2cDevice};
use crate::util::{sign_extend_8bit, sign_extend_12bit, sign_extend_16bit, sign_extend_24bit};

/// LSB to physical unit conversion factors.
///
/// These factors convert raw 24-bit ADC values to physical units (µT and °C).
/// The conversion chain is: ADC -> INA (instrumentation amp) -> LUT -> output.
///
/// Constants derived from the Bosch BMM350 Sensor API:
/// <https://github.com/boschsensortec/BMM350_SensorAPI>
mod conversion {
    /// Base scaling factor (2^20 to µT).
    pub const POWER: f32 = 1_000_000.0 / 1_048_576.0;
    /// X/Y axis magnetic sensitivity.
    pub const BXY_SENS: f32 = 14.55;
    /// Z axis magnetic sensitivity.
    pub const BZ_SENS: f32 = 9.0;
    /// Instrumentation amplifier gain for X/Y.
    pub const INA_XY_GAIN: f32 = 19.46;
    /// Instrumentation amplifier gain for Z.
    pub const INA_Z_GAIN: f32 = 31.0;
    /// ADC gain (~0.667).
    pub const ADC_GAIN: f32 = 1.0 / 1.5;
    /// Lookup table correction gain.
    pub const LUT_GAIN: f32 = 0.714_607_24;
    /// Temperature sensitivity.
    pub const TEMP_SENS: f32 = 0.00204;

    /// LSB to microtesla conversion factor for X/Y axes.
    pub const LSB_TO_UT_XY: f32 = POWER / (BXY_SENS * INA_XY_GAIN * ADC_GAIN * LUT_GAIN);
    /// LSB to microtesla conversion factor for Z axis.
    pub const LSB_TO_UT_Z: f32 = POWER / (BZ_SENS * INA_Z_GAIN * ADC_GAIN * LUT_GAIN);
    /// LSB to degrees Celsius conversion factor.
    pub const LSB_TO_DEGC: f32 = 1.0 / (TEMP_SENS * ADC_GAIN * LUT_GAIN * 1_048_576.0);

    /// Y-axis sensitivity correction.
    pub const SENS_CORR_Y: f32 = 0.01;
    /// Z-axis TCS correction.
    pub const TCS_CORR_Z: f32 = 0.0001;
}

use conversion::*;

/// Fixed (no OTP) LSB → µT scale for X and Y axes, from datasheet constants only.
/// Use this in calibration mode to avoid temperature-drift artifacts from OTP compensation.
pub const RAW_LSB_TO_UT_XY: f32 = conversion::LSB_TO_UT_XY;
/// Fixed (no OTP) LSB → µT scale for Z axis.
pub const RAW_LSB_TO_UT_Z: f32 = conversion::LSB_TO_UT_Z;

/// OTP calibration data for compensation
#[derive(Debug, Clone, Default)]
pub struct CompensationData {
    // Offset coefficients
    pub t_offs: f32,
    pub offset_x: f32,
    pub offset_y: f32,
    pub offset_z: f32,
    // Sensitivity coefficients
    pub t_sens: f32,
    pub sens_x: f32,
    pub sens_y: f32,
    pub sens_z: f32,
    // Temperature coefficient of offset
    pub tco_x: f32,
    pub tco_y: f32,
    pub tco_z: f32,
    // Temperature coefficient of sensitivity
    pub tcs_x: f32,
    pub tcs_y: f32,
    pub tcs_z: f32,
    // Reference temperature
    pub dut_t0: f32,
    // Cross-axis coupling
    pub cross_x_y: f32,
    pub cross_y_x: f32,
    pub cross_z_x: f32,
    pub cross_z_y: f32,
}

/// Shared BMM350 magnetometer driver (uses shared I2C bus)
pub struct SharedBmm350<'a, I2C> {
    i2c: SharedI2cDevice<'a, BMM350_I2C_ADDR, I2C>,
    comp: CompensationData,
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
        Ok(buf[2])
    }

    fn read_regs(&self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        // BMM350 uses 2 dummy bytes protocol
        let len = buf.len();
        let mut tmp = [0u8; 18]; // Max we'll ever need (12 bytes data + 2 dummy)

        self.i2c.read_regs(register, &mut tmp[..len + 2])?;
        buf.copy_from_slice(&tmp[2..len + 2]);
        Ok(())
    }

    /// Read a single OTP word at specified address
    fn read_otp_word<D: embedded_hal::delay::DelayNs>(
        &self,
        addr: u8,
        delay: &mut D,
    ) -> Result<u16, Error<E>> {
        // Set OTP command: read word at address (0x20 = DIR_READ)
        self.write_reg(bmm350_reg::OTP_CMD_REG, 0x20 | (addr & 0x1F))?;

        // Wait for OTP read to complete
        // Per Bosch API: poll every 300us, check for CMD_DONE (bit 0 = 1)
        for _ in 0..100 {
            delay.delay_us(300);
            let status = self.read_reg(bmm350_reg::OTP_STATUS)?;

            // Check for errors first (bits 7-5, mask 0xE0)
            if status & 0xE0 != 0 {
                defmt::warn!("OTP read error: status={:#x}", status);
                return Err(Error::OtpError);
            }

            // Bit 0 = 1 means command done (not busy!)
            if status & 0x01 != 0 {
                // Read complete, get data
                let msb = self.read_reg(bmm350_reg::OTP_DATA_MSB)?;
                let lsb = self.read_reg(bmm350_reg::OTP_DATA_LSB)?;
                return Ok(((msb as u16) << 8) | (lsb as u16));
            }
        }
        defmt::warn!("OTP read timeout");
        Err(Error::Timeout)
    }

    /// Read all OTP calibration data and extract compensation coefficients
    fn read_otp_data<D: embedded_hal::delay::DelayNs>(
        &self,
        delay: &mut D,
    ) -> Result<CompensationData, Error<E>> {
        // Read all 32 OTP words
        let mut otp = [0u16; 32];
        for (i, word) in otp.iter_mut().enumerate() {
            *word = self.read_otp_word(i as u8, delay)?;
        }

        let mut comp = CompensationData::default();

        // Temperature offset and sensitivity (word 0x0D)
        let temp_off_sens = otp[bmm350_otp::TEMP_OFF_SENS as usize];
        comp.t_offs = sign_extend_8bit(temp_off_sens as u8) as f32 / 5.0;
        comp.t_sens = sign_extend_8bit((temp_off_sens >> 8) as u8) as f32 / 512.0;

        // Magnetic offsets (12-bit signed, packed across OTP word boundaries per Bosch SensorAPI)
        // offset_x: bits [11:0] of word 0x0E
        comp.offset_x = sign_extend_12bit(otp[bmm350_otp::MAG_OFFSET_X as usize] & 0x0FFF) as f32;
        // offset_y: bits [15:12] of word 0x0E become bits [11:8], bits [7:0] of word 0x0F become bits [7:0]
        let off_y = ((otp[bmm350_otp::MAG_OFFSET_X as usize] & 0xF000) >> 4)
            | (otp[bmm350_otp::MAG_OFFSET_Y as usize] & 0x00FF);
        comp.offset_y = sign_extend_12bit(off_y) as f32;
        // offset_z: bits [11:8] of word 0x0F become bits [11:8], bits [7:0] of word 0x10 become bits [7:0]
        let off_z = (otp[bmm350_otp::MAG_OFFSET_Y as usize] & 0x0F00)
            | (otp[bmm350_otp::MAG_OFFSET_Z as usize] & 0x00FF);
        comp.offset_z = sign_extend_12bit(off_z) as f32;

        // Magnetic sensitivities (8-bit signed)
        // sens_x is upper 8 bits of word 0x10
        comp.sens_x =
            sign_extend_8bit((otp[bmm350_otp::MAG_SENS_X as usize] >> 8) as u8) as f32 / 256.0;
        // sens_y is lower 8 bits of word 0x11
        comp.sens_y = sign_extend_8bit(otp[bmm350_otp::MAG_SENS_Y as usize] as u8) as f32 / 256.0
            + SENS_CORR_Y;
        // sens_z is upper 8 bits of word 0x11
        comp.sens_z =
            sign_extend_8bit((otp[bmm350_otp::MAG_SENS_Z as usize] >> 8) as u8) as f32 / 256.0;

        // TCO (Temperature Coefficient of Offset) - 8-bit signed, divided by 32
        comp.tco_x = sign_extend_8bit(otp[bmm350_otp::MAG_TCO_X as usize] as u8) as f32 / 32.0;
        comp.tco_y = sign_extend_8bit(otp[bmm350_otp::MAG_TCO_Y as usize] as u8) as f32 / 32.0;
        comp.tco_z = sign_extend_8bit(otp[bmm350_otp::MAG_TCO_Z as usize] as u8) as f32 / 32.0;

        // TCS (Temperature Coefficient of Sensitivity) - 8-bit signed, divided by 16384
        comp.tcs_x =
            sign_extend_8bit((otp[bmm350_otp::MAG_TCS_X as usize] >> 8) as u8) as f32 / 16384.0;
        comp.tcs_y =
            sign_extend_8bit((otp[bmm350_otp::MAG_TCS_Y as usize] >> 8) as u8) as f32 / 16384.0;
        comp.tcs_z = sign_extend_8bit((otp[bmm350_otp::MAG_TCS_Z as usize] >> 8) as u8) as f32
            / 16384.0
            - TCS_CORR_Z;

        // Reference temperature DUT_T0 (16-bit signed)
        comp.dut_t0 =
            sign_extend_16bit(otp[bmm350_otp::MAG_DUT_T_0 as usize]) as f32 / 512.0 + 23.0;

        // Cross-axis coupling (8-bit signed, divided by 800)
        comp.cross_x_y = sign_extend_8bit(otp[bmm350_otp::CROSS_X_Y as usize] as u8) as f32 / 800.0;
        comp.cross_y_x =
            sign_extend_8bit((otp[bmm350_otp::CROSS_Y_X as usize] >> 8) as u8) as f32 / 800.0;
        comp.cross_z_x = sign_extend_8bit(otp[bmm350_otp::CROSS_Z_X as usize] as u8) as f32 / 800.0;
        comp.cross_z_y =
            sign_extend_8bit((otp[bmm350_otp::CROSS_Z_Y as usize] >> 8) as u8) as f32 / 800.0;

        defmt::debug!(
            "BMM350 OTP: t_offs={} offset_x={} sens_x={} tco_x={} tcs_x={} dut_t0={}",
            comp.t_offs,
            comp.offset_x,
            comp.sens_x,
            comp.tco_x,
            comp.tcs_x,
            comp.dut_t0
        );

        Ok(comp)
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
        let mut mag = Self {
            i2c,
            comp: CompensationData::default(),
        };

        // Wait for BMM350 power-up (1ms from datasheet)
        delay.delay_ms(5);

        // Soft reset to ensure clean state
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

        // Read OTP calibration data (auto-loaded after reset)
        mag.comp = mag.read_otp_data(delay)?;

        // Power off OTP after reading
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

        defmt::info!(
            "BMM350 initialized with OTP compensation (chip_id={:#x})",
            chip_id
        );
        Ok(mag)
    }

    /// Read raw magnetometer and temperature data (24-bit signed two's complement)
    pub fn read_raw(&self) -> Result<MagTempDataRaw, Error<E>> {
        let mut buf = [0u8; 12];
        self.read_regs(bmm350_reg::MAG_X_XLSB, &mut buf)?;

        // Each axis is 3 bytes (24-bit two's complement)
        // Sign extend from 24-bit to 32-bit: shift left 8, arithmetic shift right 8
        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let raw_z = (buf[6] as u32) | ((buf[7] as u32) << 8) | ((buf[8] as u32) << 16);
        let raw_t = (buf[9] as u32) | ((buf[10] as u32) << 8) | ((buf[11] as u32) << 16);

        Ok(MagTempDataRaw {
            x: sign_extend_24bit(raw_x),
            y: sign_extend_24bit(raw_y),
            z: sign_extend_24bit(raw_z),
            temp: sign_extend_24bit(raw_t),
        })
    }

    /// Read compensated magnetometer data in microtesla (µT) and temperature in °C
    ///
    /// Applies the full Bosch compensation algorithm:
    /// 1. Convert raw LSB to physical units
    /// 2. Apply sensitivity and offset correction
    /// 3. Apply temperature compensation (TCO and TCS)
    /// 4. Apply cross-axis compensation
    pub fn read(&self) -> Result<MagTempData, Error<E>> {
        let raw = self.read_raw()?;

        // Convert raw values to physical units
        let mut x = raw.x as f32 * LSB_TO_UT_XY;
        let mut y = raw.y as f32 * LSB_TO_UT_XY;
        let mut z = raw.z as f32 * LSB_TO_UT_Z;
        let temp = raw.temp as f32 * LSB_TO_DEGC - 25.49;

        // Apply temperature compensation
        let temperature = (1.0 + self.comp.t_sens) * temp + self.comp.t_offs;
        let temp_delta = temperature - self.comp.dut_t0;

        // Apply sensitivity, offset, and temperature compensation for each axis
        // Formula: value = ((value * (1 + sens) + offset + tco * dT) / (1 + tcs * dT))

        // X-axis
        x *= 1.0 + self.comp.sens_x;
        x += self.comp.offset_x;
        x += self.comp.tco_x * temp_delta;
        x /= 1.0 + self.comp.tcs_x * temp_delta;

        // Y-axis
        y *= 1.0 + self.comp.sens_y;
        y += self.comp.offset_y;
        y += self.comp.tco_y * temp_delta;
        y /= 1.0 + self.comp.tcs_y * temp_delta;

        // Z-axis
        z *= 1.0 + self.comp.sens_z;
        z += self.comp.offset_z;
        z += self.comp.tco_z * temp_delta;
        z /= 1.0 + self.comp.tcs_z * temp_delta;

        // Apply cross-axis compensation
        let cross_denom = 1.0 - self.comp.cross_y_x * self.comp.cross_x_y;
        let x_comp = (x - self.comp.cross_x_y * y) / cross_denom;
        let y_comp = (y - self.comp.cross_y_x * x) / cross_denom;
        let z_comp = z
            + (x * (self.comp.cross_y_x * self.comp.cross_z_y - self.comp.cross_z_x)
                - y * (self.comp.cross_z_y - self.comp.cross_x_y * self.comp.cross_z_x))
                / cross_denom;

        Ok(MagTempData {
            x: MagneticFluxDensity::new::<microtesla>(x_comp),
            y: MagneticFluxDensity::new::<microtesla>(y_comp),
            z: MagneticFluxDensity::new::<microtesla>(z_comp),
            temperature,
        })
    }
}

/// Raw magnetometer and temperature data (21-bit signed mag values in LSB)
#[derive(Debug, Clone, Copy, Default)]
pub struct MagTempDataRaw {
    pub x: i32,
    pub y: i32,
    pub z: i32,
    pub temp: i32,
}

/// Compensated magnetometer data in physical units
#[derive(Debug, Clone, Copy)]
pub struct MagTempData {
    pub x: MagneticFluxDensity,
    pub y: MagneticFluxDensity,
    pub z: MagneticFluxDensity,
    pub temperature: f32, // Degrees Celsius
}

// Keep backward compatibility aliases
pub type MagDataRaw = MagTempDataRaw;
pub type MagData = MagTempData;
