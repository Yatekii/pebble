//! IMU driver wrapper for BMI270 and BMM350
//!
//! BMI270 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
//! BMM350 Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf

use bmi2::config::BMI270_CONFIG_FILE;
use defmt::Format;
use embedded_hal::i2c::I2c as I2cTrait;
use esp_hal::Blocking;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::time::Rate;

// BMI270 Register addresses
mod reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const ERR_REG: u8 = 0x02;
    pub const STATUS: u8 = 0x03;
    pub const ACC_DATA_0: u8 = 0x0C;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

// BMM350 Register addresses (from BST-BMM350-DS001 datasheet)
mod bmm350_reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const PMU_CMD_AGGR_SET: u8 = 0x04;
    pub const PMU_CMD_AXIS_EN: u8 = 0x05;
    pub const PMU_CMD: u8 = 0x06;
    pub const PMU_CMD_STATUS_0: u8 = 0x07;
    // Magnetic data registers (24-bit per axis, little-endian)
    pub const MAG_X_XLSB: u8 = 0x31;
    pub const OTP_CMD_REG: u8 = 0x50;
    pub const CMD: u8 = 0x7E;
}

// BMM350 PMU command values
mod bmm350_pmu {
    pub const NORMAL: u8 = 0x01;
    pub const UPDATE_OAE: u8 = 0x02;
    pub const FLUX_GUIDE_RESET: u8 = 0x05;
    pub const BIT_RESET: u8 = 0x07;
}

// BMM350 ODR settings (bits 0-3 of PMU_CMD_AGGR_SET)
mod bmm350_odr {
    pub const ODR_100HZ: u8 = 0x04;
}

// BMM350 Averaging settings (bits 4-5 of PMU_CMD_AGGR_SET)
mod bmm350_avg {
    pub const AVG_4: u8 = 0x02 << 4;
}

const BMM350_CHIP_ID: u8 = 0x33;
const BMM350_I2C_ADDR: u8 = 0x14;

// Soft reset command for BMM350
const BMM350_SOFT_RESET: u8 = 0xB6;

const BMI270_CHIP_ID: u8 = 0x24;
const BMI270_I2C_ADDR: u8 = 0x68;

/// IMU initialization error
#[derive(Debug, Format)]
pub enum Error<E> {
    /// I2C creation error
    I2cCreate,
    /// I2C communication error
    Bus(E),
    /// Chip ID mismatch
    InvalidChipId(u8),
    /// Initialization timeout
    InitTimeout,
}

/// Accelerometer and gyroscope data
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
    pub gyr_x: i16,
    pub gyr_y: i16,
    pub gyr_z: i16,
}

impl ImuData {
    /// Convert accelerometer readings to g (assuming +/-8g range)
    /// BMI270 with 8g range: 1g = 4096 LSB
    pub fn accel_g(&self) -> (f32, f32, f32) {
        const SCALE: f32 = 1.0 / 4096.0; // 8g range
        (
            self.acc_x as f32 * SCALE,
            self.acc_y as f32 * SCALE,
            self.acc_z as f32 * SCALE,
        )
    }

    /// Convert gyroscope readings to rad/s (assuming +/-2000 dps range)
    /// BMI270 with 2000dps range: 1 dps = 16.384 LSB
    pub fn gyro_rads(&self) -> (f32, f32, f32) {
        // Convert to degrees per second first, then to radians
        const DPS_SCALE: f32 = 1.0 / 16.384; // 2000 dps range
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        const SCALE: f32 = DPS_SCALE * DEG_TO_RAD;
        (
            self.gyr_x as f32 * SCALE,
            self.gyr_y as f32 * SCALE,
            self.gyr_z as f32 * SCALE,
        )
    }
}

/// Magnetometer data (raw 24-bit signed values)
#[derive(Debug, Clone, Copy, Default)]
pub struct MagData {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// BMI270 IMU wrapper providing convenient access to accelerometer and gyroscope
pub struct Imu<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Imu<I2C>
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    fn write_reg(&mut self, register: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.addr, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_reg(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.addr, &[register], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    fn read_regs(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.addr, &[register], buf)
            .map_err(Error::Bus)
    }

    /// Create and initialize a new IMU driver
    pub fn new<D: embedded_hal::delay::DelayNs>(i2c: I2C, delay: &mut D) -> Result<Self, Error<E>> {
        let mut imu = Self {
            i2c,
            addr: BMI270_I2C_ADDR,
        };

        // Dummy read to wake up the device (required for I2C after power-on)
        let _ = imu.read_reg(reg::CHIP_ID);
        delay.delay_us(450);

        // Soft reset the device
        imu.write_reg(reg::CMD, 0xB6)?; // SOFT_RESET command
        delay.delay_ms(2);

        // Another dummy read after reset
        let _ = imu.read_reg(reg::CHIP_ID);
        delay.delay_us(450);

        // Verify chip ID
        let chip_id = imu.read_reg(reg::CHIP_ID)?;
        if chip_id != BMI270_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Disable advanced power save before config upload
        imu.write_reg(reg::PWR_CONF, 0x00)?;
        delay.delay_us(450);

        // Prepare for config load - set init_ctrl to 0
        imu.write_reg(reg::INIT_CTRL, 0x00)?;

        // Upload configuration file in small chunks
        let mut offset = 0usize;
        let chunk_size = 32usize;

        while offset < BMI270_CONFIG_FILE.len() {
            // Set the address (divided by 2 as per datasheet)
            let addr_val = (offset / 2) as u16;
            imu.write_reg(reg::INIT_ADDR_0, (addr_val & 0x0F) as u8)?;
            imu.write_reg(reg::INIT_ADDR_1, (addr_val >> 4) as u8)?;

            // Calculate chunk end
            let end = (offset + chunk_size).min(BMI270_CONFIG_FILE.len());
            let chunk = &BMI270_CONFIG_FILE[offset..end];

            // Write chunk: first byte is register address, then data
            let mut buf = [0u8; 33]; // 1 register + 32 data bytes max
            buf[0] = reg::INIT_DATA;
            buf[1..1 + chunk.len()].copy_from_slice(chunk);

            imu.i2c
                .write(imu.addr, &buf[..1 + chunk.len()])
                .map_err(Error::Bus)?;

            offset = end;
        }

        // Start initialization
        imu.write_reg(reg::INIT_CTRL, 0x01)?;

        // Wait for initialization to complete
        let mut init_ok = false;
        for _ in 0..50 {
            delay.delay_ms(10);
            let status = imu.read_reg(reg::INTERNAL_STATUS)?;
            let msg = status & 0x0F;
            if msg == 0x01 {
                init_ok = true;
                break;
            } else if msg >= 0x02 {
                defmt::warn!("BMI270 init error: {}", msg);
                break;
            }
        }

        if !init_ok {
            return Err(Error::InitTimeout);
        }

        // Disable advanced power save mode
        imu.write_reg(reg::PWR_CONF, 0x00)?;
        delay.delay_ms(1);

        // Enable accelerometer and gyroscope (no aux)
        // PWR_CTRL: bit 2 = acc_en, bit 1 = gyr_en
        imu.write_reg(reg::PWR_CTRL, 0x0E)?; // acc_en + gyr_en + temp_en
        delay.delay_ms(50);

        // Configure accelerometer: 100Hz ODR, normal mode, +/-8g
        // ACC_CONF: odr=0x08 (100Hz), bwp=0x02 (normal), filter_perf=1
        imu.write_reg(reg::ACC_CONF, 0xA8)?;
        // ACC_RANGE: 0x02 = +/-8g
        imu.write_reg(reg::ACC_RANGE, 0x02)?;

        // Configure gyroscope: 200Hz ODR, normal mode, +/-2000dps
        // GYR_CONF: odr=0x09 (200Hz), bwp=0x02 (normal), noise_perf=1, filter_perf=1
        imu.write_reg(reg::GYR_CONF, 0xA9)?;
        // GYR_RANGE: 0x00 = +/-2000dps
        imu.write_reg(reg::GYR_RANGE, 0x00)?;

        Ok(imu)
    }

    /// Read accelerometer and gyroscope data
    pub fn read(&mut self) -> Result<ImuData, Error<E>> {
        let mut buf = [0u8; 12];
        self.read_regs(reg::ACC_DATA_0, &mut buf)?;

        Ok(ImuData {
            acc_x: i16::from_le_bytes([buf[0], buf[1]]),
            acc_y: i16::from_le_bytes([buf[2], buf[3]]),
            acc_z: i16::from_le_bytes([buf[4], buf[5]]),
            gyr_x: i16::from_le_bytes([buf[6], buf[7]]),
            gyr_y: i16::from_le_bytes([buf[8], buf[9]]),
            gyr_z: i16::from_le_bytes([buf[10], buf[11]]),
        })
    }

    /// Read and log diagnostic information
    pub fn debug_status(&mut self) -> Result<(), Error<E>> {
        let status = self.read_reg(reg::STATUS)?;
        defmt::info!(
            "Status: acc_data_ready={} gyr_data_ready={}",
            (status & 0x80) != 0,
            (status & 0x40) != 0
        );

        let pwr_ctrl = self.read_reg(reg::PWR_CTRL)?;
        defmt::info!(
            "PwrCtrl: acc_en={} gyr_en={} temp_en={}",
            (pwr_ctrl & 0x04) != 0,
            (pwr_ctrl & 0x02) != 0,
            (pwr_ctrl & 0x08) != 0
        );

        let pwr_conf = self.read_reg(reg::PWR_CONF)?;
        defmt::info!("PwrConf: power_save={}", (pwr_conf & 0x01) != 0);

        let internal_status = self.read_reg(reg::INTERNAL_STATUS)?;
        defmt::info!("InternalStatus: {:#x}", internal_status);

        let errors = self.read_reg(reg::ERR_REG)?;
        defmt::info!("Errors: {:#x}", errors);

        Ok(())
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Scan the I2C bus for devices
    pub fn scan_i2c_bus(&mut self) {
        defmt::info!("Scanning I2C bus for devices...");
        let mut found = 0;
        for addr in 0x03..=0x77 {
            if self.i2c.write(addr, &[]).is_ok() {
                defmt::info!("Found device at address {:#x}", addr);
                found += 1;
            }
        }
        defmt::info!("I2C scan complete - found {} devices", found);
    }
}

/// BMM350 Magnetometer driver with direct I2C access
pub struct Bmm350<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Bmm350<I2C>
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    fn write_reg(&mut self, register: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.addr, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_reg(&mut self, register: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c
            .write_read(self.addr, &[register], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    fn read_regs(&mut self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(self.addr, &[register], buf)
            .map_err(Error::Bus)
    }

    /// Wait for PMU command to complete
    fn wait_pmu_cmd<D: embedded_hal::delay::DelayNs>(
        &mut self,
        expected_cmd: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        for _ in 0..20 {
            delay.delay_ms(2);
            let status = self.read_reg(bmm350_reg::PMU_CMD_STATUS_0)?;
            // Check pmu_cmd_busy (bit 0) is 0 and pmu_cmd_value (bits 7:5) matches
            let busy = (status & 0x01) != 0;
            let cmd_value = (status >> 5) & 0x07;
            if !busy && cmd_value == expected_cmd {
                return Ok(());
            }
        }
        defmt::warn!("BMM350 PMU command timeout");
        Ok(()) // Continue anyway
    }

    /// Create and initialize a new BMM350 driver with direct I2C access
    pub fn new<D: embedded_hal::delay::DelayNs>(i2c: I2C, delay: &mut D) -> Result<Self, Error<E>> {
        let mut mag = Self {
            i2c,
            addr: BMM350_I2C_ADDR,
        };

        defmt::info!("=== Initializing BMM350 via direct I2C ===");

        // Wait for BMM350 startup (3ms from power-on)
        delay.delay_ms(5);

        // Read chip ID
        let chip_id = mag.read_reg(bmm350_reg::CHIP_ID)?;
        defmt::info!(
            "BMM350 chip ID: {:#x} (expected: {:#x})",
            chip_id,
            BMM350_CHIP_ID
        );

        if chip_id != BMM350_CHIP_ID {
            defmt::warn!(
                "Wrong chip ID: {:#x}, expected {:#x}",
                chip_id,
                BMM350_CHIP_ID
            );
            return Err(Error::InvalidChipId(chip_id));
        }

        // Soft reset BMM350 via CMD register (0x7E)
        defmt::info!("Performing BMM350 soft reset...");
        mag.write_reg(bmm350_reg::CMD, BMM350_SOFT_RESET)?;
        delay.delay_ms(24); // BMM350 needs ~24ms after soft reset

        // Verify chip ID again after reset
        let chip_id = mag.read_reg(bmm350_reg::CHIP_ID)?;
        defmt::info!("BMM350 chip ID after reset: {:#x}", chip_id);
        if chip_id != BMM350_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Power off OTP after boot (OTP is auto-loaded on reset)
        defmt::info!("Powering off OTP...");
        mag.write_reg(bmm350_reg::OTP_CMD_REG, 0x80)?; // OTP_CMD_PWR_OFF_OTP = 0x80
        delay.delay_ms(5);

        // Perform magnetic reset sequence (required for proper operation)
        // 1. First do BR (Bit Reset)
        defmt::info!("Performing magnetic bit reset (BR)...");
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::BIT_RESET)?;
        delay.delay_ms(14);
        mag.wait_pmu_cmd(bmm350_pmu::BIT_RESET, delay)?;

        // 2. Then do FGR (Flux Guide Reset)
        defmt::info!("Performing flux guide reset (FGR)...");
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::FLUX_GUIDE_RESET)?;
        delay.delay_ms(18);
        mag.wait_pmu_cmd(bmm350_pmu::FLUX_GUIDE_RESET, delay)?;

        // Enable all axes (X, Y, Z)
        // PMU_CMD_AXIS_EN: en_x=1 (bit 0), en_y=1 (bit 1), en_z=1 (bit 2)
        defmt::info!("Enabling all axes...");
        mag.write_reg(bmm350_reg::PMU_CMD_AXIS_EN, 0x07)?;
        delay.delay_ms(1);

        // Set ODR to 100Hz with 4x averaging for good noise performance
        // PMU_CMD_AGGR_SET: bits 3:0 = ODR, bits 5:4 = AVG
        let odr_avg = bmm350_odr::ODR_100HZ | bmm350_avg::AVG_4;
        defmt::info!("Setting ODR and averaging: {:#x}", odr_avg);
        mag.write_reg(bmm350_reg::PMU_CMD_AGGR_SET, odr_avg)?;
        delay.delay_ms(1);

        // Send update command to apply ODR/AVG settings
        defmt::info!("Updating OAE settings...");
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::UPDATE_OAE)?;
        delay.delay_ms(2);
        mag.wait_pmu_cmd(bmm350_pmu::UPDATE_OAE, delay)?;

        // Set normal mode for continuous measurement
        defmt::info!("Setting normal mode...");
        mag.write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::NORMAL)?;
        delay.delay_ms(38); // Normal mode startup can take up to 38ms
        mag.wait_pmu_cmd(bmm350_pmu::NORMAL, delay)?;

        // Check PMU status to verify normal mode is active
        let pmu_status = mag.read_reg(bmm350_reg::PMU_CMD_STATUS_0)?;
        defmt::info!(
            "BMM350 PMU status: {:#x} (expect bit 3 set for normal mode)",
            pmu_status
        );

        defmt::info!("BMM350 magnetometer initialized successfully");
        Ok(mag)
    }

    /// Read magnetometer data (all 3 axes with full 24-bit precision)
    pub fn read(&mut self) -> Result<MagData, Error<E>> {
        // Read 9 bytes: X (3), Y (3), Z (3)
        let mut buf = [0u8; 9];
        self.read_regs(bmm350_reg::MAG_X_XLSB, &mut buf)?;

        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let raw_z = (buf[6] as u32) | ((buf[7] as u32) << 8) | ((buf[8] as u32) << 16);

        Ok(MagData {
            x: sign_extend_24bit(raw_x),
            y: sign_extend_24bit(raw_y),
            z: sign_extend_24bit(raw_z),
        })
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}

/// Sign extend a 24-bit value to 32-bit signed integer
fn sign_extend_24bit(value: u32) -> i32 {
    if value & 0x800000 != 0 {
        // Negative number, extend sign bit
        (value | 0xFF000000) as i32
    } else {
        value as i32
    }
}

/// Initialize the IMU with the given I2C peripheral and GPIO pins
///
/// Uses GPIO2 for SDA and GPIO3 for SCL.
pub fn init(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO2<'static>,
    scl: esp_hal::peripherals::GPIO3<'static>,
    delay: &mut impl embedded_hal::delay::DelayNs,
) -> Result<Imu<I2c<'static, Blocking>>, Error<esp_hal::i2c::master::Error>> {
    // Use 400kHz (Fast Mode) - both BMI270 and BMM350 support this
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| Error::I2cCreate)?
    .with_sda(sda)
    .with_scl(scl);

    Imu::new(i2c, delay)
}

/// Wrapper to share I2C bus between IMU and magnetometer using RefCell
pub struct SharedI2c<I2C> {
    i2c: core::cell::RefCell<I2C>,
}

impl<I2C> SharedI2c<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c: core::cell::RefCell::new(i2c),
        }
    }
}

/// IMU that uses shared I2C bus (for when BMM350 is on the same bus)
pub struct SharedImu<'a, I2C> {
    i2c: &'a SharedI2c<I2C>,
}

impl<'a, I2C, E> SharedImu<'a, I2C>
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    fn write_reg(&self, register: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .i2c
            .borrow_mut()
            .write(BMI270_I2C_ADDR, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_reg(&self, register: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];
        self.i2c
            .i2c
            .borrow_mut()
            .write_read(BMI270_I2C_ADDR, &[register], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    fn read_regs(&self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .i2c
            .borrow_mut()
            .write_read(BMI270_I2C_ADDR, &[register], buf)
            .map_err(Error::Bus)
    }

    /// Create and initialize a new shared IMU driver
    pub fn new<D: embedded_hal::delay::DelayNs>(
        i2c: &'a SharedI2c<I2C>,
        delay: &mut D,
    ) -> Result<Self, Error<E>> {
        let imu = Self { i2c };

        // Dummy read to wake up the device (required for I2C after power-on)
        let _ = imu.read_reg(reg::CHIP_ID);
        delay.delay_us(450);

        // Soft reset the device
        imu.write_reg(reg::CMD, 0xB6)?; // SOFT_RESET command
        delay.delay_ms(2);

        // Another dummy read after reset
        let _ = imu.read_reg(reg::CHIP_ID);
        delay.delay_us(450);

        // Verify chip ID
        let chip_id = imu.read_reg(reg::CHIP_ID)?;
        if chip_id != BMI270_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Disable advanced power save before config upload
        imu.write_reg(reg::PWR_CONF, 0x00)?;
        delay.delay_us(450);

        // Prepare for config load - set init_ctrl to 0
        imu.write_reg(reg::INIT_CTRL, 0x00)?;

        // Upload configuration file in small chunks
        let mut offset = 0usize;
        let chunk_size = 32usize;

        while offset < BMI270_CONFIG_FILE.len() {
            // Set the address (divided by 2 as per datasheet)
            let addr_val = (offset / 2) as u16;
            imu.write_reg(reg::INIT_ADDR_0, (addr_val & 0x0F) as u8)?;
            imu.write_reg(reg::INIT_ADDR_1, (addr_val >> 4) as u8)?;

            // Calculate chunk end
            let end = (offset + chunk_size).min(BMI270_CONFIG_FILE.len());
            let chunk = &BMI270_CONFIG_FILE[offset..end];

            // Write chunk: first byte is register address, then data
            let mut buf = [0u8; 33]; // 1 register + 32 data bytes max
            buf[0] = reg::INIT_DATA;
            buf[1..1 + chunk.len()].copy_from_slice(chunk);

            imu.i2c
                .i2c
                .borrow_mut()
                .write(BMI270_I2C_ADDR, &buf[..1 + chunk.len()])
                .map_err(Error::Bus)?;

            offset = end;
        }

        // Start initialization
        imu.write_reg(reg::INIT_CTRL, 0x01)?;

        // Wait for initialization to complete
        let mut init_ok = false;
        for _ in 0..50 {
            delay.delay_ms(10);
            let status = imu.read_reg(reg::INTERNAL_STATUS)?;
            let msg = status & 0x0F;
            if msg == 0x01 {
                init_ok = true;
                break;
            } else if msg >= 0x02 {
                defmt::warn!("BMI270 init error: {}", msg);
                break;
            }
        }

        if !init_ok {
            return Err(Error::InitTimeout);
        }

        // Disable advanced power save mode
        imu.write_reg(reg::PWR_CONF, 0x00)?;
        delay.delay_ms(1);

        // Enable accelerometer and gyroscope (no aux)
        // PWR_CTRL: bit 2 = acc_en, bit 1 = gyr_en
        imu.write_reg(reg::PWR_CTRL, 0x0E)?; // acc_en + gyr_en + temp_en
        delay.delay_ms(50);

        // Configure accelerometer: 100Hz ODR, normal mode, +/-8g
        imu.write_reg(reg::ACC_CONF, 0xA8)?;
        imu.write_reg(reg::ACC_RANGE, 0x02)?;

        // Configure gyroscope: 200Hz ODR, normal mode, +/-2000dps
        imu.write_reg(reg::GYR_CONF, 0xA9)?;
        imu.write_reg(reg::GYR_RANGE, 0x00)?;

        Ok(imu)
    }

    /// Read accelerometer and gyroscope data
    pub fn read(&self) -> Result<ImuData, Error<E>> {
        let mut buf = [0u8; 12];
        self.read_regs(reg::ACC_DATA_0, &mut buf)?;

        Ok(ImuData {
            acc_x: i16::from_le_bytes([buf[0], buf[1]]),
            acc_y: i16::from_le_bytes([buf[2], buf[3]]),
            acc_z: i16::from_le_bytes([buf[4], buf[5]]),
            gyr_x: i16::from_le_bytes([buf[6], buf[7]]),
            gyr_y: i16::from_le_bytes([buf[8], buf[9]]),
            gyr_z: i16::from_le_bytes([buf[10], buf[11]]),
        })
    }

    /// Scan the I2C bus for devices
    pub fn scan_i2c_bus(&self) {
        defmt::info!("Scanning I2C bus for devices...");
        let mut found = 0;
        for addr in 0x03..=0x77 {
            if self.i2c.i2c.borrow_mut().write(addr, &[]).is_ok() {
                defmt::info!("Found device at address {:#x}", addr);
                found += 1;
            }
        }
        defmt::info!("I2C scan complete - found {} devices", found);
    }
}

/// Shared BMM350 magnetometer driver (uses shared I2C bus)
pub struct SharedBmm350<'a, I2C> {
    i2c: &'a SharedI2c<I2C>,
}

impl<'a, I2C, E> SharedBmm350<'a, I2C>
where
    I2C: I2cTrait<Error = E>,
    E: defmt::Format,
{
    fn write_reg(&self, register: u8, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .i2c
            .borrow_mut()
            .write(BMM350_I2C_ADDR, &[register, value])
            .map_err(Error::Bus)
    }

    fn read_reg(&self, register: u8) -> Result<u8, Error<E>> {
        // BMM350 uses 2 dummy bytes protocol - first 2 bytes after register address are dummy
        let mut buf = [0u8; 3];
        self.i2c
            .i2c
            .borrow_mut()
            .write_read(BMM350_I2C_ADDR, &[register], &mut buf)
            .map_err(Error::Bus)?;
        // buf[0], buf[1] are dummy, buf[2] is actual data
        Ok(buf[2])
    }

    fn read_regs(&self, register: u8, buf: &mut [u8]) -> Result<(), Error<E>> {
        // BMM350 uses 2 dummy bytes protocol
        // We need to read len+2 bytes and skip the first 2
        let len = buf.len();
        let mut tmp = [0u8; 18]; // Max we'll ever need (9 bytes data + 2 dummy)
        self.i2c
            .i2c
            .borrow_mut()
            .write_read(BMM350_I2C_ADDR, &[register], &mut tmp[..len + 2])
            .map_err(Error::Bus)?;
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

    /// Read magnetometer data
    pub fn read(&self) -> Result<MagData, Error<E>> {
        let mut buf = [0u8; 9];
        self.read_regs(bmm350_reg::MAG_X_XLSB, &mut buf)?;

        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let raw_z = (buf[6] as u32) | ((buf[7] as u32) << 8) | ((buf[8] as u32) << 16);

        Ok(MagData {
            x: sign_extend_24bit(raw_x),
            y: sign_extend_24bit(raw_y),
            z: sign_extend_24bit(raw_z),
        })
    }
}

/// Initialize IMU and magnetometer on shared I2C bus
///
/// Returns a SharedI2c wrapper, SharedImu, and SharedBmm350.
/// The SharedI2c must be stored in a static or kept alive for the lifetime of the drivers.
pub fn init_shared(
    i2c0: esp_hal::peripherals::I2C0<'static>,
    sda: esp_hal::peripherals::GPIO2<'static>,
    scl: esp_hal::peripherals::GPIO3<'static>,
) -> Result<SharedI2c<I2c<'static, Blocking>>, Error<esp_hal::i2c::master::Error>> {
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| Error::I2cCreate)?
    .with_sda(sda)
    .with_scl(scl);

    Ok(SharedI2c::new(i2c))
}
