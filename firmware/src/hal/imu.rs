//! IMU driver wrapper for BMI270
//!
//! Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf

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
    /// Configuration upload failed
    ConfigFailed,
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

        // Enable accelerometer and gyroscope
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
    // Use 100kHz for reliable communication
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .map_err(|_| Error::I2cCreate)?
    .with_sda(sda)
    .with_scl(scl);

    Imu::new(i2c, delay)
}
