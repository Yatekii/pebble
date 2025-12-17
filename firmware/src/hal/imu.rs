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
    pub const AUX_DATA_0: u8 = 0x04;
    pub const ACC_DATA_0: u8 = 0x0C;
    pub const INTERNAL_STATUS: u8 = 0x21;
    pub const ACC_CONF: u8 = 0x40;
    pub const ACC_RANGE: u8 = 0x41;
    pub const GYR_CONF: u8 = 0x42;
    pub const GYR_RANGE: u8 = 0x43;
    pub const AUX_CONF: u8 = 0x44;
    pub const AUX_DEV_ID: u8 = 0x4B;
    pub const AUX_IF_CONF: u8 = 0x4C;
    pub const AUX_RD_ADDR: u8 = 0x4D;
    pub const AUX_WR_ADDR: u8 = 0x4E;
    pub const AUX_WR_DATA: u8 = 0x4F;
    pub const INIT_CTRL: u8 = 0x59;
    pub const INIT_ADDR_0: u8 = 0x5B;
    pub const INIT_ADDR_1: u8 = 0x5C;
    pub const INIT_DATA: u8 = 0x5E;
    pub const AUX_IF_TRIM: u8 = 0x68;
    #[allow(dead_code)]
    pub const IF_CONF: u8 = 0x6B;
    pub const PWR_CONF: u8 = 0x7C;
    pub const PWR_CTRL: u8 = 0x7D;
    pub const CMD: u8 = 0x7E;
}

// BMM350 Register addresses (from BST-BMM350-DS001 datasheet)
#[allow(dead_code)]
mod bmm350_reg {
    pub const CHIP_ID: u8 = 0x00;
    pub const REV_ID: u8 = 0x01;
    pub const ERR_REG: u8 = 0x02;
    pub const PAD_CTRL: u8 = 0x03;
    pub const PMU_CMD_AGGR_SET: u8 = 0x04;
    pub const PMU_CMD_AXIS_EN: u8 = 0x05;
    pub const PMU_CMD: u8 = 0x06;
    pub const PMU_CMD_STATUS_0: u8 = 0x07;
    pub const INT_CTRL: u8 = 0x2E;
    pub const INT_STATUS: u8 = 0x30;
    // Magnetic data registers (24-bit per axis, little-endian)
    pub const MAG_X_XLSB: u8 = 0x31;
    pub const MAG_X_LSB: u8 = 0x32;
    pub const MAG_X_MSB: u8 = 0x33;
    pub const MAG_Y_XLSB: u8 = 0x34;
    pub const MAG_Y_LSB: u8 = 0x35;
    pub const MAG_Y_MSB: u8 = 0x36;
    pub const MAG_Z_XLSB: u8 = 0x37;
    pub const MAG_Z_LSB: u8 = 0x38;
    pub const MAG_Z_MSB: u8 = 0x39;
    pub const TEMP_XLSB: u8 = 0x3A;
    pub const OTP_CMD_REG: u8 = 0x50;
    pub const OTP_STATUS_REG: u8 = 0x55;
    pub const CMD: u8 = 0x7E;
}

// BMM350 PMU command values
#[allow(dead_code)]
mod bmm350_pmu {
    pub const SUSPEND: u8 = 0x00;
    pub const NORMAL: u8 = 0x01;
    pub const UPDATE_OAE: u8 = 0x02;
    pub const FORCED_MODE: u8 = 0x03;
    pub const FORCED_MODE_FAST: u8 = 0x04;
    pub const FLUX_GUIDE_RESET: u8 = 0x05;
    pub const FLUX_GUIDE_RESET_FAST: u8 = 0x06;
    pub const BIT_RESET: u8 = 0x07;
    pub const BIT_RESET_FAST: u8 = 0x08;
}

// BMM350 ODR settings (bits 0-3 of PMU_CMD_AGGR_SET)
#[allow(dead_code)]
mod bmm350_odr {
    pub const ODR_400HZ: u8 = 0x02;
    pub const ODR_200HZ: u8 = 0x03;
    pub const ODR_100HZ: u8 = 0x04;
    pub const ODR_50HZ: u8 = 0x05;
    pub const ODR_25HZ: u8 = 0x06;
    pub const ODR_12_5HZ: u8 = 0x07;
    pub const ODR_6_25HZ: u8 = 0x08;
    pub const ODR_3_125HZ: u8 = 0x09;
    pub const ODR_1_5625HZ: u8 = 0x0A;
}

// BMM350 Averaging settings (bits 4-5 of PMU_CMD_AGGR_SET)
#[allow(dead_code)]
mod bmm350_avg {
    pub const NO_AVG: u8 = 0x00 << 4;
    pub const AVG_2: u8 = 0x01 << 4;
    pub const AVG_4: u8 = 0x02 << 4;
    pub const AVG_8: u8 = 0x03 << 4;
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

    // === Auxiliary interface methods for BMM350 magnetometer ===

    /// Write to auxiliary device register via BMI270
    fn aux_write_reg<D: embedded_hal::delay::DelayNs>(
        &mut self,
        aux_reg: u8,
        value: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Set the register address to write to
        self.write_reg(reg::AUX_WR_ADDR, aux_reg)?;
        // Set the data to write
        self.write_reg(reg::AUX_WR_DATA, value)?;
        // Wait for write to complete
        delay.delay_us(500);
        Ok(())
    }

    /// Wait for AUX interface to not be busy
    fn wait_aux_not_busy<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        for _ in 0..20 {
            let status = self.read_reg(reg::STATUS)?;
            // AUX_BUSY is bit 2 (0x04)
            if (status & 0x04) == 0 {
                return Ok(());
            }
            delay.delay_ms(10);
        }
        defmt::warn!("AUX busy timeout");
        Ok(())
    }

    /// Read from auxiliary device register via BMI270 (manual mode)
    ///
    /// According to BMM350 datasheet section 5.3.2, the first read after setting
    /// the register address returns dummy data. We need to do a second read to
    /// get actual data. But with BMI270 aux interface, we can read multiple bytes.
    fn aux_read_reg<D: embedded_hal::delay::DelayNs>(
        &mut self,
        aux_reg: u8,
        delay: &mut D,
    ) -> Result<u8, Error<E>> {
        // Wait for AUX to not be busy
        self.wait_aux_not_busy(delay)?;

        // Set the register address to read from - this triggers the read
        self.write_reg(reg::AUX_RD_ADDR, aux_reg)?;
        delay.delay_ms(5); // Give more time for the read to complete

        // Wait for AUX to not be busy (read complete)
        self.wait_aux_not_busy(delay)?;

        // Check drdy_aux bit in STATUS
        let status = self.read_reg(reg::STATUS)?;
        defmt::info!(
            "AUX read status for reg {:#x}: {:#x} (drdy_aux={}, aux_busy={})",
            aux_reg,
            status,
            (status >> 4) & 1,
            (status >> 2) & 1
        );

        // Read all 8 AUX_DATA registers to see what we get
        let mut buf = [0u8; 8];
        self.read_regs(reg::AUX_DATA_0, &mut buf)?;
        defmt::info!("AUX raw data: {:?}", buf);

        // Check the ERR_REG for any errors
        let err_reg = self.read_reg(reg::ERR_REG)?;
        if err_reg != 0 {
            defmt::warn!("BMI270 ERR_REG: {:#x}", err_reg);
        }

        // According to BMI270 datasheet, AUX_DATA_0 contains the first byte read
        // from the auxiliary device. No dummy byte handling in the BMI270 interface.
        // The dummy bytes are a BMM350 SPI-specific behavior.
        let data = buf[0];
        Ok(data)
    }

    /// Read multiple bytes from auxiliary device via BMI270 (manual mode)
    #[allow(dead_code)]
    fn aux_read_regs<D: embedded_hal::delay::DelayNs>(
        &mut self,
        aux_reg: u8,
        buf: &mut [u8],
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        // Determine burst length setting
        let burst_bits = match buf.len() {
            1 => 0x00,     // 1 byte
            2 => 0x01,     // 2 bytes
            3..=6 => 0x02, // 6 bytes
            _ => 0x03,     // 8 bytes
        };

        // Enable manual mode with appropriate burst
        // AUX_IF_CONF: aux_manual_en=1 (bit 7), man_read_burst in bits 3:2
        self.write_reg(reg::AUX_IF_CONF, 0x80 | (burst_bits << 2))?;

        // Set the register address to read from
        self.write_reg(reg::AUX_RD_ADDR, aux_reg)?;

        // Wait for read to complete
        delay.delay_us(500);

        // Read the data from AUX_DATA registers
        self.read_regs(reg::AUX_DATA_0, buf)?;
        Ok(())
    }

    /// Wait for PMU command to complete on BMM350
    fn bmm350_wait_pmu_cmd<D: embedded_hal::delay::DelayNs>(
        &mut self,
        expected_cmd: u8,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        for _ in 0..20 {
            delay.delay_ms(2);
            let status = self.aux_read_reg(bmm350_reg::PMU_CMD_STATUS_0, delay)?;
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

    /// Scan the primary I2C bus for devices
    pub fn scan_i2c_bus(&mut self) {
        defmt::info!("Scanning primary I2C bus for devices (full range)...");
        let mut found = 0;
        for addr in 0x03..=0x77 {
            // Try a simple write with no data - some devices respond to this
            if self.i2c.write(addr, &[]).is_ok() {
                defmt::info!("Found device at address {:#x} (write)", addr);
                found += 1;
            } else {
                // Also try read
                let mut buf = [0u8; 1];
                if self.i2c.read(addr, &mut buf).is_ok() {
                    defmt::info!("Found device at address {:#x} (read)", addr);
                    found += 1;
                }
            }
        }
        defmt::info!("I2C scan complete - found {} devices", found);
    }

    /// Try to read BMM350 chip ID directly via I2C (not through auxiliary interface)
    pub fn read_bmm350_direct(&mut self) -> Result<u8, Error<E>> {
        // Try reading chip ID register (0x00) directly from BMM350 at address 0x14
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(BMM350_I2C_ADDR, &[0x00], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    /// Initialize BMM350 magnetometer via auxiliary interface
    pub fn init_magnetometer<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(), Error<E>> {
        defmt::info!("=== Initializing BMM350 via AUX I2C ===");

        // 1. Disable advanced power save mode (required for aux interface)
        defmt::info!("Step 1: Disable power save");
        self.write_reg(reg::PWR_CONF, 0x00)?;
        delay.delay_ms(1);

        // 2. Enable auxiliary power in PWR_CTRL (bit 0 = aux_en)
        defmt::info!("Step 2: Enable AUX power");
        let pwr = self.read_reg(reg::PWR_CTRL)?;
        defmt::info!("  PWR_CTRL before: {:#x}", pwr);
        self.write_reg(reg::PWR_CTRL, pwr | 0x01)?;
        delay.delay_ms(10);
        let pwr_after = self.read_reg(reg::PWR_CTRL)?;
        defmt::info!("  PWR_CTRL after: {:#x}", pwr_after);

        // 3. Configure AUX interface pull-ups (stronger pull-up for reliable communication)
        defmt::info!("Step 3: Configure pull-ups");
        let trim = self.read_reg(reg::AUX_IF_TRIM)?;
        defmt::info!("  AUX_IF_TRIM before: {:#x}", trim);
        // bits 1:0 = asda_pupsel: 00=off, 01=40k, 10=10k, 11=2k
        // Also set bits 3:2 for SCL pull-up (ascx_pupsel)
        self.write_reg(reg::AUX_IF_TRIM, (trim & 0xF0) | 0x0F)?; // 2k pull-up on both SDA and SCL
        delay.delay_ms(5);
        let trim_after = self.read_reg(reg::AUX_IF_TRIM)?;
        defmt::info!("  AUX_IF_TRIM after: {:#x}", trim_after);

        // 4. Set BMM350 I2C address (0x14 << 1 = 0x28)
        defmt::info!("Step 4: Set AUX device address to 0x14 (shifted: 0x28)");
        self.write_reg(reg::AUX_DEV_ID, BMM350_I2C_ADDR << 1)?;
        delay.delay_ms(1);

        // 5. Configure AUX interface: manual mode, FCU write enable, 8-byte burst
        defmt::info!("Step 5: Enable manual mode with FCU write");
        // AUX_IF_CONF register bits:
        //   bit 7 = aux_manual_en (1 = manual mode)
        //   bit 6 = aux_fcu_write_en (1 = enable FCU write commands)
        //   bits 3:2 = man_rd_burst (burst length for manual mode: 0=1, 1=2, 2=6, 3=8 bytes)
        //   bits 1:0 = aux_rd_burst (burst length for data mode: 0=1, 1=2, 2=6, 3=8 bytes)
        // Set: manual_en=1, fcu_write_en=1, man_rd_burst=3 (8 bytes), aux_rd_burst=3 (8 bytes)
        self.write_reg(reg::AUX_IF_CONF, 0x80 | 0x40 | (0x03 << 2) | 0x03)?;
        delay.delay_ms(1);

        // 6. Configure AUX_CONF with ODR
        defmt::info!("Step 6: Configure AUX_CONF ODR");
        // AUX_CONF register (0x44): bits 3:0 = aux_odr
        // ODR values: 0x05=50Hz, 0x06=100Hz, 0x07=200Hz, 0x08=400Hz
        self.write_reg(reg::AUX_CONF, 0x06)?; // 100Hz
        delay.delay_ms(1);

        // 7. Wait for BMM350 startup (3ms from power-on)
        defmt::info!("Step 7: Wait for BMM350 startup (5ms)");
        delay.delay_ms(5);

        // 7. Check BMI270 status and error registers
        let status = self.read_reg(reg::STATUS)?;
        let err = self.read_reg(reg::ERR_REG)?;
        let if_conf = self.read_reg(reg::IF_CONF)?;
        let aux_if_conf = self.read_reg(reg::AUX_IF_CONF)?;
        let aux_dev_id = self.read_reg(reg::AUX_DEV_ID)?;
        let pwr_ctrl = self.read_reg(reg::PWR_CTRL)?;
        let pwr_conf = self.read_reg(reg::PWR_CONF)?;

        defmt::info!("  STATUS: {:#x}, ERR_REG: {:#x}", status, err);
        defmt::info!("  IF_CONF: {:#x}, AUX_IF_CONF: {:#x}", if_conf, aux_if_conf);
        defmt::info!(
            "  AUX_DEV_ID: {:#x}, PWR_CTRL: {:#x}, PWR_CONF: {:#x}",
            aux_dev_id,
            pwr_ctrl,
            pwr_conf
        );
        defmt::info!(
            "  aux_busy={}, drdy_aux={}, aux_err={}",
            (status >> 2) & 1,
            (status >> 4) & 1,
            (err >> 7) & 1
        );

        // 8. Try to read chip ID from BMM350
        defmt::info!("Step 8: Reading BMM350 chip ID...");

        // Clear any previous error
        let _ = self.read_reg(reg::ERR_REG)?;

        // Set read address to chip ID register (0x00)
        self.write_reg(reg::AUX_RD_ADDR, 0x00)?;
        delay.delay_ms(5);

        // Check status after triggering read
        let status = self.read_reg(reg::STATUS)?;
        let err = self.read_reg(reg::ERR_REG)?;
        defmt::info!(
            "  After read trigger - STATUS: {:#x}, ERR_REG: {:#x}",
            status,
            err
        );

        // Read the data
        let mut buf = [0u8; 8];
        self.read_regs(reg::AUX_DATA_0, &mut buf)?;
        defmt::info!("  AUX_DATA: {:?}", buf);

        let chip_id = buf[0];
        defmt::info!(
            "  Chip ID read: {:#x} (expected: {:#x})",
            chip_id,
            BMM350_CHIP_ID
        );

        // If aux_err is set, scan for devices
        if (err & 0x80) != 0 {
            defmt::warn!("AUX error detected - BMM350 not responding at 0x14");
            defmt::info!("Scanning AUX I2C bus for devices (0x08-0x77)...");

            for addr in 0x08u8..=0x77 {
                // Set address (shifted left by 1)
                self.write_reg(reg::AUX_DEV_ID, addr << 1)?;
                delay.delay_ms(1);

                // Clear error
                let _ = self.read_reg(reg::ERR_REG)?;

                // Trigger read of register 0x00
                self.write_reg(reg::AUX_RD_ADDR, 0x00)?;
                delay.delay_ms(3);

                // Check for error
                let scan_err = self.read_reg(reg::ERR_REG)?;
                if (scan_err & 0x80) == 0 {
                    let mut scan_buf = [0u8; 1];
                    self.read_regs(reg::AUX_DATA_0, &mut scan_buf)?;
                    defmt::info!("  Found device at {:#x}, chip_id={:#x}", addr, scan_buf[0]);
                }
            }

            return Err(Error::InvalidChipId(chip_id));
        }

        if chip_id != BMM350_CHIP_ID {
            defmt::warn!(
                "Wrong chip ID: {:#x}, expected {:#x}",
                chip_id,
                BMM350_CHIP_ID
            );
            return Err(Error::InvalidChipId(chip_id));
        }

        defmt::info!("BMM350 chip ID verified: {:#x}", chip_id);

        // Soft reset BMM350 via CMD register (0x7E)
        defmt::info!("Performing BMM350 soft reset...");
        self.aux_write_reg(bmm350_reg::CMD, BMM350_SOFT_RESET, delay)?;
        delay.delay_ms(24); // BMM350 needs ~24ms after soft reset

        // Verify chip ID again after reset
        let chip_id = self.aux_read_reg(bmm350_reg::CHIP_ID, delay)?;
        defmt::info!("BMM350 chip ID after reset: {:#x}", chip_id);
        if chip_id != BMM350_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Power off OTP after boot (OTP is auto-loaded on reset)
        defmt::info!("Powering off OTP...");
        self.aux_write_reg(bmm350_reg::OTP_CMD_REG, 0x80, delay)?; // OTP_CMD_PWR_OFF_OTP = 0x80
        delay.delay_ms(5);

        // Perform magnetic reset sequence (required for proper operation)
        // 1. First do BR (Bit Reset)
        defmt::info!("Performing magnetic bit reset (BR)...");
        self.aux_write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::BIT_RESET, delay)?;
        delay.delay_ms(14);
        self.bmm350_wait_pmu_cmd(bmm350_pmu::BIT_RESET, delay)?;

        // 2. Then do FGR (Flux Guide Reset)
        defmt::info!("Performing flux guide reset (FGR)...");
        self.aux_write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::FLUX_GUIDE_RESET, delay)?;
        delay.delay_ms(18);
        self.bmm350_wait_pmu_cmd(bmm350_pmu::FLUX_GUIDE_RESET, delay)?;

        // Enable all axes (X, Y, Z)
        // PMU_CMD_AXIS_EN: en_x=1 (bit 0), en_y=1 (bit 1), en_z=1 (bit 2)
        defmt::info!("Enabling all axes...");
        self.aux_write_reg(bmm350_reg::PMU_CMD_AXIS_EN, 0x07, delay)?;
        delay.delay_ms(1);

        // Set ODR to 100Hz with 4x averaging for good noise performance
        // PMU_CMD_AGGR_SET: bits 3:0 = ODR, bits 5:4 = AVG
        let odr_avg = bmm350_odr::ODR_100HZ | bmm350_avg::AVG_4;
        defmt::info!("Setting ODR and averaging: {:#x}", odr_avg);
        self.aux_write_reg(bmm350_reg::PMU_CMD_AGGR_SET, odr_avg, delay)?;
        delay.delay_ms(1);

        // Send update command to apply ODR/AVG settings
        defmt::info!("Updating OAE settings...");
        self.aux_write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::UPDATE_OAE, delay)?;
        delay.delay_ms(2);
        self.bmm350_wait_pmu_cmd(bmm350_pmu::UPDATE_OAE, delay)?;

        // Set normal mode for continuous measurement
        defmt::info!("Setting normal mode...");
        self.aux_write_reg(bmm350_reg::PMU_CMD, bmm350_pmu::NORMAL, delay)?;
        delay.delay_ms(38); // Normal mode startup can take up to 38ms
        self.bmm350_wait_pmu_cmd(bmm350_pmu::NORMAL, delay)?;

        // Check PMU status to verify normal mode is active
        let pmu_status = self.aux_read_reg(bmm350_reg::PMU_CMD_STATUS_0, delay)?;
        defmt::info!(
            "BMM350 PMU status: {:#x} (expect bit 3 set for normal mode)",
            pmu_status
        );

        // Configure BMI270 to automatically read magnetometer data
        // AUX_CONF: aux_odr = 0x08 (100Hz)
        self.write_reg(reg::AUX_CONF, 0x08)?;

        // Set read address for automatic mode
        // BMM350 data starts at MAG_X_XLSB = 0x31, but we need to account for 2 dummy bytes
        // So we read starting at 0x31 and get: [dummy0, dummy1, X_XLSB, X_LSB, X_MSB, Y_XLSB, Y_LSB, Y_MSB]
        self.write_reg(reg::AUX_RD_ADDR, bmm350_reg::MAG_X_XLSB)?;

        // Switch to automatic mode with 8-byte burst (maximum supported)
        // AUX_IF_CONF: aux_manual_en=0 (bit 7), aux_read_burst=3 (8 bytes, bits 1:0)
        self.write_reg(reg::AUX_IF_CONF, 0x03)?;

        defmt::info!("BMM350 magnetometer initialized successfully");
        Ok(())
    }

    /// Read magnetometer data from auxiliary data registers
    ///
    /// The BMI270 automatically reads 8 bytes from BMM350 starting at MAG_X_XLSB (0x31).
    /// Via I2C (not SPI), there are no dummy bytes.
    /// We get: [X_XLSB, X_LSB, X_MSB, Y_XLSB, Y_LSB, Y_MSB, Z_XLSB, Z_LSB]
    /// Note: We only get 2 bytes of Z data (missing Z_MSB) with 8-byte burst.
    pub fn read_mag(&mut self) -> Result<MagData, Error<E>> {
        let mut buf = [0u8; 8];
        self.read_regs(reg::AUX_DATA_0, &mut buf)?;

        // I2C mode: no dummy bytes, data starts immediately
        // buf[0], buf[1], buf[2] = X (XLSB, LSB, MSB)
        // buf[3], buf[4], buf[5] = Y (XLSB, LSB, MSB)
        // buf[6], buf[7] = Z (XLSB, LSB) - missing MSB

        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        // For Z we only have 2 bytes, so we'll need read_mag_full for accurate Z
        let raw_z = (buf[6] as u32) | ((buf[7] as u32) << 8);

        Ok(MagData {
            x: sign_extend_24bit(raw_x),
            y: sign_extend_24bit(raw_y),
            z: (raw_z as i16) as i32, // Only 16-bit for now
        })
    }

    /// Read magnetometer data with all 3 axes (uses manual mode for Z MSB)
    pub fn read_mag_full<D: embedded_hal::delay::DelayNs>(
        &mut self,
        delay: &mut D,
    ) -> Result<MagData, Error<E>> {
        // Read from automatic mode buffer
        let mut buf = [0u8; 8];
        self.read_regs(reg::AUX_DATA_0, &mut buf)?;

        // I2C mode: no dummy bytes
        // buf[0], buf[1], buf[2] = X (XLSB, LSB, MSB)
        // buf[3], buf[4], buf[5] = Y (XLSB, LSB, MSB)
        // buf[6], buf[7] = Z (XLSB, LSB) - missing MSB
        let raw_x = (buf[0] as u32) | ((buf[1] as u32) << 8) | ((buf[2] as u32) << 16);
        let raw_y = (buf[3] as u32) | ((buf[4] as u32) << 8) | ((buf[5] as u32) << 16);
        let raw_z_partial = (buf[6] as u32) | ((buf[7] as u32) << 8);

        // Now do a manual read to get Z MSB
        // Switch to manual mode with 1-byte burst
        self.write_reg(reg::AUX_IF_CONF, 0x80)?; // manual mode, 1 byte
        delay.delay_us(100);

        // Read Z_MSB (0x39)
        self.write_reg(reg::AUX_RD_ADDR, bmm350_reg::MAG_Z_MSB)?;
        delay.delay_ms(2);

        let mut z_msb_buf = [0u8; 1];
        self.read_regs(reg::AUX_DATA_0, &mut z_msb_buf)?;

        let raw_z = raw_z_partial | ((z_msb_buf[0] as u32) << 16);

        // Switch back to automatic mode
        self.write_reg(reg::AUX_RD_ADDR, bmm350_reg::MAG_X_XLSB)?;
        self.write_reg(reg::AUX_IF_CONF, 0x03)?; // automatic mode, 8 bytes

        Ok(MagData {
            x: sign_extend_24bit(raw_x),
            y: sign_extend_24bit(raw_y),
            z: sign_extend_24bit(raw_z),
        })
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
