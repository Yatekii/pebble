use bmi2::config::BMI270_CONFIG_FILE;
use embedded_hal::i2c::I2c;
use nalgebra::Vector3;
use uom::si::f32::{Acceleration, AngularVelocity};

use crate::{
    hal::{
        imu::constants::{BMI270_CHIP_ID, BMI270_I2C_ADDR, bmi270_reg},
        peripherals::i2c::{Error, SharedI2c, SharedI2cDevice},
    },
    math::rotate_90ccw,
};

/// IMU that uses shared I2C bus (for when BMM350 is on the same bus)
pub struct SharedBmi270<'a, I2C> {
    i2c: SharedI2cDevice<'a, BMI270_I2C_ADDR, I2C>,
}

impl<'a, I2C, E> SharedBmi270<'a, I2C>
where
    I2C: I2c<Error = E>,
    E: defmt::Format,
{
    /// Create and initialize a new shared IMU driver
    pub fn new<D: embedded_hal::delay::DelayNs>(
        i2c: &'a SharedI2c<I2C>,
        delay: &mut D,
    ) -> Result<Self, Error<E>> {
        let i2c = i2c.device_with_address::<BMI270_I2C_ADDR>();
        let imu = Self { i2c };

        // Dummy read to wake up the device (required for I2C after power-on)
        let _ = imu.i2c.read_reg(bmi270_reg::CHIP_ID);
        delay.delay_us(450);

        // Soft reset the device
        imu.i2c.write_reg(bmi270_reg::CMD, 0xB6)?; // SOFT_RESET command
        delay.delay_ms(2);

        // Another dummy read after reset
        let _ = imu.i2c.read_reg(bmi270_reg::CHIP_ID);
        delay.delay_us(450);

        // Verify chip ID
        let chip_id = imu.i2c.read_reg(bmi270_reg::CHIP_ID)?;
        if chip_id != BMI270_CHIP_ID {
            return Err(Error::InvalidChipId(chip_id));
        }

        // Disable advanced power save before config upload
        imu.i2c.write_reg(bmi270_reg::PWR_CONF, 0x00)?;
        delay.delay_us(450);

        // Prepare for config load - set init_ctrl to 0
        imu.i2c.write_reg(bmi270_reg::INIT_CTRL, 0x00)?;

        // Upload configuration file in small chunks
        let mut offset = 0usize;
        let chunk_size = 32usize;

        while offset < BMI270_CONFIG_FILE.len() {
            // Set the address (divided by 2 as per datasheet)
            let addr_val = (offset / 2) as u16;
            imu.i2c
                .write_reg(bmi270_reg::INIT_ADDR_0, (addr_val & 0x0F) as u8)?;
            imu.i2c
                .write_reg(bmi270_reg::INIT_ADDR_1, (addr_val >> 4) as u8)?;

            // Calculate chunk end
            let end = (offset + chunk_size).min(BMI270_CONFIG_FILE.len());
            let chunk = &BMI270_CONFIG_FILE[offset..end];

            // Write chunk: first byte is register address, then data
            let mut buf = [0u8; 33]; // 1 register + 32 data bytes max
            buf[0] = bmi270_reg::INIT_DATA;
            buf[1..1 + chunk.len()].copy_from_slice(chunk);

            imu.i2c.write(&buf[..1 + chunk.len()])?;

            offset = end;
        }

        // Start initialization
        imu.i2c.write_reg(bmi270_reg::INIT_CTRL, 0x01)?;

        // Wait for initialization to complete
        let mut init_ok = false;
        for _ in 0..50 {
            delay.delay_ms(10);
            let status = imu.i2c.read_reg(bmi270_reg::INTERNAL_STATUS)?;
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
        imu.i2c.write_reg(bmi270_reg::PWR_CONF, 0x00)?;
        delay.delay_ms(1);

        // Enable accelerometer and gyroscope (no aux)
        // PWR_CTRL: bit 2 = acc_en, bit 1 = gyr_en
        imu.i2c.write_reg(bmi270_reg::PWR_CTRL, 0x0E)?; // acc_en + gyr_en + temp_en
        delay.delay_ms(50);

        // Configure accelerometer: 100Hz ODR, normal mode, +/-8g
        imu.i2c.write_reg(bmi270_reg::ACC_CONF, 0xA8)?;
        imu.i2c.write_reg(bmi270_reg::ACC_RANGE, 0x02)?;

        // Configure gyroscope: 200Hz ODR, normal mode, +/-2000dps
        imu.i2c.write_reg(bmi270_reg::GYR_CONF, 0xA9)?;
        imu.i2c.write_reg(bmi270_reg::GYR_RANGE, 0x00)?;

        Ok(imu)
    }

    /// Read accelerometer and gyroscope data
    pub fn read(&self) -> Result<ImuData, Error<E>> {
        let mut buf = [0u8; 12];
        self.i2c.read_regs(bmi270_reg::ACC_DATA_0, &mut buf)?;

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
            if self.i2c.write(&[]).is_ok() {
                defmt::info!("Found device at address {:#x}", addr);
                found += 1;
            }
        }
        defmt::info!("I2C scan complete - found {} devices", found);
    }
}

/// Accelerometer and gyroscope data
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    acc_x: i16,
    acc_y: i16,
    acc_z: i16,
    gyr_x: i16,
    gyr_y: i16,
    gyr_z: i16,
}

impl ImuData {
    /// Return the transformed and calibrated accelerometer reading
    /// Convert accelerometer readings to g (assuming +/-8g range)s
    pub fn acceleration(&self) -> Vector3<Acceleration> {
        /// BMI270 with 8g range: 1g = 4096 LSB
        const SCALE: f32 = 1.0 / 4096.0;

        rotate_90ccw(Vector3::new(self.acc_x as f32, self.acc_y as f32, self.acc_z as f32) * SCALE)
            .map(Acceleration::new::<uom::si::acceleration::meter_per_second_squared>)
    }

    /// Return the transformed and calibrated gyroscope reading
    pub fn angular_velocity(&self) -> Vector3<AngularVelocity> {
        /// BMI270 with 2000dps range: 1 dps = 16.384 LSB
        const SCALE: f32 = 1.0 / 16.384;

        rotate_90ccw(Vector3::new(self.gyr_x as f32, self.gyr_y as f32, self.gyr_z as f32) * SCALE)
            .map(AngularVelocity::new::<uom::si::angular_velocity::degree_per_second>)
    }

    /// Return the raw accelerometer reading
    pub fn raw_acceleration(&self) -> Vector3<i16> {
        Vector3::new(self.acc_x, self.acc_y, self.acc_z)
    }

    /// Return the raw gyroscope reading
    pub fn raw_angular_velocity(&self) -> Vector3<i16> {
        Vector3::new(self.gyr_x, self.gyr_y, self.gyr_z)
    }
}
