// IMU driver wrapper for BMI270 using the bmi2 crate
// Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf

use bmi2::{
    config::BMI270_CONFIG_FILE,
    interface::{I2cAddr, I2cInterface},
    types::{
        AccBwp, AccConf, AccRange, Burst, Data, Error as Bmi2Error, GyrBwp, GyrConf, GyrRange,
        GyrRangeVal, Odr, OisRange, PerfMode, PwrCtrl,
    },
    Bmi2,
};
use embedded_hal::i2c::I2c as I2cTrait;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::peripherals::{GPIO2, GPIO3, I2C0};
use esp_hal::time::Rate;
use esp_hal::Blocking;

/// IMU initialization error
#[derive(Debug)]
pub enum ImuError<E> {
    /// I2C creation error
    I2cCreate,
    /// I2C communication error
    Bus(E),
    /// Chip ID mismatch
    InvalidChipId(u8),
    /// Configuration upload failed
    ConfigFailed(Bmi2Error<E>),
    /// Sensor configuration failed
    SensorConfig(Bmi2Error<E>),
}

/// Initialize the IMU with I2C0 peripheral and GPIO2/GPIO3 pins
pub fn init(
    i2c0: I2C0<'static>,
    sda: GPIO2<'static>,
    scl: GPIO3<'static>,
) -> Result<Imu<I2c<'static, Blocking>>, ImuError<esp_hal::i2c::master::Error>> {
    let i2c = I2c::new(
        i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| ImuError::I2cCreate)?
    .with_sda(sda)
    .with_scl(scl);

    Imu::new(i2c, I2cAddr::Default)
}

/// BMI270 IMU wrapper providing convenient access to accelerometer and gyroscope
pub struct Imu<I2C> {
    bmi: Bmi2<I2cInterface<I2C>>,
}

impl<I2C, E> Imu<I2C>
where
    I2C: I2cTrait<Error = E>,
{
    /// Create and initialize a new IMU driver
    ///
    /// # Arguments
    /// * `i2c` - I2C bus instance
    /// * `addr` - I2C address (Default = 0x68, Alternative = 0x69)
    pub fn new(i2c: I2C, addr: I2cAddr) -> Result<Self, ImuError<E>> {
        let mut bmi = Bmi2::new_i2c(i2c, addr, Burst::default());

        // Verify chip ID
        let chip_id = bmi.get_chip_id().map_err(|e| match e {
            Bmi2Error::Comm(e) => ImuError::Bus(e),
            other => ImuError::SensorConfig(other),
        })?;

        if chip_id != 0x24 {
            return Err(ImuError::InvalidChipId(chip_id));
        }

        // Upload configuration file
        bmi.init(&BMI270_CONFIG_FILE)
            .map_err(ImuError::ConfigFailed)?;

        // Configure accelerometer: 100Hz ODR, normal filter, ±8g range
        bmi.set_acc_conf(AccConf {
            odr: Odr::Odr100,
            bwp: AccBwp::NormAvg4,
            filter_perf: PerfMode::Perf,
        })
        .map_err(ImuError::SensorConfig)?;

        bmi.set_acc_range(AccRange::Range8g)
            .map_err(ImuError::SensorConfig)?;

        // Configure gyroscope: 200Hz ODR, normal filter, ±2000°/s range
        bmi.set_gyr_conf(GyrConf {
            odr: Odr::Odr200,
            bwp: GyrBwp::Norm,
            noise_perf: PerfMode::Perf,
            filter_perf: PerfMode::Perf,
        })
        .map_err(ImuError::SensorConfig)?;

        bmi.set_gyr_range(GyrRange {
            range: GyrRangeVal::Range2000,
            ois_range: OisRange::Range2000,
        })
        .map_err(ImuError::SensorConfig)?;

        // Enable accelerometer and gyroscope
        bmi.set_pwr_ctrl(PwrCtrl {
            aux_en: false,
            gyr_en: true,
            acc_en: true,
            temp_en: false,
        })
        .map_err(ImuError::SensorConfig)?;

        Ok(Self { bmi })
    }

    /// Read accelerometer and gyroscope data
    pub fn read(&mut self) -> Result<Data, ImuError<E>> {
        self.bmi.get_data().map_err(|e| match e {
            Bmi2Error::Comm(e) => ImuError::Bus(e),
            other => ImuError::SensorConfig(other),
        })
    }

    /// Get access to the underlying bmi2 driver for advanced operations
    pub fn inner(&mut self) -> &mut Bmi2<I2cInterface<I2C>> {
        &mut self.bmi
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.bmi.release()
    }
}
