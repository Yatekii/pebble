use defmt::Format;
use embedded_hal::i2c::ErrorType;

/// Wrapper to share I2C bus between IMU and magnetometer using RefCell
pub struct SharedI2c<I2C> {
    i2c: core::cell::RefCell<I2C>,
}

/// Wrapper to share I2C bus between IMU and magnetometer using RefCell
pub struct SharedI2cDevice<'a, const A: u8, I2C> {
    bus: &'a SharedI2c<I2C>,
}

impl<I2C> SharedI2c<I2C>
where
    I2C: embedded_hal::i2c::I2c,
    <I2C as ErrorType>::Error: Format,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c: core::cell::RefCell::new(i2c),
        }
    }

    pub fn device_with_address<'a, const A: u8>(&'a self) -> SharedI2cDevice<'a, A, I2C> {
        SharedI2cDevice { bus: &self }
    }

    pub fn write(&self, address: u8, buf: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c
            .borrow_mut()
            .write(address, buf)
            .map_err(Error::Bus)
    }
}

impl<'a, const A: u8, I2C> SharedI2cDevice<'a, A, I2C>
where
    I2C: embedded_hal::i2c::I2c,
    <I2C as ErrorType>::Error: Format,
{
    pub fn write(&self, buf: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.bus.i2c.borrow_mut().write(A, buf).map_err(Error::Bus)
    }

    pub fn write_reg(&self, register: u8, value: u8) -> Result<(), Error<I2C::Error>> {
        self.bus
            .i2c
            .borrow_mut()
            .write(A, &[register, value])
            .map_err(Error::Bus)
    }

    pub fn read_reg(&self, register: u8) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0u8];
        self.bus
            .i2c
            .borrow_mut()
            .write_read(A, &[register], &mut buf)
            .map_err(Error::Bus)?;
        Ok(buf[0])
    }

    pub fn read_regs(&self, register: u8, buf: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.bus
            .i2c
            .borrow_mut()
            .write_read(A, &[register], buf)
            .map_err(Error::Bus)
    }
}

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
