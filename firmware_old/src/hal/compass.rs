// Magnetometer driver wrapper for BMM350 using the bmm350 crate
// Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf
//
// Note: The BMM350 is connected via the BMI270's I2C passthrough mode.
// The BMI270 must be configured to enable auxiliary interface before the BMM350 can be accessed.
//
// Usage: Import bmm350 crate directly and use Bmm350::new_with_i2c() to create the driver.
// The bmm350 crate's interface module is private, so we cannot wrap it in a custom struct.
//
// Example:
// ```
// use bmm350::{Bmm350, DataRate, MagConfig, PerformanceMode, PowerMode};
//
// let mut bmm = Bmm350::new_with_i2c(i2c, 0x14, delay);
// bmm.init().unwrap();
// let config = MagConfig::builder()
//     .odr(DataRate::ODR100Hz)
//     .performance(PerformanceMode::Regular)
//     .mode(PowerMode::Normal)
//     .build();
// bmm.set_mag_config(config).unwrap();
// let data = bmm.read_mag_data().unwrap();
// ```

// Re-export commonly used types from bmm350 for convenience
#[allow(unused_imports)]
pub use bmm350::{Bmm350, DataRate, MagConfig, PerformanceMode, PowerMode, Sensor3DData};

/// Default BMM350 I2C address
pub const BMM350_ADDR: u8 = 0x14;
