/// A single 3-axis IMU reading.
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuReading {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Stores the history of IMU readings for plotting.
pub struct ImuHistory {
    /// Accelerometer readings (raw i16 values from sensor).
    pub accel: Vec<ImuReading>,
    /// Gyroscope readings (raw i16 values from sensor).
    pub gyro: Vec<ImuReading>,
    /// Magnetometer readings (raw i32 values from sensor).
    pub mag: Vec<ImuReading>,
    /// Maximum number of samples to keep.
    max_samples: usize,
}

impl ImuHistory {
    pub fn new(max_samples: usize) -> Self {
        Self {
            accel: Vec::with_capacity(max_samples),
            gyro: Vec::with_capacity(max_samples),
            mag: Vec::with_capacity(max_samples),
            max_samples,
        }
    }

    /// Push accelerometer reading.
    pub fn push_accel(&mut self, accel: ImuReading) {
        if self.accel.len() >= self.max_samples {
            self.accel.remove(0);
        }
        self.accel.push(accel);
    }

    /// Push gyroscope reading.
    pub fn push_gyro(&mut self, gyro: ImuReading) {
        if self.gyro.len() >= self.max_samples {
            self.gyro.remove(0);
        }
        self.gyro.push(gyro);
    }

    /// Push magnetometer reading.
    pub fn push_mag(&mut self, mag: ImuReading) {
        if self.mag.len() >= self.max_samples {
            self.mag.remove(0);
        }
        self.mag.push(mag);
    }

    pub fn len(&self) -> usize {
        self.accel.len()
    }
}
