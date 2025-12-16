use rand::Rng;

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
    /// Time counter for generating dummy data.
    time: f64,
}

impl ImuHistory {
    pub fn new(max_samples: usize) -> Self {
        Self {
            accel: Vec::with_capacity(max_samples),
            gyro: Vec::with_capacity(max_samples),
            mag: Vec::with_capacity(max_samples),
            max_samples,
            time: 0.0,
        }
    }

    /// Generate and add a new dummy reading.
    pub fn generate_dummy_reading(&mut self) {
        let mut rng = rand::thread_rng();

        // Simulate accelerometer with gravity on Z axis plus noise
        let accel = ImuReading {
            x: rng.gen_range(-0.1..0.1) + 0.05 * (self.time * 2.0).sin(),
            y: rng.gen_range(-0.1..0.1) + 0.05 * (self.time * 1.5).cos(),
            z: 1.0 + rng.gen_range(-0.05..0.05) + 0.1 * (self.time * 0.5).sin(),
        };

        // Simulate gyroscope with slow oscillations
        let gyro = ImuReading {
            x: rng.gen_range(-5.0..5.0) + 20.0 * (self.time * 0.8).sin(),
            y: rng.gen_range(-5.0..5.0) + 15.0 * (self.time * 1.2).cos(),
            z: rng.gen_range(-5.0..5.0) + 10.0 * (self.time * 0.3).sin(),
        };

        // Simulate magnetometer (Earth's magnetic field is ~25-65 microtesla)
        let mag = ImuReading {
            x: 30.0 + rng.gen_range(-2.0..2.0) + 5.0 * (self.time * 0.2).sin(),
            y: -10.0 + rng.gen_range(-2.0..2.0) + 3.0 * (self.time * 0.4).cos(),
            z: 45.0 + rng.gen_range(-2.0..2.0) + 4.0 * (self.time * 0.1).sin(),
        };

        self.push(accel, gyro, mag);
        self.time += 0.05; // 50ms per sample = 20 Hz
    }

    /// Add a new reading, removing old ones if at capacity.
    pub fn push(&mut self, accel: ImuReading, gyro: ImuReading, mag: ImuReading) {
        if self.accel.len() >= self.max_samples {
            self.accel.remove(0);
            self.gyro.remove(0);
            self.mag.remove(0);
        }
        self.accel.push(accel);
        self.gyro.push(gyro);
        self.mag.push(mag);
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
