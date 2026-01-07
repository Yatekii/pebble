use std::collections::VecDeque;

/// A single 3-axis IMU reading.
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuReading {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A single AHRS orientation reading (roll, pitch, yaw in degrees).
#[derive(Clone, Copy, Debug, Default)]
pub struct AhrsReading {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

/// Stores the history of IMU readings for plotting.
/// Uses VecDeque for O(1) push/pop operations instead of O(n) Vec::remove(0).
pub struct ImuHistory {
    /// Accelerometer readings (raw i16 values from sensor).
    pub accel: VecDeque<ImuReading>,
    /// Gyroscope readings (raw i16 values from sensor).
    pub gyro: VecDeque<ImuReading>,
    /// Magnetometer readings (raw i32 values from sensor).
    pub mag: VecDeque<ImuReading>,
    /// AHRS orientation readings (roll, pitch, yaw in degrees).
    pub ahrs: VecDeque<AhrsReading>,
    /// Maximum number of samples to keep.
    max_samples: usize,
}

impl ImuHistory {
    pub fn new(max_samples: usize) -> Self {
        Self {
            accel: VecDeque::with_capacity(max_samples),
            gyro: VecDeque::with_capacity(max_samples),
            mag: VecDeque::with_capacity(max_samples),
            ahrs: VecDeque::with_capacity(max_samples),
            max_samples,
        }
    }

    /// Push accelerometer reading.
    pub fn push_accel(&mut self, accel: ImuReading) {
        if self.accel.len() >= self.max_samples {
            self.accel.pop_front();
        }
        self.accel.push_back(accel);
    }

    /// Push gyroscope reading.
    pub fn push_gyro(&mut self, gyro: ImuReading) {
        if self.gyro.len() >= self.max_samples {
            self.gyro.pop_front();
        }
        self.gyro.push_back(gyro);
    }

    /// Push magnetometer reading.
    pub fn push_mag(&mut self, mag: ImuReading) {
        if self.mag.len() >= self.max_samples {
            self.mag.pop_front();
        }
        self.mag.push_back(mag);
    }

    /// Push AHRS orientation reading.
    pub fn push_ahrs(&mut self, ahrs: AhrsReading) {
        if self.ahrs.len() >= self.max_samples {
            self.ahrs.pop_front();
        }
        self.ahrs.push_back(ahrs);
    }

    pub fn len(&self) -> usize {
        self.accel.len()
    }

    pub fn clear(&mut self) {
        self.accel.clear();
        self.gyro.clear();
        self.mag.clear();
        self.ahrs.clear();
    }
}
