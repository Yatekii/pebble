// Orientation puzzle using the IMU
// The user must hold the box in specific orientations to solve

use bmi2::types::Data;

/// Required orientations to solve the puzzle (in order)
/// Each orientation is defined by which axis should be pointing up
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    /// Z-axis pointing up (flat on table, normal position)
    FlatUp,
    /// Z-axis pointing down (upside down)
    FlatDown,
    /// X-axis pointing up (tilted forward)
    TiltForward,
    /// X-axis pointing down (tilted backward)
    TiltBackward,
    /// Y-axis pointing up (tilted left)
    TiltLeft,
    /// Y-axis pointing down (tilted right)
    TiltRight,
}

impl Orientation {
    /// Check if IMU data matches this orientation
    /// Uses a threshold to allow some tolerance
    pub fn matches(&self, data: &Data) -> bool {
        const THRESHOLD: i16 = 12000; // ~0.75g with 16384 LSB/g at ±8g range
        const LOW_THRESHOLD: i16 = 4000; // ~0.25g - other axes should be low

        let ax = data.acc.x;
        let ay = data.acc.y;
        let az = data.acc.z;

        match self {
            Orientation::FlatUp => {
                az > THRESHOLD && ax.abs() < LOW_THRESHOLD && ay.abs() < LOW_THRESHOLD
            }
            Orientation::FlatDown => {
                az < -THRESHOLD && ax.abs() < LOW_THRESHOLD && ay.abs() < LOW_THRESHOLD
            }
            Orientation::TiltForward => {
                ax > THRESHOLD && ay.abs() < LOW_THRESHOLD && az.abs() < LOW_THRESHOLD
            }
            Orientation::TiltBackward => {
                ax < -THRESHOLD && ay.abs() < LOW_THRESHOLD && az.abs() < LOW_THRESHOLD
            }
            Orientation::TiltLeft => {
                ay > THRESHOLD && ax.abs() < LOW_THRESHOLD && az.abs() < LOW_THRESHOLD
            }
            Orientation::TiltRight => {
                ay < -THRESHOLD && ax.abs() < LOW_THRESHOLD && az.abs() < LOW_THRESHOLD
            }
        }
    }
}

/// The orientation puzzle state machine
pub struct OrientationPuzzle {
    /// Sequence of orientations required to solve
    sequence: &'static [Orientation],
    /// Current position in the sequence
    current_step: usize,
    /// Whether the current orientation has been held long enough
    hold_counter: u8,
}

/// Required hold time in update cycles (e.g., 10 = 1 second at 100ms update rate)
const REQUIRED_HOLD_CYCLES: u8 = 10;

impl OrientationPuzzle {
    /// Create a new orientation puzzle with a predefined sequence
    pub fn new() -> Self {
        // Default puzzle sequence - can be customized
        static SEQUENCE: &[Orientation] = &[
            Orientation::FlatUp,
            Orientation::TiltLeft,
            Orientation::TiltRight,
            Orientation::FlatDown,
            Orientation::FlatUp,
        ];

        Self {
            sequence: SEQUENCE,
            current_step: 0,
            hold_counter: 0,
        }
    }

    /// Update the puzzle state with new IMU data
    /// Returns true if the puzzle is now solved
    pub fn update(&mut self, data: &Data) -> bool {
        if self.is_solved() {
            return true;
        }

        let target = self.sequence[self.current_step];

        if target.matches(data) {
            self.hold_counter = self.hold_counter.saturating_add(1);

            if self.hold_counter >= REQUIRED_HOLD_CYCLES {
                // Move to next step
                self.current_step += 1;
                self.hold_counter = 0;

                if self.is_solved() {
                    return true;
                }
            }
        } else {
            // Reset hold counter if orientation doesn't match
            self.hold_counter = 0;
        }

        false
    }

    /// Check if the puzzle is solved
    pub fn is_solved(&self) -> bool {
        self.current_step >= self.sequence.len()
    }

    /// Get current progress (0.0 to 1.0)
    #[allow(dead_code)]
    pub fn progress(&self) -> f32 {
        if self.sequence.is_empty() {
            return 1.0;
        }
        self.current_step as f32 / self.sequence.len() as f32
    }

    /// Get current step index
    pub fn current_step(&self) -> usize {
        self.current_step
    }

    /// Get total number of steps
    #[allow(dead_code)]
    pub fn total_steps(&self) -> usize {
        self.sequence.len()
    }

    /// Reset the puzzle
    #[allow(dead_code)]
    pub fn reset(&mut self) {
        self.current_step = 0;
        self.hold_counter = 0;
    }
}

impl Default for OrientationPuzzle {
    fn default() -> Self {
        Self::new()
    }
}
