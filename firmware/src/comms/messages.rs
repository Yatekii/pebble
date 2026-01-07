//! Common message types for all communication protocols.
//!
//! These message types provide a protocol-agnostic interface for
//! sending and receiving data. Each protocol implementation is
//! responsible for serializing/deserializing these messages.

use crate::comms::ble::GpsBleData;
use crate::filter::watches::{DeviceStatus, LedCommand, SensorData, ServoCommand};

/// Messages that can be sent from the device to external clients.
#[derive(Clone, Copy)]
pub enum OutboundMessage {
    /// Sensor data update (accelerometer, gyroscope, magnetometer, orientation).
    SensorData(SensorData),

    /// GPS position update.
    GpsData(GpsBleData),

    /// Device status (peripheral health).
    DeviceStatus(DeviceStatus),

    /// Compass heading update.
    CompassHeading(u16),

    /// Puzzle state update.
    PuzzleState(PuzzleStateMessage),
}

/// Messages that can be received from external clients.
#[derive(Clone, Copy)]
pub enum InboundMessage {
    /// LED control command.
    LedCommand(LedCommand),

    /// Servo control command.
    ServoCommand(ServoCommand),

    /// LED brightness update.
    LedBrightness(u8),

    /// LED color chunk update (chunk index 0-2, 72 bytes of RGB data).
    LedColorChunk { chunk: u8, colors: [u8; 72] },

    /// Puzzle control command.
    PuzzleCommand(PuzzleCommandMessage),
}

/// Puzzle state information for external display.
#[derive(Clone, Copy, Default)]
pub struct PuzzleStateMessage {
    /// Current puzzle ID (0-3).
    pub current_puzzle: u8,
    /// Number of completed puzzles.
    pub completed_count: u8,
    /// Current puzzle progress (0-100).
    pub progress: u8,
    /// Whether the box is unlocked.
    pub unlocked: bool,
}

impl PuzzleStateMessage {
    /// Serialize to bytes for transmission.
    pub fn to_bytes(&self) -> [u8; 4] {
        [
            self.current_puzzle,
            self.completed_count,
            self.progress,
            if self.unlocked { 1 } else { 0 },
        ]
    }

    /// Deserialize from bytes.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 4 {
            return None;
        }
        Some(Self {
            current_puzzle: bytes[0],
            completed_count: bytes[1],
            progress: bytes[2],
            unlocked: bytes[3] != 0,
        })
    }
}

/// Commands for puzzle control from external clients.
#[derive(Clone, Copy)]
pub enum PuzzleCommandMessage {
    /// Request current puzzle state.
    GetState,
    /// Skip current puzzle (requires authorization).
    Skip,
    /// Reset all progress (requires authorization).
    Reset,
    /// Request a hint.
    Hint,
}

impl PuzzleCommandMessage {
    /// Deserialize from a single byte command.
    pub fn from_byte(byte: u8) -> Option<Self> {
        match byte {
            0 => Some(Self::GetState),
            1 => Some(Self::Skip),
            2 => Some(Self::Reset),
            3 => Some(Self::Hint),
            _ => None,
        }
    }

    /// Serialize to a single byte.
    pub fn to_byte(&self) -> u8 {
        match self {
            Self::GetState => 0,
            Self::Skip => 1,
            Self::Reset => 2,
            Self::Hint => 3,
        }
    }
}
