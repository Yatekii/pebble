// Persistent state machine for the reverse geocache puzzle box
// The state is stored in flash memory and survives power cycles

use embedded_storage::{ReadStorage, Storage};
use postcard::{from_bytes, to_slice};
use serde::{Deserialize, Serialize};

/// Flash offset for state storage (using a safe area in NVS partition)
/// This should be aligned to flash sector size (4KB)
const STATE_FLASH_OFFSET: u32 = 0;

/// Puzzle stages in the geocache
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum PuzzleStage {
    /// Initial locked state - box is closed
    Locked = 0,
    /// First puzzle: orientation puzzle using IMU
    OrientationPuzzle = 1,
    /// Second puzzle: navigate to first waypoint
    NavigateWaypoint1 = 2,
    /// Third puzzle: navigate to second waypoint
    NavigateWaypoint2 = 3,
    /// Fourth puzzle: navigate to final location
    NavigateFinal = 4,
    /// Box is unlocked and open!
    Unlocked = 5,
}

impl PuzzleStage {
    /// Get the next puzzle stage
    pub fn next(self) -> Option<Self> {
        match self {
            Self::Locked => Some(Self::OrientationPuzzle),
            Self::OrientationPuzzle => Some(Self::NavigateWaypoint1),
            Self::NavigateWaypoint1 => Some(Self::NavigateWaypoint2),
            Self::NavigateWaypoint2 => Some(Self::NavigateFinal),
            Self::NavigateFinal => Some(Self::Unlocked),
            Self::Unlocked => None,
        }
    }
}

impl Default for PuzzleStage {
    fn default() -> Self {
        Self::Locked
    }
}

/// Persistent state stored in flash
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct PuzzleState {
    /// Current puzzle stage
    pub stage: PuzzleStage,
    /// Number of attempts (for fun statistics)
    pub attempt_count: u16,
}

impl PuzzleState {
    /// Maximum serialized size (with some margin for postcard overhead)
    const MAX_SIZE: usize = 16;
}

/// State machine manager with flash persistence
pub struct StateMachine<S: ReadStorage + Storage> {
    storage: S,
    state: PuzzleState,
}

/// State machine errors
#[derive(Debug)]
pub enum StateError<E> {
    Read(E),
    Write(E),
    Serialize(postcard::Error),
    Deserialize(postcard::Error),
}

impl<S> StateMachine<S>
where
    S: ReadStorage + Storage,
{
    /// Create a new state machine and load state from flash
    pub fn new(mut storage: S) -> Self {
        let state = Self::load_state(&mut storage).unwrap_or_default();
        Self { storage, state }
    }

    /// Load state from flash storage
    fn load_state(storage: &mut S) -> Option<PuzzleState> {
        let mut bytes = [0u8; PuzzleState::MAX_SIZE];
        storage.read(STATE_FLASH_OFFSET, &mut bytes).ok()?;
        from_bytes(&bytes).ok()
    }

    /// Save current state to flash
    pub fn save(&mut self) -> Result<(), StateError<S::Error>> {
        let mut bytes = [0u8; PuzzleState::MAX_SIZE];
        let serialized = to_slice(&self.state, &mut bytes).map_err(StateError::Serialize)?;
        self.storage
            .write(STATE_FLASH_OFFSET, serialized)
            .map_err(StateError::Write)
    }

    /// Get current puzzle stage
    pub fn stage(&self) -> PuzzleStage {
        self.state.stage
    }

    /// Get attempt count
    pub fn attempt_count(&self) -> u16 {
        self.state.attempt_count
    }

    /// Increment attempt counter and save
    pub fn increment_attempts(&mut self) -> Result<(), StateError<S::Error>> {
        self.state.attempt_count = self.state.attempt_count.saturating_add(1);
        self.save()
    }

    /// Advance to the next puzzle stage
    pub fn advance(&mut self) -> Result<Option<PuzzleStage>, StateError<S::Error>> {
        if let Some(next) = self.state.stage.next() {
            self.state.stage = next;
            self.save()?;
            Ok(Some(next))
        } else {
            Ok(None)
        }
    }

    /// Check if the puzzle box is unlocked
    pub fn is_unlocked(&self) -> bool {
        self.state.stage == PuzzleStage::Unlocked
    }

    /// Reset state to initial (for testing/development)
    pub fn reset(&mut self) -> Result<(), StateError<S::Error>> {
        self.state = PuzzleState::default();
        self.save()
    }
}
