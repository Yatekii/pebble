//! Puzzle state machine.
//!
//! This module implements the main state machine that tracks puzzle progress
//! and orchestrates transitions between puzzles.

use super::events::{Action, PuzzleEvent};
use super::puzzles::Puzzle;

/// Number of real, solvable puzzles. `Unlocked` is a terminal state, not a
/// puzzle, so it is excluded from this count and from `puzzle_states`.
pub const NUM_PUZZLES: usize = 3;

/// Unique identifier for each puzzle.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[repr(u8)]
pub enum PuzzleId {
    /// First puzzle: align compass to target heading.
    #[default]
    CompassAlign = 0,
    /// Second puzzle: navigate to GPS coordinates.
    LocationFind = 1,
    /// Third puzzle: perform tilt sequence.
    TiltSequence = 2,
    /// Final puzzle: box unlocked!
    Unlocked = 3,
}

impl PuzzleId {
    /// Get the next puzzle in sequence.
    pub fn next(self) -> Option<Self> {
        match self {
            PuzzleId::CompassAlign => Some(PuzzleId::LocationFind),
            PuzzleId::LocationFind => Some(PuzzleId::TiltSequence),
            PuzzleId::TiltSequence => Some(PuzzleId::Unlocked),
            PuzzleId::Unlocked => None,
        }
    }

    /// Convert from u8 for deserialization.
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(PuzzleId::CompassAlign),
            1 => Some(PuzzleId::LocationFind),
            2 => Some(PuzzleId::TiltSequence),
            3 => Some(PuzzleId::Unlocked),
            _ => None,
        }
    }
}

/// State of a single puzzle.
#[derive(Clone, Copy, Default)]
pub struct PuzzleState {
    /// Whether this puzzle has been completed.
    pub completed: bool,
    /// Number of attempts made.
    pub attempts: u16,
    /// Puzzle-specific progress data (interpretation depends on puzzle).
    pub progress: u8,
}

/// Serializable puzzle progress for flash storage.
#[derive(Clone, Copy, Default)]
pub struct SavedProgress {
    /// Current puzzle ID.
    pub current_puzzle: u8,
    /// State of each puzzle.
    pub puzzle_states: [PuzzleState; NUM_PUZZLES],
}

impl SavedProgress {
    /// Serialize to bytes for flash storage.
    pub fn to_bytes(&self) -> [u8; 16] {
        let mut bytes = [0u8; 16];
        bytes[0] = self.current_puzzle;
        for (i, state) in self.puzzle_states.iter().enumerate() {
            let base = 1 + i * 4;
            bytes[base] = if state.completed { 1 } else { 0 };
            bytes[base + 1] = (state.attempts & 0xFF) as u8;
            bytes[base + 2] = ((state.attempts >> 8) & 0xFF) as u8;
            bytes[base + 3] = state.progress;
        }
        bytes
    }

    /// Deserialize from bytes.
    pub fn from_bytes(bytes: &[u8; 16]) -> Self {
        let mut progress = SavedProgress {
            current_puzzle: bytes[0],
            puzzle_states: [PuzzleState::default(); NUM_PUZZLES],
        };
        for (i, state) in progress.puzzle_states.iter_mut().enumerate() {
            let base = 1 + i * 4;
            state.completed = bytes[base] != 0;
            state.attempts = bytes[base + 1] as u16 | ((bytes[base + 2] as u16) << 8);
            state.progress = bytes[base + 3];
        }
        progress
    }
}

/// Main puzzle state machine.
pub struct PuzzleStateMachine {
    /// Currently active puzzle.
    current_puzzle: PuzzleId,
    /// State of each puzzle.
    puzzle_states: [PuzzleState; NUM_PUZZLES],
    /// Flag indicating progress needs to be saved.
    dirty: bool,
}

impl PuzzleStateMachine {
    /// Create a new state machine starting at the first puzzle.
    pub fn new() -> Self {
        Self {
            current_puzzle: PuzzleId::CompassAlign,
            puzzle_states: [PuzzleState::default(); NUM_PUZZLES],
            dirty: false,
        }
    }

    /// Create from saved progress.
    pub fn from_saved(saved: SavedProgress) -> Self {
        Self {
            current_puzzle: PuzzleId::from_u8(saved.current_puzzle)
                .unwrap_or(PuzzleId::CompassAlign),
            puzzle_states: saved.puzzle_states,
            dirty: false,
        }
    }

    /// Get the current puzzle ID.
    pub fn current_puzzle(&self) -> PuzzleId {
        self.current_puzzle
    }

    /// Get the state of a specific puzzle.
    pub fn puzzle_state(&self, id: PuzzleId) -> &PuzzleState {
        &self.puzzle_states[id as usize]
    }

    /// Get mutable state of the current puzzle.
    fn current_state_mut(&mut self) -> &mut PuzzleState {
        &mut self.puzzle_states[self.current_puzzle as usize]
    }

    /// True once every real puzzle has been completed (i.e. the box should open).
    pub fn all_solved(&self) -> bool {
        self.puzzle_states.iter().all(|s| s.completed)
    }

    /// Check if progress needs to be saved.
    pub fn needs_save(&self) -> bool {
        self.dirty
    }

    /// Mark progress as saved.
    pub fn mark_saved(&mut self) {
        self.dirty = false;
    }

    /// Get progress for saving.
    pub fn to_saved(&self) -> SavedProgress {
        SavedProgress {
            current_puzzle: self.current_puzzle as u8,
            puzzle_states: self.puzzle_states,
        }
    }

    /// Handle an event and return any resulting actions.
    ///
    /// The puzzle implementation is passed in to allow the state machine
    /// to delegate event handling to the active puzzle.
    pub fn handle_event<P: Puzzle>(
        &mut self,
        puzzle: &mut P,
        event: PuzzleEvent,
    ) -> Option<Action> {
        // If already unlocked, no more events to handle
        if self.current_puzzle == PuzzleId::Unlocked {
            return None;
        }

        // Delegate to the active puzzle
        let action = puzzle.handle_event(event, self.current_state_mut());

        // Check if puzzle completed
        if let Some(Action::Complete) = action {
            self.current_state_mut().completed = true;
            self.dirty = true;

            // Advance to next puzzle
            if let Some(next) = self.current_puzzle.next() {
                self.current_puzzle = next;

                // If final puzzle reached, return unlock action
                if next == PuzzleId::Unlocked {
                    return Some(Action::MoveServo(super::events::ServoPosition::Unlocked));
                }
            }

            // Return success animation instead of Complete
            return Some(Action::SetLeds(super::events::LedPattern::Success));
        }

        action
    }

    /// Reset all progress.
    pub fn reset(&mut self) {
        self.current_puzzle = PuzzleId::CompassAlign;
        self.puzzle_states = [PuzzleState::default(); NUM_PUZZLES];
        self.dirty = true;
    }
}

impl Default for PuzzleStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_puzzle_id_next() {
        assert_eq!(PuzzleId::CompassAlign.next(), Some(PuzzleId::LocationFind));
        assert_eq!(PuzzleId::LocationFind.next(), Some(PuzzleId::TiltSequence));
        assert_eq!(PuzzleId::TiltSequence.next(), Some(PuzzleId::Unlocked));
        assert_eq!(PuzzleId::Unlocked.next(), None);
    }

    #[test]
    fn test_puzzle_id_from_u8() {
        assert_eq!(PuzzleId::from_u8(0), Some(PuzzleId::CompassAlign));
        assert_eq!(PuzzleId::from_u8(1), Some(PuzzleId::LocationFind));
        assert_eq!(PuzzleId::from_u8(2), Some(PuzzleId::TiltSequence));
        assert_eq!(PuzzleId::from_u8(3), Some(PuzzleId::Unlocked));
        assert_eq!(PuzzleId::from_u8(4), None);
        assert_eq!(PuzzleId::from_u8(255), None);
    }

    #[test]
    fn test_saved_progress_roundtrip() {
        let mut progress = SavedProgress::default();
        progress.current_puzzle = 2;
        progress.puzzle_states[0].completed = true;
        progress.puzzle_states[0].attempts = 5;
        progress.puzzle_states[1].completed = true;
        progress.puzzle_states[1].progress = 50;

        let bytes = progress.to_bytes();
        let restored = SavedProgress::from_bytes(&bytes);

        assert_eq!(restored.current_puzzle, 2);
        assert!(restored.puzzle_states[0].completed);
        assert_eq!(restored.puzzle_states[0].attempts, 5);
        assert!(restored.puzzle_states[1].completed);
        assert_eq!(restored.puzzle_states[1].progress, 50);
    }

    #[test]
    fn test_state_machine_new() {
        let sm = PuzzleStateMachine::new();
        assert_eq!(sm.current_puzzle(), PuzzleId::CompassAlign);
        assert!(!sm.needs_save());
    }

    #[test]
    fn test_state_machine_reset() {
        let mut sm = PuzzleStateMachine::new();
        sm.reset();
        assert_eq!(sm.current_puzzle(), PuzzleId::CompassAlign);
        assert!(sm.needs_save());
    }

    #[test]
    fn test_state_machine_from_saved() {
        let mut saved = SavedProgress::default();
        saved.current_puzzle = 2;
        saved.puzzle_states[0].completed = true;
        saved.puzzle_states[1].completed = true;

        let sm = PuzzleStateMachine::from_saved(saved);
        assert_eq!(sm.current_puzzle(), PuzzleId::TiltSequence);
        assert!(sm.puzzle_state(PuzzleId::CompassAlign).completed);
        assert!(sm.puzzle_state(PuzzleId::LocationFind).completed);
    }
}
