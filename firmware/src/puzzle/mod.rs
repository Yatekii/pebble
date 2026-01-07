//! Control layer: puzzle logic and game state machine.
//!
//! This module implements the puzzle box game logic. The box contains multiple
//! puzzles that must be solved in sequence to unlock it.
//!
//! # Architecture
//!
//! - [`state_machine`]: Main puzzle state machine that tracks progress
//! - [`events`]: Event types for inter-layer communication
//! - [`puzzles`]: Individual puzzle implementations
//!
//! # Puzzle Flow
//!
//! 1. Device boots, loads saved puzzle state from flash
//! 2. State machine activates the current puzzle
//! 3. Puzzle receives device state updates (orientation, GPS, etc.)
//! 4. When puzzle conditions are met, it signals completion
//! 5. State machine advances to next puzzle, saves progress
//! 6. Final puzzle unlocks the servo

pub mod events;
pub mod puzzles;
pub mod state_machine;

pub use events::{Action, PuzzleEvent};
pub use state_machine::{PuzzleId, PuzzleState, PuzzleStateMachine};
