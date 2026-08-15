//! Individual puzzle implementations.
//!
//! Each puzzle is a separate module that implements the [`Puzzle`] trait.

pub mod directions;

use super::events::{Action, PuzzleEvent};
use super::state_machine::PuzzleState;

/// Trait that all puzzles must implement.
pub trait Puzzle {
    /// Handle an event and optionally return an action.
    ///
    /// # Arguments
    ///
    /// * `event` - The event to handle
    /// * `state` - Mutable reference to this puzzle's state
    ///
    /// # Returns
    ///
    /// An optional action to perform. Return `Some(Action::Complete)` when
    /// the puzzle is solved.
    fn handle_event(&mut self, event: PuzzleEvent, state: &mut PuzzleState) -> Option<Action>;

    /// Get the initial LED pattern for this puzzle.
    fn initial_pattern(&self) -> super::events::LedPattern;

    /// Get a description of this puzzle (for debugging/display).
    fn description(&self) -> &'static str;
}
