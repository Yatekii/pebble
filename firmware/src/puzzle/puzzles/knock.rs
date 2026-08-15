//! Knock puzzle: tap a secret rhythm on the box.
//!
//! Matching is on *rhythm*, not absolute tempo: the recorded inter-knock gaps
//! and the reference gaps are both normalised by their total, then compared
//! within [`TOLERANCE`]. So the player can knock the pattern fast or slow.
//!
//! Timing lives in the task (see `tasks::puzzle`): each knock arrives as a
//! [`PuzzleEvent::Tap`] carrying the gap since the previous one, and a quiet
//! period ends the attempt as a [`PuzzleEvent::Timeout`]. The first knock of an
//! attempt carries [`START_GAP`].

use crate::puzzle::events::{Action, LedPattern, PuzzleEvent};
use crate::puzzle::state_machine::PuzzleState;

use super::Puzzle;

/// Sentinel gap marking the first knock of a fresh attempt.
pub const START_GAP: u16 = u16::MAX;

/// Longest pattern we record (knocks = gaps + 1).
const MAX_GAPS: usize = 15;

/// Per-interval tolerance as a fraction, expressed as num/den (±25%).
const TOL_NUM: i64 = 1;
const TOL_DEN: i64 = 4;

/// Test rhythm: "Shave and a Haircut" — 5 knocks, gaps ~1:1:2:1. Only the
/// ratios matter; swap for the real secret per box.
pub const TEST_PATTERN: [u16; 4] = [250, 250, 500, 250];

/// Knock puzzle state.
pub struct KnockPuzzle {
    /// Reference rhythm as inter-knock gaps (ratios matter, not absolute ms).
    pattern: &'static [u16],
    /// Recorded gaps for the current attempt.
    gaps: [u16; MAX_GAPS],
    /// Number of gaps recorded.
    count: usize,
    /// Whether the first knock of an attempt has been seen.
    started: bool,
    /// More knocks than the buffer holds — attempt can only fail.
    overflow: bool,
}

impl KnockPuzzle {
    /// Create a new knock puzzle for the given reference rhythm.
    pub fn new(pattern: &'static [u16]) -> Self {
        Self {
            pattern,
            gaps: [0; MAX_GAPS],
            count: 0,
            started: false,
            overflow: false,
        }
    }

    fn reset(&mut self) {
        self.count = 0;
        self.started = false;
        self.overflow = false;
    }

    /// Do the recorded gaps match the reference rhythm within tolerance?
    fn matches(&self) -> bool {
        if self.overflow || self.count != self.pattern.len() || self.count == 0 {
            return false;
        }
        let rec_sum: i64 = self.gaps[..self.count].iter().map(|&g| g as i64).sum();
        let ref_sum: i64 = self.pattern.iter().map(|&g| g as i64).sum();
        if rec_sum == 0 || ref_sum == 0 {
            return false;
        }
        // Compare normalised intervals without dividing:
        //   |g_i/rec_sum - p_i/ref_sum| <= TOL  ==>
        //   |g_i*ref_sum - p_i*rec_sum| <= TOL * rec_sum * ref_sum
        let bound = rec_sum * ref_sum * TOL_NUM / TOL_DEN;
        for i in 0..self.count {
            let delta = (self.gaps[i] as i64 * ref_sum - self.pattern[i] as i64 * rec_sum).abs();
            if delta > bound {
                return false;
            }
        }
        true
    }
}

impl Puzzle for KnockPuzzle {
    fn handle_event(&mut self, event: PuzzleEvent, _state: &mut PuzzleState) -> Option<Action> {
        match event {
            PuzzleEvent::Tap { gap_ms } => {
                if gap_ms == START_GAP || !self.started {
                    // First knock of an attempt: reset, record no gap yet.
                    self.reset();
                    self.started = true;
                } else if self.count < MAX_GAPS {
                    self.gaps[self.count] = gap_ms;
                    self.count += 1;
                } else {
                    self.overflow = true;
                }
                None
            }
            PuzzleEvent::Timeout => {
                if !self.started {
                    return None;
                }
                let ok = self.matches();
                self.reset();
                if ok {
                    Some(Action::Complete)
                } else {
                    Some(Action::SetLeds(LedPattern::Error))
                }
            }
            _ => None,
        }
    }

    fn initial_pattern(&self) -> LedPattern {
        LedPattern::Off
    }

    fn description(&self) -> &'static str {
        "Knock the secret rhythm"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Play a full attempt: first knock, then each gap, then the quiet timeout.
    fn attempt(p: &mut KnockPuzzle, gaps: &[u16]) -> Option<Action> {
        let mut st = PuzzleState::default();
        p.handle_event(PuzzleEvent::Tap { gap_ms: START_GAP }, &mut st);
        for &g in gaps {
            p.handle_event(PuzzleEvent::Tap { gap_ms: g }, &mut st);
        }
        p.handle_event(PuzzleEvent::Timeout, &mut st)
    }

    #[test]
    fn exact_rhythm_completes() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        assert!(matches!(attempt(&mut p, &TEST_PATTERN), Some(Action::Complete)));
    }

    #[test]
    fn tempo_independent() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        // Same rhythm, half the speed.
        let slow: [u16; 4] = [500, 500, 1000, 500];
        assert!(matches!(attempt(&mut p, &slow), Some(Action::Complete)));
    }

    #[test]
    fn slight_wobble_still_completes() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        // Within ±25% per interval.
        let wobbly: [u16; 4] = [280, 230, 520, 240];
        assert!(matches!(attempt(&mut p, &wobbly), Some(Action::Complete)));
    }

    #[test]
    fn wrong_count_fails() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        assert!(matches!(
            attempt(&mut p, &[250, 250]),
            Some(Action::SetLeds(LedPattern::Error))
        ));
    }

    #[test]
    fn wrong_rhythm_fails() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        // Ratios 2:1:1:1 vs the reference 1:1:2:1.
        assert!(matches!(
            attempt(&mut p, &[500, 250, 250, 250]),
            Some(Action::SetLeds(LedPattern::Error))
        ));
    }

    #[test]
    fn timeout_without_knocks_is_ignored() {
        let mut p = KnockPuzzle::new(&TEST_PATTERN);
        let mut st = PuzzleState::default();
        assert!(p.handle_event(PuzzleEvent::Timeout, &mut st).is_none());
    }
}
