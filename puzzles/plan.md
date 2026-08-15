# Puzzle plan

The box runs a fixed sequence of puzzles. Each unlocks the next; the last opens the box.

## 1. Crossword → cardinal directions

An external crossword yields a combination of the letters N/E/W/S (configurable
sequence, e.g. `N E S W`). The player enters it by pointing the box:

- When the box points within ±5° of one of the four cardinal directions, the LED
  circle starts filling blue over ~5 s.
- Circle full + correct direction for the current step → flash green twice, advance
  the sequence, restart the fill game for the next letter.
- Circle full + wrong direction → flash red twice, reset the whole sequence.
- Sequence complete → puzzle solved.

## 2. Knock pattern

Player knocks a rhythm on the box; the accelerometer/gyro tap detection already
in firmware (`TAP_EVENT`) captures it.

- Match on *rhythm*, not absolute timing: store the pattern as N−1 inter-knock
  intervals. After a gap > ~2 s, evaluate.
- Knock count must match, then normalize both sequences to total duration = 1.0
  (tempo-independent) and require each interval within ±25–30% of the stored one.
- Pass → green flash; fail → red flash + reset (same feedback vocab as puzzle 1).
- Pattern source: something personal/derivable (e.g. "Shave and a Haircut", or a
  favorite song's hook).

Notes / open items:
- Single tap is currently the global open/close toggle (`puzzle.rs:48`). While this
  puzzle is the active FSM state, taps must feed the puzzle instead of the toggle.
- Fast knocks (<300 ms) currently collapse into double-taps (`imu.rs:80`). Knock
  capture needs the raw per-tap stream, not the single/double classification.

## 3. Waypoints (compass navigation)

Two or three programmable waypoints shown via the compass. Each waypoint has a
~100 m geofence; entering it unlocks the next waypoint.

## 4. Final unlock

The last waypoint has a tight ~10 m geofence. Reaching it opens the box.
