//! Puzzle list UI component showing puzzle progress in a tree-like view.

use gpui::prelude::FluentBuilder;
use gpui::*;
use gpui_component::ActiveTheme;

/// Status of a puzzle or sub-step
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PuzzleStatus {
    /// Not yet attempted
    Locked,
    /// Currently being worked on
    InProgress,
    /// Successfully completed
    Solved,
}

/// A sub-step within a puzzle
#[derive(Clone, Debug)]
pub struct PuzzleStep {
    pub name: &'static str,
    pub status: PuzzleStatus,
}

/// A puzzle with optional sub-steps
#[derive(Clone, Debug)]
pub struct Puzzle {
    pub name: &'static str,
    pub description: &'static str,
    pub status: PuzzleStatus,
    pub steps: Vec<PuzzleStep>,
}

impl Puzzle {
    /// Calculate overall status based on sub-steps
    pub fn effective_status(&self) -> PuzzleStatus {
        if self.steps.is_empty() {
            return self.status;
        }

        let all_solved = self.steps.iter().all(|s| s.status == PuzzleStatus::Solved);
        let any_in_progress = self
            .steps
            .iter()
            .any(|s| s.status == PuzzleStatus::InProgress);
        let any_solved = self.steps.iter().any(|s| s.status == PuzzleStatus::Solved);

        if all_solved {
            PuzzleStatus::Solved
        } else if any_in_progress || any_solved {
            PuzzleStatus::InProgress
        } else {
            PuzzleStatus::Locked
        }
    }
}

/// Sample puzzle data for mockup
pub fn sample_puzzles() -> Vec<Puzzle> {
    vec![
        Puzzle {
            name: "The Awakening",
            description: "Shake the box to wake it up",
            status: PuzzleStatus::Solved,
            steps: vec![],
        },
        Puzzle {
            name: "Cardinal Direction",
            description: "Point the box towards the four cardinal directions",
            status: PuzzleStatus::InProgress,
            steps: vec![
                PuzzleStep {
                    name: "Point North",
                    status: PuzzleStatus::Solved,
                },
                PuzzleStep {
                    name: "Point East",
                    status: PuzzleStatus::Solved,
                },
                PuzzleStep {
                    name: "Point South",
                    status: PuzzleStatus::InProgress,
                },
                PuzzleStep {
                    name: "Point West",
                    status: PuzzleStatus::Locked,
                },
            ],
        },
        Puzzle {
            name: "The Journey",
            description: "Bring the box to the secret location",
            status: PuzzleStatus::Locked,
            steps: vec![
                PuzzleStep {
                    name: "Find the first waypoint",
                    status: PuzzleStatus::Locked,
                },
                PuzzleStep {
                    name: "Find the second waypoint",
                    status: PuzzleStatus::Locked,
                },
                PuzzleStep {
                    name: "Reach the destination",
                    status: PuzzleStatus::Locked,
                },
            ],
        },
        Puzzle {
            name: "The Dance",
            description: "Perform the secret rotation sequence",
            status: PuzzleStatus::Locked,
            steps: vec![],
        },
        Puzzle {
            name: "Final Revelation",
            description: "The box opens to reveal its treasure",
            status: PuzzleStatus::Locked,
            steps: vec![],
        },
    ]
}

/// Puzzle list view element
#[derive(IntoElement)]
pub struct PuzzleListView {
    puzzles: Vec<Puzzle>,
}

impl PuzzleListView {
    pub fn new(puzzles: Vec<Puzzle>) -> Self {
        Self { puzzles }
    }
}

impl RenderOnce for PuzzleListView {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let border_color = theme.border;

        div()
            .id("puzzle-list")
            .w_full()
            .h_full()
            .overflow_y_scroll()
            .p_3()
            .bg(theme.background)
            .border_1()
            .border_color(border_color)
            .rounded_md()
            .flex()
            .flex_col()
            .gap_1()
            .child(
                div()
                    .text_sm()
                    .font_weight(FontWeight::BOLD)
                    .text_color(theme.foreground)
                    .mb_2()
                    .child("Puzzles"),
            )
            .children(
                self.puzzles
                    .into_iter()
                    .enumerate()
                    .map(|(idx, puzzle)| PuzzleItem::new(puzzle, idx)),
            )
    }
}

/// Individual puzzle item in the list
#[derive(IntoElement)]
struct PuzzleItem {
    puzzle: Puzzle,
    index: usize,
}

impl PuzzleItem {
    fn new(puzzle: Puzzle, index: usize) -> Self {
        Self { puzzle, index }
    }
}

impl RenderOnce for PuzzleItem {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let status = self.puzzle.effective_status();

        let (icon, icon_color) = match status {
            PuzzleStatus::Solved => ("✓", hsla(0.33, 0.7, 0.45, 1.0)), // green
            PuzzleStatus::InProgress => ("◐", hsla(0.15, 0.8, 0.5, 1.0)), // orange
            PuzzleStatus::Locked => ("○", theme.muted_foreground),
        };

        let text_color = match status {
            PuzzleStatus::Solved => theme.foreground,
            PuzzleStatus::InProgress => theme.foreground,
            PuzzleStatus::Locked => theme.muted_foreground,
        };

        let has_steps = !self.puzzle.steps.is_empty();
        let steps = self.puzzle.steps.clone();

        div()
            .flex()
            .flex_col()
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap_2()
                    .py_1()
                    .child(
                        div()
                            .w(px(16.0))
                            .text_center()
                            .text_color(icon_color)
                            .child(icon),
                    )
                    .child(
                        div()
                            .flex_1()
                            .text_sm()
                            .text_color(text_color)
                            .child(format!("{}. {}", self.index + 1, self.puzzle.name)),
                    ),
            )
            .when(has_steps, |this| {
                this.child(
                    div()
                        .ml(px(24.0))
                        .flex()
                        .flex_col()
                        .children(steps.into_iter().map(|step| {
                            let (step_icon, step_icon_color) = match step.status {
                                PuzzleStatus::Solved => ("✓", hsla(0.33, 0.7, 0.45, 1.0)),
                                PuzzleStatus::InProgress => ("▸", hsla(0.15, 0.8, 0.5, 1.0)),
                                PuzzleStatus::Locked => ("·", theme.muted_foreground),
                            };

                            let step_text_color = match step.status {
                                PuzzleStatus::Solved => theme.muted_foreground,
                                PuzzleStatus::InProgress => theme.foreground,
                                PuzzleStatus::Locked => theme.muted_foreground,
                            };

                            div()
                                .flex()
                                .flex_row()
                                .items_center()
                                .gap_2()
                                .py(px(2.0))
                                .child(
                                    div()
                                        .w(px(12.0))
                                        .text_center()
                                        .text_xs()
                                        .text_color(step_icon_color)
                                        .child(step_icon),
                                )
                                .child(div().text_xs().text_color(step_text_color).child(step.name))
                        })),
                )
            })
    }
}
