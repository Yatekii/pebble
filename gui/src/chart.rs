//! Simple line chart wrapper with Y-axis tick marks.

use gpui::prelude::FluentBuilder;
use gpui::*;
use gpui_component::ActiveTheme;

const SAMPLES_PER_SECOND: usize = 20; // 20 Hz sample rate

/// A wrapper around data that renders a line chart with Y-axis tick labels.
#[derive(IntoElement)]
pub struct SimpleLineChart {
    data: Vec<f64>,
    color: Hsla,
    show_x_axis: bool,
}

impl SimpleLineChart {
    pub fn new(data: Vec<f64>, color: Hsla) -> Self {
        Self {
            data,
            color,
            show_x_axis: false,
        }
    }

    pub fn show_x_axis(mut self) -> Self {
        self.show_x_axis = true;
        self
    }

    fn calc_range(data: &[f64]) -> (f64, f64) {
        if data.is_empty() {
            return (-1.0, 1.0);
        }
        let min = data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        // Ensure we have some range and include 0
        let min = min.min(0.0);
        let max = max.max(0.0);
        let range = (max - min).max(0.001);
        (min - range * 0.05, max + range * 0.05)
    }

    fn format_value(val: f64) -> String {
        if val.abs() >= 10000.0 {
            format!("{:.0}", val)
        } else if val.abs() >= 100.0 {
            format!("{:.0}", val)
        } else if val.abs() >= 1.0 {
            format!("{:.1}", val)
        } else if val.abs() < 0.001 {
            "0".to_string()
        } else {
            format!("{:.2}", val)
        }
    }

    /// Get Y-axis tick values: min, 0, max
    fn y_tick_values(min_val: f64, max_val: f64) -> Vec<f64> {
        let mut ticks = vec![min_val, max_val];
        // Add 0 if it's within range
        if min_val < 0.0 && max_val > 0.0 {
            ticks.push(0.0);
        }
        ticks.sort_by(|a, b| a.partial_cmp(b).unwrap());
        ticks
    }
}

impl RenderOnce for SimpleLineChart {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let (min_val, max_val) = Self::calc_range(&self.data);
        let range = max_val - min_val;
        let y_ticks = Self::y_tick_values(min_val, max_val);

        let data = self.data;
        let color = self.color;
        let grid_color = theme.border;
        let muted_color = theme.muted_foreground;
        let show_x_axis = self.show_x_axis;

        // Calculate X-axis ticks (every second)
        let num_samples = data.len();
        let total_seconds = num_samples / SAMPLES_PER_SECOND;

        div()
            .size_full()
            .flex()
            .flex_col()
            .child(
                // Main chart row
                div()
                    .flex_1()
                    .flex()
                    .flex_row()
                    // Y-axis labels column
                    .child(
                        div()
                            .w(px(45.0))
                            .h_full()
                            .flex()
                            .flex_col()
                            .justify_between()
                            .text_xs()
                            .text_color(theme.muted_foreground)
                            .children(
                                y_ticks
                                    .iter()
                                    .rev()
                                    .map(|&val| {
                                        div().text_right().pr_1().child(Self::format_value(val))
                                    })
                                    .collect::<Vec<_>>(),
                            ),
                    )
                    // Chart area
                    .child(
                        canvas(
                            {
                                let y_ticks = y_ticks.clone();
                                move |bounds, _window, _cx| {
                                    (
                                        bounds,
                                        data.clone(),
                                        min_val,
                                        max_val,
                                        range,
                                        y_ticks.clone(),
                                        color,
                                        grid_color,
                                        show_x_axis,
                                    )
                                }
                            },
                            move |_bounds, prepaint_state, window, _cx| {
                                let (
                                    bounds,
                                    data,
                                    min_val,
                                    _max_val,
                                    range,
                                    y_ticks,
                                    color,
                                    grid_color,
                                    show_x_axis,
                                ) = prepaint_state;

                                if data.is_empty()
                                    || bounds.size.width <= px(0.0)
                                    || bounds.size.height <= px(0.0)
                                {
                                    return;
                                }

                                let origin = bounds.origin;
                                let width = bounds.size.width;
                                let height = bounds.size.height;

                                let grid_color_faint = grid_color.opacity(0.3);

                                // Draw horizontal grid lines at y-tick positions
                                for &tick_val in &y_ticks {
                                    let y_t = ((tick_val - min_val) / range) as f32;
                                    let y = origin.y + height - height * y_t;

                                    let mut builder = PathBuilder::stroke(px(1.0));
                                    builder.move_to(point(origin.x, y));
                                    builder.line_to(point(origin.x + width, y));
                                    if let Ok(path) = builder.build() {
                                        window.paint_path(path, grid_color_faint);
                                    }
                                }

                                // Draw vertical grid lines every second (only if showing x-axis)
                                if show_x_axis && data.len() > SAMPLES_PER_SECOND {
                                    let num_seconds = data.len() / SAMPLES_PER_SECOND;
                                    for s in 1..=num_seconds {
                                        let sample_idx = s * SAMPLES_PER_SECOND;
                                        if sample_idx < data.len() {
                                            let x_t = sample_idx as f32 / (data.len() - 1) as f32;
                                            let x = origin.x + width * x_t;

                                            let mut builder = PathBuilder::stroke(px(1.0));
                                            builder.move_to(point(x, origin.y));
                                            builder.line_to(point(x, origin.y + height));
                                            if let Ok(path) = builder.build() {
                                                window.paint_path(path, grid_color_faint);
                                            }
                                        }
                                    }
                                }

                                // Draw the data line
                                if data.len() >= 2 {
                                    let mut builder = PathBuilder::stroke(px(2.0));

                                    for (i, &val) in data.iter().enumerate() {
                                        let x_t = i as f32 / (data.len() - 1) as f32;
                                        let x = origin.x + width * x_t;

                                        let y_t = ((val - min_val) / range) as f32;
                                        let y = origin.y + height - height * y_t;

                                        if i == 0 {
                                            builder.move_to(point(x, y));
                                        } else {
                                            builder.line_to(point(x, y));
                                        }
                                    }

                                    if let Ok(path) = builder.build() {
                                        window.paint_path(path, color);
                                    }
                                }
                            },
                        )
                        .flex_1()
                        .h_full(),
                    ),
            )
            // X-axis labels row (only if show_x_axis is true)
            .when(show_x_axis, |this| {
                this.child(
                    div()
                        .h(px(16.0))
                        .flex()
                        .flex_row()
                        // Spacer for Y-axis label column
                        .child(div().w(px(45.0)))
                        // X-axis labels
                        .child(
                            canvas(
                                move |bounds, _window, _cx| (bounds, num_samples, total_seconds),
                                move |_bounds, (bounds, num_samples, total_seconds), window, cx| {
                                    if num_samples <= SAMPLES_PER_SECOND {
                                        return;
                                    }

                                    let origin = bounds.origin;
                                    let width = bounds.size.width;

                                    // Draw second labels
                                    for s in 1..=total_seconds {
                                        let sample_idx = s * SAMPLES_PER_SECOND;
                                        if sample_idx < num_samples {
                                            let x_t = sample_idx as f32 / (num_samples - 1) as f32;
                                            let x = origin.x + width * x_t;

                                            let label = format!("{}s", s);
                                            let text_origin = point(x - px(8.0), origin.y);

                                            let text_run = TextRun {
                                                len: label.len(),
                                                font: window.text_style().font(),
                                                color: muted_color,
                                                background_color: None,
                                                underline: None,
                                                strikethrough: None,
                                            };

                                            if let Ok(shaped) = window.text_system().shape_text(
                                                label.into(),
                                                px(10.0),
                                                &[text_run],
                                                None,
                                                None,
                                            ) {
                                                for line in shaped {
                                                    let _ = line.paint(
                                                        text_origin,
                                                        px(10.0),
                                                        TextAlign::Center,
                                                        None,
                                                        window,
                                                        cx,
                                                    );
                                                }
                                            }
                                        }
                                    }
                                },
                            )
                            .flex_1()
                            .h_full(),
                        ),
                )
            })
    }
}
