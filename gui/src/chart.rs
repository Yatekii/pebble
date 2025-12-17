//! Simple line chart wrapper with Y-axis tick marks and multiple series support.

use gpui::prelude::FluentBuilder;
use gpui::*;
use gpui_component::ActiveTheme;

const SAMPLES_PER_SECOND: usize = 20; // 20 Hz sample rate

/// A single data series with its color and label.
#[derive(Clone)]
pub struct Series {
    pub data: Vec<f64>,
    pub color: Hsla,
    pub label: &'static str,
}

/// A line chart that can display multiple data series.
#[derive(IntoElement)]
pub struct MultiLineChart {
    series: Vec<Series>,
    show_x_axis: bool,
}

impl MultiLineChart {
    pub fn new(series: Vec<Series>) -> Self {
        Self {
            series,
            show_x_axis: true,
        }
    }

    fn calc_range(series: &[Series]) -> (f64, f64) {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;

        for s in series {
            for &val in &s.data {
                min = min.min(val);
                max = max.max(val);
            }
        }

        if min == f64::INFINITY {
            return (-1.0, 1.0);
        }

        // Use symmetric range: max(abs(min), abs(max)) for both directions
        let abs_max = min.abs().max(max.abs()).max(0.001);
        let symmetric_max = abs_max * 1.05; // Add 5% padding
        (-symmetric_max, symmetric_max)
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

impl RenderOnce for MultiLineChart {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let (min_val, max_val) = Self::calc_range(&self.series);
        let range = max_val - min_val;
        let y_ticks = Self::y_tick_values(min_val, max_val);

        let series = self.series;
        let grid_color = theme.border;
        let muted_color = theme.muted_foreground;
        let show_x_axis = self.show_x_axis;

        // Calculate X-axis ticks (every second)
        let num_samples = series.first().map(|s| s.data.len()).unwrap_or(0);
        let total_seconds = num_samples / SAMPLES_PER_SECOND;

        // Create legend items
        let legend_items: Vec<_> = series.iter().map(|s| (s.color, s.label)).collect();

        div()
            .size_full()
            .flex()
            .flex_col()
            .child(
                // Legend row
                div()
                    .flex()
                    .flex_row()
                    .gap_3()
                    .pb_1()
                    .children(legend_items.into_iter().map(|(color, label)| {
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap_1()
                            .child(div().size(px(8.0)).rounded_sm().bg(color))
                            .child(div().text_xs().text_color(color).child(label.to_string()))
                    })),
            )
            .child(
                // Main chart row
                div()
                    .flex_1()
                    .min_h_0()
                    .flex()
                    .flex_row()
                    // Y-axis labels column - use canvas for precise positioning
                    .child({
                        let y_ticks_for_labels = y_ticks.clone();
                        canvas(
                            move |bounds, _window, _cx| {
                                (bounds, y_ticks_for_labels.clone(), min_val, range)
                            },
                            move |_bounds, (bounds, y_ticks, min_val, range), window, cx| {
                                let origin = bounds.origin;
                                let height = bounds.size.height;
                                let width = bounds.size.width;

                                for &tick_val in &y_ticks {
                                    let y_t = ((tick_val - min_val) / range) as f32;
                                    let y = origin.y + height - height * y_t;

                                    let label = Self::format_value(tick_val);
                                    // Position text so it's vertically centered on the tick line
                                    let text_origin = point(origin.x, y - px(5.0));

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
                                        Some(width),
                                        None,
                                    ) {
                                        for line in shaped {
                                            let _ = line.paint(
                                                text_origin,
                                                px(10.0),
                                                TextAlign::Right,
                                                None,
                                                window,
                                                cx,
                                            );
                                        }
                                    }
                                }
                            },
                        )
                        .w(px(45.0))
                        .h_full()
                    })
                    // Chart area
                    .child(
                        canvas(
                            {
                                let y_ticks = y_ticks.clone();
                                let series = series.clone();
                                move |bounds, _window, _cx| {
                                    (
                                        bounds,
                                        series.clone(),
                                        min_val,
                                        range,
                                        y_ticks.clone(),
                                        grid_color,
                                        show_x_axis,
                                    )
                                }
                            },
                            move |_bounds, prepaint_state, window, _cx| {
                                let (
                                    bounds,
                                    series,
                                    min_val,
                                    range,
                                    y_ticks,
                                    grid_color,
                                    show_x_axis,
                                ) = prepaint_state;

                                if bounds.size.width <= px(0.0) || bounds.size.height <= px(0.0) {
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

                                // Get max data length across all series
                                let max_len =
                                    series.iter().map(|s| s.data.len()).max().unwrap_or(0);

                                // Draw vertical grid lines every second (only if showing x-axis)
                                if show_x_axis && max_len > SAMPLES_PER_SECOND {
                                    let num_seconds = max_len / SAMPLES_PER_SECOND;
                                    for s in 1..=num_seconds {
                                        let sample_idx = s * SAMPLES_PER_SECOND;
                                        if sample_idx < max_len {
                                            let x_t = sample_idx as f32 / (max_len - 1) as f32;
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

                                // Draw each series
                                for s in &series {
                                    if s.data.len() >= 2 {
                                        let mut builder = PathBuilder::stroke(px(2.0));

                                        for (i, &val) in s.data.iter().enumerate() {
                                            let x_t = i as f32 / (s.data.len() - 1) as f32;
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
                                            window.paint_path(path, s.color);
                                        }
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
