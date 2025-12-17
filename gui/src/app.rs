use std::sync::Arc;

use gpui::*;
use gpui_component::{ActiveTheme, Root, TitleBar, h_flex, v_flex};
use gpui_component_assets::Assets;
use parking_lot::Mutex;

use crate::ble::{BleMessage, BleState, ConnectionState, LedColors, start_ble_client};
use crate::chart::{MultiLineChart, Series};
use crate::data::ImuReading;
use crate::orientation::{Orientation, OrientationView, calculate_orientation, smooth_orientation};

actions!(imu_viewer, [Quit]);

fn pulsating_easing(t: f32) -> f32 {
    // Sine wave that goes 0 -> 1 -> 0
    (t * std::f32::consts::PI).sin()
}

pub struct ImuViewerApp {
    focus_handle: FocusHandle,
    ble_state: Arc<Mutex<BleState>>,
    connection_state: ConnectionState,
    smoothed_orientation: Orientation,
    led_colors: LedColors,
}

impl ImuViewerApp {
    pub fn new(window: &mut Window, cx: &mut Context<Self>) -> Self {
        let focus_handle = cx.focus_handle();

        // Create shared BLE state
        let ble_state = Arc::new(Mutex::new(BleState::new()));

        // Start the BLE client
        let ble_rx = Arc::new(Mutex::new(start_ble_client(ble_state.clone())));

        // Poll for BLE messages
        let ble_rx_for_poll = ble_rx.clone();
        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor()
                    .timer(std::time::Duration::from_millis(16))
                    .await;

                let result = this.update(cx, |this, cx| {
                    // Process all pending BLE messages
                    let rx = ble_rx_for_poll.lock();
                    let mut msg_count = 0;
                    while let Ok(msg) = rx.try_recv() {
                        this.handle_ble_message(msg);
                        msg_count += 1;
                    }
                    drop(rx);

                    if msg_count > 0 {
                        // Only log occasionally to avoid spam
                        static MSG_TOTAL: std::sync::atomic::AtomicU64 =
                            std::sync::atomic::AtomicU64::new(0);
                        let total = MSG_TOTAL
                            .fetch_add(msg_count, std::sync::atomic::Ordering::Relaxed)
                            + msg_count;
                        if total % 500 == 0 {
                            eprintln!("UI received {} total messages", total);
                        }
                    }

                    cx.notify();
                });
                if result.is_err() {
                    break;
                }
            }
        })
        .detach();

        window.focus(&focus_handle);

        Self {
            focus_handle,
            ble_state,
            connection_state: ConnectionState::Disconnected,
            smoothed_orientation: Orientation::default(),
            led_colors: [[0; 3]; 72],
        }
    }

    fn handle_ble_message(&mut self, msg: BleMessage) {
        match msg {
            BleMessage::StateChanged(state) => {
                self.connection_state = state;
            }
            BleMessage::AccelData(accel) => {
                self.ble_state.lock().imu_history.push_accel(accel);
            }
            BleMessage::GyroData(gyro) => {
                self.ble_state.lock().imu_history.push_gyro(gyro);
            }
            BleMessage::MagData(mag) => {
                self.ble_state.lock().imu_history.push_mag(mag);
            }
            BleMessage::LedColorsChunk(chunk, colors) => {
                // Copy 24 LED colors to the appropriate position
                let start = (chunk as usize) * 24;
                for (i, color) in colors.iter().enumerate() {
                    if start + i < 72 {
                        self.led_colors[start + i] = *color;
                    }
                }
            }
        }
    }

    fn render_sensor_chart(
        &self,
        title: &str,
        readings: &[ImuReading],
        cx: &Context<Self>,
    ) -> impl IntoElement {
        let theme = cx.theme();

        // Convert readings to separate X, Y, Z value vectors
        let x_data: Vec<f64> = readings.iter().map(|r| r.x).collect();
        let y_data: Vec<f64> = readings.iter().map(|r| r.y).collect();
        let z_data: Vec<f64> = readings.iter().map(|r| r.z).collect();

        let red = hsla(0.0, 0.7, 0.5, 1.0);
        let green = hsla(0.33, 0.7, 0.45, 1.0);
        let blue = hsla(0.6, 0.7, 0.5, 1.0);

        let series = vec![
            Series {
                data: x_data,
                color: red,
                label: "X",
            },
            Series {
                data: y_data,
                color: green,
                label: "Y",
            },
            Series {
                data: z_data,
                color: blue,
                label: "Z",
            },
        ];

        v_flex()
            .flex_1()
            .min_h_0()
            .h_full()
            .p_2()
            .bg(theme.background)
            .border_1()
            .border_color(theme.border)
            .rounded_md()
            .child(
                div()
                    .text_xs()
                    .font_weight(FontWeight::SEMIBOLD)
                    .text_color(theme.foreground)
                    .child(title.to_string()),
            )
            .child(
                div()
                    .flex_1()
                    .min_h_0()
                    .h_full()
                    .child(MultiLineChart::new(series)),
            )
    }

    fn connection_status_color(&self) -> Hsla {
        match &self.connection_state {
            ConnectionState::Disconnected => hsla(0.0, 0.0, 0.5, 1.0), // gray
            ConnectionState::Scanning => hsla(0.15, 0.8, 0.5, 1.0),    // orange
            ConnectionState::Connecting => hsla(0.15, 0.8, 0.5, 1.0),  // orange
            ConnectionState::Connected => hsla(0.35, 0.7, 0.45, 1.0),  // green
            ConnectionState::Error(_) => hsla(0.0, 0.7, 0.5, 1.0),     // red
        }
    }

    fn connection_status_text(&self) -> String {
        match &self.connection_state {
            ConnectionState::Disconnected => "Disconnected".to_string(),
            ConnectionState::Scanning => "Scanning for Pebble...".to_string(),
            ConnectionState::Connecting => "Connecting...".to_string(),
            ConnectionState::Connected => "Connected via BLE".to_string(),
            ConnectionState::Error(e) => format!("Error: {}", e),
        }
    }
}

impl Focusable for ImuViewerApp {
    fn focus_handle(&self, _cx: &App) -> FocusHandle {
        self.focus_handle.clone()
    }
}

impl Render for ImuViewerApp {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let theme = cx.theme();
        let is_connected = self.connection_state == ConnectionState::Connected;

        let state = self.ble_state.lock();
        let accel = state.imu_history.accel.clone();
        let gyro = state.imu_history.gyro.clone();
        let mag = state.imu_history.mag.clone();
        let sample_count = state.imu_history.len();
        drop(state);

        let status_color = self.connection_status_color();
        let status_text = self.connection_status_text();

        div()
            .id("imu-viewer-app")
            .size_full()
            .flex()
            .flex_col()
            .bg(theme.sidebar)
            .text_color(theme.foreground)
            .child(
                TitleBar::new().child(
                    div()
                        .flex()
                        .items_center()
                        .justify_center()
                        .w_full()
                        .child("Pebble IMU Viewer"),
                ),
            )
            .child(if is_connected {
                // Calculate orientation from latest sensor readings with smoothing
                if !accel.is_empty() && !mag.is_empty() {
                    let last_accel = accel.last().unwrap();
                    let last_mag = mag.last().unwrap();
                    let target = calculate_orientation(
                        (last_accel.x, last_accel.y, last_accel.z),
                        (last_mag.x, last_mag.y, last_mag.z),
                    );
                    // Smooth with alpha=0.15 (lower = smoother but more lag)
                    self.smoothed_orientation =
                        smooth_orientation(self.smoothed_orientation, target, 0.15);
                }
                let orientation = self.smoothed_orientation;
                let led_colors = self.led_colors;

                // Main content with charts on left, orientation on right
                h_flex()
                    .flex_1()
                    .min_h_0() // Allow shrinking below content size
                    .h_full()
                    .p_4()
                    .gap_4()
                    // Charts stacked on the left
                    .child(
                        v_flex()
                            .flex_1()
                            .h_full()
                            .min_h_0() // Allow shrinking
                            .gap_2()
                            .child(self.render_sensor_chart("Accelerometer (raw)", &accel, cx))
                            .child(self.render_sensor_chart("Gyroscope (raw)", &gyro, cx))
                            .child(self.render_sensor_chart("Magnetometer (raw)", &mag, cx)),
                    )
                    // Orientation view on the right
                    .child(
                        div()
                            .w(px(400.0))
                            .h_full()
                            .child(OrientationView::new(orientation, led_colors)),
                    )
                    .into_any_element()
            } else {
                // Connection status in center
                let is_scanning = matches!(
                    self.connection_state,
                    ConnectionState::Scanning | ConnectionState::Connecting
                );
                let center_dot_color = if is_scanning {
                    hsla(0.6, 0.7, 0.5, 1.0) // blue
                } else {
                    status_color
                };

                div()
                    .flex_1()
                    .flex()
                    .items_center()
                    .justify_center()
                    .child(
                        v_flex()
                            .items_center()
                            .gap_4()
                            .child(if is_scanning {
                                div()
                                    .id("pulse-dot")
                                    .size(px(16.0))
                                    .rounded_full()
                                    .bg(center_dot_color)
                                    .with_animation(
                                        "pulse",
                                        Animation::new(std::time::Duration::from_millis(1000))
                                            .repeat()
                                            .with_easing(pulsating_easing),
                                        |this, delta| this.opacity(0.4 + 0.6 * delta),
                                    )
                                    .into_any_element()
                            } else {
                                div()
                                    .size(px(16.0))
                                    .rounded_full()
                                    .bg(center_dot_color)
                                    .into_any_element()
                            })
                            .child(
                                div()
                                    .text_xl()
                                    .text_color(theme.muted_foreground)
                                    .child(status_text.clone()),
                            ),
                    )
                    .into_any_element()
            })
            .child(
                // Status bar
                div()
                    .flex()
                    .items_center()
                    .justify_between()
                    .px_4()
                    .py_2()
                    .bg(theme.title_bar)
                    .border_t_1()
                    .border_color(theme.border)
                    .text_xs()
                    .text_color(theme.muted_foreground)
                    .child(format!("Samples: {}", sample_count))
                    .child(
                        h_flex()
                            .gap_2()
                            .items_center()
                            .child(div().size_2().rounded_full().bg(status_color))
                            .child(status_text),
                    ),
            )
    }
}

pub fn run_app() {
    let app = Application::new().with_assets(Assets);

    app.run(move |cx| {
        gpui_component::init(cx);

        // Register global Cmd+Q handler
        cx.bind_keys([KeyBinding::new("cmd-q", Quit, None)]);
        cx.on_action(|_: &Quit, cx| {
            cx.quit();
        });

        cx.spawn(async move |cx| {
            let window_options = cx.update(|cx| WindowOptions {
                titlebar: Some(TitleBar::title_bar_options()),
                window_bounds: Some(WindowBounds::Windowed(Bounds::centered(
                    None,
                    size(px(1400.), px(800.)),
                    cx,
                ))),
                ..Default::default()
            })?;

            let window = cx.open_window(window_options, |window, cx| {
                let app = cx.new(|cx| ImuViewerApp::new(window, cx));
                cx.new(|cx| Root::new(app, window, cx))
            })?;

            window.update(cx, |_, window, cx| {
                window.activate_window();
                cx.on_release(|_, cx| {
                    cx.quit();
                })
                .detach();
            })?;

            cx.update(|cx| cx.activate(true))?;

            Ok::<_, anyhow::Error>(())
        })
        .detach();
    });
}
