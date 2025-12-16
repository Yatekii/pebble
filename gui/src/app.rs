use std::sync::Arc;

use gpui::*;
use gpui_component::{ActiveTheme, Root, TitleBar, h_flex, v_flex};
use gpui_component_assets::Assets;
use parking_lot::Mutex;

use crate::ble::{BleMessage, BleState, ConnectionState, start_ble_client};
use crate::chart::SimpleLineChart;
use crate::data::ImuReading;

actions!(imu_viewer, [Quit]);

pub struct ImuViewerApp {
    focus_handle: FocusHandle,
    ble_state: Arc<Mutex<BleState>>,
    connection_state: ConnectionState,
    use_dummy_data: bool,
}

impl ImuViewerApp {
    pub fn new(window: &mut Window, cx: &mut Context<Self>) -> Self {
        let focus_handle = cx.focus_handle();

        // Create shared BLE state
        let ble_state = Arc::new(Mutex::new(BleState::new()));

        // Pre-fill with some initial data
        {
            let mut state = ble_state.lock();
            for _ in 0..50 {
                state.imu_history.generate_dummy_reading();
            }
        }

        // Start the BLE client
        let ble_rx = Arc::new(Mutex::new(start_ble_client(ble_state.clone())));

        // Poll for BLE messages and generate dummy data
        let ble_state_for_poll = ble_state.clone();
        let ble_rx_for_poll = ble_rx.clone();
        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor()
                    .timer(std::time::Duration::from_millis(16))
                    .await;

                let result = this.update(cx, |this, cx| {
                    // Process all pending BLE messages
                    let rx = ble_rx_for_poll.lock();
                    while let Ok(msg) = rx.try_recv() {
                        this.handle_ble_message(msg);
                    }
                    drop(rx);

                    // Generate dummy data if not connected
                    if this.use_dummy_data && this.connection_state == ConnectionState::Disconnected
                    {
                        ble_state_for_poll
                            .lock()
                            .imu_history
                            .generate_dummy_reading();
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
            use_dummy_data: true,
        }
    }

    fn handle_ble_message(&mut self, msg: BleMessage) {
        match msg {
            BleMessage::StateChanged(state) => {
                self.connection_state = state;
                // Stop generating dummy data once we connect
                if self.connection_state == ConnectionState::Connected {
                    self.use_dummy_data = false;
                }
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

        v_flex()
            .flex_1()
            .p_3()
            .gap_2()
            .bg(theme.background)
            .border_1()
            .border_color(theme.border)
            .rounded_md()
            .child(
                div()
                    .text_sm()
                    .font_weight(FontWeight::SEMIBOLD)
                    .text_color(theme.foreground)
                    .child(title.to_string()),
            )
            // X axis chart
            .child(
                h_flex()
                    .items_center()
                    .gap_1()
                    .child(
                        h_flex()
                            .w(px(20.0))
                            .justify_center()
                            .child(div().size_3().rounded_sm().bg(red)),
                    )
                    .child(
                        div()
                            .flex_1()
                            .h(px(100.0))
                            .child(SimpleLineChart::new(x_data, red)),
                    ),
            )
            // Y axis chart
            .child(
                h_flex()
                    .items_center()
                    .gap_1()
                    .child(
                        h_flex()
                            .w(px(20.0))
                            .justify_center()
                            .child(div().size_3().rounded_sm().bg(green)),
                    )
                    .child(
                        div()
                            .flex_1()
                            .h(px(100.0))
                            .child(SimpleLineChart::new(y_data, green)),
                    ),
            )
            // Z axis chart (with X-axis labels)
            .child(
                h_flex()
                    .items_center()
                    .gap_1()
                    .child(
                        h_flex()
                            .w(px(20.0))
                            .justify_center()
                            .child(div().size_3().rounded_sm().bg(blue)),
                    )
                    .child(
                        div()
                            .flex_1()
                            .h(px(100.0))
                            .child(SimpleLineChart::new(z_data, blue).show_x_axis()),
                    ),
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
            ConnectionState::Disconnected => "Disconnected (using dummy data)".to_string(),
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
            .child(
                // Main content with three charts
                h_flex()
                    .flex_1()
                    .p_4()
                    .gap_4()
                    .child(self.render_sensor_chart("Accelerometer (raw)", &accel, cx))
                    .child(self.render_sensor_chart("Gyroscope (raw)", &gyro, cx))
                    .child(self.render_sensor_chart("Magnetometer (raw)", &mag, cx)),
            )
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
                    size(px(1400.), px(600.)),
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
