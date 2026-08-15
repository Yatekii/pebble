//! Device status panel UI component showing peripheral initialization status.

use gpui::prelude::FluentBuilder;
use gpui::*;
use gpui_component::ActiveTheme;

use crate::ble::{DeviceStatusData, PeripheralStatusData};

/// Device status panel element
#[derive(IntoElement)]
pub struct DeviceStatusPanel {
    status: Option<DeviceStatusData>,
    /// Battery voltage in millivolts, if known.
    battery_mv: Option<u16>,
}

impl DeviceStatusPanel {
    pub fn new(status: Option<DeviceStatusData>, battery_mv: Option<u16>) -> Self {
        Self { status, battery_mv }
    }

    fn render_battery(&self, cx: &App) -> impl IntoElement {
        let theme = cx.theme();
        let value = match self.battery_mv {
            Some(mv) => format!("{:.2} V", mv as f32 / 1000.0),
            None => "--".to_string(),
        };
        div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .py_1()
            .child(div().text_sm().text_color(theme.foreground).child("Battery"))
            .child(
                div()
                    .text_sm()
                    .font_weight(FontWeight::BOLD)
                    .text_color(theme.foreground)
                    .child(value),
            )
    }
}

impl RenderOnce for DeviceStatusPanel {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let border_color = theme.border;

        div()
            .id("device-status")
            .w_full()
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
                    .child("Device Status"),
            )
            .child(self.render_battery(cx))
            .child(self.render_status_list(cx))
    }
}

impl DeviceStatusPanel {
    fn render_status_list(&self, cx: &App) -> impl IntoElement {
        let theme = cx.theme();

        if let Some(status) = &self.status {
            div()
                .flex()
                .flex_col()
                .gap_1()
                .child(PeripheralStatusRow::new("LEDs", status.leds))
                .child(PeripheralStatusRow::new("GPS", status.gps))
                .child(PeripheralStatusRow::new("Servo", status.servo))
                .child(PeripheralStatusRow::new("IMU", status.imu))
                .child(PeripheralStatusRow::new(
                    "Magnetometer",
                    status.magnetometer,
                ))
        } else {
            div()
                .text_sm()
                .text_color(theme.muted_foreground)
                .child("Waiting for device connection...")
        }
    }
}

/// Individual peripheral status row
#[derive(IntoElement)]
struct PeripheralStatusRow {
    name: &'static str,
    status: PeripheralStatusData,
}

impl PeripheralStatusRow {
    fn new(name: &'static str, status: PeripheralStatusData) -> Self {
        Self { name, status }
    }
}

impl RenderOnce for PeripheralStatusRow {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();

        let (icon, icon_color) = match self.status.status {
            0 => ("-", theme.muted_foreground),      // NotInitialized
            1 => ("OK", hsla(0.33, 0.7, 0.45, 1.0)), // Ok - green
            2 => ("ERR", hsla(0.0, 0.7, 0.5, 1.0)),  // Error - red
            _ => ("?", theme.muted_foreground),      // Unknown
        };

        let text_color = match self.status.status {
            1 => theme.foreground,
            2 => hsla(0.0, 0.7, 0.5, 1.0), // Error text in red
            _ => theme.muted_foreground,
        };

        let error_text = self.status.error_text();

        div()
            .flex()
            .flex_col()
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .justify_between()
                    .py_1()
                    .child(div().text_sm().text_color(text_color).child(self.name))
                    .child(
                        div()
                            .text_xs()
                            .font_weight(FontWeight::BOLD)
                            .text_color(icon_color)
                            .child(icon),
                    ),
            )
            .when_some(error_text, |this, err| {
                this.child(
                    div()
                        .text_xs()
                        .text_color(hsla(0.0, 0.6, 0.6, 1.0))
                        .pl_2()
                        .child(err),
                )
            })
    }
}
