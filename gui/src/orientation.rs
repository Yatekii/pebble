//! 3D orientation visualization using a wireframe cube.

use gpui::*;
use gpui_component::ActiveTheme;

use crate::ble::LedColors;

/// Represents a 3D rotation as Euler angles (in radians).
#[derive(Clone, Copy, Debug)]
pub struct Orientation {
    pub roll: f64,  // rotation around X axis
    pub pitch: f64, // rotation around Y axis
    pub yaw: f64,   // rotation around Z axis
}

impl Default for Orientation {
    fn default() -> Self {
        // Default to a slight tilt so the 3D shape is visible
        Self {
            roll: 0.4,
            pitch: 0.3,
            yaw: 0.2,
        }
    }
}

/// A 3D point.
#[derive(Clone, Copy)]
struct Point3D {
    x: f64,
    y: f64,
    z: f64,
}

impl Point3D {
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Rotate around X axis (roll)
    fn rotate_x(self, angle: f64) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            x: self.x,
            y: self.y * cos - self.z * sin,
            z: self.y * sin + self.z * cos,
        }
    }

    /// Rotate around Y axis (pitch)
    fn rotate_y(self, angle: f64) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            x: self.x * cos + self.z * sin,
            y: self.y,
            z: -self.x * sin + self.z * cos,
        }
    }

    /// Rotate around Z axis (yaw)
    fn rotate_z(self, angle: f64) -> Self {
        let cos = angle.cos();
        let sin = angle.sin();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
            z: self.z,
        }
    }

    /// Apply all rotations
    fn rotate(self, orientation: Orientation) -> Self {
        self.rotate_x(orientation.roll)
            .rotate_y(orientation.pitch)
            .rotate_z(orientation.yaw)
    }

    /// Project to 2D with perspective
    fn project(self, scale: f64, distance: f64) -> (f64, f64) {
        let factor = distance / (distance + self.z);
        (self.x * scale * factor, self.y * scale * factor)
    }
}

/// A wireframe 3D box visualization for device orientation.
#[derive(IntoElement)]
pub struct OrientationView {
    orientation: Orientation,
    led_colors: LedColors,
}

impl OrientationView {
    pub fn new(orientation: Orientation, led_colors: LedColors) -> Self {
        Self {
            orientation,
            led_colors,
        }
    }
}

impl RenderOnce for OrientationView {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let orientation = self.orientation;
        let led_colors = self.led_colors;
        let border_color = theme.border;

        div()
            .w_full()
            .h_full()
            .min_h(px(150.0))
            .bg(hsla(0.0, 0.0, 0.25, 1.0)) // Dark gray background to see black LEDs
            .border_1()
            .border_color(border_color)
            .rounded_md()
            .child(
                canvas(
                    move |bounds, _window, _cx| (bounds, orientation, led_colors),
                    move |_bounds, (bounds, orientation, led_colors), window, _cx| {
                        if bounds.size.width <= px(0.0) || bounds.size.height <= px(0.0) {
                            return;
                        }

                        let center_x = bounds.origin.x + bounds.size.width * 0.5;
                        let center_y = bounds.origin.y + bounds.size.height * 0.5;
                        let size = bounds.size.width.min(bounds.size.height) * 0.7; // Fill more of the space
                        let scale = f32::from(size) as f64;
                        let distance = 4.0;

                        // LED circle radius
                        let led_radius = 0.45;

                        // Define a square box tight around the LED circle
                        let hw = led_radius + 0.08; // half width (X) - slightly larger than LED circle
                        let hh = led_radius + 0.08; // half height (Y) - same as width for square
                        let hd = 0.08; // half depth (Z) - thin like a PCB

                        let vertices = [
                            Point3D::new(-hw, -hh, -hd), // 0: back bottom left
                            Point3D::new(hw, -hh, -hd),  // 1: back bottom right
                            Point3D::new(hw, hh, -hd),   // 2: back top right
                            Point3D::new(-hw, hh, -hd),  // 3: back top left
                            Point3D::new(-hw, -hh, hd),  // 4: front bottom left
                            Point3D::new(hw, -hh, hd),   // 5: front bottom right
                            Point3D::new(hw, hh, hd),    // 6: front top right
                            Point3D::new(-hw, hh, hd),   // 7: front top left
                        ];

                        // Rotate all vertices
                        let rotated: Vec<Point3D> =
                            vertices.iter().map(|v| v.rotate(orientation)).collect();

                        // Project to 2D
                        let projected: Vec<(f64, f64)> =
                            rotated.iter().map(|v| v.project(scale, distance)).collect();

                        // Define edges with different colors for each axis
                        // Back face edges
                        let back_edges = [(0, 1), (1, 2), (2, 3), (3, 0)];
                        // Front face edges
                        let front_edges = [(4, 5), (5, 6), (6, 7), (7, 4)];
                        // Connecting edges
                        let connect_edges = [(0, 4), (1, 5), (2, 6), (3, 7)];

                        // X-axis indicator (red) - from center to +X
                        let axis_len = hw + 0.15;
                        let axis_origin = Point3D::new(0.0, 0.0, 0.0)
                            .rotate(orientation)
                            .project(scale, distance);
                        let x_axis = Point3D::new(axis_len, 0.0, 0.0)
                            .rotate(orientation)
                            .project(scale, distance);
                        let y_axis = Point3D::new(0.0, axis_len, 0.0)
                            .rotate(orientation)
                            .project(scale, distance);
                        let z_axis = Point3D::new(0.0, 0.0, hd + 0.15)
                            .rotate(orientation)
                            .project(scale, distance);

                        let red = hsla(0.0, 0.7, 0.5, 1.0);
                        let green = hsla(0.33, 0.7, 0.45, 1.0);
                        let blue = hsla(0.6, 0.7, 0.5, 1.0);
                        let edge_color = hsla(0.0, 0.0, 0.6, 0.8);

                        // Generate 72 LEDs in a circle on the top face (Z = hd)
                        let num_leds = 72;
                        let led_points: Vec<Point3D> = (0..num_leds)
                            .map(|i| {
                                let angle =
                                    2.0 * std::f64::consts::PI * (i as f64) / (num_leds as f64);
                                Point3D::new(
                                    led_radius * angle.cos(),
                                    led_radius * angle.sin(),
                                    hd + 0.01, // Slightly above the top face
                                )
                            })
                            .collect();

                        // Helper to draw a filled circle (LED dot)
                        let draw_line = |window: &mut Window,
                                         p1: (f64, f64),
                                         p2: (f64, f64),
                                         color: Hsla,
                                         width: f32| {
                            let mut builder = PathBuilder::stroke(px(width));
                            builder.move_to(point(
                                center_x + px(p1.0 as f32),
                                center_y - px(p1.1 as f32),
                            ));
                            builder.line_to(point(
                                center_x + px(p2.0 as f32),
                                center_y - px(p2.1 as f32),
                            ));
                            if let Ok(path) = builder.build() {
                                window.paint_path(path, color);
                            }
                        };

                        // Draw back face
                        for (i, j) in back_edges {
                            draw_line(window, projected[i], projected[j], edge_color, 1.5);
                        }

                        // Draw connecting edges
                        for (i, j) in connect_edges {
                            draw_line(window, projected[i], projected[j], edge_color, 1.5);
                        }

                        // Draw front face
                        for (i, j) in front_edges {
                            draw_line(window, projected[i], projected[j], edge_color, 2.0);
                        }

                        // Draw axis indicators
                        draw_line(window, axis_origin, x_axis, red, 2.5);
                        draw_line(window, axis_origin, y_axis, green, 2.5);
                        draw_line(window, axis_origin, z_axis, blue, 2.5);

                        // Draw LEDs as small colored dots
                        for (i, led_point) in led_points.iter().enumerate() {
                            let rotated = led_point.rotate(orientation);
                            let projected = rotated.project(scale, distance);

                            // Use actual LED color from BLE data (RGB bytes to normalized floats)
                            let [r, g, b] = led_colors[i];
                            let led_color: Hsla = gpui::Rgba {
                                r: r as f32 / 255.0,
                                g: g as f32 / 255.0,
                                b: b as f32 / 255.0,
                                a: 1.0,
                            }
                            .into();

                            // Draw LED as a small filled circle
                            let led_x = center_x + px(projected.0 as f32);
                            let led_y = center_y - px(projected.1 as f32);
                            let led_size = px(8.0);

                            // Use a small quad for the LED
                            window.paint_quad(gpui::fill(
                                gpui::Bounds::new(
                                    gpui::point(led_x - led_size / 2.0, led_y - led_size / 2.0),
                                    gpui::size(led_size, led_size),
                                ),
                                led_color,
                            ));
                        }
                    },
                )
                .size_full(),
            )
    }
}
