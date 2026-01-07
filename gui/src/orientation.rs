//! 3D orientation visualization using a wireframe PCB model.

use std::sync::Arc;

use gpui::*;
use gpui_component::ActiveTheme;

use crate::ble::LedColors;
use crate::mesh::Mesh3D;

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

/// A wireframe 3D visualization for device orientation.
#[derive(IntoElement)]
pub struct OrientationView {
    orientation: Orientation,
    led_colors: LedColors,
    mesh: Option<Arc<Mesh3D>>,
}

impl OrientationView {
    pub fn new(orientation: Orientation, led_colors: LedColors, mesh: Option<Arc<Mesh3D>>) -> Self {
        Self {
            orientation,
            led_colors,
            mesh,
        }
    }
}

impl RenderOnce for OrientationView {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let orientation = self.orientation;
        let led_colors = self.led_colors;
        let mesh = self.mesh;
        let border_color = theme.border;

        div()
            .w_full()
            .h_full()
            .min_h(px(150.0))
            .bg(hsla(0.0, 0.0, 0.15, 1.0)) // Dark background
            .border_1()
            .border_color(border_color)
            .rounded_md()
            .child(
                canvas(
                    move |bounds, _window, _cx| (bounds, orientation, led_colors, mesh),
                    move |_bounds, (bounds, orientation, led_colors, mesh), window, _cx| {
                        if bounds.size.width <= px(0.0) || bounds.size.height <= px(0.0) {
                            return;
                        }

                        let center_x = bounds.origin.x + bounds.size.width * 0.5;
                        let center_y = bounds.origin.y + bounds.size.height * 0.5;
                        let size = bounds.size.width.min(bounds.size.height) * 0.8;
                        let base_scale = f32::from(size) as f64;
                        let distance = 4.0;

                        let edge_color = hsla(0.0, 0.0, 0.5, 0.6);

                        // Helper to draw a line
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

                        // Render the mesh if available
                        if let Some(mesh) = &mesh {
                            // Calculate scale to fit mesh in view
                            let mesh_scale = 1.0 / mesh.max_dimension() as f64;
                            let center = mesh.center();
                            let scale = base_scale * mesh_scale;

                            // Transform and project all vertices
                            let projected: Vec<(f64, f64)> = mesh
                                .vertices
                                .iter()
                                .map(|v| {
                                    // Center the mesh, then apply scale and rotation
                                    let p = Point3D::new(
                                        (v.x - center.x) as f64,
                                        (v.y - center.y) as f64,
                                        (v.z - center.z) as f64,
                                    );
                                    p.rotate(orientation).project(scale, distance)
                                })
                                .collect();

                            // Batch edges into chunks for performance
                            // (single huge path may fail, so we chunk it)
                            const BATCH_SIZE: usize = 500;
                            for chunk in mesh.edges.chunks(BATCH_SIZE) {
                                let mut builder = PathBuilder::stroke(px(0.5));
                                for edge in chunk {
                                    if edge.0 < projected.len() && edge.1 < projected.len() {
                                        let p1 = projected[edge.0];
                                        let p2 = projected[edge.1];
                                        builder.move_to(point(
                                            center_x + px(p1.0 as f32),
                                            center_y - px(p1.1 as f32),
                                        ));
                                        builder.line_to(point(
                                            center_x + px(p2.0 as f32),
                                            center_y - px(p2.1 as f32),
                                        ));
                                    }
                                }
                                if let Ok(path) = builder.build() {
                                    window.paint_path(path, edge_color);
                                }
                            }
                        } else {
                            // Fallback: draw a simple box if no mesh
                            let hw = 0.5;
                            let hh = 0.5;
                            let hd = 0.08;

                            let vertices = [
                                Point3D::new(-hw, -hh, -hd),
                                Point3D::new(hw, -hh, -hd),
                                Point3D::new(hw, hh, -hd),
                                Point3D::new(-hw, hh, -hd),
                                Point3D::new(-hw, -hh, hd),
                                Point3D::new(hw, -hh, hd),
                                Point3D::new(hw, hh, hd),
                                Point3D::new(-hw, hh, hd),
                            ];

                            let projected: Vec<(f64, f64)> = vertices
                                .iter()
                                .map(|v| v.rotate(orientation).project(base_scale, distance))
                                .collect();

                            let edges = [
                                (0, 1),
                                (1, 2),
                                (2, 3),
                                (3, 0),
                                (4, 5),
                                (5, 6),
                                (6, 7),
                                (7, 4),
                                (0, 4),
                                (1, 5),
                                (2, 6),
                                (3, 7),
                            ];

                            for (i, j) in edges {
                                draw_line(window, projected[i], projected[j], edge_color, 1.5);
                            }
                        }

                        // Draw axis indicators
                        let axis_len = 0.3;
                        let axis_origin = Point3D::new(0.0, 0.0, 0.0)
                            .rotate(orientation)
                            .project(base_scale, distance);
                        let x_axis = Point3D::new(axis_len, 0.0, 0.0)
                            .rotate(orientation)
                            .project(base_scale, distance);
                        let y_axis = Point3D::new(0.0, axis_len, 0.0)
                            .rotate(orientation)
                            .project(base_scale, distance);
                        let z_axis = Point3D::new(0.0, 0.0, axis_len)
                            .rotate(orientation)
                            .project(base_scale, distance);

                        let red = hsla(0.0, 0.7, 0.5, 1.0);
                        let green = hsla(0.33, 0.7, 0.45, 1.0);
                        let blue = hsla(0.6, 0.7, 0.5, 1.0);

                        draw_line(window, axis_origin, x_axis, red, 2.5);
                        draw_line(window, axis_origin, y_axis, green, 2.5);
                        draw_line(window, axis_origin, z_axis, blue, 2.5);

                        // Draw LEDs in a circle - radius should match the PCB's LED ring
                        // The mesh is normalized to fit in ~1 unit, LEDs are near the outer edge
                        let led_radius = if mesh.is_some() { 0.42 } else { 0.25 };
                        let num_leds = 72;

                        for i in 0..num_leds {
                            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (num_leds as f64);
                            let led_point = Point3D::new(
                                led_radius * angle.cos(),
                                led_radius * angle.sin(),
                                0.02, // Slightly above the PCB surface
                            );

                            let rotated = led_point.rotate(orientation);
                            let projected = rotated.project(base_scale, distance);

                            let [r, g, b] = led_colors[i];
                            let led_color: Hsla = gpui::Rgba {
                                r: r as f32 / 255.0,
                                g: g as f32 / 255.0,
                                b: b as f32 / 255.0,
                                a: 1.0,
                            }
                            .into();

                            let led_x = center_x + px(projected.0 as f32);
                            let led_y = center_y - px(projected.1 as f32);
                            let led_size = px(6.0);

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
