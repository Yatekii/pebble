//! Map view using OpenStreetMap tiles.

use gpui::*;
use gpui_component::ActiveTheme;

/// GPS coordinates
#[derive(Clone, Copy, Debug, Default)]
pub struct GpsPosition {
    pub latitude: f64,
    pub longitude: f64,
}

/// Convert lat/lon to tile coordinates at a given zoom level
/// Returns (tile_x, tile_y, pixel_offset_x, pixel_offset_y)
fn lat_lon_to_tile(lat: f64, lon: f64, zoom: u32) -> (u32, u32, f64, f64) {
    let n = 2_f64.powi(zoom as i32);
    let x = (lon + 180.0) / 360.0 * n;
    let lat_rad = lat.to_radians();
    let y = (1.0 - lat_rad.tan().asinh() / std::f64::consts::PI) / 2.0 * n;

    let x_tile = x.floor() as u32;
    let y_tile = y.floor() as u32;

    // Pixel offset within tile (0-256)
    let x_offset = (x - x_tile as f64) * 256.0;
    let y_offset = (y - y_tile as f64) * 256.0;

    (x_tile, y_tile, x_offset, y_offset)
}

/// Get OSM tile URL
fn tile_url(x: u32, y: u32, zoom: u32) -> String {
    format!("https://tile.openstreetmap.org/{}/{}/{}.png", zoom, x, y)
}

/// Map view element that displays OSM tiles with a marker
#[derive(IntoElement)]
pub struct MapViewElement {
    position: GpsPosition,
    zoom: u32,
    satellites: u8,
    has_fix: bool,
}

impl MapViewElement {
    pub fn new(position: GpsPosition, satellites: u8, has_fix: bool) -> Self {
        Self {
            position,
            zoom: 15,
            satellites,
            has_fix,
        }
    }
}

impl RenderOnce for MapViewElement {
    fn render(self, _window: &mut Window, cx: &mut App) -> impl IntoElement {
        let theme = cx.theme();
        let position = self.position;
        let zoom = self.zoom;
        let border_color = theme.border;
        let bg_color = theme.background;

        // Calculate tile coordinates
        let (tile_x, tile_y, offset_x, offset_y) =
            lat_lon_to_tile(position.latitude, position.longitude, zoom);

        let offset_x = offset_x as f32;
        let offset_y = offset_y as f32;

        // Grid size - use enough tiles to cover any reasonable view
        let grid_size = 5i32;
        let half_grid = grid_size / 2;

        // Corner radius
        let radius = px(6.0);

        // Build the tile grid
        let tile_grid = div()
            .id("tile-grid")
            .absolute()
            .inset_0()
            .flex()
            .items_center()
            .justify_center()
            .child(
                div()
                    .relative()
                    .left(px(128.0 - offset_x))
                    .top(px(128.0 - offset_y))
                    .flex()
                    .flex_col()
                    .children((0..grid_size).map(|row| {
                        div()
                            .flex()
                            .flex_row()
                            .children((0..grid_size).map(move |col| {
                                let dx = col - half_grid;
                                let dy = row - half_grid;
                                let tx = (tile_x as i32 + dx) as u32;
                                let ty = (tile_y as i32 + dy) as u32;
                                let url = tile_url(tx, ty, zoom);

                                img(url).w(px(256.0)).h(px(256.0)).flex_shrink_0()
                            }))
                    })),
            );

        // Build the marker
        let marker = div()
            .id("marker")
            .absolute()
            .inset_0()
            .flex()
            .items_center()
            .justify_center()
            .child(
                div()
                    .w(px(16.0))
                    .h(px(16.0))
                    .rounded_full()
                    .bg(hsla(0.0, 0.8, 0.5, 1.0))
                    .border_2()
                    .border_color(hsla(0.0, 0.0, 1.0, 1.0))
                    .shadow_md(),
            );

        // Build the coordinates overlay
        let fix_indicator = if self.has_fix { "●" } else { "○" };
        let coords = div()
            .absolute()
            .bottom_2()
            .left_2()
            .px_2()
            .py_1()
            .bg(hsla(0.0, 0.0, 0.0, 0.7))
            .rounded_sm()
            .text_xs()
            .text_color(hsla(0.0, 0.0, 1.0, 1.0))
            .child(format!(
                "{:.4}, {:.4} | {} {} sats",
                position.latitude, position.longitude, fix_indicator, self.satellites
            ));

        // Corner masks - draw filled paths that cover the corners outside the rounded area
        let corner_size = radius + px(2.0);

        // Top-left corner mask
        let corner_tl = canvas(
            move |bounds, _window, _cx| bounds,
            move |bounds, _data, window, _cx| {
                let mut builder = PathBuilder::fill();
                let origin = bounds.origin;
                // Start at top-left
                builder.move_to(origin);
                // Go right along top edge
                builder.line_to(point(origin.x + corner_size, origin.y));
                // Arc to bottom of corner area
                builder.arc_to(
                    point(radius, radius), // radii
                    px(0.0),               // x_rotation
                    false,                 // large_arc
                    false,                 // sweep (counter-clockwise)
                    point(origin.x, origin.y + corner_size),
                );
                // Close
                builder.line_to(origin);
                if let Ok(path) = builder.build() {
                    window.paint_path(path, bg_color);
                }
            },
        )
        .absolute()
        .top_0()
        .left_0()
        .w(corner_size)
        .h(corner_size);

        // Top-right corner mask
        let corner_tr = canvas(
            move |bounds, _window, _cx| bounds,
            move |bounds, _data, window, _cx| {
                let mut builder = PathBuilder::fill();
                let right = bounds.origin.x + bounds.size.width;
                let top = bounds.origin.y;
                // Start at top-right
                builder.move_to(point(right, top));
                // Go down along right edge
                builder.line_to(point(right, top + corner_size));
                // Arc to left of corner area
                builder.arc_to(
                    point(radius, radius),
                    px(0.0),
                    false,
                    false,
                    point(right - corner_size, top),
                );
                // Close
                builder.line_to(point(right, top));
                if let Ok(path) = builder.build() {
                    window.paint_path(path, bg_color);
                }
            },
        )
        .absolute()
        .top_0()
        .right_0()
        .w(corner_size)
        .h(corner_size);

        // Bottom-left corner mask
        let corner_bl = canvas(
            move |bounds, _window, _cx| bounds,
            move |bounds, _data, window, _cx| {
                let mut builder = PathBuilder::fill();
                let left = bounds.origin.x;
                let bottom = bounds.origin.y + bounds.size.height;
                // Start at bottom-left
                builder.move_to(point(left, bottom));
                // Go up along left edge
                builder.line_to(point(left, bottom - corner_size));
                // Arc to right of corner area
                builder.arc_to(
                    point(radius, radius),
                    px(0.0),
                    false,
                    false,
                    point(left + corner_size, bottom),
                );
                // Close
                builder.line_to(point(left, bottom));
                if let Ok(path) = builder.build() {
                    window.paint_path(path, bg_color);
                }
            },
        )
        .absolute()
        .bottom_0()
        .left_0()
        .w(corner_size)
        .h(corner_size);

        // Bottom-right corner mask
        let corner_br = canvas(
            move |bounds, _window, _cx| bounds,
            move |bounds, _data, window, _cx| {
                let mut builder = PathBuilder::fill();
                let right = bounds.origin.x + bounds.size.width;
                let bottom = bounds.origin.y + bounds.size.height;
                // Start at bottom-right
                builder.move_to(point(right, bottom));
                // Go left along bottom edge
                builder.line_to(point(right - corner_size, bottom));
                // Arc to top of corner area
                builder.arc_to(
                    point(radius, radius),
                    px(0.0),
                    false,
                    false,
                    point(right, bottom - corner_size),
                );
                // Close
                builder.line_to(point(right, bottom));
                if let Ok(path) = builder.build() {
                    window.paint_path(path, bg_color);
                }
            },
        )
        .absolute()
        .bottom_0()
        .right_0()
        .w(corner_size)
        .h(corner_size);

        // Border overlay with rounded corners
        let border_overlay = div()
            .absolute()
            .inset_0()
            .rounded_md()
            .border_1()
            .border_color(border_color);

        // Main container
        div()
            .id("map-view")
            .w_full()
            .h_full()
            .min_h(px(200.0))
            .overflow_hidden()
            .relative()
            .child(tile_grid)
            .child(marker)
            .child(coords)
            .child(corner_tl)
            .child(corner_tr)
            .child(corner_bl)
            .child(corner_br)
            .child(border_overlay)
    }
}
