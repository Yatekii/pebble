//! Map view using OpenStreetMap tiles.

use gpui::*;
use gpui_component::ActiveTheme;

use crate::ble::{GnssType, SatelliteInfo};

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
    has_fix: bool,
    satellite_info: Vec<SatelliteInfo>,
}

impl MapViewElement {
    pub fn new(position: GpsPosition, has_fix: bool, satellite_info: Vec<SatelliteInfo>) -> Self {
        Self {
            position,
            zoom: 15,
            has_fix,
            satellite_info,
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
        // Show satellites in view (from satellite_info) rather than fix satellites
        let sats_in_view = self.satellite_info.len();
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
                "{:.4}, {:.4} | {} {} in view",
                position.latitude, position.longitude, fix_indicator, sats_in_view
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

        // Build satellite info panel
        let satellite_panel = self.build_satellite_panel(theme);

        // Map container
        let map_container = div()
            .id("map-container")
            .w_full()
            .flex_1()
            .min_h(px(150.0))
            .overflow_hidden()
            .relative()
            .child(tile_grid)
            .child(marker)
            .child(coords)
            .child(corner_tl)
            .child(corner_tr)
            .child(corner_bl)
            .child(corner_br)
            .child(border_overlay);

        // Main container with satellite panel above map
        div()
            .id("map-view")
            .w_full()
            .h_full()
            .flex()
            .flex_col()
            .gap_2()
            .child(satellite_panel)
            .child(map_container)
    }
}

impl MapViewElement {
    /// Build the satellite info panel showing constellation breakdown
    fn build_satellite_panel(&self, theme: &gpui_component::theme::Theme) -> impl IntoElement {
        // Count satellites by constellation
        let mut gps_count = 0u8;
        let mut galileo_count = 0u8;
        let mut beidou_count = 0u8;
        let mut glonass_count = 0u8;

        // Collect satellites with signal (SNR > 0)
        let mut tracked_sats: Vec<&SatelliteInfo> = Vec::new();

        for sat in &self.satellite_info {
            if sat.snr > 0 {
                tracked_sats.push(sat);
            }
            match sat.gnss_type {
                GnssType::Gps => gps_count += 1,
                GnssType::Galileo => galileo_count += 1,
                GnssType::BeiDou => beidou_count += 1,
                GnssType::Glonass => glonass_count += 1,
                _ => {}
            }
        }

        // Sort by SNR descending
        tracked_sats.sort_by(|a, b| b.snr.cmp(&a.snr));

        // Total satellites in view for header
        let total_in_view = self.satellite_info.len();

        // Build satellite bars showing signal strength
        let sat_bars: Vec<_> = tracked_sats
            .iter()
            .take(12) // Show top 12 by signal
            .map(|sat| {
                let color = match sat.gnss_type {
                    GnssType::Gps => hsla(0.6, 0.7, 0.5, 1.0),     // Blue
                    GnssType::Galileo => hsla(0.1, 0.8, 0.5, 1.0), // Orange
                    GnssType::BeiDou => hsla(0.0, 0.8, 0.5, 1.0),  // Red
                    GnssType::Glonass => hsla(0.3, 0.7, 0.5, 1.0), // Green
                    _ => hsla(0.0, 0.0, 0.5, 1.0),                 // Gray
                };
                // SNR typically 0-50 dB-Hz, normalize to bar height
                let bar_height = ((sat.snr as f32 / 50.0) * 24.0).clamp(2.0, 24.0);

                div()
                    .flex()
                    .flex_col()
                    .items_center()
                    .gap_px()
                    .child(div().w(px(8.0)).h(px(bar_height)).bg(color).rounded_t_sm())
                    .child(
                        div()
                            .text_color(theme.muted_foreground)
                            .text_size(px(8.0))
                            .child(format!("{}", sat.prn)),
                    )
            })
            .collect();

        // Legend items for constellations that are present (with counts)
        let mut legend_items: Vec<(String, Hsla)> = Vec::new();
        if gps_count > 0 {
            legend_items.push((format!("GPS:{}", gps_count), hsla(0.6, 0.7, 0.5, 1.0)));
        }
        if galileo_count > 0 {
            legend_items.push((format!("GAL:{}", galileo_count), hsla(0.1, 0.8, 0.5, 1.0)));
        }
        if beidou_count > 0 {
            legend_items.push((format!("BDS:{}", beidou_count), hsla(0.0, 0.8, 0.5, 1.0)));
        }
        if glonass_count > 0 {
            legend_items.push((format!("GLO:{}", glonass_count), hsla(0.3, 0.7, 0.5, 1.0)));
        }

        let legend = div()
            .flex()
            .flex_row()
            .gap_2()
            .children(legend_items.into_iter().map(|(name, color)| {
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap_1()
                    .child(div().w(px(8.0)).h(px(8.0)).bg(color).rounded_sm())
                    .child(
                        div()
                            .text_color(theme.muted_foreground)
                            .text_size(px(9.0))
                            .child(name),
                    )
            }));

        div()
            .id("satellite-panel")
            .w_full()
            .px_2()
            .py_1()
            .bg(theme.background)
            .border_1()
            .border_color(theme.border)
            .rounded_md()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap_1()
                    // Header
                    .child(
                        div()
                            .text_xs()
                            .font_weight(FontWeight::SEMIBOLD)
                            .text_color(theme.foreground)
                            .child(format!("Satellites ({})", total_in_view)),
                    )
                    // Signal strength bars and legend
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_end()
                            .justify_between()
                            .child(
                                div()
                                    .flex()
                                    .flex_row()
                                    .items_end()
                                    .gap_1()
                                    .h(px(36.0))
                                    .children(sat_bars),
                            )
                            .child(legend),
                    ),
            )
    }
}
