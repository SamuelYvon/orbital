use crate::body::{Body, OrbitalBodies, TrailParameter};
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};
use ringbuffer::RingBuffer;

/// Converts coordinates from the universe into coordinates to the screen
#[inline]
pub fn universe_coord_to_screen(
    universe_coords: (f32, f32),
    scale: f32,
    universe_center: (f32, f32),
    screen_center: i32,
) -> (i32, i32) {
    let (x, y) = universe_coords;
    let (ux, uy) = universe_center;
    let (dx, dy) = (x - ux, y - uy);
    let (scaled_x, scaled_y) = (dx * scale, dy * scale);
    let (screen_x, screen_y) = (
        scaled_x + screen_center as f32,
        scaled_y + screen_center as f32,
    );

    (screen_x as i32, screen_y as i32)
}

/// Convert screen coordinates to universe coordinates
#[inline]
pub fn screen_coords_to_universe(
    screen_coords: (i32, i32),
    scale: f32,
    universe_center: (f32, f32),
    screen_center: i32,
) -> (f32, f32) {
    let (x, y) = screen_coords;
    let (dx, dy) = (x - screen_center, y - screen_center);
    let (scaled_dx, scaled_dy) = (dx as f32 / scale, dy as f32 / scale);

    let (ux, uy) = universe_center;

    (ux + scaled_dx, uy + scaled_dy)
}

fn draw_body_lines(
    handle: &mut RaylibDrawHandle,
    body_lines: &[(f32, f32)],
    universe_center: (f32, f32),
    scale: f32,
) {
    let boundary = handle.get_screen_height();
    let screen_center = boundary / 2;

    for i in 1..body_lines.len() {
        let prev =
            universe_coord_to_screen(body_lines[i - 1], scale, universe_center, screen_center);
        let next = universe_coord_to_screen(body_lines[i], scale, universe_center, screen_center);

        handle.draw_line(prev.0, prev.1, next.0, next.1, Color::WHITE)
    }
}

pub fn draw_universe_relative(
    handle: &mut RaylibDrawHandle,
    bodies: &OrbitalBodies,
    universe_center: (f32, f32),
    scale: f32,
) {
    let boundary = handle.get_screen_height();
    let screen_center = boundary / 2;

    for body in bodies.iter() {
        let (screen_x, screen_y) =
            universe_coord_to_screen(body.pos(), scale, universe_center, screen_center);

        // TODO: you can do this better
        if body.trail_parameter == TrailParameter::Trail {
            draw_body_lines(
                handle,
                body.pos_list.iter().cloned().collect::<Vec<_>>().as_ref(),
                universe_center,
                scale,
            );
        }

        // Outside the range
        if screen_x >= boundary || screen_y >= boundary || screen_x < 0 || screen_y < 0 {
            continue;
        }

        handle.draw_circle(screen_x, screen_y, body.radius, body.color);
    }
}

pub fn click_in_body(
    screen_pos: (i32, i32),
    universe_center: (f32, f32),
    screen_center: i32,
    scale: f32,
    body: &Body,
) -> bool {
    // TODO: this will take into account the scaling, but the drawing of the radius does not.
    let universe_diameter = body.radius / scale;

    // Convert the click to universe pos
    let (cx, cy) = screen_coords_to_universe(screen_pos, scale, universe_center, screen_center);

    let (bx, by) = body.pos();

    // Check if within radius
    ((bx - cx).powf(2.) + (by - cy).powf(2.)).sqrt() <= universe_diameter
}
