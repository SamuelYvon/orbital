use crate::body::Body;
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};

/// Converts coordinates from the universe into coordinates to the screen
#[inline]
fn universe_coord_to_screen(
    coords: (f32, f32),
    scale: f32,
    universe_center: (f32, f32),
    screen_center: f32,
) -> (f32, f32) {
    let (x, y) = coords;
    let (ux, uy) = universe_center;
    let (dx, dy) = (x - ux, y - uy);
    let (scaled_x, scaled_y) = (dx * scale, dy * scale);
    let (screen_x, screen_y) = (scaled_x + screen_center, scaled_y + screen_center);

    (screen_x, screen_y)
}

fn draw_body_lines(
    handle: &mut RaylibDrawHandle,
    body_lines: &[(f32, f32)],
    universe_center: (f32, f32),
    scale: f32,
) {
    let boundary = handle.get_screen_height() as f32;
    let screen_center = boundary / 2.;

    for i in 1..body_lines.len() {
        let prev =
            universe_coord_to_screen(body_lines[i - 1], scale, universe_center, screen_center);
        let next = universe_coord_to_screen(body_lines[i], scale, universe_center, screen_center);

        handle.draw_line(
            prev.0 as i32,
            prev.1 as i32,
            next.0 as i32,
            next.1 as i32,
            Color::WHITE,
        )
    }
}

pub fn draw(
    handle: &mut RaylibDrawHandle,
    bodies: &[Body],
    universe_center: (f32, f32),
    scale: f32,
) {
    let boundary = handle.get_screen_height() as f32;
    let screen_center = boundary / 2.;

    for body in bodies {
        let (screen_x, screen_y) =
            universe_coord_to_screen(body.pos(), scale, universe_center, screen_center);

        // Outside the range
        if screen_x >= boundary || screen_y >= boundary || screen_x < 0. || screen_y < 0. {
            continue;
        }

        handle.draw_circle(screen_x as i32, screen_y as i32, body.radius, body.color);

        draw_body_lines(handle, &body.pos_list, universe_center, scale);
    }
}
