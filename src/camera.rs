use crate::body::Body;
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};

pub struct ScaleInfo {
    pub scale: f32,
    pub center: (f32, f32),
}

#[inline]
fn pos_to_canvas(pos: (f32, f32), scale: &ScaleInfo) -> (i32, i32) {
    // Move every body along the shift applied to the main one
    let (x, y) = pos;
    let (x, y) = (x - scale.center.0, y - scale.center.1);
    let (x, y) = (x * scale.scale, y * scale.scale);
    let (x, y) = ((x + scale.center.0) as i32, (y + scale.center.1) as i32);

    (x, y)
}

/// Draw a body on the canvas, taking into account scaling and shifting to the center of the
/// visible area.
fn draw_body(body: &Body, scale: &ScaleInfo, raylib_draw_handle: &mut RaylibDrawHandle) {
    let height = raylib_draw_handle.get_screen_height();

    let (x, y) = pos_to_canvas(body.pos(), scale);

    if x >= height || y >= height || y < 0 || x < 0 {
        return;
    }

    for i in 1..body.pos_list.len() {
        let prev = pos_to_canvas(body.pos_list[i - 1], scale);
        let next = pos_to_canvas(body.pos_list[i], scale);
        raylib_draw_handle.draw_line(prev.0, prev.1, next.0, next.1, Color::WHITE);
    }

    raylib_draw_handle.draw_circle(x, y, body.radius, body.color);
}

pub fn camera_fixed(
    bodies: &[Body],
    raylib_draw_handle: &mut RaylibDrawHandle,
    scale_info: &ScaleInfo,
) {
    for body in bodies.iter() {
        draw_body(body, &scale_info, raylib_draw_handle);
    }
}

// pub fn camera_tracking(
//     center: &Body,
//     bodies: &[Body],
//     raylib_draw_handle: &mut RaylibDrawHandle,
//     scale_info: &ScaleInfo,
// ) {
//     let height = raylib_draw_handle.get_screen_height();
//
//     let middle = (height / 2) as f32;
//     let pos = center.pos();
//
//     let dx = pos.0 - middle;
//     let dy = pos.1 - middle;
//
//     for body in bodies.iter() {
//         draw_body(body, &scale_info, raylib_draw_handle, Some((dx, dy)));
//     }
// }
