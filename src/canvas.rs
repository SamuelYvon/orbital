use crate::SimulationState;
use crate::body::OrbitalBodies;
use crate::constants::SPACE_SIZE;
use crate::physics::Kinematics;
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};

pub fn draw_hud(
    dh: &mut RaylibDrawHandle,
    simulation_state: &SimulationState,
    bodies: &OrbitalBodies,
    kin: &Box<dyn Kinematics>,
) {
    let mut total_text_width = 0;
    dh.draw_text(kin.name(), 14, SPACE_SIZE as i32 - 14 * 2, 14, Color::WHITE);
    total_text_width += dh.measure_text(kin.name(), 14);

    let bodies_text = &format!("{0} bodies", bodies.len());
    dh.draw_text(
        bodies_text,
        total_text_width + 2 * 14,
        SPACE_SIZE as i32 - 14 * 2,
        14,
        Color::WHITE,
    );
    total_text_width += dh.measure_text(bodies_text, 14);

    let collisions_text = match simulation_state.compute_collisions {
        true => "Collisions on",
        false => "Collisions off",
    };
    dh.draw_text(
        collisions_text,
        total_text_width + 4 * 14,
        SPACE_SIZE as i32 - 14 * 2,
        14,
        Color::WHITE,
    );
    total_text_width += dh.measure_text(collisions_text, 14);

    let paused_text = match simulation_state.paused {
        true => "Paused",
        false => "",
    };

    dh.draw_text(
        paused_text,
        total_text_width + 6 * 14,
        SPACE_SIZE as i32 - 14 * 2,
        14,
        Color::WHITE,
    );
    total_text_width += dh.measure_text(paused_text, 14);
}
