use crate::SimulationState;
use crate::body::OrbitalBodies;
use crate::constants::SPACE_SIZE;
use crate::physics::Kinematics;
use raylib::color::Color;
use raylib::drawing::{RaylibDraw, RaylibDrawHandle};
use std::time::Duration;

pub struct HudParams {
    pub compute_time: Duration,
    pub energy_delta: f64,
}

pub fn draw_hud(
    dh: &mut RaylibDrawHandle,
    simulation_state: &SimulationState,
    bodies: &OrbitalBodies,
    kin: &Box<dyn Kinematics>,
    params: HudParams,
) {
    let mut all_text = vec![kin.name()];

    let n_bodies_text = format!("{0} bodies", bodies.len());
    all_text.push(&n_bodies_text);

    all_text.push(match simulation_state.compute_collisions {
        true => "Collisions on",
        false => "Collisions off",
    });

    all_text.push(match simulation_state.paused {
        true => "Paused",
        false => "",
    });

    let speedup_text = format!("Speedup: {0:.1}", simulation_state.speedup);
    all_text.push(&speedup_text);

    all_text.push(if simulation_state.dt_factor < 0. {
        "Reversed"
    } else {
        ""
    });

    let HudParams {
        compute_time,
        energy_delta,
    } = params;

    let compute_text = format!("{0}ms", compute_time.as_millis());
    all_text.push(&compute_text);

    let energy_delta_text = format!("E: {0:.2} (%)", energy_delta * 100.);
    all_text.push(&energy_delta_text);

    dh.draw_text(
        &all_text.join("  "),
        14,
        SPACE_SIZE as i32 - 14 * 2,
        14,
        Color::WHITE,
    );
}
