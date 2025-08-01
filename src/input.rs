use crate::body::OrbitalBodies;
use crate::camera::{click_in_body, screen_coords_to_universe};
use crate::constants::SPACE_SIZE;
use crate::physics::Kinematics;
use crate::{CameraPosition, SimulationState};
use raylib::RaylibHandle;
use raylib::consts::{KeyboardKey, MouseButton};

/// Handle inputs, return if the window should be closed immediately.
pub fn handle_input<'k>(
    rl: &mut RaylibHandle,
    simulation_state: &mut SimulationState,
    kin: &mut &'k Box<dyn Kinematics>,
    bodies: &OrbitalBodies,
    kinematics: &'k [Box<dyn Kinematics>],
) -> bool {
    let mouse_wheel = rl.get_mouse_wheel_move() as f64;

    if mouse_wheel > 0. {
        simulation_state.scale *= mouse_wheel * 1.1;
    } else if mouse_wheel < 0. {
        simulation_state.scale /= mouse_wheel.abs() * 1.1;
    }

    // Center to the selection position
    if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
        let screen_position = (rl.get_mouse_x(), rl.get_mouse_y());
        let universe_center = simulation_state.get_universe_center(&bodies);
        let screen_center = (SPACE_SIZE / 2) as i32;

        let mut set = false;
        for (i, body) in bodies.tier0.iter() {
            if click_in_body(
                screen_position,
                universe_center,
                screen_center,
                simulation_state.scale,
                body,
            ) {
                simulation_state.camera_position = CameraPosition::BodyRelative(*i);
                set = true;
                break;
            }
        }

        if !set {
            let universe_center = screen_coords_to_universe(
                screen_position,
                simulation_state.scale,
                universe_center,
                screen_center,
            );

            simulation_state.camera_position = CameraPosition::UniverseAbsolute(universe_center);
        }
    }

    match rl.get_key_pressed() {
        Some(KeyboardKey::KEY_K) => {
            simulation_state.kinematics_index =
                (simulation_state.kinematics_index + 1) % kinematics.len();
            *kin = &kinematics[simulation_state.kinematics_index];
        }
        Some(KeyboardKey::KEY_C) => {
            simulation_state.compute_collisions = !simulation_state.compute_collisions;
        }
        Some(KeyboardKey::KEY_Q) => {
            return true;
        }
        Some(KeyboardKey::KEY_P) => {
            simulation_state.paused = !simulation_state.paused;
        }
        Some(KeyboardKey::KEY_R) => {
            simulation_state.dt_factor = -1. * simulation_state.dt_factor;
        }
        Some(KeyboardKey::KEY_EQUAL) => {
            simulation_state.speedup += 0.1;
        }
        Some(KeyboardKey::KEY_MINUS) => {
            simulation_state.speedup -= 0.1;
        }
        _ => (),
    };

    false
}
