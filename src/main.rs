mod body;
mod camera;
mod canvas;
mod constants;
mod physics;

use crate::body::{Body, BodyId, OrbitalBodies, bodies_to_map, create_asteroid_belt};
use crate::camera::{click_in_body, draw_universe_relative, screen_coords_to_universe};
use crate::canvas::draw_hud;
use crate::constants::{SPACE_SIZE, SUN_EARTH_DISTANCE, SUN_MASS};
use crate::physics::Kinematics;
use crate::physics::collisions::handle_collisions;
use crate::physics::euler::Euler;
use crate::physics::leapfrog::{Leapfrog, LeapfrogKDK};
use constants::{
    AU, EARTH_MASS, EARTH_MOON_DISTANCE, EARTH_RADIUS, EARTH_SUN_VELOCITY, HALEYS_COMET_MASS,
    HALEYS_COMET_VELOCITY, HALEYS_RADIUS, MARS_MASS, MARS_RADIUS, MARS_VELOCITY,
    MOON_EARTH_VELOCITY, MOON_MASS, MOON_RADIUS, SUN_HALEY_DISTANCE, SUN_MARS_DISTANCE, SUN_RADIUS,
};
use raylib::prelude::*;

enum CameraPosition {
    UniverseAbsolute((f64, f64)),
    BodyRelative(BodyId),
}

pub struct SimulationState {
    paused: bool,
    compute_collisions: bool,
    scale: f64,
    camera_position: CameraPosition,
    dt_factor: f64,
    kinematics_index: usize,
}

impl Default for SimulationState {
    fn default() -> Self {
        Self {
            paused: false,
            compute_collisions: true,
            scale: (1. / (SUN_EARTH_DISTANCE)) * 200.,
            camera_position: CameraPosition::BodyRelative(0),
            dt_factor: 1.0,
            kinematics_index: 0,
        }
    }
}

impl SimulationState {
    fn get_universe_center(&self, bodies: &OrbitalBodies) -> (f64, f64) {
        match self.camera_position {
            CameraPosition::UniverseAbsolute(pos) => pos,
            CameraPosition::BodyRelative(body) => bodies.get_by_id(body).unwrap().pos(),
        }
    }
}

fn main() {
    let (mut rl, thread) = init()
        .size(SPACE_SIZE as i32, SPACE_SIZE as i32)
        .title("Space")
        .build();

    rl.set_target_fps(60);

    let sun = Body::new(
        SUN_MASS,
        (0., 0.),
        SUN_RADIUS,
        20.,
        Color::YELLOW,
        (0.0, 0.0),
        (0.0, 0.0),
    );
    let sun_id = sun.id();

    let belt = bodies_to_map(create_asteroid_belt(&sun, 10_000, AU));

    let mut bodies = OrbitalBodies {
        tier0: bodies_to_map(vec![
            sun,
            // Mars
            Body::new(
                MARS_MASS,
                (0., 0. + SUN_MARS_DISTANCE),
                MARS_RADIUS,
                8.,
                Color::RED,
                (MARS_VELOCITY, 0.),
                (0.0, 0.0),
            ),
            // Earth
            Body::new(
                EARTH_MASS,
                (0., 0. + SUN_EARTH_DISTANCE),
                EARTH_RADIUS,
                10.,
                Color::BLUE,
                (EARTH_SUN_VELOCITY, 0.0),
                (0.0, 0.0),
            ),
            // Moon
            Body::new(
                MOON_MASS,
                (0., 0. + SUN_EARTH_DISTANCE + EARTH_MOON_DISTANCE),
                MOON_RADIUS,
                3.0,
                Color::GRAY,
                (EARTH_SUN_VELOCITY + MOON_EARTH_VELOCITY, 0.),
                (0., 0.),
            ),
            // Haley's comet
            Body::new(
                HALEYS_COMET_MASS,
                (0. + SUN_HALEY_DISTANCE, 0.),
                HALEYS_RADIUS,
                3.0,
                Color::ORANGERED,
                (0., HALEYS_COMET_VELOCITY),
                (0.0, 0.0),
            ),
        ]),
        tier1: belt,
    };

    let mut simulation_state = SimulationState::default();

    let kinematics: [Box<dyn Kinematics>; 3] =
        [Box::new(Leapfrog), Box::new(LeapfrogKDK), Box::new(Euler)];

    bodies.init_sun(sun_id);
    let mut kin = &kinematics[simulation_state.kinematics_index];

    let e0 = kin.step(&mut bodies, 0.01);

    while !rl.window_should_close() {
        // Handle mouse zoom
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

                simulation_state.camera_position =
                    CameraPosition::UniverseAbsolute(universe_center);
            }
        }

        // Swap kinematics based on keybind
        match rl.get_key_pressed() {
            Some(KeyboardKey::KEY_K) => {
                simulation_state.kinematics_index =
                    (simulation_state.kinematics_index + 1) % kinematics.len();
                kin = &kinematics[simulation_state.kinematics_index];
            }
            Some(KeyboardKey::KEY_C) => {
                simulation_state.compute_collisions = !simulation_state.compute_collisions;
            }
            Some(KeyboardKey::KEY_Q) => {
                break;
            }
            Some(KeyboardKey::KEY_P) => {
                simulation_state.paused = !simulation_state.paused;
            }
            Some(KeyboardKey::KEY_R) => {
                simulation_state.dt_factor = -1. * simulation_state.dt_factor;
            }
            _ => (),
        };

        // Simulate
        // dt in secs
        if !simulation_state.paused {
            let step_kinematics = kin.step(&mut bodies, simulation_state.dt_factor * 1800. * 24.);
            #[cfg(debug_assertions)]
            {
                let delta_energy_rel = (step_kinematics - e0) / e0.total();
                println!("Energy delta: ${delta_energy_rel:.3}");
            }

            if simulation_state.compute_collisions {
                handle_collisions(&mut bodies);
            }
        }

        // Draw
        let mut draw_handle = rl.begin_drawing(&thread);
        draw_handle.clear_background(Color::BLACK);

        draw_universe_relative(
            &mut draw_handle,
            &bodies,
            simulation_state.get_universe_center(&bodies),
            simulation_state.scale,
        );

        draw_hud(&mut draw_handle, &simulation_state, &bodies, &kin);
    }
}
