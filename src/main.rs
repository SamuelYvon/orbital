mod body;
mod camera;
mod canvas;
mod constants;
mod input;
mod physics;

use crate::body::{Body, BodyId, OrbitalBodies, bodies_to_map, create_asteroid_belt};
use crate::camera::draw_universe_relative;
use crate::canvas::{HudParams, draw_hud};
use crate::constants::{SPACE_SIZE, SUN_EARTH_DISTANCE, SUN_MASS};
use crate::input::handle_input;
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
use std::time::Instant;

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
    speedup: f64,
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
            speedup: 1.,
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
        if handle_input(
            &mut rl,
            &mut simulation_state,
            &mut kin,
            &bodies,
            &kinematics,
        ) {
            break;
        }

        // Simulate
        let before_step = Instant::now();

        let energy_delta = if !simulation_state.paused {
            let step_kinematics = kin.step(
                &mut bodies,
                simulation_state.dt_factor * simulation_state.speedup * 1800. * 24.,
            );
            let delta_energy_rel = (step_kinematics - e0) / e0.total();
            #[cfg(debug_assertions)]
            {
                println!("Energy delta: ${delta_energy_rel:.3}");
            }

            if simulation_state.compute_collisions {
                handle_collisions(&mut bodies);
            }
            delta_energy_rel
        } else {
            0.
        };

        let after_step = Instant::now();
        let hud_text = HudParams {
            compute_time: after_step - before_step,
            energy_delta,
        };

        // Draw
        let mut draw_handle = rl.begin_drawing(&thread);
        draw_handle.clear_background(Color::BLACK);

        draw_universe_relative(
            &mut draw_handle,
            &bodies,
            simulation_state.get_universe_center(&bodies),
            simulation_state.scale,
        );

        draw_hud(&mut draw_handle, &simulation_state, &bodies, &kin, hud_text);
    }
}
