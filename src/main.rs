mod body;
mod camera;
mod physics;

use crate::body::{Body, BodyId, OrbitalBodies, bodies_to_map, create_asteroid_belt};
use crate::camera::{click_in_body, draw_universe_relative, screen_coords_to_universe};
use crate::physics::Kinematics;
use crate::physics::euler::Euler;
use crate::physics::leapfrog::{Leapfrog, LeapfrogKDK};
use raylib::prelude::*;
use std::collections::HashMap;

const SPACE_SIZE: usize = 1500;

const EARTH_MASS: f32 = 5.972 * 1E24;
const MOON_MASS: f32 = 7.32 * 1E22;
const SUN_MASS: f32 = 1.989 * 1E30;
const MARS_MASS: f32 = 6.39 * 1E23;
const HALEYS_COMET_MASS: f32 = 2.2 * 1E14;
const MARS_VELOCITY: f32 = 24.077 * 1000.0; // m/s
const EARTH_SUN_VELOCITY: f32 = 29_784.8;
const MOON_EARTH_VELOCITY: f32 = 1023.;
const HALEYS_COMET_VELOCITY: f32 = 54.6 * 1000.;

const SUN_MARS_DISTANCE: f32 = 243230000000.;
const SUN_EARTH_DISTANCE: f32 = 149597870700.;
const EARTH_MOON_DISTANCE: f32 = 384400000.;
const SUN_HALEY_DISTANCE: f32 = 87_831_000. * 1000.;

const SUN_RADIUS: f32 = 6.957e+8;
const EARTH_RADIUS: f32 = 6.378e+6;
const MARS_RADIUS: f32 = 3389500.;
const MOON_RADIUS: f32 = 1737400.;
const HALEYS_RADIUS: f32 = 5500.;

enum CameraPosition {
    UniverseAbsolute((f32, f32)),
    BodyRelative(BodyId),
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
        true,
    );

    let belt = bodies_to_map(create_asteroid_belt(&sun, 10_000, SUN_EARTH_DISTANCE));

    let mut simulated = OrbitalBodies {
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
                false,
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
                false,
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
                false,
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
                false,
            ),
        ]),
        tier1: belt,
    };

    let mut scale = (1. / (SUN_EARTH_DISTANCE)) * 200.;
    let mut camera_position = CameraPosition::BodyRelative(0);

    let mut kinematics_index: usize = 0;
    let kinematics: [Box<dyn Kinematics>; 3] =
        [Box::new(Leapfrog), Box::new(LeapfrogKDK), Box::new(Euler)];

    let mut kin = &kinematics[kinematics_index];

    macro_rules! get_universe_center {
        () => {{
            match camera_position {
                CameraPosition::UniverseAbsolute(pos) => pos,
                CameraPosition::BodyRelative(body) => simulated.get_by_id(body).unwrap().pos(),
            }
        }};
    }

    while !rl.window_should_close() {
        // Handle mouse zoom
        let mouse_wheel = rl.get_mouse_wheel_move();

        if mouse_wheel > 0. {
            scale *= mouse_wheel * 1.1;
        } else if mouse_wheel < 0. {
            scale /= mouse_wheel.abs() * 1.1;
        }

        // Center to the selection position
        if rl.is_mouse_button_pressed(MouseButton::MOUSE_BUTTON_LEFT) {
            // TODO: check if we clicked on a body, for now just center on click

            let screen_position = (rl.get_mouse_x(), rl.get_mouse_y());
            let universe_center = get_universe_center!();
            let screen_center = (SPACE_SIZE / 2) as i32;

            let mut set = false;
            for (i, body) in simulated.tier0.iter() {
                if click_in_body(screen_position, universe_center, screen_center, scale, body) {
                    camera_position = CameraPosition::BodyRelative(*i);
                    set = true;
                    break;
                }
            }

            if !set {
                let universe_center = screen_coords_to_universe(
                    screen_position,
                    scale,
                    universe_center,
                    screen_center,
                );

                camera_position = CameraPosition::UniverseAbsolute(universe_center);
            }
        }

        // Swap kinematics based on keybind
        match rl.get_key_pressed() {
            Some(KeyboardKey::KEY_K) => {
                kinematics_index = (kinematics_index + 1) % kinematics.len();
                kin = &kinematics[kinematics_index];
            }
            Some(KeyboardKey::KEY_Q) => {
                break;
            }
            _ => (),
        };

        let mut draw_handle = rl.begin_drawing(&thread);
        draw_handle.clear_background(Color::BLACK);
        draw_universe_relative(&mut draw_handle, &simulated, get_universe_center!(), scale);

        // dt in secs
        kin.step(&mut simulated, 1800. * 24.);

        draw_handle.draw_text(kin.name(), 14, SPACE_SIZE as i32 - 14 * 2, 14, Color::WHITE);
    }
}
