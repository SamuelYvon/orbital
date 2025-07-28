mod body;
mod camera;
mod physics;

use crate::body::Body;
use crate::camera::{click_in_body, draw_universe_relative, screen_coords_to_universe};
use crate::physics::Kinematics;
use crate::physics::euler::Euler;
use raylib::prelude::*;

const SPACE_SIZE: usize = 1500;

const EARTH_MASS: f32 = 5.972 * 1E24;
const MOON_MASS: f32 = 7.32 * 1E22;
const SUN_MASS: f32 = 1.989 * 1E30;
const MARS_MASS: f32 = 6.39 * 1E23;
const MARS_VELOCITY: f32 = 24.077 * 1000.0; // m/s
const EARTH_SUN_VELOCITY: f32 = 29780.;
const MOON_EARTH_VELOCITY: f32 = 1023.;

const SUN_MARS_DISTANCE: f32 = 243230000000.;
const SUN_EARTH_DISTANCE: f32 = 149597870700.;
const EARTH_MOON_DISTANCE: f32 = 384400000.;

enum CameraPosition {
    UniverseAbsolute((f32, f32)),
    BodyRelative(usize),
}

fn main() {
    let (mut rl, thread) = init()
        .size(SPACE_SIZE as i32, SPACE_SIZE as i32)
        .title("Space")
        .build();

    rl.set_target_fps(60);

    let mut bodies = [
        // Sun
        Body::new(
            SUN_MASS,
            (0., 0.),
            20.,
            Color::YELLOW,
            (0.0, 0.0),
            (0.0, 0.0),
            true,
        ),
        // Mars
        Body::new(
            MARS_MASS,
            (0., 0. + SUN_MARS_DISTANCE),
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
            3.0,
            Color::GRAY,
            (EARTH_SUN_VELOCITY + MOON_EARTH_VELOCITY, 0.),
            (0., 0.),
            false,
        ),
    ];

    let mut scale = (1. / (SUN_EARTH_DISTANCE)) * 200.;
    let mut camera_position = CameraPosition::BodyRelative(2);

    let mut kin = Euler;

    macro_rules! get_universe_center {
        () => {{
            match camera_position {
                CameraPosition::UniverseAbsolute(pos) => pos,
                CameraPosition::BodyRelative(body) => bodies[body].pos(),
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
            for (i, body) in bodies.iter().enumerate() {
                if click_in_body(screen_position, universe_center, screen_center, scale, body) {
                    camera_position = CameraPosition::BodyRelative(i);
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

        let mut draw_handle = rl.begin_drawing(&thread);
        draw_handle.clear_background(Color::BLACK);
        draw_universe_relative(&mut draw_handle, &bodies, get_universe_center!(), scale);

        kin.step(&mut bodies, 1800. * 24.);
        // kin.step(&mut bodies, 60.0 * 10.);

        // for (i, body) in bodies.iter().enumerate() {
        //     println!("======================================");
        //     println!(
        //         "Body {i}: {0:.3}10^22 kg, R_x:{1:.3} R_y:{2:.3}",
        //         body.mass / 10E22,
        //         body.pos.0,
        //         body.pos.1
        //     );
        //     println!(
        //         "                       V_x:{0:.3} V_y:{1:.3}",
        //         body.velocity.0, body.velocity.1
        //     );
        //     println!(
        //         "                       A_x:{0:.3} A_y:{1:.3}",
        //         body.accel.0, body.accel.1
        //     );
        // }
        // println!()
    }
}
