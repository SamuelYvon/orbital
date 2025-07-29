use crate::body::{Body, OrbitalBodies};
use crate::physics::{Kinematics, update_acceleration};
use raylib::drawing::RaylibDrawHandle;

pub struct Euler;

impl Kinematics for Euler {
    fn step(&self, bodies: &mut OrbitalBodies, dt: f64) {
        // Rn+1 = Rn + Vn*dt
        // Vn+1 = Vn + An*dt
        update_acceleration(bodies);

        for body in bodies.iter_mut() {
            if body.fixed {
                continue;
            }

            let (rx, ry) = body.pos();

            body.velocity.0 += body.accel.0 * dt;
            body.velocity.1 += body.accel.1 * dt;

            let rx_1 = rx + body.velocity.0 * dt;
            let ry_1 = ry + body.velocity.1 * dt;

            body.set_pos((rx_1, ry_1));
        }
    }

    fn name(&self) -> &'static str {
        "Euler"
    }
}
