use crate::body::Body;
use crate::physics::{Kinematics, update_acceleration};
use raylib::drawing::RaylibDrawHandle;

pub struct Euler;

impl Kinematics for Euler {
    fn step(&mut self, bodies: &mut [Body], dt: f32) {
        // Rn+1 = Rn + Vn*dt
        // Vn+1 = Vn + An*dt
        update_acceleration(bodies);

        for body in bodies {
            if body.fixed {
                continue;
            }

            body.velocity.0 += body.accel.0 * dt;
            body.velocity.1 += body.accel.1 * dt;

            let pos = body.pos();

            let x = pos.0 + body.velocity.0 * dt;
            let y = pos.1 + body.velocity.1 * dt;

            body.set_pos((x, y));
        }
    }

    fn draw(&self, drawing_handle: &mut RaylibDrawHandle) {}
}
