pub mod euler;

use raylib::drawing::RaylibDrawHandle;
use crate::body::Body;

/// Gravity constant
pub const G: f32 = 6.6674 * 1E-11;

/// Implementations of the maths to compute the new position of a list of
/// bodies.
pub trait Kinematics {
    /// Compute a time step
    fn step(&mut self, bodies: &mut [Body], secs: f32);

    /// Hook to draw information about the kinematics. Can be used to draw paths of
    /// objects.
    fn draw(&self, drawing_handle : &mut RaylibDrawHandle);
}

/// Compute the euclidian distance between two bodies. Returns
/// two values, (d^2, d) where d is the euclidian distance.
#[inline]
pub fn distance(body1: &Body, body2: &Body) -> (f32, f32) {
    let pos_1 = body1.pos();
    let pos_2 = body2.pos();

    let dx2 = (pos_1.0 - pos_2.0).powf(2.0);
    let dy2 = (pos_1.1 - pos_2.1).powf(2.0);
    let sum = dx2 + dy2;

    (sum, sum.sqrt())
}

/// Update the acceleration of each bodies relative to one another.
pub fn update_acceleration(bodies: &mut [Body]) {
    for i in 0..bodies.len() {
        if bodies[i].fixed {
            continue;
        }

        let mut x_acc = 0.0;
        let mut y_acc = 0.0;

        for j in 0..bodies.len() {
            if i == j {
                continue;
            }

            let bi = &bodies[i];
            let bj = &bodies[j];

            let pos_i = bi.pos();
            let pos_j = bj.pos();

            let (distance_pow, distance) = distance(bi, bj);

            let mj = bj.mass;

            let grav = (-G * mj) / distance_pow;

            x_acc += grav * ((pos_i.0 - pos_j.0) / distance);
            y_acc += grav * ((pos_i.1 - pos_j.1) / distance);
        }

        bodies[i].accel = (x_acc, y_acc);
    }
}
