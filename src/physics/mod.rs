pub mod euler;

use crate::body::Body;
use raylib::drawing::RaylibDrawHandle;

/// Gravity constant
pub const G: f32 = 6.6674 * 1E-11;

/// Implementations of the maths to compute the new position of a list of
/// bodies.
pub trait Kinematics {
    /// Compute a time step
    fn step(&mut self, bodies: &mut [Body], secs: f32);

    /// Hook to draw information about the kinematics. Can be used to draw paths of
    /// objects.
    fn draw(&self, drawing_handle: &mut RaylibDrawHandle);
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

/// Given an angle in radians, a radius (a distance in meters), computes the position of the second
/// body relative to the one at [pos], assuming it is on a circle around the first body.
pub fn orient(theta: f32, radius: f32, pos: (f32, f32)) -> (f32, f32) {
    let dy = theta.sin() * radius;
    let dx = theta.cos() * radius;
    let (x, y) = pos;
    (x + dx, y + dy)
}

/// Parameters to create an orbit
pub struct OrbitParameters {
    /// Semi-major axis, meters
    pub a: f32,
    /// Eccentricity
    pub e: f32,
    /// True anomaly, radians
    pub theta: f32,
}

/// Configure an orbit between two bodies. 
/// 
/// Warning: 
/// Overwrites the position of the bodies, as well as their velocities.
/// This means you should set the mass of the bodies and place them both 
/// anywhere, then configure the orbit.
pub fn kepler_orbit(orb: OrbitParameters, body1: &mut Body, body2: &mut Body) {
    let mu = G * (body1.mass + body2.mass);

    let p = orb.a * (1. - orb.e.powf(2.0));

    // Position
    let r = (orb.a * p) / (1. + orb.e * orb.theta.cos());
    let (rx, ry) = (orb.theta.cos() * r, orb.theta.sin() * r);

    // Velocity
    let factor = (mu / p).sqrt();
    let (vx, vy) = (
        -orb.theta.sin() * factor,
        (orb.theta.cos() + orb.e) * factor,
    );

    let m1_mt = (body1.mass) / (body1.mass + body2.mass);
    let m2_mt = (body2.mass) / (body1.mass + body2.mass);

    body1.set_pos((-m2_mt * rx, -m2_mt * ry));
    body2.set_pos((m1_mt * rx, m1_mt * ry));

    body1.velocity = (-m2_mt * vx, -m2_mt * vy);
    body2.velocity = (m1_mt * vx, m1_mt * vy);
}
