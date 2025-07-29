pub mod euler;
pub mod leapfrog;

use crate::body::{Body, BodyId, OrbitalBodies};
use std::collections::HashMap;

/// Gravity constant
pub const G: f32 = 6.6674 * 1E-11;

/// Implementations of the maths to compute the new position of a list of
/// bodies.
pub trait Kinematics {
    /// Compute a time step
    fn step(&self, bodies: &mut OrbitalBodies, dt: f32);

    fn name(&self) -> &'static str;
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

fn pairwise_acceleration(pullee: &Body, pulling: &Body) -> (f32, f32) {
    let bi = pullee;
    let bj = pulling;

    let pos_i = bi.pos();
    let pos_j = bj.pos();

    let (d2, d) = distance(bi, bj);
    let d3 = d * d2;

    let mj = bj.mass;

    let body_grav_constant = -G * mj;

    let x_acc = (body_grav_constant * (pos_i.0 - pos_j.0)) / d3;
    let y_acc = (body_grav_constant * (pos_i.1 - pos_j.1)) / d3;

    (x_acc, y_acc)
}

/// Update the acceleration of each bodies relative to one another.
/// This is an expensive operation, because the acceleration of a body
/// depends on **all the other bodies**. This means the performance is
/// expected to be within `O(n^2)`.
///
/// This function takes into account the tier of each body.
pub fn update_acceleration(bodies: &mut OrbitalBodies) -> HashMap<BodyId, (f32, f32)> {
    let mut accelerations = HashMap::new();

    // Pullee are tier 0, only pulled by tier0
    for pullee_id in bodies.tier0.keys().copied().collect::<Vec<_>>() {
        let pullee = bodies.tier0.get(&pullee_id).unwrap();

        let mut x_acc = 0.0;
        let mut y_acc = 0.0;

        if !pullee.fixed {
            for pulling_id in bodies.tier0.keys().cloned() {
                if pullee_id == pulling_id {
                    continue;
                }

                let pulling = bodies.tier0.get(&pulling_id).unwrap();

                let (x, y) = pairwise_acceleration(pullee, pulling);
                x_acc += x;
                y_acc += y;
            }
        }

        // Re-borrow for mutability
        let pullee = &mut bodies.tier0.get_mut(&pullee_id).unwrap();
        pullee.accel = (x_acc, y_acc);
        accelerations.insert(pullee.id(), (x_acc, y_acc));
    }

    // Pullee are tier1, only pulled by tier0
    for (pullee_id, pullee) in bodies.tier1.iter_mut() {
        let mut x_acc = 0.0;
        let mut y_acc = 0.0;

        if !pullee.fixed {
            for (_, pulling) in bodies.tier0.iter() {
                let (x, y) = pairwise_acceleration(pullee, pulling);
                x_acc += x;
                y_acc += y;
            }
        }

        pullee.accel = (x_acc, y_acc);
        accelerations.insert(*pullee_id, (x_acc, y_acc));
    }

    accelerations
}

/// Given an angle in radians, a radius (a distance in meters), computes the position of the second
/// body relative to the one at [pos], assuming it is on a circle around the first body.
pub fn orient(theta: f32, radius: f32, pos: (f32, f32)) -> (f32, f32) {
    let dx = theta.cos() * radius;
    let dy = theta.sin() * radius;
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

/// Configure the orbit of a body around another one.
///
/// Warning:
/// Overwrites the position of the orbiting body, as well as the velocity.
/// This means you should set the mass of the bodies and place them both
/// anywhere, then configure the orbit.
pub fn kepler_orbit(orb: OrbitParameters, orbiting_body: &mut Body, point_of_reference: &Body) {
    let mu = G * (orbiting_body.mass + point_of_reference.mass);

    let p = orb.a * (1. - orb.e.powf(2.0));

    // Position
    let radius = p / (1. + orb.e * orb.theta.cos());
    let (rx, ry) = (orb.theta.cos() * radius, orb.theta.sin() * radius);

    // Velocity
    let factor = (mu / p).sqrt();
    let (vx, vy) = (
        -orb.theta.sin() * factor,
        (orb.theta.cos() + orb.e) * factor,
    );

    let total_mass = orbiting_body.mass + point_of_reference.mass;
    let m2_mt = (point_of_reference.mass) / total_mass;

    orbiting_body.set_pos((-m2_mt * rx, -m2_mt * ry));
    orbiting_body.velocity = (-m2_mt * vx, -m2_mt * vy);
}
