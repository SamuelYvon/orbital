use crate::body::OrbitalBodies;
use crate::physics::{Kinematics, update_acceleration, KinematicsDiagnostic};
use std::collections::HashMap;

pub struct Leapfrog;

impl Kinematics for Leapfrog {
    fn step(&self, bodies: &mut OrbitalBodies, dt: f64) -> KinematicsDiagnostic {
        let acceleration = bodies
            .iter()
            .map(|body| (body.id(), body.accel))
            .collect::<HashMap<_, _>>();
        
        let mut potential_energy = 0.0;
        let mut kinetic_energy = 0.0;

        for body in bodies.iter_mut() {
            if body.fixed {
                continue;
            }

            let (ax, ay) = acceleration.get(&body.id()).unwrap();
            let (rx, ry) = body.pos();
            let (vx, vy) = body.velocity;

            let rx1 = rx + vx * dt + (1. / 2.) * ax * dt.powf(2.0);
            let ry1 = ry + vy * dt + (1. / 2.) * ay * dt.powf(2.0);

            body.set_pos((rx1, ry1));
        }

        let acceleration_updated = update_acceleration(bodies, &mut potential_energy);

        for body in bodies.iter_mut() {
            if body.fixed {
                continue;
            }

            let (ax, ay) = acceleration.get(&body.id()).unwrap();
            let (ax1, ay1) = acceleration_updated.get(&body.id()).unwrap();
            let (vx, vy) = body.velocity;

            let vx1 = vx + (1. / 2.) * (ax + ax1) * dt;
            let vy1 = vy + (1. / 2.) * (ay + ay1) * dt;

            body.velocity = (vx1, vy1);
            kinetic_energy += body.kinetic_energy();
        }
        
        KinematicsDiagnostic {
            kinetic_energy,
            potential_energy
        }
    }

    fn name(&self) -> &'static str {
        "Leapfrog"
    }
}

pub struct LeapfrogKDK;

impl Kinematics for LeapfrogKDK {
    fn step(&self, bodies: &mut OrbitalBodies, dt: f64) -> KinematicsDiagnostic {
        // kick-drift-kick format
        let acceleration = bodies.iter().map(|body| body.accel).collect::<Vec<_>>();

        let mut potential_energy = 0.0;
        let mut kinetic_energy = 0.0;

        let mut velocities_half = Vec::with_capacity(bodies.tier0.len() + bodies.tier1.len());

        for (i, body) in bodies.iter_mut().enumerate() {
            if body.fixed {
                velocities_half.push((0., 0.));
                continue;
            }

            let (ax, ay) = acceleration[i];
            let (vx, vy) = body.velocity;
            let (rx, ry) = body.pos();

            let vx_i_half = vx + (1. / 2.) * ax * dt;
            let vy_i_half = vy + (1. / 2.) * ay * dt;

            let rx_i_1 = rx + vx_i_half * dt;
            let ry_i_1 = ry + vy_i_half * dt;

            body.set_pos((rx_i_1, ry_i_1));
            velocities_half.push((vx_i_half, vy_i_half));
        }

        let acceleration_updated = update_acceleration(bodies, &mut potential_energy);

        for (i, body) in bodies.iter_mut().enumerate() {
            let (ax_1, ay_1) = acceleration_updated[&body.id()];
            let (vx_i_half, vy_i_half) = velocities_half[i];

            let vx_i_1 = vx_i_half + (1. / 2.) * ax_1 * dt;
            let vy_i_1 = vy_i_half + (1. / 2.) * ay_1 * dt;

            body.velocity = (vx_i_1, vy_i_1);
            kinetic_energy += body.kinetic_energy();
        }

        KinematicsDiagnostic {
            kinetic_energy,
            potential_energy
        }
    }

    fn name(&self) -> &'static str {
        "Leapfrog (KDK)"
    }
}
