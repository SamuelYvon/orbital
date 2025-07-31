use crate::body::OrbitalBodies;
use crate::physics::{Kinematics, KinematicsDiagnostic, update_acceleration};

pub struct Euler;

impl Kinematics for Euler {
    fn step(&self, bodies: &mut OrbitalBodies, dt: f64) -> KinematicsDiagnostic {
        // Rn+1 = Rn + Vn*dt
        // Vn+1 = Vn + An*dt

        let mut potential_energy = 0.;
        let mut kinetic_energy = 0.;

        update_acceleration(bodies, &mut potential_energy);

        for body in bodies.iter_mut() {
            let (rx, ry) = body.pos();

            body.velocity.0 += body.accel.0 * dt;
            body.velocity.1 += body.accel.1 * dt;

            let rx_1 = rx + body.velocity.0 * dt;
            let ry_1 = ry + body.velocity.1 * dt;

            body.set_pos((rx_1, ry_1));

            kinetic_energy += body.kinetic_energy();
        }

        KinematicsDiagnostic {
            potential_energy,
            kinetic_energy,
        }
    }

    fn name(&self) -> &'static str {
        "Euler"
    }
}
