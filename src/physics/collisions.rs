use crate::AU;
use crate::body::{Body, BodyId, OrbitalBodies};
use crate::physics::distance;
use kdtree::distance::squared_euclidean;
use rayon::prelude::*;
use std::cmp::Ordering;
use std::collections::HashSet;
use std::sync::Arc;
use std::sync::RwLock;
use std::time::Instant;

const MAX_DISTANCE_DEFAULT: f64 = AU * 10.;
const BIN_WIDTH_DEFAULT: f64 = AU / 2.;

struct BinBodiesParam {
    max_distance: f64,
    bin_width: f64,
}

impl Default for BinBodiesParam {
    fn default() -> Self {
        Self {
            max_distance: MAX_DISTANCE_DEFAULT,
            bin_width: BIN_WIDTH_DEFAULT,
        }
    }
}

enum CollisionResult {
    Merge {
        body_id: BodyId,
        new_mass: f64,
        new_velocity: (f64, f64),
    },
    Destroyed {
        body_id: BodyId,
    },
}

fn collides(body1: &Body, body2: &Body) -> bool {
    let (_, dist) = distance(body1, body2);
    dist <= (body1.physical_radius + body2.physical_radius)
}

fn compute_merger(winner: &Body, destroyed: &Body) -> CollisionResult {
    let new_mass = winner.mass + destroyed.mass;

    let (vx1, vy1) = winner.velocity;
    let (vx2, vy2) = destroyed.velocity;

    let vx = ((vx1 * winner.mass) + (vx2 * destroyed.mass)) / new_mass;
    let vy = ((vy1 * winner.mass) + (vy2 * destroyed.mass)) / new_mass;

    CollisionResult::Merge {
        body_id: winner.id(),
        new_mass,
        new_velocity: (vx, vy),
    }
}

fn append_collision(body1: &Body, body2: &Body, collisions: &mut Vec<CollisionResult>) {
    if collides(body1, body2) {
        let (largest, smallest) = if body1.mass > body2.mass {
            (body1, body2)
        } else {
            (body2, body1)
        };

        collisions.push(CollisionResult::Destroyed {
            body_id: smallest.id(),
        });

        collisions.push(compute_merger(largest, smallest));
    }
}

fn compute_pairwise_collision_slice(bodies: &[&Body]) -> Vec<CollisionResult> {
    let mut collisions = vec![];
    for i in 0..bodies.len() {
        for j in 0..bodies.len() {
            if i == j {
                continue;
            }

            append_collision(bodies[i], bodies[j], &mut collisions);
        }
    }

    collisions
}

/// Check all pairs of bodies and returns the list of results
#[allow(unused)]
fn compute_pairwise_collisions(orbital_bodies: &OrbitalBodies) -> Vec<CollisionResult> {
    let bodies = orbital_bodies.iter().collect::<Vec<_>>();
    compute_pairwise_collision_slice(bodies.as_slice())
}

/// Bins of fixed width
fn bin_bodies(orbital_bodies: &OrbitalBodies, params: BinBodiesParam) -> Vec<HashSet<BodyId>> {
    let BinBodiesParam {
        max_distance,
        bin_width,
    } = params;

    let body_in_range = |body: &Body| -> bool {
        let (x, y) = body.pos();
        (x.powf(2.) + y.powf(2.)).sqrt() < max_distance
    };

    let width = orbital_bodies
        .iter()
        .filter(|b| body_in_range(b))
        .map(|body| body.pos())
        .map(|(x, y)| {
            let xabs = x.abs();
            let yabs = y.abs();
            xabs.max(yabs)
        })
        .max_by(|a, b| {
            if a >= b {
                Ordering::Greater
            } else {
                Ordering::Less
            }
        })
        .unwrap()
        * 2.0;

    let bin_count = (width / bin_width) as usize;

    let mut bins: Vec<HashSet<BodyId>> = (0..bin_count * bin_count)
        .map(|_| HashSet::new())
        .collect::<Vec<_>>();

    if bins.len() > 0 {
        for body in orbital_bodies.iter().filter(|b| body_in_range(b)) {
            let (x, y) = body.pos();
            let offset = width / 2.;

            let (x, y) = (x + offset, y + offset);
            assert!(
                x >= 0. && y >= 0.,
                "Offset coordinates should always be in a positive area"
            );

            let bx = ((x / bin_width) as usize).min(bin_count - 1);
            let by = ((y / bin_width) as usize).min(bin_count - 1);

            let index = bx + by * bin_count;
            bins[index].insert(body.id());
        }
    }

    #[cfg(debug_assertions)]
    {
        let min = bins.iter().map(HashSet::len).min().unwrap_or(0);
        let max = bins.iter().map(HashSet::len).max().unwrap_or(0);
        let total = bins.iter().map(HashSet::len).sum::<usize>();
        let avg = if bins.len() == 0 {
            0
        } else {
            total / bins.len()
        };

        println!("Bin stats: max={max} min={min} avg={avg}");
    }

    bins
}

fn compute_kdtree_collisions(orbital_bodies: &OrbitalBodies) -> Vec<CollisionResult> {
    let collisions = Arc::new(RwLock::new(vec![]));
    let kd = Arc::new(RwLock::new(kdtree::KdTree::new(2)));

    {
        let mut kd = kd.write().unwrap();
        for body in orbital_bodies.iter() {
            kd.add(body.pos_arr(), body).unwrap();
        }
    }

    let bodies = orbital_bodies.iter().collect::<Vec<_>>();

    bodies.par_iter().for_each({
        let kd = kd.clone();
        let collisions = collisions.clone();

        move |body| {
            let kd = kd.read().unwrap();

            let neigh = kd
                .nearest(&body.pos_arr(), 100, &squared_euclidean)
                .unwrap();

            neigh.par_iter().for_each({
                let collisions = collisions.clone();
                move |(_, other)| {
                    if body.id() == other.id() {
                        return;
                    }

                    let mut collisions = collisions.write().unwrap();
                    append_collision(body, other, &mut collisions);
                }
            });
        }
    });

    vec![]
}

/// Compute collisions using spatial hashing
#[allow(unused)]
fn compute_collisions_spatial_hash(orbital_bodies: &OrbitalBodies) -> Vec<CollisionResult> {
    let bins = bin_bodies(orbital_bodies, BinBodiesParam::default());

    bins.par_iter()
        .map(|bin| {
            let bodies = bin
                .iter()
                .map(|id| orbital_bodies.get_by_id(*id).unwrap())
                .collect::<Vec<_>>();

            compute_pairwise_collision_slice(bodies.as_slice())
        })
        .flatten()
        .collect::<Vec<_>>()
}

/// Handle the collisions for the orbital system
pub fn handle_collisions(orbital_bodies: &mut OrbitalBodies) {
    let start = Instant::now();
    let collisions = compute_collisions_spatial_hash(orbital_bodies);
    let end = Instant::now();
    let delta = end - start;

    #[cfg(debug_assertions)]
    println!("Collision time: {0}ms", delta.as_millis());

    for collision in collisions {
        match collision {
            CollisionResult::Merge {
                body_id,
                new_mass,
                new_velocity,
            } => {
                if let Some(body) = orbital_bodies.get_mut_by_id(body_id) {
                    body.mass = new_mass;
                    body.velocity = new_velocity;
                }
            }
            CollisionResult::Destroyed { body_id } => {
                orbital_bodies.remove(body_id);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::body::{Body, OrbitalBodies, bodies_to_map};
    use crate::physics::collisions::{BinBodiesParam, bin_bodies};
    use raylib::color::Color;

    #[test]
    fn test_bin_bodies() {
        fn body(x: i32, y: i32) -> Body {
            Body::new(
                0.,
                (x as f64, y as f64),
                1.,
                1.,
                Color::WHITE,
                (0., 0.),
                (0., 0.),
            )
        }

        let b0 = body(-10, -10);
        let b1 = body(-5, -5);
        let b2 = body(5, 5);
        let b3 = body(10, 10);
        let b4 = body(0, 0);
        let b5 = body(2, 2);
        let b6 = body(1, 1);
        let b6_id = *&b6.id();

        let bodies = OrbitalBodies {
            tier0: bodies_to_map(vec![b0, b1, b2, b3, b4, b5, b6]),
            tier1: bodies_to_map(vec![]),
        };

        let binned = bin_bodies(
            &bodies,
            BinBodiesParam {
                max_distance: 100.,
                bin_width: 2.,
            },
        );

        assert!(
            binned.iter().all(|b| b.len() <= 1 || b.contains(&b6_id)),
            "There should not be more than a body per bin"
        );
    }
}
