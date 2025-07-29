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

/// Check all pairs of bodies and returns the list of results
#[allow(unused)]
fn compute_pairwise_collisions(orbital_bodies: &OrbitalBodies) -> Vec<CollisionResult> {
    let mut collisions = vec![];
    let bodies = orbital_bodies.iter().collect::<Vec<_>>();

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

/// Bins of fixed width
fn bin_bodies(orbital_bodies: &OrbitalBodies) -> Vec<HashSet<BodyId>> {
    let max_distance = (AU * 10.).sqrt();
    let bin_width = AU / 2.;

    let width = orbital_bodies
        .iter()
        .filter(|body| {
            let (x, y) = body.pos();
            (x.powf(2.) + y.powf(2.)) < max_distance
        })
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
        .unwrap();

    let bin_count = (width / bin_width) as usize;
    let mut bins = Vec::<HashSet<BodyId>>::with_capacity(bin_count * bin_count);

    for body in orbital_bodies.iter() {
        let (x, y) = body.pos();
        let offset = width / 2.;

        let (x, y) = (x + offset, y + offset);
        assert!(
            x >= 0. && y >= 0.,
            "Offset coordinates should always be in a positive area"
        );

        let bx = (x / bin_width) as usize;
        let by = (y / bin_width) as usize;

        let index = bx + by * bin_count;
        bins[index].insert(body.id());
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

/// Handle the collisions for the orbital system
pub fn handle_collisions(orbital_bodies: &mut OrbitalBodies) {
    let start = Instant::now();
    let collisions = compute_kdtree_collisions(orbital_bodies);
    let end = Instant::now();
    let delta = end - start;

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
