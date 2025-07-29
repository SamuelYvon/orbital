use crate::physics::{OrbitParameters, kepler_orbit};
use rand::Rng;
use raylib::color::Color;
use ringbuffer::{AllocRingBuffer, RingBuffer};
use std::sync::atomic::{AtomicUsize, Ordering};

const ASTEROID_HIGH_SEMI_MAJOR_AXIS: f32 = 3.3;

const ASTEROID_LOW_SEMI_MAJOR_AXIS: f32 = 2.1;

const MAXIMUM_POSITION_HISTORY: usize = 1000;

/// Largest asteroid mass in Kg
const ASTEROID_MASS_HIGH: f32 = 1E18;

/// Smallest asteroid mass in Kg
const ASTEROID_MASS_LOW: f32 = 1E5;

pub type BodyId = usize;

static NEXT_ID: AtomicUsize = AtomicUsize::new(0);

/// The collection of bodies being simulated
pub struct OrbitalBodies {
    /// Tier 0 bodies have a gravity effect on all objects,
    /// including themselves
    pub tier0: Vec<Body>,
    /// Tier 2 bodies are influenced by tier 0, but do not
    /// influence other bodies
    pub tier1: Vec<Body>,
}

impl OrbitalBodies {
    pub fn iter(&self) -> impl Iterator<Item = &Body> {
        self.tier0.iter().chain(self.tier1.iter())
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Body> {
        self.tier0.iter_mut().chain(self.tier1.iter_mut())
    }
}

#[derive(Debug, Eq, PartialEq)]
pub enum TrailParameter {
    Trail,
    NoTrail,
}

pub struct Body {
    /// The unique Id of the body, used for tracking it.
    id: BodyId,
    /// Mass of the body in KG
    pub mass: f32,
    /// Center position of the space body
    pos: (f32, f32),
    /// Radius in pixels of the body
    pub radius: f32,
    /// Color to use for the body
    pub color: Color,
    /// Velocity in m/s
    pub velocity: (f32, f32),
    /// Acceleration in m/s^2
    pub accel: (f32, f32),
    /// If the body is fixed (won't be updated)
    pub fixed: bool,
    /// Drawing parameters
    pub trail_parameter: TrailParameter,
    /// The list of position of this body
    pub pos_list: AllocRingBuffer<(f32, f32)>,
}

impl Body {
    pub fn new(
        mass: f32,
        pos: (f32, f32),
        radius: f32,
        color: Color,
        velocity: (f32, f32),
        accel: (f32, f32),
        fixed: bool,
    ) -> Self {
        Self {
            id: NEXT_ID.fetch_add(1, Ordering::Relaxed),
            mass,
            pos,
            radius,
            color,
            velocity,
            accel,
            fixed,
            trail_parameter: TrailParameter::Trail,
            pos_list: AllocRingBuffer::new(MAXIMUM_POSITION_HISTORY),
        }
    }

    pub fn id(&self) -> BodyId {
        self.id
    }

    pub fn pos(&self) -> (f32, f32) {
        self.pos
    }

    pub fn set_pos(&mut self, pos: (f32, f32)) {
        self.pos_list.enqueue(pos);
        self.pos = pos
    }
}

/// Create an asteroid belt of `asteroids` bodies. They will be randomly placed
/// around the body.
pub fn create_asteroid_belt(
    reference_body: &Body,
    asteroids: usize,
    average_distance: f32,
) -> Vec<Body> {
    let mut ret = Vec::with_capacity(asteroids);
    let mut rng = rand::rng();

    for _ in 0..asteroids {
        let a = ((rng.random::<f32>()
            * (ASTEROID_HIGH_SEMI_MAJOR_AXIS - ASTEROID_LOW_SEMI_MAJOR_AXIS))
            + ASTEROID_LOW_SEMI_MAJOR_AXIS)
            * average_distance;
        let theta = rng.random::<f32>() * 2.0 * std::f32::consts::PI;
        let e = rng.random::<f32>() * 0.15;

        let mass =
            (rng.random::<f32>() * (ASTEROID_MASS_HIGH - ASTEROID_MASS_LOW)) + ASTEROID_MASS_LOW;

        let mut asteroid = Body::new(
            mass,
            (0., 0.),
            1., // always 1px
            Color::WHITESMOKE,
            (0., 0.),
            (0., 0.),
            false,
        );
        asteroid.trail_parameter = TrailParameter::NoTrail;

        kepler_orbit(
            OrbitParameters { a, e, theta },
            &mut asteroid,
            &reference_body,
        );

        ret.push(asteroid);
    }

    ret
}
