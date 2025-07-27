use raylib::color::Color;

pub struct Body {
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
    /// The list of position of this body
    pub pos_list: Vec<(f32, f32)>,
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
            mass,
            pos,
            radius,
            color,
            velocity,
            accel,
            fixed,
            pos_list: vec![pos],
        }
    }

    pub fn pos(&self) -> (f32, f32) {
        self.pos
    }

    pub fn set_pos(&mut self, pos: (f32, f32)) {
        self.pos_list.push(pos);
        self.pos = pos
    }
}
