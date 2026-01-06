use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, capsule_sdf, smooth_min};

/// A helical tube segment that spirals downward
pub struct SpiralTube {
    /// Number of complete loops
    pub loops: f32,
    /// Total vertical descent
    pub descent: f32,
    /// Radius of the helix (distance from center)
    pub helix_radius: f32,
    /// Radius of the tube itself
    pub tube_radius: f32,
    /// Amplitude of small bumps along the path
    pub bump_amplitude: f32,
    /// Number of points to generate for the spine
    pub segments: usize,
    /// Cached spine points
    spine: Vec<Vec3>,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl SpiralTube {
    pub fn new(
        loops: f32,
        descent: f32,
        helix_radius: f32,
        tube_radius: f32,
        entry_port: Port,
    ) -> Self {
        let segments = (loops * 50.0) as usize + 1;
        let bump_amplitude = 0.5;

        let mut tube = Self {
            loops,
            descent,
            helix_radius,
            tube_radius,
            bump_amplitude,
            segments,
            spine: Vec::new(),
            entry: entry_port.clone(),
            exit: Port::default(),
            bounds: AABB::default(),
        };

        tube.generate_spine();
        tube
    }

    /// Create with default starting position at origin
    pub fn at_origin(loops: f32, descent: f32, helix_radius: f32, tube_radius: f32) -> Self {
        let start_pos = Vec3::new(helix_radius, descent + 2.0, 0.0);
        let entry = Port::new(
            start_pos,
            Vec3::new(0.0, -0.3, 1.0).normalize(), // Slightly downward, forward
            Vec3::Y,
            tube_radius,
        );
        Self::new(loops, descent, helix_radius, tube_radius, entry)
    }

    fn generate_spine(&mut self) {
        self.spine.clear();

        let start_y = self.entry.position.y;

        for i in 0..=self.segments {
            let t = i as f32 / self.segments as f32;
            let angle = t * std::f32::consts::TAU * self.loops;

            // Spiral with varying radius
            let radius = self.helix_radius + 1.5 * (angle * 0.5).sin();
            let x = radius * angle.cos();
            let z = radius * angle.sin();

            // Descending with small bumps (cosine so starts going downhill)
            let base_descent = self.descent * t;
            let bumps = self.bump_amplitude * (angle * 2.0).cos();
            let y = start_y - base_descent + bumps;

            self.spine.push(Vec3::new(x, y, z));
        }

        // Compute bounds
        self.bounds = AABB::from_points(&self.spine, self.tube_radius + 0.5);

        // Compute exit port
        if self.spine.len() >= 2 {
            let last = self.spine[self.spine.len() - 1];
            let prev = self.spine[self.spine.len() - 2];
            let dir = (last - prev).normalize();

            self.exit = Port::new(last, dir, Vec3::Y, self.tube_radius);
        }

        // Update entry port to match actual spine start
        if !self.spine.is_empty() {
            self.entry.position = self.spine[0];
            if self.spine.len() >= 2 {
                self.entry.direction = (self.spine[1] - self.spine[0]).normalize();
            }
        }
    }

    /// Get the spine points for external use
    pub fn spine(&self) -> &[Vec3] {
        &self.spine
    }
}

impl Segment for SpiralTube {
    fn sdf(&self, point: Vec3) -> f32 {
        if self.spine.len() < 2 {
            return f32::MAX;
        }

        let mut dist = capsule_sdf(point, self.spine[0], self.spine[1], self.tube_radius);

        for i in 1..self.spine.len() - 1 {
            let seg_dist = capsule_sdf(point, self.spine[i], self.spine[i + 1], self.tube_radius);
            dist = smooth_min(dist, seg_dist, 0.5);
        }

        // Negate so collision is on INSIDE of tube
        -dist
    }

    fn entry_port(&self) -> Port {
        self.entry.clone()
    }

    fn exit_ports(&self) -> Vec<Port> {
        vec![self.exit.clone()]
    }

    fn bounds(&self) -> AABB {
        self.bounds.clone()
    }

    fn draw_debug_gizmos(&self, gizmos: &mut Gizmos, color: Color) {
        // Draw spine
        for i in 0..self.spine.len() - 1 {
            gizmos.line(self.spine[i], self.spine[i + 1], color);
        }

        // Draw cross-section circles every few points
        for (i, &point) in self.spine.iter().enumerate() {
            if i % 5 == 0 && i + 1 < self.spine.len() {
                let dir = (self.spine[i + 1] - point).normalize_or_zero();
                let right = dir.cross(Vec3::Y).normalize_or_zero();
                let up = right.cross(dir).normalize_or_zero();

                // Draw circle
                let segments = 16;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = point + (right * angle1.cos() + up * angle1.sin()) * self.tube_radius;
                    let p2 = point + (right * angle2.cos() + up * angle2.sin()) * self.tube_radius;

                    gizmos.line(p1, p2, color.with_alpha(0.5));
                }
            }
        }

        // Draw entry/exit ports
        let entry = self.entry_port();
        gizmos.sphere(Isometry3d::from_translation(entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(entry.position, entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        let exit = self.primary_exit_port();
        gizmos.sphere(Isometry3d::from_translation(exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(exit.position, exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "SpiralTube"
    }

    fn descent(&self) -> f32 {
        self.descent
    }
}
