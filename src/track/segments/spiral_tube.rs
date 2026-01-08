use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, infinite_cylinder_sdf, smooth_min};

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

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0; // Matches tube_radius

impl Segment for SpiralTube {
    fn sdf(&self, point: Vec3) -> f32 {
        if self.spine.len() < 2 {
            return f32::MAX;
        }

        // Find the closest point along the spine and compute radial distance
        let mut min_dist = f32::MAX;

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let segment_dir = (b - a).normalize_or_zero();

            // Use infinite cylinder for this segment
            let cyl_dist = infinite_cylinder_sdf(point, a, segment_dir, self.tube_radius);

            // Check axial bounds within this spine segment
            let to_point = point - a;
            let seg_len = (b - a).length();
            let axial = to_point.dot(segment_dir);

            // Only count if within this spine segment's axial range (no entry/exit caps)
            if axial >= 0.0 && axial <= seg_len {
                min_dist = min_dist.min(cyl_dist);
            } else {
                // Blend for nearby spine segments
                min_dist = smooth_min(min_dist, cyl_dist, 0.5);
            }
        }

        // Negate so collision is on INSIDE of tube
        -min_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        if self.spine.len() < 2 {
            return false;
        }

        // Calculate total spine length to determine threshold
        let mut total_length = 0.0;
        for i in 0..self.spine.len() - 1 {
            total_length += (self.spine[i + 1] - self.spine[i]).length();
        }

        // Find the cumulative distance along the spine to the closest point
        let mut min_dist_sq = f32::MAX;
        let mut closest_along = 0.0;
        let mut cumulative_length = 0.0;

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let ab = b - a;
            let segment_length = ab.length();
            let ap = point - a;
            let t = (ap.dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
            let closest = a + ab * t;
            let dist_sq = (point - closest).length_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_along = cumulative_length + t * segment_length;
            }

            cumulative_length += segment_length;
        }

        // Core region excludes the overlap zones at each end (OVERLAP_DISTANCE)
        // This ensures junctions get special soft collision handling
        closest_along > OVERLAP_DISTANCE && closest_along < total_length - OVERLAP_DISTANCE
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

        // Draw extended capsule endpoints (overlap regions)
        if !self.spine.is_empty() {
            let extended_entry = self.spine[0] - self.entry.direction * OVERLAP_DISTANCE;
            let last_idx = self.spine.len() - 1;
            let extended_exit = self.spine[last_idx] + self.exit.direction * OVERLAP_DISTANCE;

            // Draw extended lines in yellow
            gizmos.line(extended_entry, self.spine[0], Color::srgb(1.0, 1.0, 0.0));
            gizmos.line(self.spine[last_idx], extended_exit, Color::srgb(1.0, 1.0, 0.0));

            // Draw spheres at extended endpoints
            gizmos.sphere(Isometry3d::from_translation(extended_entry), self.tube_radius, Color::srgb(1.0, 1.0, 0.0));
            gizmos.sphere(Isometry3d::from_translation(extended_exit), self.tube_radius, Color::srgb(1.0, 0.5, 0.0));
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
