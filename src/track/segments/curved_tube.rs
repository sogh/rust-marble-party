use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, capsule_sdf, smooth_min};

/// A tube that curves in a horizontal arc
pub struct CurvedTube {
    /// Arc angle in radians (positive = left, negative = right)
    pub arc_angle: f32,
    /// Radius of the arc (distance from center of curve to tube center)
    pub arc_radius: f32,
    /// Radius of the tube itself
    pub tube_radius: f32,
    /// Number of segments for SDF approximation
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

impl CurvedTube {
    pub fn new(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        let segments = ((arc_angle.abs() / std::f32::consts::TAU) * 32.0).max(8.0) as usize;

        let mut tube = Self {
            arc_angle,
            arc_radius,
            tube_radius,
            segments,
            spine: Vec::new(),
            entry: entry_port,
            exit: Port::default(),
            bounds: AABB::default(),
        };

        tube.generate_spine();
        tube
    }

    /// Create a left-curving tube at origin
    pub fn left(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        Self::new(arc_angle.abs(), arc_radius, tube_radius, entry_port)
    }

    /// Create a right-curving tube at origin
    pub fn right(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        Self::new(-arc_angle.abs(), arc_radius, tube_radius, entry_port)
    }

    fn generate_spine(&mut self) {
        self.spine.clear();

        // Get the entry direction projected onto horizontal plane for curve calculation
        let entry_dir_horizontal = Vec3::new(self.entry.direction.x, 0.0, self.entry.direction.z).normalize_or_zero();

        // If entry is nearly vertical, use a default horizontal direction
        let entry_dir_horizontal = if entry_dir_horizontal.length() < 0.1 {
            Vec3::NEG_Z
        } else {
            entry_dir_horizontal
        };

        // Calculate perpendicular direction (to the right of travel)
        // Y cross direction gives right, direction cross Y gives left
        let right = Vec3::Y.cross(entry_dir_horizontal).normalize();

        // Calculate the center of the arc
        let curve_center = if self.arc_angle > 0.0 {
            // Curving left: center is to the left
            self.entry.position - right * self.arc_radius
        } else {
            // Curving right: center is to the right
            self.entry.position + right * self.arc_radius
        };

        // Calculate the starting angle from center to entry point
        let to_entry = self.entry.position - curve_center;
        let start_angle = to_entry.z.atan2(to_entry.x);

        // Generate spine points along the arc
        for i in 0..=self.segments {
            let t = i as f32 / self.segments as f32;
            let angle = start_angle + self.arc_angle * t;

            let x = curve_center.x + self.arc_radius * angle.cos();
            let z = curve_center.z + self.arc_radius * angle.sin();
            let y = self.entry.position.y; // Keep same height for horizontal curve

            self.spine.push(Vec3::new(x, y, z));
        }

        // Compute bounds
        self.bounds = AABB::from_points(&self.spine, self.tube_radius + 0.5);

        // Compute exit port
        if self.spine.len() >= 2 {
            let last = self.spine[self.spine.len() - 1];
            let prev = self.spine[self.spine.len() - 2];
            let exit_dir = (last - prev).normalize();

            self.exit = Port::new(last, exit_dir, self.entry.up, self.tube_radius);
        }
    }

    pub fn spine(&self) -> &[Vec3] {
        &self.spine
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0; // Matches tube_radius

impl Segment for CurvedTube {
    fn sdf(&self, point: Vec3) -> f32 {
        if self.spine.len() < 2 {
            return f32::MAX;
        }

        // Extend the first capsule backwards past entry for smooth junction blending
        let extended_entry = self.spine[0] - self.entry.direction * OVERLAP_DISTANCE;
        let mut dist = capsule_sdf(point, extended_entry, self.spine[1], self.tube_radius);

        // Middle capsules (no extension needed)
        for i in 1..self.spine.len() - 2 {
            let seg_dist = capsule_sdf(point, self.spine[i], self.spine[i + 1], self.tube_radius);
            dist = smooth_min(dist, seg_dist, 0.3);
        }

        // Extend the last capsule forwards past exit for smooth junction blending
        if self.spine.len() >= 2 {
            let last_idx = self.spine.len() - 1;
            let extended_exit = self.spine[last_idx] + self.exit.direction * OVERLAP_DISTANCE;
            let seg_dist = capsule_sdf(point, self.spine[last_idx - 1], extended_exit, self.tube_radius);
            dist = smooth_min(dist, seg_dist, 0.3);
        }

        // Negate for inside collision
        -dist
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

        // Draw cross-sections at intervals
        for (i, &point) in self.spine.iter().enumerate() {
            if i % 4 == 0 && i + 1 < self.spine.len() {
                let dir = (self.spine[i + 1] - point).normalize_or_zero();
                let right = dir.cross(Vec3::Y).normalize_or_zero();
                let up = right.cross(dir).normalize_or_zero();

                let segments = 12;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = point + (right * angle1.cos() + up * angle1.sin()) * self.tube_radius;
                    let p2 = point + (right * angle2.cos() + up * angle2.sin()) * self.tube_radius;

                    gizmos.line(p1, p2, color.with_alpha(0.5));
                }
            }
        }

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(self.entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(self.entry.position, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(self.exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(self.exit.position, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "CurvedTube"
    }
}
