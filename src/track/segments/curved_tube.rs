use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, infinite_cylinder_sdf, smooth_min};

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

        // Add a short straight lead-in aligned with entry direction
        // This ensures tangent continuity at the junction
        let lead_in_length = 0.5;
        self.spine.push(self.entry.position);
        self.spine.push(self.entry.position + entry_dir_horizontal * lead_in_length);

        // Adjust the arc to start from the lead-in end point
        let arc_start = self.entry.position + entry_dir_horizontal * lead_in_length;
        let to_arc_start = arc_start - curve_center;
        let adjusted_start_angle = to_arc_start.z.atan2(to_arc_start.x);

        // Generate spine points along the arc (skip first since we have lead-in)
        for i in 1..=self.segments {
            let t = i as f32 / self.segments as f32;
            let angle = adjusted_start_angle + self.arc_angle * t;

            let x = curve_center.x + self.arc_radius * angle.cos();
            let z = curve_center.z + self.arc_radius * angle.sin();
            let y = self.entry.position.y;

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

        // Negate for inside collision
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
