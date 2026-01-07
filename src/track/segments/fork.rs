use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, capsule_sdf, smooth_min};

/// A Y-fork that splits one track into two paths
/// Marbles naturally take one path or the other based on their position
pub struct Fork {
    /// Length of the fork section
    pub length: f32,
    /// Radius of the tubes
    pub tube_radius: f32,
    /// Angle of split (radians, each side)
    pub split_angle: f32,
    /// Entry port
    entry: Port,
    /// Left exit port
    exit_left: Port,
    /// Right exit port
    exit_right: Port,
    /// Spine points for the left path
    spine_left: Vec<Vec3>,
    /// Spine points for the right path
    spine_right: Vec<Vec3>,
    /// Bounding box
    bounds: AABB,
}

impl Fork {
    pub fn new(length: f32, tube_radius: f32, split_angle: f32, entry_port: Port) -> Self {
        let segments = 8;

        // Calculate the right vector for splitting
        let right = entry_port.direction.cross(entry_port.up).normalize();

        // Generate spine points for both paths
        let mut spine_left = Vec::with_capacity(segments + 1);
        let mut spine_right = Vec::with_capacity(segments + 1);

        // Both paths start at the entry
        spine_left.push(entry_port.position);
        spine_right.push(entry_port.position);

        for i in 1..=segments {
            let t = i as f32 / segments as f32;

            // Gradually split apart
            let angle = split_angle * t;
            let forward = entry_port.direction * (length * t);
            let split = right * (length * t * angle.sin());

            spine_left.push(entry_port.position + forward - split);
            spine_right.push(entry_port.position + forward + split);
        }

        // Calculate exit directions
        let left_dir = if spine_left.len() >= 2 {
            (spine_left[spine_left.len() - 1] - spine_left[spine_left.len() - 2]).normalize()
        } else {
            entry_port.direction
        };

        let right_dir = if spine_right.len() >= 2 {
            (spine_right[spine_right.len() - 1] - spine_right[spine_right.len() - 2]).normalize()
        } else {
            entry_port.direction
        };

        let exit_left = Port::new(
            spine_left[spine_left.len() - 1],
            left_dir,
            entry_port.up,
            tube_radius,
        );

        let exit_right = Port::new(
            spine_right[spine_right.len() - 1],
            right_dir,
            entry_port.up,
            tube_radius,
        );

        // Compute bounds from all spine points
        let mut all_points = spine_left.clone();
        all_points.extend(spine_right.iter());
        let bounds = AABB::from_points(&all_points, tube_radius + 0.5);

        Self {
            length,
            tube_radius,
            split_angle,
            entry: entry_port,
            exit_left,
            exit_right,
            spine_left,
            spine_right,
            bounds,
        }
    }

    /// Create a fork at origin
    pub fn at_origin(length: f32, tube_radius: f32, split_angle: f32) -> Self {
        let entry = Port::new(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::Y,
            tube_radius,
        );
        Self::new(length, tube_radius, split_angle, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for Fork {
    fn sdf(&self, point: Vec3) -> f32 {
        // Compute SDF for both paths and take the minimum (union)
        let mut min_dist = f32::MAX;

        // Left path
        if self.spine_left.len() >= 2 {
            // Extended entry
            let extended_entry = self.spine_left[0] - self.entry.direction * OVERLAP_DISTANCE;
            let mut dist_left = capsule_sdf(point, extended_entry, self.spine_left[1], self.tube_radius);

            for i in 1..self.spine_left.len() - 1 {
                let seg_dist = capsule_sdf(point, self.spine_left[i], self.spine_left[i + 1], self.tube_radius);
                dist_left = smooth_min(dist_left, seg_dist, 0.5);
            }

            // Extended exit
            let last = self.spine_left.len() - 1;
            let extended_exit = self.spine_left[last] + self.exit_left.direction * OVERLAP_DISTANCE;
            let seg_dist = capsule_sdf(point, self.spine_left[last], extended_exit, self.tube_radius);
            dist_left = smooth_min(dist_left, seg_dist, 0.5);

            min_dist = dist_left;
        }

        // Right path
        if self.spine_right.len() >= 2 {
            // Note: Entry is shared, so we don't extend it again for right path
            let mut dist_right = capsule_sdf(point, self.spine_right[0], self.spine_right[1], self.tube_radius);

            for i in 1..self.spine_right.len() - 1 {
                let seg_dist = capsule_sdf(point, self.spine_right[i], self.spine_right[i + 1], self.tube_radius);
                dist_right = smooth_min(dist_right, seg_dist, 0.5);
            }

            // Extended exit
            let last = self.spine_right.len() - 1;
            let extended_exit = self.spine_right[last] + self.exit_right.direction * OVERLAP_DISTANCE;
            let seg_dist = capsule_sdf(point, self.spine_right[last], extended_exit, self.tube_radius);
            dist_right = smooth_min(dist_right, seg_dist, 0.5);

            min_dist = smooth_min(min_dist, dist_right, 0.5);
        }

        // Negate for inside collision
        -min_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        // Check distance along the entry direction
        let to_point = point - self.entry.position;
        let along = to_point.dot(self.entry.direction);

        // Core region is past the entry overlap but before the exit overlaps
        along > OVERLAP_DISTANCE && along < self.length - OVERLAP_DISTANCE
    }

    fn entry_port(&self) -> Port {
        self.entry.clone()
    }

    fn exit_ports(&self) -> Vec<Port> {
        vec![self.exit_left.clone(), self.exit_right.clone()]
    }

    fn bounds(&self) -> AABB {
        self.bounds.clone()
    }

    fn draw_debug_gizmos(&self, gizmos: &mut Gizmos, color: Color) {
        // Draw left spine
        for i in 0..self.spine_left.len() - 1 {
            gizmos.line(self.spine_left[i], self.spine_left[i + 1], color);
        }

        // Draw right spine
        for i in 0..self.spine_right.len() - 1 {
            gizmos.line(self.spine_right[i], self.spine_right[i + 1], color);
        }

        // Draw cross-sections at key points
        let draw_circle = |gizmos: &mut Gizmos, center: Vec3, dir: Vec3, radius: f32, col: Color| {
            let right = dir.cross(Vec3::Y).normalize_or_zero();
            let up = right.cross(dir).normalize_or_zero();

            let segments = 12;
            for j in 0..segments {
                let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                let p1 = center + (right * angle1.cos() + up * angle1.sin()) * radius;
                let p2 = center + (right * angle2.cos() + up * angle2.sin()) * radius;

                gizmos.line(p1, p2, col);
            }
        };

        // Entry circle
        draw_circle(gizmos, self.entry.position, self.entry.direction, self.tube_radius, color);

        // Exit circles
        draw_circle(gizmos, self.exit_left.position, self.exit_left.direction, self.tube_radius, Color::srgb(0.8, 0.4, 0.8));
        draw_circle(gizmos, self.exit_right.position, self.exit_right.direction, self.tube_radius, Color::srgb(0.4, 0.8, 0.8));

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(self.entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(self.entry.position, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(self.exit_left.position), 0.15, Color::srgb(1.0, 0.0, 0.5));
        gizmos.ray(self.exit_left.position, self.exit_left.direction * 0.5, Color::srgb(1.0, 0.0, 0.5));

        gizmos.sphere(Isometry3d::from_translation(self.exit_right.position), 0.15, Color::srgb(0.5, 0.0, 1.0));
        gizmos.ray(self.exit_right.position, self.exit_right.direction * 0.5, Color::srgb(0.5, 0.0, 1.0));
    }

    fn type_name(&self) -> &'static str {
        "Fork"
    }
}
