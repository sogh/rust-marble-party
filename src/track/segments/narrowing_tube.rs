use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A tube that transitions from a larger radius to a smaller radius
pub struct NarrowingTube {
    /// Length of the tube
    pub length: f32,
    /// Radius at entry (larger)
    pub entry_radius: f32,
    /// Radius at exit (smaller)
    pub exit_radius: f32,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl NarrowingTube {
    pub fn new(length: f32, entry_radius: f32, exit_radius: f32, entry_port: Port) -> Self {
        let exit_pos = entry_port.position + entry_port.direction * length;
        let exit = Port::new(
            exit_pos,
            entry_port.direction,
            entry_port.up,
            exit_radius,
        );

        // Use larger radius for bounds padding
        let max_radius = entry_radius.max(exit_radius);
        let bounds = AABB::from_points(
            &[entry_port.position, exit_pos],
            max_radius + 0.5,
        );

        Self {
            length,
            entry_radius,
            exit_radius,
            entry: entry_port,
            exit,
            bounds,
        }
    }

    /// Create at origin pointing in -Z direction
    pub fn at_origin(length: f32, entry_radius: f32, exit_radius: f32) -> Self {
        let entry = Port::new(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::Y,
            entry_radius,
        );
        Self::new(length, entry_radius, exit_radius, entry)
    }

    /// Get the interpolated radius at a given t (0 = entry, 1 = exit)
    fn radius_at(&self, t: f32) -> f32 {
        self.entry_radius + (self.exit_radius - self.entry_radius) * t.clamp(0.0, 1.0)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for NarrowingTube {
    fn sdf(&self, point: Vec3) -> f32 {
        // Vector from entry to exit
        let axis = self.exit.position - self.entry.position;
        let axis_length = axis.length();
        let axis_dir = axis / axis_length;

        // Project point onto axis
        let to_point = point - self.entry.position;
        let along = to_point.dot(axis_dir);

        // Get t for radius interpolation (clamped to valid range)
        let t = (along / axis_length).clamp(0.0, 1.0);
        let radius = self.radius_at(t);

        // Distance from axis (radial distance with interpolated radius)
        let on_axis = self.entry.position + axis_dir * along;
        let radial_dist = (point - on_axis).length() - radius;

        // Pure radial distance - no axial walls
        // Adjacent segments handle their own regions seamlessly
        -radial_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let axis = self.exit.position - self.entry.position;
        let axis_length = axis.length();
        let axis_dir = axis / axis_length;

        let to_point = point - self.entry.position;
        let along = to_point.dot(axis_dir);

        // Core region excludes the overlap zones at each end
        along > OVERLAP_DISTANCE && along < axis_length - OVERLAP_DISTANCE
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
        let axis = self.exit.position - self.entry.position;
        let axis_dir = axis.normalize();

        // Draw center line
        gizmos.line(self.entry.position, self.exit.position, color);

        // Draw cross-sections at entry, middle, and exit
        let positions = [0.0, 0.5, 1.0];
        for t in positions {
            let pos = self.entry.position + axis * t;
            let radius = self.radius_at(t);

            let right = axis_dir.cross(self.entry.up).normalize_or_zero();
            let up = right.cross(axis_dir).normalize_or_zero();

            let segments = 16;
            for j in 0..segments {
                let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                let p1 = pos + (right * angle1.cos() + up * angle1.sin()) * radius;
                let p2 = pos + (right * angle2.cos() + up * angle2.sin()) * radius;

                gizmos.line(p1, p2, color);
            }
        }

        // Draw lines connecting entry to exit circles
        let right = axis_dir.cross(self.entry.up).normalize_or_zero();
        let up = right.cross(axis_dir).normalize_or_zero();
        for i in 0..4 {
            let angle = (i as f32 / 4.0) * std::f32::consts::TAU;
            let dir = right * angle.cos() + up * angle.sin();
            let p1 = self.entry.position + dir * self.entry_radius;
            let p2 = self.exit.position + dir * self.exit_radius;
            gizmos.line(p1, p2, color.with_alpha(0.5));
        }

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(self.entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(self.entry.position, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(self.exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(self.exit.position, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "NarrowingTube"
    }
}
