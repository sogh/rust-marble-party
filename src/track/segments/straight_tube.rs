use bevy::prelude::*;
use crate::track::{Port, AABB, Segment, capsule_sdf};

/// A simple straight tube segment
pub struct StraightTube {
    /// Length of the tube
    pub length: f32,
    /// Radius of the tube
    pub tube_radius: f32,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl StraightTube {
    pub fn new(length: f32, tube_radius: f32, entry_port: Port) -> Self {
        let exit_pos = entry_port.position + entry_port.direction * length;
        let exit = Port::new(
            exit_pos,
            entry_port.direction,
            entry_port.up,
            tube_radius,
        );

        let bounds = AABB::from_points(
            &[entry_port.position, exit_pos],
            tube_radius + 0.5,
        );

        Self {
            length,
            tube_radius,
            entry: entry_port,
            exit,
            bounds,
        }
    }

    /// Create at origin pointing in -Z direction
    pub fn at_origin(length: f32, tube_radius: f32) -> Self {
        let entry = Port::new(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::Y,
            tube_radius,
        );
        Self::new(length, tube_radius, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0; // Matches tube_radius

impl Segment for StraightTube {
    fn sdf(&self, point: Vec3) -> f32 {
        // Extend the capsule past both endpoints for smooth junction blending
        let extended_entry = self.entry.position - self.entry.direction * OVERLAP_DISTANCE;
        let extended_exit = self.exit.position + self.exit.direction * OVERLAP_DISTANCE;

        let dist = capsule_sdf(
            point,
            extended_entry,
            extended_exit,
            self.tube_radius,
        );
        // Negate for inside collision
        -dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        // Check if point is in the cylindrical core (not in end cap hemispheres)
        let to_point = point - self.entry.position;
        let along = to_point.dot(self.entry.direction);
        let tube_length = (self.exit.position - self.entry.position).length();

        // Core region excludes the overlap zones at each end (OVERLAP_DISTANCE)
        // This ensures junctions get special soft collision handling
        along > OVERLAP_DISTANCE && along < tube_length - OVERLAP_DISTANCE
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
        // Draw center line
        gizmos.line(self.entry.position, self.exit.position, color);

        // Draw extended capsule endpoints (overlap regions)
        let extended_entry = self.entry.position - self.entry.direction * OVERLAP_DISTANCE;
        let extended_exit = self.exit.position + self.exit.direction * OVERLAP_DISTANCE;

        // Draw extended lines in yellow
        gizmos.line(extended_entry, self.entry.position, Color::srgb(1.0, 1.0, 0.0));
        gizmos.line(self.exit.position, extended_exit, Color::srgb(1.0, 1.0, 0.0));

        // Draw spheres at extended endpoints
        gizmos.sphere(Isometry3d::from_translation(extended_entry), self.tube_radius, Color::srgb(1.0, 1.0, 0.0));
        gizmos.sphere(Isometry3d::from_translation(extended_exit), self.tube_radius, Color::srgb(1.0, 0.5, 0.0));

        // Draw entry and exit circles
        for (port, port_color) in [
            (&self.entry, Color::srgb(0.0, 1.0, 0.0)),
            (&self.exit, Color::srgb(1.0, 0.0, 0.0)),
        ] {
            let right = port.direction.cross(port.up).normalize_or_zero();
            let up = right.cross(port.direction).normalize_or_zero();

            let segments = 16;
            for j in 0..segments {
                let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                let p1 = port.position + (right * angle1.cos() + up * angle1.sin()) * self.tube_radius;
                let p2 = port.position + (right * angle2.cos() + up * angle2.sin()) * self.tube_radius;

                gizmos.line(p1, p2, port_color);
            }

            // Direction arrow
            gizmos.ray(port.position, port.direction * 0.3, port_color);
        }
    }

    fn type_name(&self) -> &'static str {
        "StraightTube"
    }
}
