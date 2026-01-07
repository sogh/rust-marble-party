use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A half-pipe (open half-cylinder) that marbles can roll along
/// Marbles can ride on the curved walls and do tricks
pub struct HalfPipe {
    /// Length of the half pipe
    pub length: f32,
    /// Radius of the half-cylinder
    pub radius: f32,
    /// Wall thickness
    pub wall_thickness: f32,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
    /// Direction along the half pipe
    direction: Vec3,
    /// Right vector (perpendicular, toward wall)
    right: Vec3,
}

impl HalfPipe {
    pub fn new(length: f32, radius: f32, entry_port: Port) -> Self {
        let direction = entry_port.direction;
        let right = direction.cross(Vec3::Y).normalize_or_zero();

        // If direction is vertical, use a default right
        let right = if right.length() < 0.1 {
            Vec3::X
        } else {
            right
        };

        let exit_pos = entry_port.position + direction * length;
        let exit = Port::new(
            exit_pos,
            direction,
            entry_port.up,
            radius,
        );

        // Bounds include the curved walls
        let bounds = AABB::from_points(
            &[
                entry_port.position + right * radius + Vec3::Y * radius,
                entry_port.position - right * radius,
                exit_pos + right * radius + Vec3::Y * radius,
                exit_pos - right * radius,
            ],
            0.5,
        );

        Self {
            length,
            radius,
            wall_thickness: 0.2,
            entry: entry_port,
            exit,
            bounds,
            direction,
            right,
        }
    }

    /// Create at origin pointing in -Z direction
    pub fn at_origin(length: f32, radius: f32) -> Self {
        let entry = Port::new(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::Y,
            radius,
        );
        Self::new(length, radius, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for HalfPipe {
    fn sdf(&self, point: Vec3) -> f32 {
        // Transform point to local space
        // Local: X = right, Y = up, Z = direction
        let local_origin = self.entry.position;
        let to_point = point - local_origin;

        let local_z = to_point.dot(self.direction); // Along half pipe
        let local_x = to_point.dot(self.right);     // Across half pipe
        let local_y = to_point.dot(Vec3::Y);        // Up

        // Clamp Z to pipe length (with overlap)
        let z_before = -local_z - OVERLAP_DISTANCE;
        let z_after = local_z - self.length - OVERLAP_DISTANCE;

        // Distance from the center axis (at floor level)
        // The half-pipe is a semicircle with center at y=0
        let center_y = 0.0;
        let dy = local_y - center_y;

        // Only consider the bottom half (y <= radius from center)
        // The half pipe is like a U shape when viewed from the end

        // Distance to the curved floor
        let dist_from_center = (local_x * local_x + dy * dy).sqrt();
        let curved_dist = dist_from_center - self.radius;

        // Only apply the curved surface for the lower half
        // Above the rim, it's open space
        if local_y > self.radius {
            // Above the half pipe - open
            let rim_dist = local_y - self.radius;

            // But still bound by X (wall edges)
            if local_x.abs() > self.radius {
                // Outside the walls
                let wall_dist = local_x.abs() - self.radius;
                return (rim_dist.powi(2) + wall_dist.powi(2)).sqrt();
            }
            return -rim_dist; // Open above
        }

        // Handle end caps
        if z_before > 0.0 || z_after > 0.0 {
            let end_dist = z_before.max(z_after);
            return -curved_dist.min(-end_dist);
        }

        // Inside the half pipe region
        -curved_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let to_point = point - self.entry.position;
        let local_z = to_point.dot(self.direction);

        // Core region excludes the overlap zones at each end
        local_z > OVERLAP_DISTANCE && local_z < self.length - OVERLAP_DISTANCE
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
        // Draw the half-pipe shape at entry, middle, and exit
        let positions = [0.0, 0.5, 1.0];

        for t in positions {
            let center = self.entry.position + self.direction * (self.length * t);

            // Draw semicircle
            let segments = 12;
            for j in 0..segments {
                // Semicircle from -90 to +90 degrees
                let angle1 = std::f32::consts::PI * (j as f32 / segments as f32 - 0.5);
                let angle2 = std::f32::consts::PI * ((j + 1) as f32 / segments as f32 - 0.5);

                let p1 = center + self.right * (angle1.sin() * self.radius) + Vec3::Y * (angle1.cos() * self.radius);
                let p2 = center + self.right * (angle2.sin() * self.radius) + Vec3::Y * (angle2.cos() * self.radius);

                gizmos.line(p1, p2, color);
            }
        }

        // Draw edges along the length
        let edge_angles = [-std::f32::consts::FRAC_PI_2, 0.0, std::f32::consts::FRAC_PI_2];
        for angle in edge_angles {
            let offset = self.right * (angle.sin() * self.radius) + Vec3::Y * (angle.cos() * self.radius);
            let start = self.entry.position + offset;
            let end = self.exit.position + offset;
            gizmos.line(start, end, color.with_alpha(0.5));
        }

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(self.entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(self.entry.position, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(self.exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(self.exit.position, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "HalfPipe"
    }
}
