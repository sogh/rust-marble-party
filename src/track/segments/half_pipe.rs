use bevy::prelude::*;
use crate::track::{Port, PortProfile, AABB, Segment};

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
    /// Up vector (perpendicular to direction and right, for sloped half-pipes)
    up: Vec3,
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

        // Up vector is perpendicular to both direction and right
        // This handles sloped half-pipes correctly
        let up = right.cross(direction).normalize();

        let exit_pos = entry_port.position + direction * length;

        // Create entry port with HalfPipe profile
        let entry_with_profile = Port::with_profile(
            entry_port.position,
            entry_port.direction,
            entry_port.up,
            radius,
            PortProfile::half_pipe(radius),
        );

        let exit = Port::with_profile(
            exit_pos,
            direction,
            entry_port.up,
            radius,
            PortProfile::half_pipe(radius),
        );

        // Bounds include the curved walls (use up instead of Vec3::Y for slopes)
        let bounds = AABB::from_points(
            &[
                entry_port.position + right * radius + up * radius,
                entry_port.position - right * radius - up * radius,
                exit_pos + right * radius + up * radius,
                exit_pos - right * radius - up * radius,
            ],
            0.5,
        );

        Self {
            length,
            radius,
            wall_thickness: 0.2,
            entry: entry_with_profile,
            exit,
            bounds,
            direction,
            right,
            up,
        }
    }

    /// Create at origin pointing in -Z direction
    pub fn at_origin(length: f32, radius: f32) -> Self {
        let entry = Port::with_profile(
            Vec3::ZERO,
            Vec3::NEG_Z,
            Vec3::Y,
            radius,
            PortProfile::half_pipe(radius),
        );
        Self::new(length, radius, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
/// Experiment: one marble diameter (0.4)
const OVERLAP_DISTANCE: f32 = 0.4;

impl Segment for HalfPipe {
    fn sdf(&self, point: Vec3) -> f32 {
        // Transform point to local space
        // Local: X = right, Y = up (perpendicular to slope), Z = direction
        let local_origin = self.entry.position;
        let to_point = point - local_origin;

        let local_z = to_point.dot(self.direction); // Along half pipe
        let local_x = to_point.dot(self.right);     // Across half pipe
        let local_y = to_point.dot(self.up);        // Up (perpendicular to slope)

        // Reject points outside the segment bounds (like tubes do)
        // Allow OVERLAP_DISTANCE before entry for seamless tube connection
        if local_z < -OVERLAP_DISTANCE || local_z > self.length + OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Half-pipe geometry:
        // - The center axis runs along direction at local_y = 0, local_x = 0
        // - The curved floor is a semicircle below the axis (local_y < 0)
        // - The rim is at local_y = 0, the floor bottom at local_y = -radius
        // - Above the rim (local_y > 0) is open air

        // Above the rim = open air, not part of the half-pipe
        if local_y > 0.0 {
            return f32::MAX;
        }

        // Below the rim: use distance from the curved floor surface
        // The floor is at distance = radius from the center axis
        let dist_from_center = (local_x * local_x + local_y * local_y).sqrt();

        // SDF for bowl/gutter geometry:
        // - Positive when inside the curve (free space where marbles roll)
        // - Zero at the curved surface
        // - Negative when penetrating the floor (solid)
        self.radius - dist_from_center
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

            // Draw semicircle using self.up for proper slope orientation
            // Negate up so the curved floor is at the bottom (where marbles roll)
            let segments = 12;
            for j in 0..segments {
                // Semicircle from -90 to +90 degrees
                let angle1 = std::f32::consts::PI * (j as f32 / segments as f32 - 0.5);
                let angle2 = std::f32::consts::PI * ((j + 1) as f32 / segments as f32 - 0.5);

                let p1 = center + self.right * (angle1.sin() * self.radius) - self.up * (angle1.cos() * self.radius);
                let p2 = center + self.right * (angle2.sin() * self.radius) - self.up * (angle2.cos() * self.radius);

                gizmos.line(p1, p2, color);
            }
        }

        // Draw edges along the length (floor at bottom, rims at sides)
        let edge_angles = [-std::f32::consts::FRAC_PI_2, 0.0, std::f32::consts::FRAC_PI_2];
        for angle in edge_angles {
            let offset = self.right * (angle.sin() * self.radius) - self.up * (angle.cos() * self.radius);
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
