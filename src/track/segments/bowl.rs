use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A spherical bowl where marbles can roll around
/// Has an entry at the rim and an exit hole at the bottom
pub struct Bowl {
    /// Radius of the bowl
    pub radius: f32,
    /// Radius of the exit hole at the bottom
    pub exit_hole_radius: f32,
    /// Center position of the bowl (at the rim level)
    center: Vec3,
    /// Entry port (at rim)
    entry: Port,
    /// Exit port (at bottom center)
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl Bowl {
    pub fn new(radius: f32, exit_hole_radius: f32, entry_port: Port) -> Self {
        // Bowl center is at the entry position
        let center = entry_port.position;

        // Exit is at the bottom of the bowl
        let exit_pos = center - Vec3::Y * radius;
        let exit = Port::new(
            exit_pos,
            Vec3::NEG_Y, // Exit pointing down
            Vec3::NEG_Z,
            exit_hole_radius,
        );

        let bounds = AABB::from_points(
            &[
                center + Vec3::new(radius, 0.0, radius),
                center + Vec3::new(-radius, -radius, -radius),
            ],
            0.5,
        );

        Self {
            radius,
            exit_hole_radius,
            center,
            entry: entry_port,
            exit,
            bounds,
        }
    }

    /// Create a bowl at origin
    pub fn at_origin(radius: f32, exit_hole_radius: f32) -> Self {
        let entry = Port::new(
            Vec3::new(radius, 0.0, 0.0), // Entry at rim
            Vec3::NEG_X, // Pointing into bowl
            Vec3::Y,
            radius * 0.3,
        );
        Self::new(radius, exit_hole_radius, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for Bowl {
    fn sdf(&self, point: Vec3) -> f32 {
        // Bowl center is at self.center, bowl curves down from there
        // The bowl is the lower hemisphere of a sphere centered at self.center

        let bowl_sphere_center = self.center;
        let to_point = point - bowl_sphere_center;

        // Distance from sphere center
        let dist_from_center = to_point.length();

        // Distance to sphere surface (inside is negative)
        let sphere_dist = dist_from_center - self.radius;

        // Only the lower hemisphere is the bowl
        // Above the rim (y > center.y), it's open
        if point.y > self.center.y + OVERLAP_DISTANCE {
            // Above the bowl - open space
            return -(point.y - self.center.y - OVERLAP_DISTANCE);
        }

        // Check for exit hole at bottom
        let bottom_y = self.center.y - self.radius;
        if point.y < bottom_y + self.exit_hole_radius {
            // Near the exit hole
            let xz_dist = Vec2::new(to_point.x, to_point.z).length();
            if xz_dist < self.exit_hole_radius {
                // In the exit hole - it's open (extending down)
                let hole_edge_dist = self.exit_hole_radius - xz_dist;
                return -hole_edge_dist.min(bottom_y + OVERLAP_DISTANCE - point.y);
            }
        }

        // Inside the bowl region
        -sphere_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        // Core is the main bowl area, not the rim or exit hole
        let to_point = point - self.center;
        let dist = to_point.length();

        // Must be inside the sphere and below the rim
        point.y < self.center.y - OVERLAP_DISTANCE &&
            point.y > self.center.y - self.radius + OVERLAP_DISTANCE &&
            dist < self.radius
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
        // Draw horizontal circles at different depths
        let depths = [0.0, 0.25, 0.5, 0.75, 0.9];

        for d in depths {
            let y = self.center.y - self.radius * d;
            // Radius at this depth (from sphere equation)
            let r = (self.radius * self.radius - (self.radius * d).powi(2)).sqrt();

            if r > 0.1 {
                let segments = 24;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = self.center + Vec3::new(angle1.cos() * r, y - self.center.y, angle1.sin() * r);
                    let p2 = self.center + Vec3::new(angle2.cos() * r, y - self.center.y, angle2.sin() * r);

                    gizmos.line(p1, p2, color);
                }
            }
        }

        // Draw vertical arcs
        for i in 0..4 {
            let angle = (i as f32 / 4.0) * std::f32::consts::TAU;
            let dir = Vec3::new(angle.cos(), 0.0, angle.sin());

            let arc_segments = 12;
            for j in 0..arc_segments {
                let t1 = j as f32 / arc_segments as f32;
                let t2 = (j + 1) as f32 / arc_segments as f32;

                // Arc from rim down to bottom
                let phi1 = t1 * std::f32::consts::FRAC_PI_2;
                let phi2 = t2 * std::f32::consts::FRAC_PI_2;

                let p1 = self.center + dir * (phi1.cos() * self.radius) - Vec3::Y * (phi1.sin() * self.radius);
                let p2 = self.center + dir * (phi2.cos() * self.radius) - Vec3::Y * (phi2.sin() * self.radius);

                gizmos.line(p1, p2, color.with_alpha(0.5));
            }
        }

        // Draw exit hole
        let exit_y = self.center.y - self.radius;
        let hole_segments = 12;
        for j in 0..hole_segments {
            let angle1 = (j as f32 / hole_segments as f32) * std::f32::consts::TAU;
            let angle2 = ((j + 1) as f32 / hole_segments as f32) * std::f32::consts::TAU;

            let p1 = Vec3::new(
                self.center.x + angle1.cos() * self.exit_hole_radius,
                exit_y,
                self.center.z + angle1.sin() * self.exit_hole_radius,
            );
            let p2 = Vec3::new(
                self.center.x + angle2.cos() * self.exit_hole_radius,
                exit_y,
                self.center.z + angle2.sin() * self.exit_hole_radius,
            );

            gizmos.line(p1, p2, Color::srgb(1.0, 0.5, 0.0));
        }

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(self.entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(self.entry.position, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(self.exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(self.exit.position, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "Bowl"
    }
}
