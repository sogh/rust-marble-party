use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A funnel that collects marbles from a wide opening to a narrow exit
/// The funnel has angled walls that guide marbles toward the center
pub struct Funnel {
    /// Vertical depth of the funnel
    pub depth: f32,
    /// Radius at top (entry, larger)
    pub top_radius: f32,
    /// Radius at bottom (exit, smaller)
    pub bottom_radius: f32,
    /// Wall thickness
    pub wall_thickness: f32,
    /// Entry port (at top)
    entry: Port,
    /// Exit port (at bottom)
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl Funnel {
    pub fn new(depth: f32, top_radius: f32, bottom_radius: f32, entry_port: Port) -> Self {
        // Funnel goes downward from entry
        let exit_pos = entry_port.position - Vec3::Y * depth;
        let exit = Port::new(
            exit_pos,
            Vec3::NEG_Y, // Exit pointing down
            Vec3::NEG_Z, // Arbitrary up for vertical exit
            bottom_radius,
        );

        let bounds = AABB::from_points(
            &[
                entry_port.position + Vec3::new(top_radius, 0.0, top_radius),
                entry_port.position + Vec3::new(-top_radius, 0.0, -top_radius),
                exit_pos + Vec3::new(bottom_radius, 0.0, bottom_radius),
                exit_pos + Vec3::new(-bottom_radius, 0.0, -bottom_radius),
            ],
            0.5,
        );

        Self {
            depth,
            top_radius,
            bottom_radius,
            wall_thickness: 0.2,
            entry: entry_port,
            exit,
            bounds,
        }
    }

    /// Create a funnel at origin
    pub fn at_origin(depth: f32, top_radius: f32, bottom_radius: f32) -> Self {
        let entry = Port::new(
            Vec3::new(0.0, depth, 0.0),
            Vec3::NEG_Y, // Entry pointing down into funnel
            Vec3::NEG_Z,
            top_radius,
        );
        Self::new(depth, top_radius, bottom_radius, entry)
    }

    /// Get the radius at a given height (0 = bottom, 1 = top)
    fn radius_at(&self, t: f32) -> f32 {
        self.bottom_radius + (self.top_radius - self.bottom_radius) * t.clamp(0.0, 1.0)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for Funnel {
    fn sdf(&self, point: Vec3) -> f32 {
        let top_y = self.entry.position.y;
        let bottom_y = self.exit.position.y;
        let center_xz = Vec3::new(self.entry.position.x, 0.0, self.entry.position.z);

        // Height within funnel (0 at bottom, 1 at top)
        let t = ((point.y - bottom_y) / self.depth).clamp(0.0, 1.0);

        // Radius at this height
        let radius = self.radius_at(t);

        // Horizontal distance from center axis
        let point_xz = Vec3::new(point.x, 0.0, point.z);
        let radial_dist = (point_xz - center_xz).length();

        // Distance to cone surface
        let cone_dist = radial_dist - radius;

        // Distance to top and bottom planes (with overlap)
        let above_top = point.y - top_y - OVERLAP_DISTANCE;
        let below_bottom = bottom_y - OVERLAP_DISTANCE - point.y;

        // If above the funnel
        if above_top > 0.0 {
            // Hemispherical cap at top
            let dist_to_rim = ((radial_dist - self.top_radius).max(0.0).powi(2) + above_top.powi(2)).sqrt();
            return -dist_to_rim.min(-cone_dist);
        }

        // If below the funnel
        if below_bottom > 0.0 {
            // Cylindrical extension below
            let dist_in_cylinder = radial_dist - self.bottom_radius;
            return -dist_in_cylinder.max(below_bottom);
        }

        // Inside the funnel body
        -cone_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let top_y = self.entry.position.y;
        let bottom_y = self.exit.position.y;

        // Core region excludes overlap zones at top and bottom
        point.y > bottom_y + OVERLAP_DISTANCE && point.y < top_y - OVERLAP_DISTANCE
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
        let center = Vec3::new(self.entry.position.x, 0.0, self.entry.position.z);
        let top_y = self.entry.position.y;
        let bottom_y = self.exit.position.y;

        // Draw circles at different heights
        let heights = [0.0, 0.25, 0.5, 0.75, 1.0];
        for t in heights {
            let y = bottom_y + self.depth * t;
            let radius = self.radius_at(t);

            let segments = 24;
            for j in 0..segments {
                let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                let p1 = center + Vec3::new(angle1.cos() * radius, y, angle1.sin() * radius);
                let p2 = center + Vec3::new(angle2.cos() * radius, y, angle2.sin() * radius);

                gizmos.line(p1, p2, color);
            }
        }

        // Draw vertical lines connecting top to bottom
        for i in 0..8 {
            let angle = (i as f32 / 8.0) * std::f32::consts::TAU;
            let top_point = center + Vec3::new(angle.cos() * self.top_radius, top_y, angle.sin() * self.top_radius);
            let bottom_point = center + Vec3::new(angle.cos() * self.bottom_radius, bottom_y, angle.sin() * self.bottom_radius);
            gizmos.line(top_point, bottom_point, color.with_alpha(0.5));
        }

        // Entry/exit markers
        let entry_center = Vec3::new(center.x, top_y, center.z);
        let exit_center = Vec3::new(center.x, bottom_y, center.z);

        gizmos.sphere(Isometry3d::from_translation(entry_center), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(entry_center, Vec3::NEG_Y * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(exit_center), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(exit_center, Vec3::NEG_Y * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "Funnel"
    }
}
