use bevy::prelude::*;
use crate::track::{Port, PortProfile, AABB, Segment};

/// A finish line segment where marbles collect at the end of the track.
/// Features:
/// - Wide flat area for marbles to collect
/// - Front wall stops marbles
/// - Side walls contain marbles
/// - Completely flat floor (no slope)
/// - No exit - this is the track terminus
pub struct FinishLine {
    /// Width of the finish area (perpendicular to travel direction)
    pub width: f32,
    /// Length/depth of the finish area (along travel direction)
    pub length: f32,
    /// Height of side and front walls
    pub wall_height: f32,
    /// Entry port (FlatFloor profile)
    entry: Port,
    /// Bounding box
    bounds: AABB,
    /// Cached corner positions
    corners: FinishLineCorners,
    /// Forward direction
    forward: Vec3,
    /// Right direction
    right: Vec3,
}

struct FinishLineCorners {
    // Floor corners (entry = back, front = wall)
    back_left: Vec3,
    back_right: Vec3,
    front_left: Vec3,
    front_right: Vec3,
    // Wall top corners
    wall_front_left: Vec3,
    wall_front_right: Vec3,
    wall_front_top_left: Vec3,
    wall_front_top_right: Vec3,
    wall_side_left_top: Vec3,
    wall_side_right_top: Vec3,
}

impl FinishLine {
    pub fn new(width: f32, length: f32, wall_height: f32, entry_port: Port) -> Self {
        // Get the forward direction (direction marbles will travel)
        let forward = Vec3::new(
            entry_port.direction.x,
            0.0,
            entry_port.direction.z,
        ).normalize_or_zero();

        let forward = if forward.length_squared() < 0.01 {
            Vec3::NEG_Z
        } else {
            forward
        };

        let right = forward.cross(Vec3::Y).normalize();

        // Get the floor Y from the incoming port's profile, or fall back to position.y
        let entry_floor_y = match &entry_port.profile {
            PortProfile::FlatFloor { floor_y, .. } => *floor_y,
            _ => entry_port.position.y,
        };

        // Calculate end position - follow forward direction at floor level
        // The floor stays flat (no descent)
        let end_pos = Vec3::new(
            entry_port.position.x + forward.x * length,
            entry_floor_y, // Keep at floor level
            entry_port.position.z + forward.z * length,
        );

        // Create entry with FlatFloor profile at the correct floor level
        let entry_with_profile = Port::with_profile(
            Vec3::new(entry_port.position.x, entry_floor_y, entry_port.position.z),
            entry_port.direction,
            entry_port.up,
            width / 2.0,
            PortProfile::flat_floor(width, entry_floor_y),
        );

        // Calculate corners using the floor-level entry position
        let half_width = width / 2.0;
        let entry_floor_pos = entry_with_profile.position;
        let corners = FinishLineCorners {
            back_left: entry_floor_pos - right * half_width,
            back_right: entry_floor_pos + right * half_width,
            front_left: end_pos - right * half_width,
            front_right: end_pos + right * half_width,
            // Front wall corners
            wall_front_left: end_pos - right * half_width,
            wall_front_right: end_pos + right * half_width,
            wall_front_top_left: end_pos - right * half_width + Vec3::Y * wall_height,
            wall_front_top_right: end_pos + right * half_width + Vec3::Y * wall_height,
            // Side wall tops
            wall_side_left_top: entry_floor_pos - right * half_width + Vec3::Y * wall_height,
            wall_side_right_top: entry_floor_pos + right * half_width + Vec3::Y * wall_height,
        };

        // Compute bounds
        let all_corners = [
            corners.back_left,
            corners.back_right,
            corners.front_left,
            corners.front_right,
            corners.wall_front_top_left,
            corners.wall_front_top_right,
            corners.wall_side_left_top,
            corners.wall_side_right_top,
        ];
        let bounds = AABB::from_points(&all_corners, 0.5);

        Self {
            width,
            length,
            wall_height,
            entry: entry_with_profile,
            bounds,
            corners,
            forward,
            right,
        }
    }

    /// Create a finish line for a given number of marbles
    /// Automatically calculates width based on marble count and size
    pub fn for_marbles(
        num_marbles: usize,
        marble_radius: f32,
        length: f32,
        entry_pos: Vec3,
        forward_dir: Vec3,
    ) -> Self {
        // Each marble needs diameter + gap
        let marble_spacing = marble_radius * 2.0 + 0.1;
        let width = marble_spacing * num_marbles as f32 + 0.2; // Extra margin

        let entry = Port::with_profile(
            entry_pos,
            forward_dir.normalize(),
            Vec3::Y,
            width / 2.0,
            PortProfile::flat_floor(width, entry_pos.y),
        );

        Self::new(width, length, 1.5, entry)
    }
}

/// How far to extend the SDF past segment entry for smooth blending
const OVERLAP_DISTANCE: f32 = 2.5;

impl Segment for FinishLine {
    fn sdf(&self, point: Vec3) -> f32 {
        // Get geometry
        let entry_pos = self.entry.position;
        let floor_y = entry_pos.y; // Flat floor

        // Project point onto forward direction
        let local = point - entry_pos;
        let along = local.dot(self.forward);

        // Reject points too far behind entry
        if along < -OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Reject points too far past the front wall
        if along > self.length + 0.5 {
            return f32::MAX;
        }

        // Lateral position
        let across = local.dot(self.right);
        let half_width = self.width / 2.0;

        // Height above floor
        let height_above_floor = point.y - floor_y;

        // Distance to surfaces
        let dist_to_floor = height_above_floor;
        let dist_to_left_wall = across + half_width;
        let dist_to_right_wall = half_width - across;
        let dist_to_front_wall = self.length - along;

        // Below floor
        if dist_to_floor < 0.0 {
            return dist_to_floor;
        }

        // Outside walls
        if dist_to_left_wall < 0.0 {
            return dist_to_left_wall;
        }
        if dist_to_right_wall < 0.0 {
            return dist_to_right_wall;
        }

        // Front wall collision
        if along > 0.0 && dist_to_front_wall < 0.0 {
            return dist_to_front_wall;
        }

        // Above walls - open space
        if height_above_floor > self.wall_height {
            if dist_to_left_wall > 0.0 && dist_to_right_wall > 0.0 && dist_to_front_wall > 0.0 {
                return height_above_floor - self.wall_height;
            }
        }

        // Inside the box - return distance to nearest surface
        let mut min_dist = dist_to_floor;
        min_dist = min_dist.min(dist_to_left_wall);
        min_dist = min_dist.min(dist_to_right_wall);

        // Front wall counts when inside the segment
        if along > 0.0 {
            min_dist = min_dist.min(dist_to_front_wall);
        }

        min_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let entry_pos = self.entry.position;
        let local = point - entry_pos;
        let along = local.dot(self.forward);

        // Core region is the middle section
        along > OVERLAP_DISTANCE && along < self.length - 0.5
    }

    fn entry_port(&self) -> Port {
        self.entry.clone()
    }

    fn exit_ports(&self) -> Vec<Port> {
        // No exits - this is the track terminus
        vec![]
    }

    fn bounds(&self) -> AABB {
        self.bounds.clone()
    }

    fn draw_debug_gizmos(&self, gizmos: &mut Gizmos, color: Color) {
        let c = &self.corners;

        // Draw floor
        gizmos.line(c.back_left, c.back_right, color);
        gizmos.line(c.front_left, c.front_right, color);
        gizmos.line(c.back_left, c.front_left, color);
        gizmos.line(c.back_right, c.front_right, color);

        // Draw front wall (distinct color - yellow for visibility)
        let wall_color = Color::srgb(1.0, 1.0, 0.0);
        gizmos.line(c.front_left, c.wall_front_top_left, wall_color);
        gizmos.line(c.front_right, c.wall_front_top_right, wall_color);
        gizmos.line(c.wall_front_top_left, c.wall_front_top_right, wall_color);

        // Draw side walls
        gizmos.line(c.back_left, c.wall_side_left_top, color);
        gizmos.line(c.front_left, c.wall_front_top_left, color);
        gizmos.line(c.wall_side_left_top, c.wall_front_top_left, color);

        gizmos.line(c.back_right, c.wall_side_right_top, color);
        gizmos.line(c.front_right, c.wall_front_top_right, color);
        gizmos.line(c.wall_side_right_top, c.wall_front_top_right, color);

        // Entry marker (back center)
        let back_center = (c.back_left + c.back_right) / 2.0;
        gizmos.sphere(Isometry3d::from_translation(back_center), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(back_center, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        // Front wall center marker (shows this is the terminus)
        let front_center = (c.front_left + c.front_right) / 2.0 + Vec3::Y * (self.wall_height / 2.0);
        gizmos.sphere(Isometry3d::from_translation(front_center), 0.2, wall_color);
    }

    fn type_name(&self) -> &'static str {
        "FinishLine"
    }

    fn descent(&self) -> f32 {
        0.0 // Completely flat
    }

    fn sdf_gradient(&self, point: Vec3) -> Vec3 {
        let entry_pos = self.entry.position;
        let floor_y = entry_pos.y;

        let local = point - entry_pos;
        let along = local.dot(self.forward);
        let across = local.dot(self.right);

        // Calculate distances to each surface
        let dist_to_floor = point.y - floor_y;
        let half_width = self.width / 2.0;
        let dist_to_left = across + half_width;
        let dist_to_right = half_width - across;
        let dist_to_front = self.length - along;

        // Return gradient pointing away from nearest surface
        let min_dist = dist_to_floor
            .min(dist_to_left)
            .min(dist_to_right)
            .min(if along > 0.0 { dist_to_front } else { f32::MAX });

        if min_dist == dist_to_floor {
            // Floor - gradient points up
            Vec3::Y
        } else if min_dist == dist_to_left {
            // Left wall - gradient points right
            self.right
        } else if min_dist == dist_to_right {
            // Right wall - gradient points left
            -self.right
        } else {
            // Front wall - gradient points backward
            -self.forward
        }
    }
}
