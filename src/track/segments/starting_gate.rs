use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A wide starting platform where marbles line up horizontally for a fair start.
/// Features:
/// - Wide flat area for marbles to sit side-by-side
/// - Back wall prevents backward rolling
/// - Side walls contain marbles
/// - Slight forward slope gives initial momentum
/// - Open front connects to next segment (typically a funnel)
pub struct StartingGate {
    /// Width of the starting area (perpendicular to travel direction)
    pub width: f32,
    /// Length/depth of the starting area (along travel direction)
    pub length: f32,
    /// Height of side and back walls
    pub wall_height: f32,
    /// Slope angle in radians (positive = descending forward)
    pub slope_angle: f32,
    /// Entry port (at the back wall - for reference, marbles spawn here)
    entry: Port,
    /// Exit port (at the front, where marbles exit)
    exit: Port,
    /// Bounding box
    bounds: AABB,
    /// Cached corner positions
    corners: StartingGateCorners,
}

struct StartingGateCorners {
    // Floor corners (entry = back, exit = front)
    back_left: Vec3,
    back_right: Vec3,
    front_left: Vec3,
    front_right: Vec3,
    // Wall top corners
    wall_back_left: Vec3,
    wall_back_right: Vec3,
    wall_front_left: Vec3,
    wall_front_right: Vec3,
}

impl StartingGate {
    pub fn new(width: f32, length: f32, wall_height: f32, slope_angle: f32, entry_port: Port) -> Self {
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

        // Calculate exit position (front of the gate)
        let horizontal_length = length * slope_angle.cos();
        let descent = length * slope_angle.sin();

        let exit_pos = entry_port.position
            + forward * horizontal_length
            - Vec3::Y * descent;

        let exit_dir = (exit_pos - entry_port.position).normalize();

        let exit = Port::new(
            exit_pos,
            exit_dir,
            Vec3::Y,
            width / 2.0,
        );

        // Calculate corners
        let half_width = width / 2.0;
        let corners = StartingGateCorners {
            back_left: entry_port.position - right * half_width,
            back_right: entry_port.position + right * half_width,
            front_left: exit_pos - right * half_width,
            front_right: exit_pos + right * half_width,
            wall_back_left: entry_port.position - right * half_width + Vec3::Y * wall_height,
            wall_back_right: entry_port.position + right * half_width + Vec3::Y * wall_height,
            wall_front_left: exit_pos - right * half_width + Vec3::Y * wall_height,
            wall_front_right: exit_pos + right * half_width + Vec3::Y * wall_height,
        };

        // Compute bounds
        let all_corners = [
            corners.back_left,
            corners.back_right,
            corners.front_left,
            corners.front_right,
            corners.wall_back_left,
            corners.wall_back_right,
            corners.wall_front_left,
            corners.wall_front_right,
        ];
        let bounds = AABB::from_points(&all_corners, 0.5);

        Self {
            width,
            length,
            wall_height,
            slope_angle,
            entry: entry_port,
            exit,
            bounds,
            corners,
        }
    }

    /// Create a starting gate for a given number of marbles
    /// Automatically calculates width based on marble count and size
    pub fn for_marbles(
        num_marbles: usize,
        marble_radius: f32,
        length: f32,
        slope_angle: f32,
        entry_pos: Vec3,
        forward_dir: Vec3,
    ) -> Self {
        // Each marble needs diameter + gap
        let marble_spacing = marble_radius * 2.0 + 0.1;
        let width = marble_spacing * num_marbles as f32 + 0.2; // Extra margin

        let entry = Port::new(
            entry_pos,
            forward_dir.normalize(),
            Vec3::Y,
            width / 2.0,
        );

        Self::new(width, length, 1.0, slope_angle, entry)
    }

    /// Get spawn positions for marbles (evenly spaced across the width)
    pub fn get_spawn_positions(&self, num_marbles: usize, marble_radius: f32) -> Vec<Vec3> {
        // Use the actual slope geometry from entry to exit
        let slope_vec = self.exit.position - self.entry.position;
        let slope_dir = slope_vec.normalize();
        let slope_length = slope_vec.length();

        // Calculate right vector the same way the SDF does
        let horizontal_dir = Vec3::new(slope_vec.x, 0.0, slope_vec.z).normalize_or_zero();
        let right = if horizontal_dir.length_squared() > 0.01 {
            horizontal_dir.cross(Vec3::Y).normalize_or_zero()
        } else {
            Vec3::X
        };

        // Spawn marbles at the center of the gate (t=0.5)
        let t = 0.5;
        let spawn_on_slope = self.entry.position + slope_vec * t;

        // Place marble center slightly INTO the floor so first collision is detected
        // The physics will push it up. Using 0.9 * radius ensures reliable collision.
        let spawn_center = spawn_on_slope + Vec3::Y * (marble_radius * 0.9);

        // Calculate spacing
        let marble_spacing = marble_radius * 2.0 + 0.15;
        let total_marble_width = (num_marbles as f32 - 1.0) * marble_spacing;
        let start_offset = -total_marble_width / 2.0;

        let mut positions = Vec::new();
        for i in 0..num_marbles {
            let lateral_offset = start_offset + i as f32 * marble_spacing;
            let pos = spawn_center + right * lateral_offset;
            positions.push(pos);
        }

        positions
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
/// The starting gate has MINIMAL overlap since marbles fall off the front edge
const OVERLAP_DISTANCE: f32 = 0.5;

impl Segment for StartingGate {
    fn sdf(&self, point: Vec3) -> f32 {
        // Get geometry
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let slope_vec = exit_pos - entry_pos;
        let slope_length = slope_vec.length();
        let slope_dir = slope_vec.normalize();

        // Project point onto slope direction
        let local = point - entry_pos;
        let along = local.dot(slope_dir);

        // Reject points too far behind or past exit
        // Very minimal overlap at exit - marbles fall off the edge into funnel
        if along < -0.5 || along > slope_length + OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Get right vector
        let horizontal_dir = Vec3::new(slope_vec.x, 0.0, slope_vec.z).normalize_or_zero();
        let right = if horizontal_dir.length_squared() > 0.01 {
            horizontal_dir.cross(Vec3::Y).normalize_or_zero()
        } else {
            self.entry.direction.cross(Vec3::Y).normalize_or_zero()
        };

        // Clamp along for floor height calculation
        let t = (along / slope_length).clamp(0.0, 1.0);
        let floor_point = entry_pos + slope_vec * t;
        let floor_y = floor_point.y;

        // Lateral position
        let across = local.dot(right);
        let half_width = self.width / 2.0;

        // Height above floor
        let height_above_floor = point.y - floor_y;

        // Distance to surfaces
        let dist_to_floor = height_above_floor;
        let dist_to_left_wall = across + half_width;
        let dist_to_right_wall = half_width - across;
        let dist_to_back_wall = along + 0.1; // Small buffer at back

        // PAST THE EXIT: Only provide floor collision (no walls)
        // This allows marbles to roll off the edge cleanly
        if along > slope_length {
            // Only floor matters past exit - marbles are falling
            if dist_to_floor < 0.0 {
                return dist_to_floor;
            }
            // Above floor = no collision from gate
            return f32::MAX;
        }

        // Below floor
        if dist_to_floor < 0.0 {
            return dist_to_floor;
        }

        // Outside walls (only before exit)
        if dist_to_left_wall < 0.0 {
            return dist_to_left_wall;
        }
        if dist_to_right_wall < 0.0 {
            return dist_to_right_wall;
        }

        // Behind back wall
        if along < 0.0 && dist_to_back_wall < 0.0 {
            return dist_to_back_wall;
        }

        // Above walls - open space
        if height_above_floor > self.wall_height {
            if dist_to_left_wall > 0.0 && dist_to_right_wall > 0.0 {
                return height_above_floor - self.wall_height;
            }
        }

        // Inside the box - return distance to nearest surface
        let mut min_dist = dist_to_floor;
        min_dist = min_dist.min(dist_to_left_wall);
        min_dist = min_dist.min(dist_to_right_wall);

        // Back wall only counts if we're near it
        if along < 1.0 {
            min_dist = min_dist.min(dist_to_back_wall);
        }

        min_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let slope_vec = exit_pos - entry_pos;
        let slope_length = slope_vec.length();
        let slope_dir = slope_vec.normalize();

        let local = point - entry_pos;
        let along = local.dot(slope_dir);

        // Core region is the middle section
        along > 0.5 && along < slope_length - OVERLAP_DISTANCE
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
        let c = &self.corners;

        // Draw floor
        gizmos.line(c.back_left, c.back_right, color);
        gizmos.line(c.front_left, c.front_right, color);
        gizmos.line(c.back_left, c.front_left, color);
        gizmos.line(c.back_right, c.front_right, color);

        // Draw back wall
        gizmos.line(c.back_left, c.wall_back_left, color);
        gizmos.line(c.back_right, c.wall_back_right, color);
        gizmos.line(c.wall_back_left, c.wall_back_right, color);

        // Draw side walls
        gizmos.line(c.back_left, c.wall_back_left, color);
        gizmos.line(c.front_left, c.wall_front_left, color);
        gizmos.line(c.wall_back_left, c.wall_front_left, color);

        gizmos.line(c.back_right, c.wall_back_right, color);
        gizmos.line(c.front_right, c.wall_front_right, color);
        gizmos.line(c.wall_back_right, c.wall_front_right, color);

        // Draw wall tops (no front wall - open exit)
        gizmos.line(c.wall_front_left, c.wall_back_left, color);
        gizmos.line(c.wall_front_right, c.wall_back_right, color);

        // Entry marker (back center)
        let back_center = (c.back_left + c.back_right) / 2.0;
        gizmos.sphere(Isometry3d::from_translation(back_center), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(back_center, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        // Exit marker (front center)
        let front_center = (c.front_left + c.front_right) / 2.0;
        gizmos.sphere(Isometry3d::from_translation(front_center), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(front_center, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));

        // Draw spawn position indicators
        let spawn_positions = self.get_spawn_positions(8, 0.2);
        for pos in spawn_positions {
            gizmos.sphere(Isometry3d::from_translation(pos), 0.1, Color::srgb(0.5, 0.5, 1.0));
        }
    }

    fn type_name(&self) -> &'static str {
        "StartingGate"
    }

    fn descent(&self) -> f32 {
        self.length * self.slope_angle.sin()
    }

    fn sdf_gradient(&self, point: Vec3) -> Vec3 {
        // Get geometry
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let slope_vec = exit_pos - entry_pos;
        let slope_dir = slope_vec.normalize();

        // Get right vector
        let horizontal_dir = Vec3::new(slope_vec.x, 0.0, slope_vec.z).normalize_or_zero();
        let right = if horizontal_dir.length_squared() > 0.01 {
            horizontal_dir.cross(Vec3::Y).normalize_or_zero()
        } else {
            self.entry.direction.cross(Vec3::Y).normalize_or_zero()
        };

        let local = point - entry_pos;
        let along = local.dot(slope_dir);
        let across = local.dot(right);

        // Calculate distances to each surface
        let t = (along / slope_vec.length()).clamp(0.0, 1.0);
        let floor_point = entry_pos + slope_vec * t;
        let floor_y = floor_point.y;

        let dist_to_floor = point.y - floor_y;
        let half_width = self.width / 2.0;
        let dist_to_left = across + half_width;
        let dist_to_right = half_width - across;
        let dist_to_back = along;

        // Return gradient pointing away from nearest surface
        let min_dist = dist_to_floor
            .min(dist_to_left)
            .min(dist_to_right)
            .min(if along < 1.0 { dist_to_back } else { f32::MAX });

        if min_dist == dist_to_floor {
            // Floor - gradient points up (perpendicular to sloped floor)
            // Use right.cross(slope_dir) to get upward-pointing normal
            let floor_normal = right.cross(slope_dir).normalize();
            return floor_normal;
        } else if min_dist == dist_to_left {
            // Left wall - gradient points right
            return right;
        } else if min_dist == dist_to_right {
            // Right wall - gradient points left
            return -right;
        } else {
            // Back wall - gradient points forward
            return slope_dir;
        }
    }
}
