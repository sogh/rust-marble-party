use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A flat angled slope with side walls (like a trough or gutter)
pub struct FlatSlope {
    /// Length of the slope
    pub length: f32,
    /// Width of the channel
    pub width: f32,
    /// Height of side walls
    pub wall_height: f32,
    /// Slope angle in radians (positive = descending)
    pub slope_angle: f32,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
    /// Corners for visualization and SDF
    corners: FlatSlopeCorners,
}

struct FlatSlopeCorners {
    // Floor corners
    floor_entry_left: Vec3,
    floor_entry_right: Vec3,
    floor_exit_left: Vec3,
    floor_exit_right: Vec3,
    // Wall top corners
    wall_entry_left: Vec3,
    wall_entry_right: Vec3,
    wall_exit_left: Vec3,
    wall_exit_right: Vec3,
}

impl FlatSlope {
    pub fn new(length: f32, width: f32, wall_height: f32, slope_angle: f32, entry_port: Port) -> Self {
        // Calculate the slope direction (entry direction rotated down by slope_angle)
        let right = entry_port.direction.cross(entry_port.up).normalize();

        // Calculate descent
        let horizontal_length = length * slope_angle.cos();
        let descent = length * slope_angle.sin();

        // Exit position
        let exit_pos = entry_port.position
            + entry_port.direction * horizontal_length
            - Vec3::Y * descent;

        // Exit direction (same as entry, following the slope)
        let exit_dir = (exit_pos - entry_port.position).normalize();

        let exit = Port::new(
            exit_pos,
            exit_dir,
            entry_port.up,
            width / 2.0, // Use half-width as "radius" equivalent
        );

        // Calculate corners
        let half_width = width / 2.0;
        let corners = FlatSlopeCorners {
            floor_entry_left: entry_port.position - right * half_width,
            floor_entry_right: entry_port.position + right * half_width,
            floor_exit_left: exit_pos - right * half_width,
            floor_exit_right: exit_pos + right * half_width,
            wall_entry_left: entry_port.position - right * half_width + Vec3::Y * wall_height,
            wall_entry_right: entry_port.position + right * half_width + Vec3::Y * wall_height,
            wall_exit_left: exit_pos - right * half_width + Vec3::Y * wall_height,
            wall_exit_right: exit_pos + right * half_width + Vec3::Y * wall_height,
        };

        // Compute bounds from all corners
        let all_corners = [
            corners.floor_entry_left,
            corners.floor_entry_right,
            corners.floor_exit_left,
            corners.floor_exit_right,
            corners.wall_entry_left,
            corners.wall_entry_right,
            corners.wall_exit_left,
            corners.wall_exit_right,
        ];
        let bounds = AABB::from_points(&all_corners, 0.5);

        Self {
            length,
            width,
            wall_height,
            slope_angle,
            entry: entry_port,
            exit,
            bounds,
            corners,
        }
    }

    /// Create a slope at origin going in -Z direction
    pub fn at_origin(length: f32, width: f32, wall_height: f32, slope_angle: f32) -> Self {
        let entry = Port::new(
            Vec3::new(0.0, length * slope_angle.sin(), 0.0),
            Vec3::NEG_Z,
            Vec3::Y,
            width / 2.0,
        );
        Self::new(length, width, wall_height, slope_angle, entry)
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
const OVERLAP_DISTANCE: f32 = 1.0; // Matches tube_radius

impl Segment for FlatSlope {
    fn sdf(&self, point: Vec3) -> f32 {
        // Get the slope geometry
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let slope_vec = exit_pos - entry_pos;
        let slope_length = slope_vec.length();

        // Horizontal direction (project slope onto XZ plane)
        let horizontal_dir = Vec3::new(slope_vec.x, 0.0, slope_vec.z).normalize_or_zero();
        let right = horizontal_dir.cross(Vec3::Y).normalize_or_zero();

        // If horizontal_dir is zero (vertical slope), use entry direction
        let horizontal_dir = if horizontal_dir.length_squared() < 0.01 {
            Vec3::new(self.entry.direction.x, 0.0, self.entry.direction.z).normalize_or_zero()
        } else {
            horizontal_dir
        };
        let right = if right.length_squared() < 0.01 {
            self.entry.direction.cross(Vec3::Y).normalize_or_zero()
        } else {
            right
        };

        // Project point relative to entry
        let local = point - entry_pos;

        // Distance along the slope direction (use actual 3D slope direction for t)
        let slope_dir = slope_vec.normalize();
        let along = local.dot(slope_dir);
        let t = (along / slope_length).clamp(0.0, 1.0);

        // Find the closest point on the slope centerline
        let closest_on_line = entry_pos + slope_vec * t;

        // Distance across (perpendicular to slope in XZ plane)
        let across = local.dot(right);

        // Height above the floor at this point
        let floor_y = closest_on_line.y;
        let height_above_floor = point.y - floor_y;

        // Distance to floor (positive = above floor, which is good)
        let floor_dist = -height_above_floor; // Negative when above floor

        // Distance to walls
        let half_width = self.width / 2.0;
        let left_wall_dist = -half_width - across;
        let right_wall_dist = across - half_width;
        let wall_dist = left_wall_dist.max(right_wall_dist);

        // Distance to end caps (with overlap for blending)
        let entry_dist = -along - OVERLAP_DISTANCE;
        let exit_dist = along - slope_length - OVERLAP_DISTANCE;
        let cap_dist = entry_dist.max(exit_dist);

        // Combine all constraints
        // Inside trough when: above floor (floor_dist < 0), between walls (wall_dist < 0), between caps (cap_dist < 0)
        let outside_dist = floor_dist.max(wall_dist).max(cap_dist);

        // If above wall height and within bounds, it's open space
        let above_walls = height_above_floor - self.wall_height;
        if above_walls > 0.0 && wall_dist < 0.0 && cap_dist < 0.0 {
            return -above_walls;
        }

        outside_dist
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        // Get the slope geometry
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let slope_vec = exit_pos - entry_pos;
        let slope_length = slope_vec.length();
        let slope_dir = slope_vec.normalize();

        // Project point onto slope direction
        let local = point - entry_pos;
        let along = local.dot(slope_dir);

        // Core region excludes the overlap zones at each end (OVERLAP_DISTANCE)
        // This ensures junctions get special soft collision handling
        along > OVERLAP_DISTANCE && along < slope_length - OVERLAP_DISTANCE
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
        gizmos.line(c.floor_entry_left, c.floor_entry_right, color);
        gizmos.line(c.floor_exit_left, c.floor_exit_right, color);
        gizmos.line(c.floor_entry_left, c.floor_exit_left, color);
        gizmos.line(c.floor_entry_right, c.floor_exit_right, color);

        // Draw walls
        gizmos.line(c.floor_entry_left, c.wall_entry_left, color);
        gizmos.line(c.floor_entry_right, c.wall_entry_right, color);
        gizmos.line(c.floor_exit_left, c.wall_exit_left, color);
        gizmos.line(c.floor_exit_right, c.wall_exit_right, color);

        // Draw wall tops
        gizmos.line(c.wall_entry_left, c.wall_exit_left, color);
        gizmos.line(c.wall_entry_right, c.wall_exit_right, color);

        // Entry/exit markers
        let entry_center = (c.floor_entry_left + c.floor_entry_right) / 2.0;
        let exit_center = (c.floor_exit_left + c.floor_exit_right) / 2.0;

        gizmos.sphere(Isometry3d::from_translation(entry_center), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(entry_center, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(exit_center), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(exit_center, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "FlatSlope"
    }

    fn descent(&self) -> f32 {
        self.length * self.slope_angle.sin()
    }
}
