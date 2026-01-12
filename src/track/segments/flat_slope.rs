use bevy::prelude::*;
use crate::track::{Port, PortProfile, AABB, Segment};

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
        // Get horizontal direction (project entry direction onto XZ plane)
        let entry_horizontal = Vec3::new(
            entry_port.direction.x,
            0.0,
            entry_port.direction.z,
        ).normalize_or_zero();

        // If entry is purely vertical, use a default direction
        let horizontal_dir = if entry_horizontal.length_squared() < 0.01 {
            Vec3::NEG_Z
        } else {
            entry_horizontal
        };

        let right = horizontal_dir.cross(Vec3::Y).normalize();

        // Calculate exit position using ABSOLUTE slope angle from horizontal
        // This avoids accumulating slope from the entry direction
        let horizontal_length = length * slope_angle.cos();
        let descent = length * slope_angle.sin();

        let exit_pos = entry_port.position
            + horizontal_dir * horizontal_length
            - Vec3::Y * descent;

        // Exit direction follows the slope (not the entry direction)
        let exit_dir = (exit_pos - entry_port.position).normalize();

        // Create ports with FlatFloor profiles
        // The floor_y is the actual Y coordinate of the floor at each end
        let entry_floor_y = entry_port.position.y;
        let exit_floor_y = exit_pos.y;

        let entry_with_profile = Port::with_profile(
            entry_port.position,
            entry_port.direction,
            entry_port.up,
            width / 2.0,
            PortProfile::flat_floor(width, entry_floor_y),
        );

        let exit = Port::with_profile(
            exit_pos,
            exit_dir,
            entry_port.up,
            width / 2.0, // Use half-width as "radius" equivalent
            PortProfile::flat_floor(width, exit_floor_y),
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
            entry: entry_with_profile,
            exit,
            bounds,
            corners,
        }
    }

    /// Create a slope at origin going in -Z direction
    pub fn at_origin(length: f32, width: f32, wall_height: f32, slope_angle: f32) -> Self {
        let entry_y = length * slope_angle.sin();
        let entry = Port::with_profile(
            Vec3::new(0.0, entry_y, 0.0),
            Vec3::NEG_Z,
            Vec3::Y,
            width / 2.0,
            PortProfile::flat_floor(width, entry_y),
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
        let slope_dir = slope_vec.normalize();

        // Project point relative to entry
        let local = point - entry_pos;
        let along = local.dot(slope_dir);

        // Reject points outside the segment bounds
        // For FlatSlope, NO overlap at entry - marble drops onto the trough
        // Small overlap at exit for transition to next segment
        if along < 0.0 || along > slope_length + OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Horizontal direction (project slope onto XZ plane)
        let horizontal_dir = Vec3::new(slope_vec.x, 0.0, slope_vec.z).normalize_or_zero();
        let right = if horizontal_dir.length_squared() > 0.01 {
            horizontal_dir.cross(Vec3::Y).normalize_or_zero()
        } else {
            self.entry.direction.cross(Vec3::Y).normalize_or_zero()
        };

        // Clamp t to [0,1] for closest point calculation
        let t = (along / slope_length).clamp(0.0, 1.0);
        let closest_on_line = entry_pos + slope_vec * t;

        // Distance across (perpendicular to slope in XZ plane)
        let across = local.dot(right);

        // Height above the floor at this point
        let floor_y = closest_on_line.y;
        let height_above_floor = point.y - floor_y;

        // Distance to walls
        let half_width = self.width / 2.0;
        let dist_to_left_wall = across + half_width;   // Positive when inside
        let dist_to_right_wall = half_width - across;  // Positive when inside

        // For a trough/channel, marble is "inside" when:
        // - Above the floor (height_above_floor > 0)
        // - Between the walls (both wall distances > 0)
        // - Within longitudinal bounds (checked above)

        // SDF convention: positive = inside (free space), negative = in wall
        // The SDF is the minimum distance to any surface (floor or walls)
        // Take min of floor distance and wall distances

        if height_above_floor < 0.0 {
            // Below floor - return negative (in solid)
            return height_above_floor;
        }

        if height_above_floor > self.wall_height {
            // Above walls - open space, return clearance above walls
            if dist_to_left_wall > 0.0 && dist_to_right_wall > 0.0 {
                return height_above_floor - self.wall_height;
            }
        }

        // Inside the channel - return distance to nearest surface
        // This is the minimum clearance to floor or walls
        let floor_clearance = height_above_floor;
        let wall_clearance = dist_to_left_wall.min(dist_to_right_wall);

        // Return the minimum clearance (but negative if we're outside walls)
        if wall_clearance < 0.0 {
            // Outside a wall
            return wall_clearance;
        }

        // Inside channel - return clearance to nearest surface
        floor_clearance.min(wall_clearance)
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
