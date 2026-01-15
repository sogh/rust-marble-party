use bevy::prelude::*;
use crate::track::{Port, PortProfile, AABB, Segment};

/// A transition segment that morphs from flat-with-walls (FlatFloor) to tube (Tube).
/// This provides smooth transitions when connecting flat segments to tube segments.
pub struct TubeAdapter {
    /// Length of the adapter segment
    pub length: f32,
    /// Entry width (FlatFloor width)
    pub entry_width: f32,
    /// Exit tube radius
    pub tube_radius: f32,
    /// Wall height at entry
    pub wall_height: f32,
    /// Entry port (FlatFloor profile)
    entry: Port,
    /// Exit port (Tube profile)
    exit: Port,
    /// Bounding box
    bounds: AABB,
    /// Direction along the segment (normalized)
    direction: Vec3,
    /// Right vector (perpendicular to direction, in XZ plane)
    right: Vec3,
    /// Up vector
    up: Vec3,
    /// Entry floor Y position
    entry_floor_y: f32,
}

impl TubeAdapter {
    pub fn new(length: f32, entry_width: f32, tube_radius: f32, entry_port: Port) -> Self {
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
        let up = Vec3::Y;

        // Calculate exit position - follow the horizontal direction
        // with a slight descent to ensure forward progress
        let slope_angle: f32 = 0.15; // Gentle descent
        let horizontal_length = length * slope_angle.cos();
        let descent = length * slope_angle.sin();

        let exit_pos = entry_port.position
            + horizontal_dir * horizontal_length
            - Vec3::Y * descent;

        // Exit direction follows the slope
        let exit_dir = (exit_pos - entry_port.position).normalize();

        // Entry floor_y comes from the incoming port
        let entry_floor_y = match &entry_port.profile {
            PortProfile::FlatFloor { floor_y, .. } => *floor_y,
            _ => entry_port.position.y,
        };

        // Create entry port with FlatFloor profile
        let entry_with_profile = Port::with_profile(
            entry_port.position,
            entry_port.direction,
            entry_port.up,
            entry_width / 2.0,
            PortProfile::flat_floor(entry_width, entry_floor_y),
        );

        // Create exit port with Tube profile
        // The tube center should be positioned so the tube floor aligns with
        // where the morphed floor ends up (at exit_floor_y - but tube is centered)
        let exit = Port::with_profile(
            exit_pos,
            exit_dir,
            up,
            tube_radius,
            PortProfile::tube(tube_radius),
        );

        // Calculate bounds - account for both trough width at entry and tube radius at exit
        let half_entry = entry_width / 2.0;
        let wall_height = 1.0; // Default wall height
        let all_points = [
            entry_port.position + right * half_entry + up * wall_height,
            entry_port.position - right * half_entry,
            entry_port.position + up * wall_height,
            exit_pos + right * tube_radius + up * tube_radius,
            exit_pos - right * tube_radius - up * tube_radius,
            exit_pos + up * tube_radius,
            exit_pos - up * tube_radius,
        ];
        let bounds = AABB::from_points(&all_points, 0.5);

        Self {
            length,
            entry_width,
            tube_radius,
            wall_height,
            entry: entry_with_profile,
            exit,
            bounds,
            direction: exit_dir,
            right,
            up,
            entry_floor_y,
        }
    }

    /// Smoothstep interpolation for smooth blending
    fn smoothstep(t: f32) -> f32 {
        let t = t.clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    }

    /// Linear interpolation
    fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }

    /// Compute SDF for the trough cross-section at blend parameter t
    /// Returns positive = inside (free space), negative = in wall
    fn trough_sdf_2d(&self, local_x: f32, local_y: f32, t: f32) -> f32 {
        // Interpolate dimensions based on t
        let half_width = Self::lerp(self.entry_width / 2.0, self.tube_radius, t);
        let wall_h = Self::lerp(self.wall_height, self.tube_radius * 2.0, t);

        // Height above the floor (floor is at y=0 in local coords)
        let height_above_floor = local_y;

        // Distance to walls
        let dist_to_left = local_x + half_width;   // Positive when inside
        let dist_to_right = half_width - local_x;  // Positive when inside

        if height_above_floor < 0.0 {
            // Below floor - return negative (in solid)
            return height_above_floor;
        }

        if height_above_floor > wall_h {
            // Above walls - open space
            if dist_to_left > 0.0 && dist_to_right > 0.0 {
                return height_above_floor - wall_h;
            }
        }

        // Inside the channel - return distance to nearest surface
        let floor_clearance = height_above_floor;
        let wall_clearance = dist_to_left.min(dist_to_right);

        if wall_clearance < 0.0 {
            // Outside a wall
            return wall_clearance;
        }

        // Inside channel - return clearance to nearest surface
        floor_clearance.min(wall_clearance)
    }

    /// Compute SDF for the tube cross-section at blend parameter t
    /// Returns positive = inside (free space), negative = in wall
    fn tube_sdf_2d(&self, local_x: f32, local_y: f32, t: f32) -> f32 {
        // Interpolate radius
        let radius = Self::lerp(self.entry_width / 2.0, self.tube_radius, t);

        // The tube center transitions from floor level to centered
        // At t=0: center is at floor level (y = 0)
        // At t=1: center is at tube center (y = 0 for a centered tube)
        // But we want the floor of the tube to align with the trough floor
        // So at t=1, the center should be at y = tube_radius
        let center_y = Self::lerp(0.0, self.tube_radius, t);

        let dy = local_y - center_y;
        let dist_from_center = (local_x * local_x + dy * dy).sqrt();

        // Tube SDF: positive inside, negative outside
        radius - dist_from_center
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
/// Experiment: one marble diameter (0.4)
const OVERLAP_ENTRY: f32 = 0.4;
const OVERLAP_EXIT: f32 = 0.4;

impl Segment for TubeAdapter {
    fn sdf(&self, point: Vec3) -> f32 {
        // Get the segment geometry
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let segment_vec = exit_pos - entry_pos;
        let segment_length = segment_vec.length();
        let segment_dir = segment_vec.normalize();

        // Project point onto segment axis
        let local = point - entry_pos;
        let along = local.dot(segment_dir);

        // Reject points outside axial bounds (with overlap)
        if along < -OVERLAP_ENTRY || along > segment_length + OVERLAP_EXIT {
            return f32::MAX;
        }

        // Calculate blend parameter t (0 = entry FlatFloor, 1 = exit Tube)
        let t = (along / segment_length).clamp(0.0, 1.0);

        // Get the closest point on the segment axis
        let clamped_along = along.clamp(0.0, segment_length);
        let closest_on_axis = entry_pos + segment_dir * clamped_along;

        // Calculate the floor Y at this point (interpolated descent)
        let floor_y_at_t = Self::lerp(self.entry_floor_y, self.exit.position.y - self.tube_radius, t);

        // Transform point to local cross-section coordinates
        // local_x = distance across (perpendicular to segment)
        // local_y = height above floor
        let to_point = point - closest_on_axis;
        let local_x = to_point.dot(self.right);
        let local_y = point.y - floor_y_at_t;

        // Compute both SDFs at this cross-section
        let trough_sdf = self.trough_sdf_2d(local_x, local_y, t);
        let tube_sdf = self.tube_sdf_2d(local_x, local_y, t);

        // Smooth blend between them
        let blend = Self::smoothstep(t);
        let blended_sdf = trough_sdf * (1.0 - blend) + tube_sdf * blend;

        // Only claim points that are reasonably close
        let max_claim = self.entry_width.max(self.tube_radius * 2.0) * 1.5;
        if blended_sdf < -max_claim {
            return f32::MAX;
        }

        blended_sdf
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let segment_vec = exit_pos - entry_pos;
        let segment_length = segment_vec.length();
        let segment_dir = segment_vec.normalize();

        let local = point - entry_pos;
        let along = local.dot(segment_dir);

        // Core region excludes the overlap zones
        along > OVERLAP_ENTRY && along < segment_length - OVERLAP_EXIT
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
        let entry_pos = self.entry.position;
        let exit_pos = self.exit.position;
        let segment_vec = exit_pos - entry_pos;
        let segment_length = segment_vec.length();
        let segment_dir = segment_vec.normalize();

        // Draw center line
        gizmos.line(entry_pos, exit_pos, color);

        // Draw cross-sections at various t values showing the morphing shape
        let t_values = [0.0, 0.25, 0.5, 0.75, 1.0];

        for &t in &t_values {
            let pos_along = entry_pos + segment_vec * t;
            let blend = Self::smoothstep(t);

            // Interpolate dimensions
            let half_width = Self::lerp(self.entry_width / 2.0, self.tube_radius, t);
            let wall_h = Self::lerp(self.wall_height, self.tube_radius * 2.0, t);
            let tube_center_y = Self::lerp(0.0, self.tube_radius, t);

            // Get floor Y at this position
            let floor_y = Self::lerp(self.entry_floor_y, self.exit.position.y - self.tube_radius, t);
            let floor_pos = Vec3::new(pos_along.x, floor_y, pos_along.z);

            // Draw blended cross-section
            if blend < 0.5 {
                // More trough-like: draw floor and walls
                let left_floor = floor_pos - self.right * half_width;
                let right_floor = floor_pos + self.right * half_width;
                let left_top = left_floor + self.up * wall_h;
                let right_top = right_floor + self.up * wall_h;

                gizmos.line(left_floor, right_floor, color);
                gizmos.line(left_floor, left_top, color);
                gizmos.line(right_floor, right_top, color);
            } else {
                // More tube-like: draw circle
                let tube_center = floor_pos + self.up * tube_center_y;
                let radius = half_width;
                let segments = 16;

                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = tube_center + (self.right * angle1.cos() + self.up * angle1.sin()) * radius;
                    let p2 = tube_center + (self.right * angle2.cos() + self.up * angle2.sin()) * radius;

                    gizmos.line(p1, p2, color);
                }
            }
        }

        // Draw connecting lines from entry trough corners to exit tube
        let entry_left = entry_pos - self.right * (self.entry_width / 2.0);
        let entry_right = entry_pos + self.right * (self.entry_width / 2.0);
        let exit_center = self.exit.position;

        // Lines showing the transition
        gizmos.line(entry_left, exit_center - self.right * self.tube_radius, color.with_alpha(0.5));
        gizmos.line(entry_right, exit_center + self.right * self.tube_radius, color.with_alpha(0.5));

        // Entry/exit markers
        gizmos.sphere(Isometry3d::from_translation(entry_pos), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(entry_pos, self.entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        gizmos.sphere(Isometry3d::from_translation(exit_pos), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(exit_pos, self.exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "TubeAdapter"
    }

    fn descent(&self) -> f32 {
        self.entry.position.y - self.exit.position.y
    }
}
