use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A tube that curves in a horizontal arc
pub struct CurvedTube {
    /// Arc angle in radians (positive = left, negative = right)
    pub arc_angle: f32,
    /// Radius of the arc (distance from center of curve to tube center)
    pub arc_radius: f32,
    /// Radius of the tube itself
    pub tube_radius: f32,
    /// Number of segments for SDF approximation
    pub segments: usize,
    /// Cached spine points
    spine: Vec<Vec3>,
    /// Entry port
    entry: Port,
    /// Exit port
    exit: Port,
    /// Bounding box
    bounds: AABB,
}

impl CurvedTube {
    pub fn new(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        // Many segments for smooth gradient transitions at physics timestep (64 Hz)
        // At ~5 m/s marble speed, each physics step moves ~0.08m
        // We want segment boundaries to be much smaller than this
        // 512 base gives ~128 segments for 90° turn = ~0.04m per segment at radius 5
        let segments = ((arc_angle.abs() / std::f32::consts::TAU) * 512.0).max(64.0) as usize;

        let mut tube = Self {
            arc_angle,
            arc_radius,
            tube_radius,
            segments,
            spine: Vec::new(),
            entry: entry_port,
            exit: Port::default(),
            bounds: AABB::default(),
        };

        tube.generate_spine();
        tube
    }

    /// Create a left-curving tube at origin
    pub fn left(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        Self::new(arc_angle.abs(), arc_radius, tube_radius, entry_port)
    }

    /// Create a right-curving tube at origin
    pub fn right(arc_angle: f32, arc_radius: f32, tube_radius: f32, entry_port: Port) -> Self {
        Self::new(-arc_angle.abs(), arc_radius, tube_radius, entry_port)
    }

    fn generate_spine(&mut self) {
        self.spine.clear();

        // Calculate the descent rate (Y change per unit distance)
        let horizontal_speed = Vec3::new(self.entry.direction.x, 0.0, self.entry.direction.z).length();
        let descent_rate = if horizontal_speed > 0.01 {
            self.entry.direction.y / horizontal_speed
        } else {
            0.0
        };

        // === LEAD-IN: Start with a straight section matching the entry direction ===
        // This ensures seamless transition from the previous segment
        // Use a longer lead-in (3 units) to give marble time to establish position
        let lead_in_length = 3.0;
        let lead_in_end = self.entry.position + self.entry.direction * lead_in_length;

        self.spine.push(self.entry.position);
        self.spine.push(lead_in_end);

        // === ARC: Generate the curved section starting from lead-in end ===

        // Get the entry direction projected onto horizontal plane
        let entry_dir_horizontal = Vec3::new(self.entry.direction.x, 0.0, self.entry.direction.z).normalize_or_zero();
        let entry_dir_horizontal = if entry_dir_horizontal.length() < 0.1 {
            Vec3::NEG_Z
        } else {
            entry_dir_horizontal
        };

        // Calculate perpendicular direction (to the right of travel)
        let right = entry_dir_horizontal.cross(Vec3::Y).normalize();

        // Calculate the center of the arc (on horizontal plane) from the lead-in end
        let lead_in_end_xz = Vec2::new(lead_in_end.x, lead_in_end.z);
        let curve_center_xz = if self.arc_angle > 0.0 {
            lead_in_end_xz - Vec2::new(right.x, right.z) * self.arc_radius
        } else {
            lead_in_end_xz + Vec2::new(right.x, right.z) * self.arc_radius
        };

        // Starting angle from center to lead-in end
        let to_start = lead_in_end_xz - curve_center_xz;
        let start_angle = to_start.y.atan2(to_start.x);

        // Total arc length for descent calculation
        let arc_length = self.arc_radius * self.arc_angle.abs();

        // Generate arc points (skip first since lead-in end is already added)
        for i in 1..=self.segments {
            let t = i as f32 / self.segments as f32;
            let angle = start_angle - self.arc_angle * t;

            let x = curve_center_xz.x + self.arc_radius * angle.cos();
            let z = curve_center_xz.y + self.arc_radius * angle.sin();

            // Continue descent along the arc
            let distance_along = lead_in_length + arc_length * t;
            let y = self.entry.position.y + descent_rate * distance_along;

            self.spine.push(Vec3::new(x, y, z));
        }

        // Compute bounds
        self.bounds = AABB::from_points(&self.spine, self.tube_radius + 0.5);

        // Keep original entry port - don't modify it so it matches the previous segment's exit
        // Just ensure spine[0] is at the entry position (it should be by construction)

        // Compute exit port from the last spine segment
        if self.spine.len() >= 2 {
            let last = self.spine[self.spine.len() - 1];
            let prev = self.spine[self.spine.len() - 2];
            let exit_dir = (last - prev).normalize();

            self.exit = Port::new(last, exit_dir, self.entry.up, self.tube_radius);
        }
    }

    pub fn spine(&self) -> &[Vec3] {
        &self.spine
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
/// Experiment: one marble diameter (0.4)
const OVERLAP_DISTANCE: f32 = 0.4;

impl Segment for CurvedTube {
    fn sdf(&self, point: Vec3) -> f32 {
        if self.spine.len() < 2 {
            return f32::MAX;
        }

        // Reject points far behind the entry to avoid interference with previous segment
        // Use a generous margin (3.0) to allow smooth transition from the previous segment
        let to_point_entry = point - self.entry.position;
        let along_entry = to_point_entry.dot(self.entry.direction);
        if along_entry < -3.0 {
            return f32::MAX;
        }

        // Find distance to closest point on the spine polyline
        // Track which segment and where on it the closest point is
        let mut min_dist_sq = f32::MAX;
        let mut closest_segment_idx = 0;
        let mut closest_t = 0.0f32;

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let ab = b - a;
            let ab_len_sq = ab.dot(ab);

            if ab_len_sq < 0.0001 {
                let dist_sq = (point - a).length_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    closest_segment_idx = i;
                    closest_t = 0.0;
                }
                continue;
            }

            let ap = point - a;
            // Clamp t to [0,1] - this is critical!
            let t = (ap.dot(ab) / ab_len_sq).clamp(0.0, 1.0);
            let closest_point = a + ab * t;
            let dist_sq = (point - closest_point).length_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_segment_idx = i;
                closest_t = t;
            }
        }

        // Reject points that project past the end of the spine
        // If closest is on the last segment with t=1.0, point is at or past the exit
        let last_segment_idx = self.spine.len() - 2;
        if closest_segment_idx == last_segment_idx && closest_t > 0.99 {
            // Check if point is actually past the exit (not just at the exit)
            let to_point_exit = point - self.exit.position;
            let along_exit = to_point_exit.dot(self.exit.direction);
            if along_exit > 0.1 {
                return f32::MAX;
            }
        }

        // SDF = distance to spine minus tube radius
        // Positive = inside tube (free space), Negative = in wall
        let dist_to_spine = min_dist_sq.sqrt();
        -(dist_to_spine - self.tube_radius)
    }

    fn is_in_core_region(&self, point: Vec3) -> bool {
        if self.spine.len() < 2 {
            return false;
        }

        // Calculate total spine length to determine threshold
        let mut total_length = 0.0;
        for i in 0..self.spine.len() - 1 {
            total_length += (self.spine[i + 1] - self.spine[i]).length();
        }

        // Find the cumulative distance along the spine to the closest point
        let mut min_dist_sq = f32::MAX;
        let mut closest_along = 0.0;
        let mut cumulative_length = 0.0;

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let ab = b - a;
            let segment_length = ab.length();
            let ap = point - a;
            let t = (ap.dot(ab) / ab.dot(ab)).clamp(0.0, 1.0);
            let closest = a + ab * t;
            let dist_sq = (point - closest).length_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_along = cumulative_length + t * segment_length;
            }

            cumulative_length += segment_length;
        }

        // Core region excludes the overlap zones at each end (OVERLAP_DISTANCE)
        // This ensures junctions get special soft collision handling
        closest_along > OVERLAP_DISTANCE && closest_along < total_length - OVERLAP_DISTANCE
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
        // Draw spine with small spheres at each point
        for i in 0..self.spine.len() - 1 {
            gizmos.line(self.spine[i], self.spine[i + 1], color);
        }
        // Mark spine points
        for (i, &point) in self.spine.iter().enumerate() {
            let point_color = if i == 0 {
                Color::srgb(0.0, 1.0, 1.0) // Cyan for first
            } else if i == self.spine.len() - 1 {
                Color::srgb(1.0, 0.0, 1.0) // Magenta for last
            } else {
                Color::srgb(1.0, 1.0, 1.0) // White for middle
            };
            gizmos.sphere(Isometry3d::from_translation(point), 0.05, point_color);
        }

        // Draw extended capsule endpoints (overlap regions)
        if !self.spine.is_empty() {
            let extended_entry = self.entry.position - self.entry.direction * OVERLAP_DISTANCE;
            let extended_exit = self.exit.position + self.exit.direction * OVERLAP_DISTANCE;

            // Draw extended lines in yellow
            gizmos.line(extended_entry, self.entry.position, Color::srgb(1.0, 1.0, 0.0));
            gizmos.line(self.exit.position, extended_exit, Color::srgb(1.0, 1.0, 0.0));

            // Draw spheres at extended endpoints
            gizmos.sphere(Isometry3d::from_translation(extended_entry), self.tube_radius, Color::srgb(1.0, 1.0, 0.0));
            gizmos.sphere(Isometry3d::from_translation(extended_exit), self.tube_radius, Color::srgb(1.0, 0.5, 0.0));
        }

        // Draw cross-sections at intervals
        for (i, &point) in self.spine.iter().enumerate() {
            if i % 4 == 0 && i + 1 < self.spine.len() {
                let dir = (self.spine[i + 1] - point).normalize_or_zero();
                let right = dir.cross(Vec3::Y).normalize_or_zero();
                let up = right.cross(dir).normalize_or_zero();

                let segments = 12;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = point + (right * angle1.cos() + up * angle1.sin()) * self.tube_radius;
                    let p2 = point + (right * angle2.cos() + up * angle2.sin()) * self.tube_radius;

                    gizmos.line(p1, p2, color.with_alpha(0.5));
                }
            }
        }

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
            gizmos.ray(port.position, port.direction * 0.5, port_color);
        }
    }

    fn type_name(&self) -> &'static str {
        "CurvedTube"
    }

    /// Analytic gradient for smooth physics
    /// Points radially inward from the tube surface (since SDF is negated)
    fn sdf_gradient(&self, point: Vec3) -> Vec3 {
        if self.spine.len() < 2 {
            return Vec3::Y;
        }

        // Find the closest point on the spine (same algorithm as SDF)
        let mut min_dist_sq = f32::MAX;
        let mut closest_point = self.spine[0];

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let ab = b - a;
            let ab_len_sq = ab.dot(ab);

            if ab_len_sq < 0.0001 {
                let dist_sq = (point - a).length_squared();
                if dist_sq < min_dist_sq {
                    min_dist_sq = dist_sq;
                    closest_point = a;
                }
                continue;
            }

            let ap = point - a;
            let t = (ap.dot(ab) / ab_len_sq).clamp(0.0, 1.0);
            let cp = a + ab * t;
            let dist_sq = (point - cp).length_squared();

            if dist_sq < min_dist_sq {
                min_dist_sq = dist_sq;
                closest_point = cp;
            }
        }

        // Gradient points from marble toward spine center (inward for SDF convention)
        // Our SDF is negative when outside, so gradient should point inward
        let radial_dir = closest_point - point;
        if radial_dir.length_squared() < 0.0001 {
            // Point is on the spine - use up vector as fallback
            return Vec3::Y;
        }

        radial_dir.normalize()
    }
}
