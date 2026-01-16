use bevy::prelude::*;
use crate::track::{Port, AABB, Segment};

/// A helical tube segment that spirals downward
pub struct SpiralTube {
    /// Number of complete loops
    pub loops: f32,
    /// Total vertical descent
    pub descent: f32,
    /// Radius of the helix (distance from center)
    pub helix_radius: f32,
    /// Radius of the tube itself
    pub tube_radius: f32,
    /// Amplitude of small bumps along the path
    pub bump_amplitude: f32,
    /// Number of points to generate for the spine
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

impl SpiralTube {
    pub fn new(
        loops: f32,
        descent: f32,
        helix_radius: f32,
        tube_radius: f32,
        entry_port: Port,
    ) -> Self {
        let segments = (loops * 50.0) as usize + 1;
        let bump_amplitude = 0.5;

        let mut tube = Self {
            loops,
            descent,
            helix_radius,
            tube_radius,
            bump_amplitude,
            segments,
            spine: Vec::new(),
            entry: entry_port.clone(),
            exit: Port::default(),
            bounds: AABB::default(),
        };

        tube.generate_spine();
        tube
    }

    /// Create with default starting position at origin
    pub fn at_origin(loops: f32, descent: f32, helix_radius: f32, tube_radius: f32) -> Self {
        let start_pos = Vec3::new(helix_radius, descent + 2.0, 0.0);
        let entry = Port::new(
            start_pos,
            Vec3::new(0.0, -0.3, 1.0).normalize(), // Slightly downward, forward
            Vec3::Y,
            tube_radius,
        );
        Self::new(loops, descent, helix_radius, tube_radius, entry)
    }

    fn generate_spine(&mut self) {
        self.spine.clear();

        // Start from the entry port position
        let start_pos = self.entry.position;
        let start_dir = self.entry.direction;

        // Get horizontal entry direction
        let forward = Vec3::new(start_dir.x, 0.0, start_dir.z).normalize_or_zero();
        let forward = if forward.length_squared() < 0.01 { Vec3::NEG_Z } else { forward };

        // No lead-in - spiral starts immediately from entry for better momentum
        self.spine.push(start_pos);

        // For a vertical helix (spiral staircase), the axis is vertical
        // The helix center is offset from entry so that point is ON the circle
        // and the tangent matches the forward direction

        // Tangent of circle at angle θ is perpendicular to radius
        // If we want tangent = forward, radius must be perpendicular to forward
        let right = forward.cross(Vec3::Y).normalize();

        // Center is to the left of entry (so tangent points forward)
        let center = Vec3::new(
            start_pos.x - right.x * self.helix_radius,
            0.0, // Center Y doesn't matter, we track Y separately
            start_pos.z - right.z * self.helix_radius,
        );

        // Starting angle: from center to entry point
        let to_spiral_start = Vec2::new(start_pos.x - center.x, start_pos.z - center.z);
        let start_angle = to_spiral_start.y.atan2(to_spiral_start.x);

        // Generate spiral points
        for i in 1..=self.segments {
            let t = i as f32 / self.segments as f32;
            // Clockwise rotation (so tangent at entry points forward)
            let angle = start_angle - t * std::f32::consts::TAU * self.loops;

            // Position on the circle in XZ plane
            let x = center.x + self.helix_radius * angle.cos();
            let z = center.z + self.helix_radius * angle.sin();

            // Y descends smoothly from entry height
            let y = start_pos.y - self.descent * t;

            self.spine.push(Vec3::new(x, y, z));
        }

        // === COMPUTE EXIT PORT ===
        // Use direction from last two spine points, but ensure a minimum descent angle
        // to maintain momentum through consecutive spirals
        if self.spine.len() >= 2 {
            let last = self.spine[self.spine.len() - 1];
            let second_last = self.spine[self.spine.len() - 2];
            let spine_dir = (last - second_last).normalize();

            // Ensure at least 20 degree descent (sin(20°) ≈ 0.34)
            let min_descent = -0.34;
            let exit_dir = if spine_dir.y > min_descent {
                // Blend with a steeper descent
                let horizontal = Vec3::new(spine_dir.x, 0.0, spine_dir.z).normalize();
                Vec3::new(horizontal.x * 0.94, min_descent, horizontal.z * 0.94).normalize()
            } else {
                spine_dir
            };

            self.exit = Port::new(last, exit_dir, Vec3::Y, self.tube_radius);
        }

        // Compute bounds - include the exit position for proper AABB
        let mut bound_points = self.spine.clone();
        bound_points.push(self.exit.position);
        self.bounds = AABB::from_points(&bound_points, self.tube_radius + 0.5);
    }

    /// Get the spine points for external use
    pub fn spine(&self) -> &[Vec3] {
        &self.spine
    }
}

/// How far to extend the SDF past segment endpoints for smooth blending
/// Larger overlap (1.0) to ensure smooth handoff between spiral and next segment
const OVERLAP_DISTANCE: f32 = 1.0;

impl Segment for SpiralTube {
    fn sdf(&self, point: Vec3) -> f32 {
        if self.spine.len() < 2 {
            return f32::MAX;
        }

        // Reject points far behind the entry to avoid interference with previous segment
        let to_point_entry = point - self.entry.position;
        let along_entry = to_point_entry.dot(self.entry.direction);
        if along_entry < -OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Reject points far past the exit to avoid interference with next segment
        let to_point_exit = point - self.exit.position;
        let along_exit = to_point_exit.dot(self.exit.direction);
        if along_exit > OVERLAP_DISTANCE {
            return f32::MAX;
        }

        // Find distance to closest point on the FINITE spine polyline
        // (Same approach as CurvedTube - don't use infinite cylinders!)
        let mut min_dist_sq = f32::MAX;

        for i in 0..self.spine.len() - 1 {
            let a = self.spine[i];
            let b = self.spine[i + 1];
            let ab = b - a;
            let ab_len_sq = ab.dot(ab);

            if ab_len_sq < 0.0001 {
                let dist_sq = (point - a).length_squared();
                min_dist_sq = min_dist_sq.min(dist_sq);
                continue;
            }

            let ap = point - a;
            // Clamp t to [0,1] so each segment is FINITE
            let t = (ap.dot(ab) / ab_len_sq).clamp(0.0, 1.0);
            let closest_point = a + ab * t;
            let dist_sq = (point - closest_point).length_squared();

            min_dist_sq = min_dist_sq.min(dist_sq);
        }

        // SDF = tube_radius - distance_to_spine
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
        // Draw spine
        for i in 0..self.spine.len() - 1 {
            gizmos.line(self.spine[i], self.spine[i + 1], color);
        }

        // Draw transition zone from last spiral point to exit (handled by straight tube)
        if !self.spine.is_empty() {
            let extended_entry = self.spine[0] - self.entry.direction * OVERLAP_DISTANCE;
            let last_idx = self.spine.len() - 1;

            // Draw extended entry line in yellow
            gizmos.line(extended_entry, self.spine[0], Color::srgb(1.0, 1.0, 0.0));
            gizmos.sphere(Isometry3d::from_translation(extended_entry), self.tube_radius, Color::srgb(1.0, 1.0, 0.0));

            // Draw transition zone to exit in orange (this region is handled by straight tube)
            gizmos.line(self.spine[last_idx], self.exit.position, Color::srgb(1.0, 0.5, 0.0));
            gizmos.sphere(Isometry3d::from_translation(self.exit.position), self.tube_radius, Color::srgb(1.0, 0.5, 0.0));
        }

        // Draw cross-section circles every few points
        for (i, &point) in self.spine.iter().enumerate() {
            if i % 5 == 0 && i + 1 < self.spine.len() {
                let dir = (self.spine[i + 1] - point).normalize_or_zero();
                let right = dir.cross(Vec3::Y).normalize_or_zero();
                let up = right.cross(dir).normalize_or_zero();

                // Draw circle
                let segments = 16;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = point + (right * angle1.cos() + up * angle1.sin()) * self.tube_radius;
                    let p2 = point + (right * angle2.cos() + up * angle2.sin()) * self.tube_radius;

                    gizmos.line(p1, p2, color.with_alpha(0.5));
                }
            }
        }

        // Draw entry/exit ports
        let entry = self.entry_port();
        gizmos.sphere(Isometry3d::from_translation(entry.position), 0.15, Color::srgb(0.0, 1.0, 0.0));
        gizmos.ray(entry.position, entry.direction * 0.5, Color::srgb(0.0, 1.0, 0.0));

        let exit = self.primary_exit_port();
        gizmos.sphere(Isometry3d::from_translation(exit.position), 0.15, Color::srgb(1.0, 0.0, 0.0));
        gizmos.ray(exit.position, exit.direction * 0.5, Color::srgb(1.0, 0.0, 0.0));
    }

    fn type_name(&self) -> &'static str {
        "SpiralTube"
    }

    fn descent(&self) -> f32 {
        self.descent
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
