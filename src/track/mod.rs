use bevy::prelude::*;

pub mod segments;
pub mod generator;

// Re-export segment types
pub use segments::*;
pub use generator::{TrackGenerator, GeneratorConfig, ProceduralTrack};

// ============================================================================
// PORT - Connection point between segments
// ============================================================================

/// A connection point on a track segment
#[derive(Clone, Debug)]
pub struct Port {
    /// World position of the connection point
    pub position: Vec3,
    /// Direction the track continues (normalized, points "out" of segment)
    pub direction: Vec3,
    /// Up vector for orientation
    pub up: Vec3,
    /// Channel radius at this point
    pub radius: f32,
}

impl Port {
    pub fn new(position: Vec3, direction: Vec3, up: Vec3, radius: f32) -> Self {
        Self {
            position,
            direction: direction.normalize(),
            up: up.normalize(),
            radius,
        }
    }

    /// Check if another port can connect to this one
    /// Directions should be roughly opposite, positions should match
    pub fn can_connect(&self, other: &Port) -> bool {
        let position_match = (self.position - other.position).length() < 0.1;
        let direction_match = self.direction.dot(other.direction) < -0.5; // ~120° tolerance
        let radius_match = (self.radius - other.radius).abs() < 0.1;

        position_match && direction_match && radius_match
    }

    /// Get the right vector (perpendicular to direction and up)
    pub fn right(&self) -> Vec3 {
        self.direction.cross(self.up).normalize()
    }
}

impl Default for Port {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            direction: Vec3::NEG_Z,
            up: Vec3::Y,
            radius: 1.0,
        }
    }
}

// ============================================================================
// AABB - Axis-Aligned Bounding Box
// ============================================================================

/// Axis-aligned bounding box for spatial optimization
#[derive(Clone, Debug)]
pub struct AABB {
    pub min: Vec3,
    pub max: Vec3,
}

impl AABB {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Create AABB from a set of points with padding
    pub fn from_points(points: &[Vec3], padding: f32) -> Self {
        if points.is_empty() {
            return Self::new(Vec3::ZERO, Vec3::ZERO);
        }

        let mut min = points[0];
        let mut max = points[0];

        for p in points {
            min = min.min(*p);
            max = max.max(*p);
        }

        Self {
            min: min - Vec3::splat(padding),
            max: max + Vec3::splat(padding),
        }
    }

    /// Check if point is inside the bounding box
    pub fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Expand the bounding box by a margin
    pub fn expanded(&self, margin: f32) -> Self {
        Self {
            min: self.min - Vec3::splat(margin),
            max: self.max + Vec3::splat(margin),
        }
    }

    /// Merge two AABBs
    pub fn union(&self, other: &AABB) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Get the center of the AABB
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Get the size of the AABB
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }
}

impl Default for AABB {
    fn default() -> Self {
        Self {
            min: Vec3::splat(f32::MAX),
            max: Vec3::splat(f32::MIN),
        }
    }
}

// ============================================================================
// SEGMENT TRAIT - Interface for all track segments
// ============================================================================

/// Trait implemented by all track segment types
pub trait Segment: Send + Sync {
    /// Compute signed distance from point to this segment's surface
    /// Negative = inside (free space), Positive = outside/in wall
    fn sdf(&self, point: Vec3) -> f32;

    /// Check if point is in the core region (not in end cap overlap zones)
    /// Returns true if collisions should be applied at this point
    fn is_in_core_region(&self, point: Vec3) -> bool;

    /// Get the entry port for this segment
    fn entry_port(&self) -> Port;

    /// Get all exit ports (usually 1, but forks have 2)
    fn exit_ports(&self) -> Vec<Port>;

    /// Get the primary exit port (first one)
    fn primary_exit_port(&self) -> Port {
        self.exit_ports().into_iter().next().unwrap_or_default()
    }

    /// Get the axis-aligned bounding box
    fn bounds(&self) -> AABB;

    /// Draw debug visualization using gizmos
    fn draw_debug_gizmos(&self, gizmos: &mut Gizmos, color: Color);

    /// Get the segment type name for debug display
    fn type_name(&self) -> &'static str;

    /// Get total vertical descent of this segment
    fn descent(&self) -> f32 {
        let entry = self.entry_port();
        let exit = self.primary_exit_port();
        entry.position.y - exit.position.y
    }
}

// ============================================================================
// TRACK RESOURCE - Container for multiple segments
// ============================================================================

/// Resource containing the complete track made of segments
#[derive(Resource)]
pub struct Track {
    segments: Vec<Box<dyn Segment>>,
    bounds: AABB,
    smooth_k: f32,
}

impl Track {
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
            bounds: AABB::default(),
            smooth_k: 0.0, // No blending - segments connect seamlessly via overlap
        }
    }

    /// Add a segment to the track
    pub fn add_segment(&mut self, segment: Box<dyn Segment>) {
        let seg_bounds = segment.bounds();
        if self.segments.is_empty() {
            self.bounds = seg_bounds;
        } else {
            self.bounds = self.bounds.union(&seg_bounds);
        }
        self.segments.push(segment);
    }

    /// Remove the last segment
    pub fn remove_last(&mut self) -> Option<Box<dyn Segment>> {
        let removed = self.segments.pop();
        self.recompute_bounds();
        removed
    }

    /// Recompute bounds from all segments
    fn recompute_bounds(&mut self) {
        self.bounds = AABB::default();
        for seg in &self.segments {
            if self.bounds.min.x == f32::MAX {
                self.bounds = seg.bounds();
            } else {
                self.bounds = self.bounds.union(&seg.bounds());
            }
        }
    }

    /// Get the exit port of the last segment (for connecting new segments)
    pub fn last_port(&self) -> Option<Port> {
        self.segments.last().map(|s| s.primary_exit_port())
    }

    /// Get the entry port of the first segment
    pub fn first_port(&self) -> Option<Port> {
        self.segments.first().map(|s| s.entry_port())
    }

    /// Compute SDF with bounding box optimization
    pub fn sdf(&self, point: Vec3) -> f32 {
        if self.segments.is_empty() {
            return f32::MAX;
        }

        let mut min_dist = f32::MAX;

        for segment in &self.segments {
            // Bounding box optimization: skip if clearly far away
            let bounds = segment.bounds();
            let expanded = bounds.expanded(min_dist.abs());
            if !expanded.contains(point) {
                continue;
            }

            let dist = segment.sdf(point);
            min_dist = smooth_min(min_dist, dist, self.smooth_k);
        }

        min_dist
    }

    /// Fast SDF check using only nearby segments (for known marble position)
    /// Much faster than full sdf() when marble's current segment is known
    pub fn sdf_near_segment(&self, point: Vec3, segment_hint: i32) -> f32 {
        if self.segments.is_empty() {
            return f32::MAX;
        }

        let hint = segment_hint.max(0) as usize;
        let mut min_dist = f32::MAX;

        // Only check current segment and neighbors
        let start = hint.saturating_sub(1);
        let end = (hint + 2).min(self.segments.len());

        for i in start..end {
            let dist = self.segments[i].sdf(point);
            min_dist = min_dist.min(dist);
        }

        min_dist
    }

    /// Fast segment detection using locality (check nearby segments first)
    pub fn find_segment_near(&self, point: Vec3, segment_hint: i32) -> i32 {
        if self.segments.is_empty() {
            return -1;
        }

        let hint = segment_hint.max(0) as usize;

        // Check current and next segment only (marbles move forward)
        for i in hint..(hint + 2).min(self.segments.len()) {
            let bounds = self.segments[i].bounds();
            if bounds.expanded(1.0).contains(point) {
                let dist = self.segments[i].sdf(point);
                if dist > 0.0 {
                    return i as i32;
                }
            }
        }

        // Fallback: current segment if still valid
        if hint < self.segments.len() {
            let dist = self.segments[hint].sdf(point);
            if dist > 0.0 {
                return hint as i32;
            }
        }

        segment_hint // Keep current if nothing better found
    }

    /// Compute gradient (normal) from the nearest segment
    /// Uses the nearest segment's SDF directly to avoid blending artifacts at junctions
    pub fn sdf_gradient(&self, point: Vec3) -> Vec3 {
        const EPS: f32 = 0.01;

        // Find the nearest segment
        let mut nearest_idx = 0;
        let mut nearest_dist = f32::MAX;

        for (i, segment) in self.segments.iter().enumerate() {
            let bounds = segment.bounds();
            if !bounds.expanded(2.0).contains(point) {
                continue;
            }

            let dist = segment.sdf(point).abs();
            if dist < nearest_dist {
                nearest_dist = dist;
                nearest_idx = i;
            }
        }

        if self.segments.is_empty() {
            return Vec3::Y;
        }

        // Compute gradient from the nearest segment only
        let segment = &self.segments[nearest_idx];
        let dx = segment.sdf(point + Vec3::X * EPS) - segment.sdf(point - Vec3::X * EPS);
        let dy = segment.sdf(point + Vec3::Y * EPS) - segment.sdf(point - Vec3::Y * EPS);
        let dz = segment.sdf(point + Vec3::Z * EPS) - segment.sdf(point - Vec3::Z * EPS);

        Vec3::new(dx, dy, dz).normalize_or_zero()
    }

    /// Compute gradient from a specific segment
    /// Used to maintain consistent collision normals through junctions
    pub fn segment_gradient(&self, segment_idx: usize, point: Vec3) -> Vec3 {
        const EPS: f32 = 0.01;

        if segment_idx >= self.segments.len() {
            return self.sdf_gradient(point);
        }

        let segment = &self.segments[segment_idx];
        let dx = segment.sdf(point + Vec3::X * EPS) - segment.sdf(point - Vec3::X * EPS);
        let dy = segment.sdf(point + Vec3::Y * EPS) - segment.sdf(point - Vec3::Y * EPS);
        let dz = segment.sdf(point + Vec3::Z * EPS) - segment.sdf(point - Vec3::Z * EPS);

        Vec3::new(dx, dy, dz).normalize_or_zero()
    }

    /// Get number of segments
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Get all segments for iteration
    pub fn segments(&self) -> &[Box<dyn Segment>] {
        &self.segments
    }

    /// Clear all segments
    pub fn clear(&mut self) {
        self.segments.clear();
        self.bounds = AABB::default();
    }

    /// Check if point is in any segment's core region (not in end caps)
    pub fn is_in_any_core_region(&self, point: Vec3) -> bool {
        for segment in &self.segments {
            if segment.is_in_core_region(point) {
                return true;
            }
        }
        false
    }

    /// Find which segment contains a point (returns segment index, or -1 if none)
    /// A point is "in" a segment if its SDF value is positive (inside the surface)
    pub fn find_segment(&self, point: Vec3) -> i32 {
        let mut best_idx: i32 = -1;
        let mut best_dist = f32::NEG_INFINITY;

        for (i, segment) in self.segments.iter().enumerate() {
            let bounds = segment.bounds();
            if !bounds.expanded(1.0).contains(point) {
                continue;
            }

            let dist = segment.sdf(point);
            // Positive SDF means inside the surface
            if dist > best_dist {
                best_dist = dist;
                best_idx = i as i32;
            }
        }

        // Only return a segment if we're actually inside it
        if best_dist > 0.0 {
            best_idx
        } else {
            -1
        }
    }
}

impl Default for Track {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/// Smooth minimum for blending SDFs
pub fn smooth_min(a: f32, b: f32, k: f32) -> f32 {
    if k <= 0.0 {
        return a.min(b);
    }
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b * (1.0 - h) + a * h - k * h * (1.0 - h)
}

/// Capsule SDF helper (has hemispherical end caps)
pub fn capsule_sdf(point: Vec3, a: Vec3, b: Vec3, radius: f32) -> f32 {
    let pa = point - a;
    let ba = b - a;
    let h = (pa.dot(ba) / ba.dot(ba)).clamp(0.0, 1.0);
    (pa - ba * h).length() - radius
}

/// Infinite cylinder SDF - distance to infinite line, no end caps
/// Use this for tube segments to avoid hemispherical bumps at junctions
pub fn infinite_cylinder_sdf(point: Vec3, a: Vec3, direction: Vec3, radius: f32) -> f32 {
    let pa = point - a;
    // Project out the axial component, leaving only radial
    let axial = pa.dot(direction);
    let radial_vec = pa - direction * axial;
    radial_vec.length() - radius
}
