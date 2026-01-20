use bevy::prelude::*;

pub mod segments;
pub mod generator;

// Re-export segment types
pub use segments::*;
pub use generator::{TrackGenerator, GeneratorConfig, ProceduralTrack};

// ============================================================================
// PORT PROFILE - Cross-sectional shape of a connection point
// ============================================================================

/// Describes the cross-sectional profile of a port.
/// This determines what types of segments can connect seamlessly.
#[derive(Clone, Debug, PartialEq)]
pub enum PortProfile {
    /// Full circular tube - marble enclosed on all sides.
    /// Tubes can only connect to tubes/half-pipes with matching diameter.
    Tube {
        /// Inner diameter of the tube
        diameter: f32,
    },

    /// Half-pipe (roofless tube) - semicircular floor, open top.
    /// Treated like a tube for connection purposes (same diameter rules).
    HalfPipe {
        /// Diameter of the half-pipe curve
        diameter: f32,
    },

    /// Flat floor with walls (trough/gutter style).
    /// When connecting from a tube, the tube's bottom must be at or above the floor.
    FlatFloor {
        /// Width between walls
        width: f32,
        /// Y-coordinate of the floor at this port (world space)
        floor_y: f32,
    },

    /// Funnel or bowl opening - can accept any profile
    /// Used for collectors that gather marbles from various sources
    Open {
        /// Opening diameter/width
        size: f32,
    },
}

impl PortProfile {
    /// Create a tube profile from radius
    pub fn tube(radius: f32) -> Self {
        Self::Tube { diameter: radius * 2.0 }
    }

    /// Create a half-pipe profile from radius
    pub fn half_pipe(radius: f32) -> Self {
        Self::HalfPipe { diameter: radius * 2.0 }
    }

    /// Create a flat floor profile
    pub fn flat_floor(width: f32, floor_y: f32) -> Self {
        Self::FlatFloor { width, floor_y }
    }

    /// Create an open (accepting any) profile
    pub fn open(size: f32) -> Self {
        Self::Open { size }
    }

    /// Get the effective diameter/width of this profile
    pub fn effective_width(&self) -> f32 {
        match self {
            Self::Tube { diameter } => *diameter,
            Self::HalfPipe { diameter } => *diameter,
            Self::FlatFloor { width, .. } => *width,
            Self::Open { size } => *size,
        }
    }

    /// Check if this is a circular profile (tube or half-pipe)
    pub fn is_circular(&self) -> bool {
        matches!(self, Self::Tube { .. } | Self::HalfPipe { .. })
    }

    /// Check if this is a flat floor profile
    pub fn is_flat(&self) -> bool {
        matches!(self, Self::FlatFloor { .. })
    }
}

/// Error when segments cannot connect
#[derive(Clone, Debug)]
pub struct ConnectionError {
    pub reason: String,
}

impl std::fmt::Display for ConnectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.reason)
    }
}

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
    /// Channel radius at this point (legacy, use profile for new code)
    pub radius: f32,
    /// Cross-sectional profile at this port
    pub profile: PortProfile,
}

impl Port {
    /// Create a new port with a tube profile (default for backwards compatibility)
    pub fn new(position: Vec3, direction: Vec3, up: Vec3, radius: f32) -> Self {
        Self {
            position,
            direction: direction.normalize(),
            up: up.normalize(),
            radius,
            profile: PortProfile::tube(radius),
        }
    }

    /// Create a new port with a specific profile
    pub fn with_profile(position: Vec3, direction: Vec3, up: Vec3, radius: f32, profile: PortProfile) -> Self {
        Self {
            position,
            direction: direction.normalize(),
            up: up.normalize(),
            radius,
            profile,
        }
    }

    /// Check if another port can connect to this one (legacy simple check)
    /// Directions should be roughly opposite, positions should match
    pub fn can_connect(&self, other: &Port) -> bool {
        self.validate_connection(other).is_ok()
    }

    /// Validate if this port (as an exit) can connect to another port (as an entrance).
    /// Returns Ok(()) if compatible, Err with reason if not.
    ///
    /// Connection rules:
    /// - Tube-to-Tube: Diameters must match
    /// - HalfPipe-to-HalfPipe: Diameters must match
    /// - Tube-to-HalfPipe (either direction): Diameters must match (both circular)
    /// - Tube/HalfPipe-to-FlatFloor: Tube bottom must be at or above floor
    /// - FlatFloor-to-Tube/HalfPipe: Floor must be at or below tube bottom
    /// - FlatFloor-to-FlatFloor: Exit floor must be at or above entrance floor
    /// - Open profile: Accepts anything
    pub fn validate_connection(&self, other: &Port) -> Result<(), ConnectionError> {
        const POSITION_TOLERANCE: f32 = 0.5;
        const SIZE_TOLERANCE: f32 = 0.2;

        // Check position alignment
        let pos_diff = (self.position - other.position).length();
        if pos_diff > POSITION_TOLERANCE {
            return Err(ConnectionError {
                reason: format!("Positions too far apart: {:.2} units", pos_diff),
            });
        }

        // Check direction alignment (should be roughly parallel for exit->entrance)
        // Both point in the direction of travel (forward through the track)
        let dir_dot = self.direction.dot(other.direction);
        if dir_dot < 0.5 {
            return Err(ConnectionError {
                reason: format!("Directions not aligned (dot={:.2}, need >=0.5)", dir_dot),
            });
        }

        // Open profiles accept anything
        if matches!(self.profile, PortProfile::Open { .. }) || matches!(other.profile, PortProfile::Open { .. }) {
            return Ok(());
        }

        // Profile-specific validation
        match (&self.profile, &other.profile) {
            // Circular-to-Circular (Tube/HalfPipe combinations): diameters must match
            (PortProfile::Tube { diameter: d1 }, PortProfile::Tube { diameter: d2 })
            | (PortProfile::Tube { diameter: d1 }, PortProfile::HalfPipe { diameter: d2 })
            | (PortProfile::HalfPipe { diameter: d1 }, PortProfile::Tube { diameter: d2 })
            | (PortProfile::HalfPipe { diameter: d1 }, PortProfile::HalfPipe { diameter: d2 }) => {
                if (d1 - d2).abs() > SIZE_TOLERANCE {
                    return Err(ConnectionError {
                        reason: format!("Diameter mismatch: {:.2} vs {:.2}", d1, d2),
                    });
                }
            }

            // Circular-to-FlatFloor: compatible (height is geometry, not connection type)
            (PortProfile::Tube { .. } | PortProfile::HalfPipe { .. }, PortProfile::FlatFloor { .. }) => {
                // Heights are adjusted during segment creation, not validated here
            }

            // FlatFloor-to-Circular: compatible
            (PortProfile::FlatFloor { .. }, PortProfile::Tube { .. } | PortProfile::HalfPipe { .. }) => {
                // Heights are adjusted during segment creation, not validated here
            }

            // FlatFloor-to-FlatFloor: width should be similar
            (PortProfile::FlatFloor { width: w1, .. }, PortProfile::FlatFloor { width: w2, .. }) => {
                if (w1 - w2).abs() > SIZE_TOLERANCE {
                    return Err(ConnectionError {
                        reason: format!("Width mismatch: {:.2} vs {:.2}", w1, w2),
                    });
                }
                // Height validation removed - floor heights are geometry, not connection constraints
            }

            // Open to anything (already handled above, but for completeness)
            (PortProfile::Open { .. }, _) | (_, PortProfile::Open { .. }) => {}
        }

        Ok(())
    }

    /// Calculate the Y position of the lowest point of this port's profile
    pub fn floor_y(&self) -> f32 {
        match &self.profile {
            PortProfile::Tube { diameter } => self.position.y - diameter / 2.0,
            PortProfile::HalfPipe { diameter } => self.position.y - diameter / 2.0,
            PortProfile::FlatFloor { floor_y, .. } => *floor_y,
            PortProfile::Open { size } => self.position.y - size / 2.0,
        }
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
            profile: PortProfile::tube(1.0),
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

    /// Compute the SDF gradient (surface normal) at a point
    /// Default implementation uses central differences, but segments can override
    /// with analytic gradients for smoother physics
    fn sdf_gradient(&self, point: Vec3) -> Vec3 {
        const EPS: f32 = 0.01;
        let dx = self.sdf(point + Vec3::X * EPS) - self.sdf(point - Vec3::X * EPS);
        let dy = self.sdf(point + Vec3::Y * EPS) - self.sdf(point - Vec3::Y * EPS);
        let dz = self.sdf(point + Vec3::Z * EPS) - self.sdf(point - Vec3::Z * EPS);
        Vec3::new(dx, dy, dz).normalize_or_zero()
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
    /// Y-coordinate below which marbles are considered "fallen" and should be reset.
    /// Calculated as the lowest point of any segment minus a margin.
    kill_plane_y: f32,
}

/// Margin below the lowest segment for the kill plane
const KILL_PLANE_MARGIN: f32 = 10.0;

impl Track {
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
            bounds: AABB::default(),
            smooth_k: 0.0, // No blending - segments connect seamlessly via overlap
            kill_plane_y: -50.0, // Default fallback
        }
    }

    /// Add a segment to the track without validation
    pub fn add_segment(&mut self, segment: Box<dyn Segment>) {
        let seg_bounds = segment.bounds();
        if self.segments.is_empty() {
            self.bounds = seg_bounds;
        } else {
            self.bounds = self.bounds.union(&seg_bounds);
        }
        self.segments.push(segment);
        self.update_kill_plane();
    }

    /// Update the kill plane based on current bounds
    fn update_kill_plane(&mut self) {
        if self.bounds.min.y < f32::MAX * 0.5 {
            self.kill_plane_y = self.bounds.min.y - KILL_PLANE_MARGIN;
        }
    }

    /// Get the kill plane Y coordinate (marbles below this should be reset)
    pub fn kill_plane_y(&self) -> f32 {
        self.kill_plane_y
    }

    /// Validate if a segment can be added to the track.
    /// Returns Ok(()) if valid, Err with reason if not.
    pub fn validate_segment(&self, segment: &dyn Segment) -> Result<(), ConnectionError> {
        // First segment always valid
        if self.segments.is_empty() {
            return Ok(());
        }

        // Get the last segment's exit port and new segment's entry port
        let exit_port = self.segments.last().unwrap().primary_exit_port();
        let entry_port = segment.entry_port();

        // Validate connection
        exit_port.validate_connection(&entry_port)
    }

    /// Add a segment with validation. Returns Err if the segment cannot connect.
    pub fn try_add_segment(&mut self, segment: Box<dyn Segment>) -> Result<(), ConnectionError> {
        self.validate_segment(segment.as_ref())?;
        self.add_segment(segment);
        Ok(())
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
        self.update_kill_plane();
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

        let hint = (segment_hint.max(0) as usize).min(self.segments.len() - 1);

        // Check current, previous, and next segments
        // Use MIN of all accepting segments to get the most constraining floor
        // This ensures smooth transitions at segment boundaries
        let curr_dist = self.segments[hint].sdf(point);

        let prev_dist = if hint > 0 {
            self.segments[hint - 1].sdf(point)
        } else {
            f32::MAX
        };

        let next_dist = if hint + 1 < self.segments.len() {
            self.segments[hint + 1].sdf(point)
        } else {
            f32::MAX
        };

        // Take MIN of all segments that accept the point
        let mut best = f32::MAX;
        if curr_dist < f32::MAX * 0.5 {
            best = best.min(curr_dist);
        }
        if prev_dist < f32::MAX * 0.5 {
            best = best.min(prev_dist);
        }
        if next_dist < f32::MAX * 0.5 {
            best = best.min(next_dist);
        }

        if best < f32::MAX * 0.5 {
            return best;
        }

        100.0
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
                // Must be positive AND not f32::MAX (which means segment rejected the point)
                if dist > 0.0 && dist < f32::MAX * 0.5 {
                    return i as i32;
                }
            }
        }

        // Fallback: current segment if still valid
        if hint < self.segments.len() {
            let dist = self.segments[hint].sdf(point);
            if dist > 0.0 && dist < f32::MAX * 0.5 {
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
        if segment_idx >= self.segments.len() {
            return self.sdf_gradient(point);
        }

        // Use the segment's own gradient method (can be analytic for smooth curves)
        self.segments[segment_idx].sdf_gradient(point)
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
