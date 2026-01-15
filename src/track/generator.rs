use bevy::prelude::*;
use rand::prelude::*;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use std::collections::VecDeque;

use super::{
    Port, PortProfile, Segment, Track,
    StraightTube, CurvedTube, FlatSlope,
    NarrowingTube, WideningTube, HalfPipe, SpiralTube,
    StartingGate, Funnel, TubeAdapter,
};

/// Configuration for procedural track generation
#[derive(Clone)]
pub struct GeneratorConfig {
    /// Random seed for reproducibility
    pub seed: u64,
    /// Number of marbles (determines starting gate width)
    pub num_marbles: usize,
    /// Marble radius
    pub marble_radius: f32,
    /// Minimum segment length
    pub min_length: f32,
    /// Maximum segment length
    pub max_length: f32,
    /// Base tube radius
    pub tube_radius: f32,
    /// Maximum turn angle (radians)
    pub max_turn_angle: f32,
    /// Probability weights for each segment type
    pub segment_weights: SegmentWeights,
    /// Whether to ensure track always descends
    pub force_descent: bool,
    /// How many recent segments to track for repeat penalty (0 = disabled)
    pub history_depth: usize,
    /// Weight multiplier applied for each occurrence in history (0.0-1.0)
    /// e.g., 0.25 means each repeat quarters the weight
    pub repeat_penalty: f32,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        Self {
            seed: 42,
            num_marbles: 8,
            marble_radius: 0.2,
            min_length: 4.0,
            max_length: 12.0,
            tube_radius: 1.0,
            max_turn_angle: std::f32::consts::FRAC_PI_2,
            segment_weights: SegmentWeights::default(),
            force_descent: true,
            history_depth: 3,
            repeat_penalty: 0.25,
        }
    }
}

/// Probability weights for segment selection
#[derive(Clone)]
pub struct SegmentWeights {
    pub straight: f32,
    pub curved_left: f32,
    pub curved_right: f32,
    pub spiral: f32,
    pub flat_slope: f32,
    pub narrowing: f32,
    pub widening: f32,
    pub half_pipe: f32,
}

impl Default for SegmentWeights {
    fn default() -> Self {
        Self {
            straight: 3.0,
            curved_left: 2.0,
            curved_right: 2.0,
            spiral: 0.5,
            flat_slope: 1.5,
            narrowing: 0.5,
            widening: 0.5,
            half_pipe: 0.5,
        }
    }
}

impl SegmentWeights {
    /// Get total weight for normalization
    fn total(&self) -> f32 {
        self.straight + self.curved_left + self.curved_right +
        self.spiral + self.flat_slope + self.narrowing +
        self.widening + self.half_pipe
    }

    /// Select a segment type based on weights
    fn select(&self, rng: &mut impl Rng) -> SegmentType {
        let total = self.total();
        let mut value = rng.r#gen::<f32>() * total;

        value -= self.straight;
        if value < 0.0 { return SegmentType::Straight; }

        value -= self.curved_left;
        if value < 0.0 { return SegmentType::CurvedLeft; }

        value -= self.curved_right;
        if value < 0.0 { return SegmentType::CurvedRight; }

        value -= self.spiral;
        if value < 0.0 { return SegmentType::Spiral; }

        value -= self.flat_slope;
        if value < 0.0 { return SegmentType::FlatSlope; }

        value -= self.narrowing;
        if value < 0.0 { return SegmentType::Narrowing; }

        value -= self.widening;
        if value < 0.0 { return SegmentType::Widening; }

        SegmentType::HalfPipe
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum SegmentType {
    Straight,
    CurvedLeft,
    CurvedRight,
    Spiral,
    FlatSlope,
    Narrowing,
    Widening,
    HalfPipe,
    TubeAdapter, // Auto-inserted only, not randomly selected
}

/// Procedural track generator
pub struct TrackGenerator {
    rng: ChaCha8Rng,
    config: GeneratorConfig,
    /// Track the current tube radius (for narrowing/widening sequences)
    current_radius: f32,
    /// Recent segment types for repeat penalty calculation
    recent_segments: VecDeque<SegmentType>,
}

impl TrackGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        let rng = ChaCha8Rng::seed_from_u64(config.seed);
        let current_radius = config.tube_radius;
        let recent_segments = VecDeque::with_capacity(config.history_depth + 1);
        Self { rng, config, current_radius, recent_segments }
    }

    /// Create with default config but custom seed
    pub fn with_seed(seed: u64) -> Self {
        let mut config = GeneratorConfig::default();
        config.seed = seed;
        Self::new(config)
    }

    /// Generate a complete track with the specified number of segments
    pub fn generate(&mut self, num_segments: usize) -> Track {
        let mut track = Track::new();

        // Calculate gate dimensions based on marble count
        let marble_spacing = self.config.marble_radius * 2.0 + 0.1;
        let gate_width = marble_spacing * self.config.num_marbles as f32 + 0.2;
        let gate_radius = gate_width / 2.0;
        let gate_length = 3.5;
        let gate_slope: f32 = 0.25; // Radians

        // Position the starting gate
        let start_y = 12.0;
        let gate_entry_y = start_y + gate_length * gate_slope.sin();
        let gate_entry_z = gate_length * gate_slope.cos();

        let gate_entry = Port::new(
            Vec3::new(0.0, gate_entry_y, gate_entry_z),
            Vec3::NEG_Z,
            Vec3::Y,
            gate_radius,
        );
        let starting_gate = StartingGate::new(gate_width, gate_length, 1.0, gate_slope, gate_entry);
        let gate_exit = starting_gate.exit_ports()[0].clone();
        track.add_segment(Box::new(starting_gate));

        // Randomly select a collection segment type that works with the wide gate exit
        let collection_type = self.rng.gen_range(0..5);

        match collection_type {
            0 => {
                // Funnel - vertical drop, collects marbles to center
                let funnel_depth = self.rng.gen_range(8.0..12.0);
                let funnel_entry = Port::new(
                    gate_exit.position - Vec3::Y * 0.3, // Slightly below gate exit
                    Vec3::NEG_Y,
                    Vec3::NEG_Z,
                    gate_radius,
                );
                let funnel = Funnel::new(funnel_depth, gate_radius, self.config.tube_radius, funnel_entry);
                track.add_segment(Box::new(funnel));
            }
            1 => {
                // TubeAdapter -> NarrowingTube - smooth transition from flat gate to tube
                let adapter = TubeAdapter::new(8.0, gate_width, gate_radius, gate_exit);
                let adapter_exit = adapter.exit_ports()[0].clone();
                track.add_segment(Box::new(adapter));

                let length = self.rng.gen_range(8.0..15.0);
                let narrowing = NarrowingTube::new(length, gate_radius, self.config.tube_radius, adapter_exit);
                track.add_segment(Box::new(narrowing));
            }
            2 => {
                // TubeAdapter -> SpiralTube at gate width, then narrowing
                let adapter = TubeAdapter::new(8.0, gate_width, gate_radius, gate_exit);
                let adapter_exit = adapter.exit_ports()[0].clone();
                track.add_segment(Box::new(adapter));

                let turns = self.rng.gen_range(0.5..1.5);
                let height = self.rng.gen_range(6.0..10.0);
                let helix_radius = self.rng.gen_range(3.0..5.0);
                let spiral = SpiralTube::new(turns, height, helix_radius, gate_radius, adapter_exit);
                let spiral_exit = spiral.exit_ports()[0].clone();
                track.add_segment(Box::new(spiral));

                // Add narrowing after spiral
                let narrowing = NarrowingTube::new(6.0, gate_radius, self.config.tube_radius, spiral_exit);
                track.add_segment(Box::new(narrowing));
            }
            3 => {
                // FlatSlope - wide open slope, then TubeAdapter, then narrowing
                let length = self.rng.gen_range(6.0..10.0);
                let slope_angle = self.rng.gen_range(0.2..0.4);
                let slope = FlatSlope::new(length, gate_width, 1.0, slope_angle, gate_exit);
                let slope_exit = slope.exit_ports()[0].clone();
                track.add_segment(Box::new(slope));

                // TubeAdapter before narrowing tube
                let adapter = TubeAdapter::new(8.0, gate_width, gate_radius, slope_exit);
                let adapter_exit = adapter.exit_ports()[0].clone();
                track.add_segment(Box::new(adapter));

                let narrowing = NarrowingTube::new(6.0, gate_radius, self.config.tube_radius, adapter_exit);
                track.add_segment(Box::new(narrowing));
            }
            _ => {
                // TubeAdapter -> HalfPipe, then narrowing
                let adapter = TubeAdapter::new(8.0, gate_width, gate_radius, gate_exit);
                let adapter_exit = adapter.exit_ports()[0].clone();
                track.add_segment(Box::new(adapter));

                let length = self.rng.gen_range(8.0..12.0);
                let half_pipe = HalfPipe::new(length, gate_radius, adapter_exit);
                let pipe_exit = half_pipe.exit_ports()[0].clone();
                track.add_segment(Box::new(half_pipe));

                // Add narrowing after half-pipe
                let narrowing = NarrowingTube::new(6.0, gate_radius, self.config.tube_radius, pipe_exit);
                track.add_segment(Box::new(narrowing));
            }
        }

        // Set current radius to tube radius (after collection segment narrows)
        self.current_radius = self.config.tube_radius;

        // Generate remaining segments
        let segments_added = track.segments().len();
        let mut segment_names: Vec<&str> = track.segments().iter()
            .map(|s| s.type_name())
            .collect();

        for _ in segments_added..num_segments {
            if let Some(segment) = self.generate_next(&track) {
                segment_names.push(segment.type_name());
                track.add_segment(segment);
            }
        }

        // Log the generated track
        info!("Generated track: {}", segment_names.join(" -> "));

        track
    }

    /// Generate a track and return spawn positions for marbles
    /// This creates the track and calculates where marbles should spawn on the starting gate
    pub fn generate_with_spawns(&mut self, num_segments: usize) -> (Track, Vec<Vec3>) {
        // Calculate gate dimensions (same as in generate)
        let marble_spacing = self.config.marble_radius * 2.0 + 0.1;
        let gate_width = marble_spacing * self.config.num_marbles as f32 + 0.2;
        let gate_radius = gate_width / 2.0;
        let gate_length = 3.5;
        let gate_slope: f32 = 0.25;

        let start_y = 12.0;
        let gate_entry_y = start_y + gate_length * gate_slope.sin();
        let gate_entry_z = gate_length * gate_slope.cos();

        let gate_entry = Port::new(
            Vec3::new(0.0, gate_entry_y, gate_entry_z),
            Vec3::NEG_Z,
            Vec3::Y,
            gate_radius,
        );

        // Create starting gate to get spawn positions
        let starting_gate = StartingGate::new(gate_width, gate_length, 1.0, gate_slope, gate_entry);
        let spawn_positions = starting_gate.get_spawn_positions(self.config.num_marbles, self.config.marble_radius);

        // Generate the full track
        let track = self.generate(num_segments);

        (track, spawn_positions)
    }

    /// Generate the next segment based on current track state.
    /// Uses profile-aware selection to ensure valid connections.
    pub fn generate_next(&mut self, track: &Track) -> Option<Box<dyn Segment>> {
        let exit_port = track.last_port()?;

        // Determine which segment types are valid for this exit profile
        let valid_types = self.get_valid_segment_types(&exit_port);

        if valid_types.is_empty() {
            // Fallback: always allow straight tube with matching profile
            return Some(Box::new(StraightTube::new(
                self.config.min_length,
                self.current_radius,
                exit_port,
            )));
        }

        // Select from valid types using weights
        let segment_type = self.select_weighted_from(&valid_types);

        // AUTO-INSERT: If exiting a FlatFloor and going to a tube type, insert TubeAdapter first
        if let PortProfile::FlatFloor { width, floor_y } = &exit_port.profile {
            if self.is_tube_entry_type(segment_type) {
                // Insert TubeAdapter to transition from flat floor to tube
                self.record_segment(SegmentType::TubeAdapter);
                return Some(Box::new(TubeAdapter::new(
                    8.0, // Default adapter length
                    *width,
                    self.current_radius,
                    exit_port,
                )));
            }
        }

        // Generate length variation
        let length = self.rng.gen_range(self.config.min_length..self.config.max_length);

        let segment: Box<dyn Segment> = match segment_type {
            SegmentType::Straight => {
                Box::new(StraightTube::new(length, self.current_radius, exit_port.clone()))
            }

            SegmentType::CurvedLeft => {
                let angle = self.rng.gen_range(0.3..self.config.max_turn_angle);
                let arc_radius = self.rng.gen_range(3.0..8.0);
                Box::new(CurvedTube::new(angle, arc_radius, self.current_radius, exit_port.clone()))
            }

            SegmentType::CurvedRight => {
                let angle = self.rng.gen_range(0.3..self.config.max_turn_angle);
                let arc_radius = self.rng.gen_range(3.0..8.0);
                Box::new(CurvedTube::new(-angle, arc_radius, self.current_radius, exit_port.clone()))
            }

            SegmentType::Spiral => {
                let turns = self.rng.gen_range(0.5..1.5);
                let height = self.rng.gen_range(5.0..12.0);
                let radius = self.rng.gen_range(3.0..6.0);
                Box::new(SpiralTube::new(turns, height, radius, self.current_radius, exit_port.clone()))
            }

            SegmentType::FlatSlope => {
                // FlatSlope must have floor at or below the exit's floor level
                let width = self.rng.gen_range(2.0..4.0);
                let wall_height = self.rng.gen_range(0.5..1.5);
                let slope_angle = self.rng.gen_range(0.15..0.4);

                // Adjust entry port to have floor at correct height
                let floor_y = exit_port.floor_y();
                let adjusted_port = Port::with_profile(
                    exit_port.position,
                    exit_port.direction,
                    exit_port.up,
                    width / 2.0,
                    PortProfile::flat_floor(width, floor_y),
                );

                Box::new(FlatSlope::new(length, width, wall_height, slope_angle, adjusted_port))
            }

            SegmentType::Narrowing => {
                let new_radius = self.current_radius * 0.85;
                let segment = NarrowingTube::new(length, self.current_radius, new_radius, exit_port.clone());
                self.current_radius = new_radius;
                Box::new(segment)
            }

            SegmentType::Widening => {
                let new_radius = (self.current_radius * 1.15).min(self.config.tube_radius);
                let segment = WideningTube::new(length, self.current_radius, new_radius, exit_port.clone());
                self.current_radius = new_radius;
                Box::new(segment)
            }

            SegmentType::HalfPipe => {
                Box::new(HalfPipe::new(length, self.current_radius * 1.5, exit_port.clone()))
            }

            SegmentType::TubeAdapter => {
                // TubeAdapter is only auto-inserted above, should never be selected here
                // But we need to handle the case for exhaustive matching
                unreachable!("TubeAdapter should only be auto-inserted, not selected")
            }
        };

        // Validate the segment can connect (extra safety check)
        if let Err(e) = track.validate_segment(segment.as_ref()) {
            // Log warning and try a fallback
            eprintln!("Warning: Generated invalid segment ({}), falling back to straight tube", e);
            self.record_segment(SegmentType::Straight);
            return Some(Box::new(StraightTube::new(length, self.current_radius, exit_port)));
        }

        // Record this segment type for repeat penalty calculation
        self.record_segment(segment_type);

        Some(segment)
    }

    /// Get segment types that are valid for the given exit port profile
    fn get_valid_segment_types(&self, exit_port: &Port) -> Vec<SegmentType> {
        let mut valid = Vec::new();

        match &exit_port.profile {
            // Tube exits can connect to: tubes, half-pipes, and flat floors (if tube is above floor)
            PortProfile::Tube { diameter } => {
                // Tube-compatible segments
                valid.push(SegmentType::Straight);
                valid.push(SegmentType::CurvedLeft);
                valid.push(SegmentType::CurvedRight);
                valid.push(SegmentType::Spiral);

                // Narrowing/widening based on current radius
                if self.current_radius > self.config.tube_radius * 0.7 {
                    valid.push(SegmentType::Narrowing);
                }
                if self.current_radius < self.config.tube_radius {
                    valid.push(SegmentType::Widening);
                }

                // HalfPipe with matching diameter
                valid.push(SegmentType::HalfPipe);

                // FlatSlope - tube bottom must be at or above the flat floor
                // Since we can position the flat floor at the tube bottom, this is always valid
                valid.push(SegmentType::FlatSlope);
            }

            // HalfPipe exits - similar to tube
            PortProfile::HalfPipe { diameter } => {
                valid.push(SegmentType::Straight);
                valid.push(SegmentType::CurvedLeft);
                valid.push(SegmentType::CurvedRight);
                valid.push(SegmentType::Spiral);
                valid.push(SegmentType::HalfPipe);

                if self.current_radius > self.config.tube_radius * 0.7 {
                    valid.push(SegmentType::Narrowing);
                }
                if self.current_radius < self.config.tube_radius {
                    valid.push(SegmentType::Widening);
                }

                // FlatSlope works if floor is at or below half-pipe bottom
                valid.push(SegmentType::FlatSlope);
            }

            // FlatFloor exits - can connect to flat floors or tube types
            // Note: When a tube type is selected, generate_next() will auto-insert
            // a TubeAdapter first to provide a smooth transition
            PortProfile::FlatFloor { width, floor_y } => {
                // FlatSlope to FlatSlope (descending or level)
                valid.push(SegmentType::FlatSlope);

                // Tube types - TubeAdapter will be auto-inserted before these
                valid.push(SegmentType::Straight);
                valid.push(SegmentType::CurvedLeft);
                valid.push(SegmentType::CurvedRight);
                valid.push(SegmentType::HalfPipe);
            }

            // Open profiles accept anything
            PortProfile::Open { .. } => {
                valid.push(SegmentType::Straight);
                valid.push(SegmentType::CurvedLeft);
                valid.push(SegmentType::CurvedRight);
                valid.push(SegmentType::Spiral);
                valid.push(SegmentType::FlatSlope);
                valid.push(SegmentType::HalfPipe);

                if self.current_radius > self.config.tube_radius * 0.7 {
                    valid.push(SegmentType::Narrowing);
                }
                if self.current_radius < self.config.tube_radius {
                    valid.push(SegmentType::Widening);
                }
            }
        }

        valid
    }

    /// Select a segment type from the valid list using configured weights
    fn select_weighted_from(&mut self, valid_types: &[SegmentType]) -> SegmentType {
        // Calculate total weight for valid types only (with repeat penalty applied)
        let mut total_weight = 0.0;
        for t in valid_types {
            total_weight += self.effective_weight_for(*t);
        }

        if total_weight <= 0.0 {
            return valid_types[0];
        }

        // Weighted selection using effective weights
        let mut value = self.rng.r#gen::<f32>() * total_weight;
        for t in valid_types {
            value -= self.effective_weight_for(*t);
            if value < 0.0 {
                return *t;
            }
        }

        valid_types[valid_types.len() - 1]
    }

    /// Get the weight for a segment type
    fn weight_for(&self, segment_type: SegmentType) -> f32 {
        match segment_type {
            SegmentType::Straight => self.config.segment_weights.straight,
            SegmentType::CurvedLeft => self.config.segment_weights.curved_left,
            SegmentType::CurvedRight => self.config.segment_weights.curved_right,
            SegmentType::Spiral => self.config.segment_weights.spiral,
            SegmentType::FlatSlope => self.config.segment_weights.flat_slope,
            SegmentType::Narrowing => self.config.segment_weights.narrowing,
            SegmentType::Widening => self.config.segment_weights.widening,
            SegmentType::HalfPipe => self.config.segment_weights.half_pipe,
            SegmentType::TubeAdapter => 0.0, // Not randomly selected, only auto-inserted
        }
    }

    /// Reset the generator with a new seed
    pub fn reseed(&mut self, seed: u64) {
        self.rng = ChaCha8Rng::seed_from_u64(seed);
        self.current_radius = self.config.tube_radius;
        self.recent_segments.clear();
    }

    /// Check if a segment type requires a tube entry (and thus needs TubeAdapter from FlatFloor)
    fn is_tube_entry_type(&self, segment_type: SegmentType) -> bool {
        matches!(segment_type,
            SegmentType::Straight |
            SegmentType::CurvedLeft |
            SegmentType::CurvedRight |
            SegmentType::Spiral |
            SegmentType::Narrowing |
            SegmentType::Widening |
            SegmentType::HalfPipe
        )
    }

    /// Get effective weight for a segment type, applying repeat penalty
    fn effective_weight_for(&self, segment_type: SegmentType) -> f32 {
        let base_weight = self.weight_for(segment_type);

        if self.config.history_depth == 0 {
            return base_weight;
        }

        // Count occurrences of this type in recent history
        let occurrences = self.recent_segments
            .iter()
            .filter(|&&t| t == segment_type)
            .count();

        if occurrences == 0 {
            base_weight
        } else {
            // Apply exponential penalty: weight * (penalty ^ occurrences)
            base_weight * self.config.repeat_penalty.powi(occurrences as i32)
        }
    }

    /// Record a segment type in recent history
    fn record_segment(&mut self, segment_type: SegmentType) {
        self.recent_segments.push_back(segment_type);
        while self.recent_segments.len() > self.config.history_depth {
            self.recent_segments.pop_front();
        }
    }
}

/// Bevy resource for procedural generation
#[derive(Resource)]
pub struct ProceduralTrack {
    pub generator: TrackGenerator,
    pub target_segments: usize,
}

impl Default for ProceduralTrack {
    fn default() -> Self {
        Self {
            generator: TrackGenerator::with_seed(42),
            target_segments: 15,
        }
    }
}

impl TrackGenerator {
    /// Generate a sequence of segment types without creating actual geometry.
    /// Useful for testing distribution and variety.
    pub(crate) fn generate_segment_sequence(&mut self, num_segments: usize) -> Vec<SegmentType> {
        let mut sequence = Vec::with_capacity(num_segments);

        // Simulate starting with a tube exit (like after StartingGate + collection)
        let simulated_exit = Port::new(
            Vec3::new(0.0, 10.0, 0.0),
            Vec3::NEG_Z,
            Vec3::Y,
            self.config.tube_radius,
        );

        // Generate sequence using the same logic as generate_next
        for _ in 0..num_segments {
            let valid_types = self.get_valid_segment_types(&simulated_exit);

            if valid_types.is_empty() {
                sequence.push(SegmentType::Straight);
                self.record_segment(SegmentType::Straight);
                continue;
            }

            let segment_type = self.select_weighted_from(&valid_types);
            sequence.push(segment_type);
            self.record_segment(segment_type);
        }

        sequence
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// Test that segment distribution roughly matches configured weights
    /// and that variety is maintained across many tracks.
    #[test]
    fn test_segment_distribution_across_tracks() {
        let num_tracks: usize = 1000;
        let segments_per_track: usize = 15;

        let mut total_counts: HashMap<SegmentType, usize> = HashMap::new();
        let mut tracks_with_dominant_type = 0;

        for seed in 0..num_tracks {
            let config = GeneratorConfig {
                seed: seed as u64,
                ..Default::default()
            };
            let mut generator = TrackGenerator::new(config);
            let sequence = generator.generate_segment_sequence(segments_per_track);

            // Count segments in this track
            let mut track_counts: HashMap<SegmentType, usize> = HashMap::new();
            for seg_type in &sequence {
                *track_counts.entry(*seg_type).or_insert(0) += 1;
                *total_counts.entry(*seg_type).or_insert(0) += 1;
            }

            // Check if any single type dominates this track (>60% of segments)
            let max_count = track_counts.values().max().unwrap_or(&0);
            if *max_count > (segments_per_track * 6 / 10) {
                tracks_with_dominant_type += 1;
            }
        }

        let total_segments = num_tracks * segments_per_track;

        // Print distribution for debugging
        println!("\n=== Segment Distribution across {} tracks ===", num_tracks);
        let mut sorted_counts: Vec<_> = total_counts.iter().collect();
        sorted_counts.sort_by(|a, b| b.1.cmp(a.1));
        for (seg_type, count) in &sorted_counts {
            let percentage = (**count as f64 / total_segments as f64) * 100.0;
            println!("{:?}: {} ({:.1}%)", seg_type, count, percentage);
        }
        println!("\nTracks with dominant type (>60%): {} ({:.1}%)",
            tracks_with_dominant_type,
            (tracks_with_dominant_type as f64 / num_tracks as f64) * 100.0
        );

        // Assertions

        // 1. Straight tubes should be most common (weight 3.0)
        let straight_count = *total_counts.get(&SegmentType::Straight).unwrap_or(&0);
        let straight_pct = straight_count as f64 / total_segments as f64;
        assert!(straight_pct > 0.15, "Straight tubes too rare: {:.1}%", straight_pct * 100.0);
        assert!(straight_pct < 0.45, "Straight tubes too common: {:.1}%", straight_pct * 100.0);

        // 2. Curves should be common (weight 2.0 each)
        let curved_left = *total_counts.get(&SegmentType::CurvedLeft).unwrap_or(&0);
        let curved_right = *total_counts.get(&SegmentType::CurvedRight).unwrap_or(&0);
        let total_curves = curved_left + curved_right;
        let curves_pct = total_curves as f64 / total_segments as f64;
        assert!(curves_pct > 0.20, "Curves too rare: {:.1}%", curves_pct * 100.0);

        // 3. No type should be absent
        assert!(total_counts.len() >= 6, "Too few segment types generated: {}", total_counts.len());

        // 4. Repeat penalty should prevent too many tracks with dominant types
        // With history_depth=3 and penalty=0.25, < 10% of tracks should be dominated
        let dominant_pct = tracks_with_dominant_type as f64 / num_tracks as f64;
        assert!(dominant_pct < 0.15,
            "Too many tracks with dominant type: {:.1}% (repeat penalty may not be working)",
            dominant_pct * 100.0
        );
    }

    /// Test that consecutive identical segments are rare within a single track
    #[test]
    fn test_repeat_penalty_effectiveness() {
        let num_tracks: usize = 500;
        let segments_per_track: usize = 20;

        let mut total_runs_of_3_plus = 0;
        let mut total_runs_of_4_plus = 0;

        for seed in 0..num_tracks {
            let config = GeneratorConfig {
                seed: seed as u64,
                ..Default::default()
            };
            let mut generator = TrackGenerator::new(config);
            let sequence = generator.generate_segment_sequence(segments_per_track);

            // Count runs of 3+ and 4+ identical segments
            let mut current_type = None;
            let mut run_length = 0;

            for seg_type in &sequence {
                if Some(*seg_type) == current_type {
                    run_length += 1;
                } else {
                    current_type = Some(*seg_type);
                    run_length = 1;
                }

                if run_length == 3 {
                    total_runs_of_3_plus += 1;
                }
                if run_length == 4 {
                    total_runs_of_4_plus += 1;
                }
            }
        }

        println!("\n=== Repeat Penalty Test ({} tracks) ===", num_tracks);
        println!("Runs of 3+ identical: {}", total_runs_of_3_plus);
        println!("Runs of 4+ identical: {}", total_runs_of_4_plus);

        // With penalty=0.25, runs of 3+ should be rare, runs of 4+ very rare
        let runs_3_per_track = total_runs_of_3_plus as f64 / num_tracks as f64;
        let runs_4_per_track = total_runs_of_4_plus as f64 / num_tracks as f64;

        assert!(runs_3_per_track < 1.0,
            "Too many runs of 3+: {:.2} per track", runs_3_per_track);
        assert!(runs_4_per_track < 0.2,
            "Too many runs of 4+: {:.2} per track", runs_4_per_track);
    }

    /// Test that different seeds produce different tracks
    #[test]
    fn test_seed_produces_variety() {
        let mut unique_tracks = std::collections::HashSet::new();
        let num_seeds: usize = 100;
        let segments_per_track: usize = 10;

        for seed in 0..num_seeds {
            let config = GeneratorConfig {
                seed: seed as u64,
                ..Default::default()
            };
            let mut generator = TrackGenerator::new(config);
            let sequence = generator.generate_segment_sequence(segments_per_track);

            // Convert to string for easy comparison
            let track_str: String = sequence.iter()
                .map(|t| format!("{:?}", t).chars().next().unwrap())
                .collect();

            unique_tracks.insert(track_str);
        }

        let unique_pct = (unique_tracks.len() as f64 / num_seeds as f64) * 100.0;
        println!("\n=== Seed Variety Test ===");
        println!("Unique tracks: {} out of {} ({:.1}%)", unique_tracks.len(), num_seeds, unique_pct);

        // At least 80% of seeds should produce unique tracks
        assert!(unique_tracks.len() >= num_seeds * 80 / 100,
            "Not enough variety: only {} unique tracks out of {}", unique_tracks.len(), num_seeds);
    }
}
