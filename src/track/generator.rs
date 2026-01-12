use bevy::prelude::*;
use rand::prelude::*;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use super::{
    Port, Segment, Track,
    StraightTube, CurvedTube, FlatSlope,
    NarrowingTube, WideningTube, HalfPipe, SpiralTube,
    StartingGate, Funnel,
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

#[derive(Clone, Copy, Debug, PartialEq)]
enum SegmentType {
    Straight,
    CurvedLeft,
    CurvedRight,
    Spiral,
    FlatSlope,
    Narrowing,
    Widening,
    HalfPipe,
}

/// Procedural track generator
pub struct TrackGenerator {
    rng: ChaCha8Rng,
    config: GeneratorConfig,
    /// Track the current tube radius (for narrowing/widening sequences)
    current_radius: f32,
}

impl TrackGenerator {
    pub fn new(config: GeneratorConfig) -> Self {
        let rng = ChaCha8Rng::seed_from_u64(config.seed);
        let current_radius = config.tube_radius;
        Self { rng, config, current_radius }
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
                // NarrowingTube - gradually narrows from gate width to tube radius
                let length = self.rng.gen_range(8.0..15.0);
                let narrowing = NarrowingTube::new(length, gate_radius, self.config.tube_radius, gate_exit);
                track.add_segment(Box::new(narrowing));
            }
            2 => {
                // SpiralTube at gate width, then narrowing
                let turns = self.rng.gen_range(0.5..1.5);
                let height = self.rng.gen_range(6.0..10.0);
                let helix_radius = self.rng.gen_range(3.0..5.0);
                let spiral = SpiralTube::new(turns, height, helix_radius, gate_radius, gate_exit);
                let spiral_exit = spiral.exit_ports()[0].clone();
                track.add_segment(Box::new(spiral));

                // Add narrowing after spiral
                let narrowing = NarrowingTube::new(6.0, gate_radius, self.config.tube_radius, spiral_exit);
                track.add_segment(Box::new(narrowing));
            }
            3 => {
                // FlatSlope - wide open slope, then narrowing
                let length = self.rng.gen_range(6.0..10.0);
                let slope_angle = self.rng.gen_range(0.2..0.4);
                let slope = FlatSlope::new(length, gate_width, 1.0, slope_angle, gate_exit);
                let slope_exit = slope.exit_ports()[0].clone();
                track.add_segment(Box::new(slope));

                // Add narrowing after slope
                let narrowing = NarrowingTube::new(6.0, gate_radius, self.config.tube_radius, slope_exit);
                track.add_segment(Box::new(narrowing));
            }
            _ => {
                // HalfPipe - wide half-pipe, then narrowing
                let length = self.rng.gen_range(8.0..12.0);
                let half_pipe = HalfPipe::new(length, gate_radius, gate_exit);
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
        for _ in segments_added..num_segments {
            if let Some(segment) = self.generate_next(&track) {
                track.add_segment(segment);
            }
        }

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

    /// Generate the next segment based on current track state
    pub fn generate_next(&mut self, track: &Track) -> Option<Box<dyn Segment>> {
        let exit_port = track.last_port()?;

        // Select segment type
        let mut segment_type = self.config.segment_weights.select(&mut self.rng);

        // Avoid widening if already at max radius
        if segment_type == SegmentType::Widening && self.current_radius >= self.config.tube_radius {
            segment_type = SegmentType::Straight;
        }

        // Avoid narrowing if already at min radius
        if segment_type == SegmentType::Narrowing && self.current_radius <= self.config.tube_radius * 0.7 {
            segment_type = SegmentType::Straight;
        }

        // Generate length variation
        let length = self.rng.gen_range(self.config.min_length..self.config.max_length);

        let segment: Box<dyn Segment> = match segment_type {
            SegmentType::Straight => {
                Box::new(StraightTube::new(length, self.current_radius, exit_port))
            }

            SegmentType::CurvedLeft => {
                let angle = self.rng.gen_range(0.3..self.config.max_turn_angle);
                let arc_radius = self.rng.gen_range(3.0..8.0);
                Box::new(CurvedTube::new(angle, arc_radius, self.current_radius, exit_port))
            }

            SegmentType::CurvedRight => {
                let angle = self.rng.gen_range(0.3..self.config.max_turn_angle);
                let arc_radius = self.rng.gen_range(3.0..8.0);
                Box::new(CurvedTube::new(-angle, arc_radius, self.current_radius, exit_port))
            }

            SegmentType::Spiral => {
                let turns = self.rng.gen_range(0.5..1.5);
                let height = self.rng.gen_range(5.0..12.0);
                let radius = self.rng.gen_range(3.0..6.0);
                Box::new(SpiralTube::new(turns, height, radius, self.current_radius, exit_port))
            }

            SegmentType::FlatSlope => {
                let width = self.rng.gen_range(2.0..4.0);
                let wall_height = self.rng.gen_range(0.5..1.5);
                let slope_angle = self.rng.gen_range(0.15..0.4);
                Box::new(FlatSlope::new(length, width, wall_height, slope_angle, exit_port))
            }

            SegmentType::Narrowing => {
                let new_radius = self.current_radius * 0.85;
                let segment = NarrowingTube::new(length, self.current_radius, new_radius, exit_port);
                self.current_radius = new_radius;
                Box::new(segment)
            }

            SegmentType::Widening => {
                let new_radius = (self.current_radius * 1.15).min(self.config.tube_radius);
                let segment = WideningTube::new(length, self.current_radius, new_radius, exit_port);
                self.current_radius = new_radius;
                Box::new(segment)
            }

            SegmentType::HalfPipe => {
                Box::new(HalfPipe::new(length, self.current_radius * 1.5, exit_port))
            }
        };

        Some(segment)
    }

    /// Reset the generator with a new seed
    pub fn reseed(&mut self, seed: u64) {
        self.rng = ChaCha8Rng::seed_from_u64(seed);
        self.current_radius = self.config.tube_radius;
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
