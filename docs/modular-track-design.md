# Modular Track Segment System Design

## Overview

Design a system where different SDF-based track segments can connect seamlessly to create varied, interesting marble runs. Each segment type has unique geometry but shares a common connection interface.

## Connection System

### Port Definition

Each segment has **entry** and **exit** ports (some segments like forks have multiple exits):

```rust
struct Port {
    position: Vec3,      // World position of connection point
    direction: Vec3,     // Tangent direction (normalized, points "into" segment)
    up: Vec3,            // Up vector for orientation
    radius: f32,         // Channel radius at this point
    channel_type: ChannelType,  // Tube, HalfPipe, Flat, etc.
}

enum ChannelType {
    Tube,       // Fully enclosed circular
    HalfPipe,   // Open top, U-shaped
    Flat,       // Open flat surface
    Funnel,     // Conical transition
}
```

### Connection Rules

1. **Position**: Exit port of segment A must match entry port of segment B exactly
2. **Direction**: Directions must be opposite (A.exit.direction ≈ -B.entry.direction)
3. **Slope Limit**: Angle between directions must be < 30° to prevent harsh transitions
4. **Radius Match**: Radii must match, or use a transition segment (narrowing/widening)
5. **Channel Type**: Must match, or use explicit transition segment

### Seamless Blending

At connection boundaries, use `smooth_min` with a blend zone:

```rust
fn blend_segments(p: Vec3, seg_a: &Segment, seg_b: &Segment, blend_dist: f32) -> f32 {
    let dist_a = seg_a.sdf(p);
    let dist_b = seg_b.sdf(p);
    smooth_min(dist_a, dist_b, blend_dist)
}
```

## Segment Types (12)

### 1. Spiral Tube (Current)
- **Shape**: Helical descent, fully enclosed tube
- **Ports**: 1 entry (top), 1 exit (bottom)
- **Parameters**: loops, descent_height, radius, tube_radius
- **SDF**: Capsule chain with smooth-min blending

### 2. Funnel
- **Shape**: Wide cone narrowing to tube
- **Ports**: 1 entry (wide top), 1 exit (narrow bottom)
- **Parameters**: entry_radius, exit_radius, height, wall_thickness
- **SDF**: Cone SDF with hollow interior
- **Use**: Collect scattered marbles, dramatic entry points

### 3. Half Pipe
- **Shape**: U-shaped channel, open top
- **Ports**: 1 entry, 1 exit (can curve)
- **Parameters**: width, wall_height, curve_path
- **SDF**: Difference of two cylinders (outer - inner), cut by plane
- **Use**: Marbles can jump out, allows side-swinging

### 4. Flat Slope
- **Shape**: Flat angled surface with side walls
- **Ports**: 1 entry (top), 1 exit (bottom)
- **Parameters**: width, length, slope_angle, wall_height
- **SDF**: Box SDF with plane intersection
- **Use**: Simple acceleration sections, transitions

### 5. Narrowing Channel
- **Shape**: Tube that gradually reduces radius
- **Ports**: 1 entry (wide), 1 exit (narrow)
- **Parameters**: entry_radius, exit_radius, length
- **SDF**: Tapered capsule/cone hybrid
- **Use**: Transition between different tube sizes, speed up marbles

### 6. Widening Channel
- **Shape**: Tube that gradually increases radius
- **Ports**: 1 entry (narrow), 1 exit (wide)
- **Parameters**: entry_radius, exit_radius, length
- **SDF**: Inverse of narrowing
- **Use**: Slow down marbles, prepare for funnel/bowl

### 7. Fork (Splitter)
- **Shape**: One channel splits into two diverging paths
- **Ports**: 1 entry, 2 exits (left/right)
- **Parameters**: entry_radius, exit_radius, split_angle, divider_length
- **SDF**: Union of two tapered tubes with central wedge
- **Use**: Create parallel paths, marble separation

### 8. Merge (Combiner)
- **Shape**: Two channels join into one
- **Ports**: 2 entries (left/right), 1 exit
- **Parameters**: Same as fork but reversed
- **SDF**: Same as fork (symmetric)
- **Use**: Bring paths back together, marble collisions

### 9. Loop-de-Loop
- **Shape**: Vertical circular loop
- **Ports**: 1 entry, 1 exit (same height, opposite sides)
- **Parameters**: loop_radius, tube_radius
- **SDF**: Torus section SDF
- **Use**: Dramatic feature, requires speed to complete

### 10. Corkscrew
- **Shape**: Tight helical spiral (tighter than spiral tube)
- **Ports**: 1 entry, 1 exit
- **Parameters**: turns, height, helix_radius, tube_radius
- **SDF**: Dense capsule helix
- **Use**: Rapid descent, visual interest

### 11. Jump Ramp
- **Shape**: Upward ramp followed by gap, then landing ramp
- **Ports**: 1 entry, 1 exit (with gap between)
- **Parameters**: ramp_angle, gap_distance, landing_angle
- **SDF**: Two separate box/ramp SDFs (NOT connected during gap)
- **Use**: Airtime, dramatic jumps, skill element

### 12. Bowl/Basin
- **Shape**: Circular depression marbles spiral around
- **Ports**: 1 entry (rim), 1 exit (center bottom drain)
- **Parameters**: bowl_radius, depth, drain_radius
- **SDF**: Sphere section SDF (inverted dome)
- **Use**: Marbles swirl and collect, dramatic gathering point

## Bounding Box Optimization

### Segment Bounding Boxes

Each segment computes an AABB (Axis-Aligned Bounding Box):

```rust
struct Segment {
    sdf: Box<dyn Fn(Vec3) -> f32>,
    bounds: AABB,
    entry_port: Port,
    exit_ports: Vec<Port>,
}

struct AABB {
    min: Vec3,
    max: Vec3,
}

impl AABB {
    fn contains(&self, point: Vec3) -> bool {
        point.x >= self.min.x && point.x <= self.max.x &&
        point.y >= self.min.y && point.y <= self.max.y &&
        point.z >= self.min.z && point.z <= self.max.z
    }

    fn expanded(&self, margin: f32) -> AABB {
        AABB {
            min: self.min - Vec3::splat(margin),
            max: self.max + Vec3::splat(margin),
        }
    }
}
```

### Collision Detection Acceleration

```rust
fn track_sdf_optimized(p: Vec3, segments: &[Segment]) -> f32 {
    let mut min_dist = f32::MAX;

    for segment in segments {
        // Skip segments whose bounding box is far away
        let expanded_bounds = segment.bounds.expanded(min_dist);
        if !expanded_bounds.contains(p) {
            continue;
        }

        let dist = (segment.sdf)(p);
        min_dist = smooth_min(min_dist, dist, SMOOTH_K);
    }

    min_dist
}
```

### Spatial Partitioning (Future)

For many segments, use a BVH (Bounding Volume Hierarchy) or grid:

```rust
struct TrackGrid {
    cell_size: f32,
    cells: HashMap<(i32, i32, i32), Vec<usize>>,  // segment indices
}
```

## Track Assembly

### Track Builder API

```rust
let track = TrackBuilder::new()
    .add(Funnel::new(radius: 3.0, exit_radius: 1.0, height: 2.0))
    .add(SpiralTube::new(loops: 2, descent: 10.0))
    .add(Fork::new(split_angle: 30.0))
    .branch(0, |b| b
        .add(HalfPipe::curve_left(radius: 5.0))
        .add(Bowl::new(radius: 4.0))
    )
    .branch(1, |b| b
        .add(LoopDeLoop::new(radius: 3.0))
        .add(Corkscrew::new(turns: 3))
    )
    .merge()
    .add(JumpRamp::new(gap: 2.0))
    .add(FlatSlope::new(angle: 15.0, length: 5.0))
    .build();
```

### Validation

On build, validate:
1. All ports connected (no dangling exits except final)
2. Slope angles within limits
3. Radius compatibility
4. No self-intersection (bounding box check)

## Implementation Phases

### Phase 1: Core Infrastructure
- [ ] Port and Segment traits
- [ ] AABB computation
- [ ] Basic track container with multiple segments
- [ ] Optimized SDF evaluation with bounding boxes

### Phase 2: Basic Segments
- [ ] Straight tube
- [ ] Curved tube
- [ ] Flat slope
- [ ] Half pipe

### Phase 3: Transition Segments
- [ ] Narrowing/widening
- [ ] Tube-to-halfpipe transition
- [ ] Funnel

### Phase 4: Complex Segments
- [ ] Fork/merge
- [ ] Loop-de-loop
- [ ] Corkscrew
- [ ] Bowl

### Phase 5: Special Segments
- [ ] Jump ramp
- [ ] Custom/procedural segments

### Phase 6: Procedural Generation
- [ ] Weight tables and track categories
- [ ] Generation algorithm
- [ ] Constraint validation
- [ ] Category presets

## Procedural Track Generation

### Overview

Generate random tracks by selecting segments based on weighted probabilities, with support for themed "track categories" that dramatically shift the distribution.

### Generation Parameters

```rust
struct GenerationConfig {
    min_segments: u32,          // Minimum number of segments
    max_segments: u32,          // Maximum number of segments
    min_total_descent: f32,     // Minimum vertical drop
    max_total_descent: f32,     // Maximum vertical drop
    category: TrackCategory,    // Theme affecting probabilities
    seed: Option<u64>,          // RNG seed for reproducibility
    allow_forks: bool,          // Whether branching paths are allowed
    max_fork_depth: u32,        // How many nested forks allowed
}
```

### Base Segment Weights

Default probability weights (before category modifiers):

| Segment | Base Weight | Rarity | Notes |
|---------|-------------|--------|-------|
| Straight Tube | 100 | Common | Bread and butter |
| Curved Tube | 80 | Common | Direction changes |
| Flat Slope | 70 | Common | Simple acceleration |
| Spiral Tube | 40 | Moderate | Interesting but long |
| Half Pipe | 35 | Moderate | Opens up gameplay |
| Narrowing | 30 | Moderate | Transition piece |
| Widening | 30 | Moderate | Transition piece |
| Corkscrew | 20 | Uncommon | Visually striking |
| Funnel | 15 | Uncommon | Usually at start |
| Fork | 15 | Uncommon | Adds complexity |
| Merge | 15 | Uncommon | Must pair with fork |
| Bowl | 10 | Rare | Dramatic setpiece |
| Loop-de-Loop | 8 | Rare | Requires speed |
| Jump Ramp | 5 | Very Rare | High risk segment |

### Track Categories

Each category applies multipliers to base weights:

#### 1. Classic (Default)
Balanced mix of everything.
```rust
multipliers: all 1.0x
```

#### 2. Funnel Frenzy
Heavy emphasis on funnels and bowls - marbles constantly collecting and dispersing.
```rust
Funnel: 5.0x
Bowl: 4.0x
Widening: 3.0x
Narrowing: 2.0x
Fork: 2.0x
```

#### 3. Drag Race
Mostly straight downhill slopes for maximum speed.
```rust
Flat Slope: 5.0x
Straight Tube: 3.0x
Narrowing: 2.0x
Curved Tube: 0.3x
Spiral Tube: 0.1x
Bowl: 0.0x        // Disabled
Fork: 0.0x        // Disabled
```

#### 4. Twister
Lots of spirals, corkscrews, and curves.
```rust
Spiral Tube: 5.0x
Corkscrew: 5.0x
Curved Tube: 3.0x
Loop-de-Loop: 2.0x
Straight Tube: 0.3x
Flat Slope: 0.2x
```

#### 5. Half Pipe Heaven
Open channels where marbles can swing side to side.
```rust
Half Pipe: 5.0x
Bowl: 3.0x
Widening: 2.0x
Funnel: 2.0x
Straight Tube: 0.5x
```

#### 6. Daredevil
High risk segments - loops, jumps, and tight corkscrews.
```rust
Jump Ramp: 5.0x
Loop-de-Loop: 5.0x
Corkscrew: 3.0x
Bowl: 0.5x
Flat Slope: 0.3x
```

#### 7. Labyrinth
Many forks and merges creating a maze-like structure.
```rust
Fork: 5.0x
Merge: 5.0x
Curved Tube: 2.0x
Straight Tube: 0.5x
allow_forks: true (forced)
max_fork_depth: 3
```

#### 8. Scenic Route
Long, gentle segments with varied scenery.
```rust
Spiral Tube: 3.0x
Curved Tube: 2.0x
Half Pipe: 2.0x
Bowl: 2.0x
min_segments: 1.5x
max_segments: 1.5x
```

### Generation Algorithm

```rust
fn generate_track(config: &GenerationConfig) -> Track {
    let mut rng = config.seed.map(|s| StdRng::seed_from_u64(s))
        .unwrap_or_else(StdRng::from_entropy);

    let mut track = TrackBuilder::new();
    let mut current_port: Port = starting_port();
    let mut segment_count = 0;
    let mut total_descent = 0.0;
    let mut open_forks: Vec<ForkState> = vec![];

    // Target segment count
    let target_segments = rng.gen_range(config.min_segments..=config.max_segments);

    while segment_count < target_segments {
        // Get valid next segments (matching port type, radius, slope limits)
        let valid_segments = get_valid_segments(&current_port, &open_forks, &config);

        if valid_segments.is_empty() {
            // Dead end - backtrack or terminate
            break;
        }

        // Apply category weight multipliers
        let weights: Vec<f32> = valid_segments.iter()
            .map(|s| s.base_weight * config.category.multiplier(s.segment_type))
            .collect();

        // Weighted random selection
        let selected = weighted_random_choice(&valid_segments, &weights, &mut rng);

        // Add segment with randomized parameters within valid ranges
        let segment = selected.instantiate(&current_port, &mut rng);
        track.add(segment);

        // Update state
        current_port = segment.exit_port();
        total_descent += segment.descent();
        segment_count += 1;

        // Handle forks
        if segment.is_fork() {
            open_forks.push(ForkState::new(segment));
        }
        if segment.is_merge() && !open_forks.is_empty() {
            open_forks.pop();
        }

        // Early termination if we've descended enough
        if total_descent >= config.max_total_descent {
            break;
        }
    }

    // Close any remaining forks with merges
    while !open_forks.is_empty() {
        track.add(Merge::new());
        open_forks.pop();
    }

    // Add finale segment (bowl, funnel, or flat ending)
    track.add(generate_finale(&config, &mut rng));

    track.build()
}
```

### Constraint Validation

During generation, validate:

```rust
fn get_valid_segments(port: &Port, forks: &[ForkState], config: &Config) -> Vec<SegmentType> {
    ALL_SEGMENTS.iter().filter(|seg| {
        // Port compatibility
        seg.compatible_with_port(port) &&

        // Slope within limits (30° max change)
        seg.entry_slope_compatible(port.direction) &&

        // Radius match or transition
        seg.can_connect_radius(port.radius) &&

        // Fork depth limit
        (!seg.is_fork() || forks.len() < config.max_fork_depth) &&

        // Must have open fork to merge
        (!seg.is_merge() || !forks.is_empty()) &&

        // Category allows this segment (weight > 0)
        config.category.multiplier(seg) > 0.0
    }).collect()
}
```

### Segment Parameter Randomization

Each segment type has randomizable parameters:

```rust
impl SpiralTube {
    fn randomize(rng: &mut impl Rng, port: &Port) -> Self {
        Self {
            loops: rng.gen_range(1.0..=3.0),
            descent_per_loop: rng.gen_range(3.0..=8.0),
            radius: rng.gen_range(4.0..=7.0),
            tube_radius: port.radius, // Match incoming
            direction: if rng.gen_bool(0.5) { Clockwise } else { CounterClockwise },
        }
    }
}
```

### Track Presets (Pre-built Seeds)

Store known-good seeds for curated experiences:

```rust
const PRESET_TRACKS: &[(&str, GenerationConfig)] = &[
    ("Tutorial", GenerationConfig {
        min_segments: 5,
        max_segments: 8,
        category: Classic,
        seed: Some(12345),
        allow_forks: false,
        ..
    }),
    ("Speed Demon", GenerationConfig {
        category: DragRace,
        seed: Some(99999),
        min_total_descent: 50.0,
        ..
    }),
    ("The Gauntlet", GenerationConfig {
        category: Daredevil,
        seed: Some(66666),
        max_segments: 30,
        ..
    }),
];
```

### API Usage

```rust
// Random track with category
let track = generate_track(&GenerationConfig {
    min_segments: 10,
    max_segments: 20,
    category: TrackCategory::TwisterMania,
    ..default()
});

// Reproducible track
let track = generate_track(&GenerationConfig {
    seed: Some(42),
    category: TrackCategory::FunnelFrenzy,
    ..default()
});

// Preset track
let track = generate_track(&PRESET_TRACKS["Speed Demon"]);
```

## Debug Mode (In-Engine Track Builder)

For testing segments and connections without recompiling:

### Activation

```rust
// Launch with debug mode
cargo run -- --debug-track
// Or press F1 in-game to toggle
```

### Debug Controls

| Key | Action |
|-----|--------|
| `F1` | Toggle debug mode |
| `Tab` | Cycle through segment types |
| `Enter` | Place current segment at end of track |
| `Backspace` | Remove last segment |
| `[` / `]` | Adjust current segment parameters (length, radius, etc.) |
| `Arrow Keys` | Rotate/adjust segment orientation |
| `P` | Toggle pause (freeze marbles) |
| `R` | Reset marbles to start |
| `G` | Generate random track with current category |
| `1-8` | Switch track category |
| `F2` | Print current track config to console |

### Debug UI Overlay

When debug mode is active, display:

```
=== DEBUG MODE ===
Segment: [SpiralTube] (Tab to change)
Parameters: loops=2.0, descent=5.0, radius=5.0
Category: [Classic] (1-8 to change)
Segments placed: 7
Total descent: 23.4m

[Enter] Place  [Backspace] Undo  [G] Generate  [P] Pause
```

### Debug Gizmos

- **Green wireframe**: Current segment preview (before placing)
- **Blue spheres**: Port connection points
- **Red lines**: Slope direction at ports
- **Yellow box**: Segment bounding box
- **Orange highlight**: Connection validation errors

### Debug State Resource

```rust
#[derive(Resource, Default)]
struct DebugTrackBuilder {
    enabled: bool,
    paused: bool,
    current_segment_type: SegmentType,
    current_params: SegmentParams,
    current_category: TrackCategory,
    placed_segments: Vec<Box<dyn Segment>>,
    preview_segment: Option<Box<dyn Segment>>,
}
```

### Debug Systems

```rust
fn debug_input_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut debug: ResMut<DebugTrackBuilder>,
    mut track: ResMut<Track>,
) {
    if keyboard.just_pressed(KeyCode::F1) {
        debug.enabled = !debug.enabled;
    }

    if !debug.enabled { return; }

    if keyboard.just_pressed(KeyCode::Tab) {
        debug.current_segment_type = debug.current_segment_type.next();
        debug.update_preview();
    }

    if keyboard.just_pressed(KeyCode::Enter) {
        if let Some(segment) = debug.preview_segment.take() {
            if segment.can_connect_to(track.last_port()) {
                track.add_segment(segment);
                debug.placed_segments.push(segment.clone());
                debug.update_preview();
            }
        }
    }

    if keyboard.just_pressed(KeyCode::Backspace) {
        if let Some(_) = debug.placed_segments.pop() {
            track.remove_last_segment();
            debug.update_preview();
        }
    }
}

fn debug_preview_gizmos(
    mut gizmos: Gizmos,
    debug: Res<DebugTrackBuilder>,
    track: Res<Track>,
) {
    if !debug.enabled { return; }

    if let Some(preview) = &debug.preview_segment {
        // Draw wireframe preview
        preview.draw_debug_gizmos(&mut gizmos, Color::srgba(0.0, 1.0, 0.0, 0.5));

        // Draw connection ports
        let port = track.last_port();
        gizmos.sphere(port.position, 0.2, Color::srgb(0.0, 0.0, 1.0));
        gizmos.ray(port.position, port.direction, Color::srgb(1.0, 0.0, 0.0));
    }
}
```

### Segment Quick-Test Mode

Press `T` to spawn a single isolated segment for testing:

```rust
fn spawn_test_segment(segment_type: SegmentType) {
    // Clear track, spawn just this segment centered at origin
    // Useful for tuning individual segment SDFs
}
```

## Future Considerations

1. **Per-segment physics overrides**: Different friction/restitution for ice, rubber, etc. (save for later)
2. **Difficulty scaling**: Easy/medium/hard variants of categories
3. **Segment variants**: Each segment type could have visual/behavioral variants
