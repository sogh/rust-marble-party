# Modular Track System - Implementation Prompts

Work through these prompts in order to implement the modular track system.

## Phase 1: Core Infrastructure

### Prompt 1.1: Port and Segment Traits
```
Refactor the codebase to support modular track segments.

Create these core types in a new `src/track/mod.rs` module:
- `Port` struct: position (Vec3), direction (Vec3), up (Vec3), radius (f32)
- `AABB` struct: min (Vec3), max (Vec3), with `contains()` and `expanded()` methods
- `Segment` trait with methods:
  - `sdf(&self, point: Vec3) -> f32`
  - `entry_port(&self) -> Port`
  - `exit_ports(&self) -> Vec<Port>`
  - `bounds(&self) -> AABB`
  - `draw_debug_gizmos(&self, gizmos: &mut Gizmos)`

Keep the existing spiral track working while adding this infrastructure.
Export everything from `src/track/mod.rs`.
```

### Prompt 1.2: Track Container
```
Create a `Track` resource that holds multiple segments.

In `src/track/mod.rs`:
- `Track` struct containing `Vec<Box<dyn Segment>>` and pre-computed bounds
- `Track::sdf()` that evaluates all segments with bounding box optimization
- `Track::sdf_gradient()` using the same gradient approach as before
- `Track::add_segment()` that validates port connections
- `Track::last_port()` returns the exit port of the last segment

Update `marble_physics` to use `Track` resource instead of `TrackSpine`.
The track should still work with just one segment (the spiral).
```

### Prompt 1.3: Convert Spiral to Segment
```
Convert the existing spiral tube track into a proper Segment implementation.

Create `src/track/segments/spiral_tube.rs`:
- `SpiralTube` struct with parameters: loops, descent, helix_radius, tube_radius
- Implement `Segment` trait for `SpiralTube`
- Generate spine points internally
- Calculate proper entry/exit ports
- Compute tight AABB bounds

Update setup to create a `Track` with a `SpiralTube` segment.
Everything should work exactly as before but using the new architecture.
```

---

## Phase 2: Basic Segments

### Prompt 2.1: Straight Tube Segment
```
Implement the simplest segment: a straight tube.

Create `src/track/segments/straight_tube.rs`:
- `StraightTube` struct: length, radius, entry_port
- SDF: capsule from entry to exit
- Entry port at start, exit port at end (translated by length along direction)
- Debug gizmos: draw tube outline

Test by building a track with: SpiralTube -> StraightTube
Verify marbles transition smoothly between segments.
```

### Prompt 2.2: Curved Tube Segment
```
Implement a tube that curves in a horizontal arc.

Create `src/track/segments/curved_tube.rs`:
- `CurvedTube` struct: arc_angle (radians), arc_radius, tube_radius, entry_port
- SDF: series of capsules along the arc (like spiral but flat)
- Properly compute exit port position and rotated direction
- Support both left and right curves (positive/negative angle)

Test by building: Straight -> CurvedLeft -> CurvedRight -> Straight
```

### Prompt 2.3: Flat Slope Segment
```
Implement a flat angled slope with side walls.

Create `src/track/segments/flat_slope.rs`:
- `FlatSlope` struct: length, width, slope_angle, wall_height, entry_port
- SDF: box with angled floor, side walls
- Entry at top, exit at bottom of slope
- Ensure marbles stay contained by walls

Test by building: SpiralTube -> FlatSlope -> StraightTube
```

---

## Phase 3: Transitions

### Prompt 3.1: Narrowing/Widening Segments
```
Implement tube segments that change radius.

Create `src/track/segments/transition.rs`:
- `NarrowingTube` struct: length, entry_radius, exit_radius, entry_port
- `WideningTube` (same struct, just entry < exit)
- SDF: tapered capsule/cone hybrid
- Smoothly interpolate radius along length

Test connecting tubes of different radii through transitions.
```

### Prompt 3.2: Funnel Segment
```
Implement a wide cone that narrows to a tube.

Create `src/track/segments/funnel.rs`:
- `Funnel` struct: entry_radius, exit_radius, height, wall_thickness
- SDF: hollow cone
- Entry port is wide (top), exit port is narrow (bottom)
- Marbles should swirl around before funneling down

Test as a track starting segment.
```

---

## Phase 4: Complex Segments

### Prompt 4.1: Half Pipe Segment
```
Implement an open-top U-shaped channel.

Create `src/track/segments/half_pipe.rs`:
- `HalfPipe` struct: length, width, wall_height, curve (optional arc)
- SDF: difference of two cylinders, cut by horizontal plane
- Marbles can swing side-to-side and potentially jump out
- Support straight and curved variants

Test standalone and in combination with tubes.
```

### Prompt 4.2: Bowl/Basin Segment
```
Implement a circular depression where marbles swirl.

Create `src/track/segments/bowl.rs`:
- `Bowl` struct: bowl_radius, depth, drain_radius, entry_port
- SDF: inverted sphere section with drain hole
- Entry at rim (tangent), exit at center bottom
- Marbles should spiral around before draining

Test as a mid-track feature.
```

### Prompt 4.3: Fork and Merge Segments
```
Implement path splitting and joining.

Create `src/track/segments/fork.rs`:
- `Fork` struct: entry_radius, exit_radius, split_angle, entry_port
- Two exit ports (left and right paths)
- SDF: Y-junction with smooth blend
- Central divider to separate marble streams

Create `src/track/segments/merge.rs`:
- Inverse of fork (two entries, one exit)

Test: Straight -> Fork -> (parallel straights) -> Merge -> Straight
```

---

## Phase 5: Debug Mode

### Prompt 5.1: Debug State and Toggle
```
Implement debug mode toggle and state.

Create `src/debug.rs`:
- `DebugTrackBuilder` resource with enabled, paused, current_segment_type, etc.
- F1 to toggle debug mode
- P to pause/unpause marble physics
- When paused, marbles freeze in place

Add debug state to app, integrate with existing systems.
```

### Prompt 5.2: Segment Cycling and Preview
```
Add segment selection and preview in debug mode.

- Tab cycles through available segment types
- Show green wireframe gizmo of segment preview at track end
- Display current segment type name in console (or on screen later)
- Preview updates when cycling or adjusting parameters
```

### Prompt 5.3: Segment Placement and Undo
```
Implement placing and removing segments in debug mode.

- Enter places preview segment (if valid connection)
- Backspace removes last placed segment
- Track rebuilds SDF after each change
- Reset marble positions when track changes
```

### Prompt 5.4: Parameter Adjustment
```
Add real-time parameter adjustment in debug mode.

- [ and ] adjust primary parameter (length, loops, etc.)
- Arrow keys adjust secondary parameters or orientation
- Preview updates in real-time
- Show current parameter values
```

---

## Phase 6: Procedural Generation

### Prompt 6.1: Segment Weights and Categories
```
Implement the weight system for random generation.

Create `src/track/generation.rs`:
- `SegmentType` enum for all segment types
- Base weight table (HashMap<SegmentType, f32>)
- `TrackCategory` enum with 8 categories
- Category multiplier tables
- `get_weighted_segments()` returns valid segments with final weights
```

### Prompt 6.2: Generation Algorithm
```
Implement the track generation algorithm.

In `src/track/generation.rs`:
- `GenerationConfig` struct with all parameters
- `generate_track()` function implementing the algorithm from design doc
- Constraint validation (port compatibility, slope limits)
- Proper fork/merge handling
- Finale segment generation
```

### Prompt 6.3: Debug Mode Integration
```
Add procedural generation to debug mode.

- G key generates random track with current category
- 1-8 keys switch between track categories
- Show current category name
- Seed display/control (optional)
```

---

## Testing Checkpoints

After each phase, verify:
- [ ] Marbles still roll correctly
- [ ] Collisions work (track and marble-marble)
- [ ] No visual glitches in gizmos
- [ ] Performance acceptable
- [ ] R key still resets properly

## File Structure Target

```
src/
├── main.rs              # App setup, core systems
├── marble.rs            # Marble component and physics
├── debug.rs             # Debug mode systems
└── track/
    ├── mod.rs           # Port, AABB, Segment trait, Track resource
    ├── generation.rs    # Procedural generation
    └── segments/
        ├── mod.rs       # Re-exports all segments
        ├── spiral_tube.rs
        ├── straight_tube.rs
        ├── curved_tube.rs
        ├── flat_slope.rs
        ├── transition.rs
        ├── funnel.rs
        ├── half_pipe.rs
        ├── bowl.rs
        └── fork.rs
```
