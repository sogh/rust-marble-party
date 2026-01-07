# Rust Marble Party

A Bevy-based marble physics simulation with custom SDF (Signed Distance Field) collision detection. Watch a steel ball bearing roll through a spiraling tube track!

## Features

- **Custom Physics Solver** - No third-party physics engine; hand-rolled collision detection and response
- **SDF-Based Track** - Track defined as a spine of points, with capsule SDFs and smooth-minimum blending
- **Gradient-Based Normals** - Surface normals computed via central differences for accurate collision response
- **Steel Ball Bearing Physics** - Tuned for realistic metal-on-metal feel with proper restitution and friction
- **Gizmos Visualization** - See the track spine and tube cross-sections rendered with Bevy Gizmos
- **Modular Track Segments** - 10 different segment types that connect via ports
- **Build Mode** - Add and remove segments interactively with keyboard controls
- **Procedural Generation** - Generate random tracks with weighted segment selection
- **Debug UI** - Real-time display of FPS, physics speed, marble state, and collision info

## Running

```bash
cargo run
```

## How It Works

### Track Definition
The track is defined as a `Vec<Vec3>` spine - a series of 3D points forming a descending helix. Each segment between points creates a capsule SDF, and all capsules are blended together using smooth-minimum for seamless transitions.

### SDF Collision
The signed distance field returns:
- **Negative values** when inside the tube (free space)
- **Positive values** when penetrating the tube wall

When the marble's position + radius exceeds the SDF distance, a collision is detected.

### Collision Response
1. Compute surface normal via SDF gradient (central differences)
2. Push marble out of penetration
3. Decompose velocity into normal and tangent components
4. Apply restitution (bounce) to normal component
5. Apply friction to tangent component

### Physics Constants
```rust
GRAVITY: -20.0        // Snappy acceleration
MARBLE_RADIUS: 0.2    // Small ball bearing
TRACK_RADIUS: 1.0     // Tube inner radius
RESTITUTION: 0.6      // Steel bounce
FRICTION: 0.995       // Slippery steel-on-metal
```

## Controls

### General
| Key | Action |
|-----|--------|
| `R` | Restart all marbles at start position |
| `[` | Decrease physics speed (0.5x) |
| `]` | Increase physics speed (2x) |
| `\` | Reset physics speed to 1x |

### Procedural Generation
| Key | Action |
|-----|--------|
| `G` | Generate new random track (increments seed) |

### Build Mode
| Key | Action |
|-----|--------|
| `B` | Toggle build mode on/off |
| `1` | Select StraightTube |
| `2` | Select CurvedLeft |
| `3` | Select CurvedRight |
| `4` | Select NarrowingTube |
| `5` | Select WideningTube |
| `6` | Select FlatSlope |
| `7` | Select HalfPipe |
| `Enter` | Add selected segment to track |
| `Backspace` | Remove last segment from track |

### Automatic Behavior
- Camera follows the red marble (marble 0)
- Marbles reset to start if they fall below y=-20
- Debug UI shows FPS, physics speed, segment count, and red marble info

## Track Segments

| Segment | Description |
|---------|-------------|
| SpiralTube | Helical descent tube |
| StraightTube | Linear tube section |
| CurvedTube | Horizontal arc turn |
| FlatSlope | Open channel with walls |
| NarrowingTube | Tube that tapers smaller |
| WideningTube | Tube that expands larger |
| HalfPipe | Open half-cylinder |
| Funnel | Vertical cone (unused in builder) |
| Bowl | Spherical depression (unused in builder) |
| Fork | Y-splitter with two exits (unused in builder) |

## Dependencies

- [Bevy 0.15](https://bevyengine.org/) - Game engine
- [rand 0.8](https://crates.io/crates/rand) - Random number generation
- [rand_chacha 0.3](https://crates.io/crates/rand_chacha) - Deterministic RNG
