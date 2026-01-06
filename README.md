# Rust Marble Party

A Bevy-based marble physics simulation with custom SDF (Signed Distance Field) collision detection. Watch a steel ball bearing roll through a spiraling tube track!

## Features

- **Custom Physics Solver** - No third-party physics engine; hand-rolled collision detection and response
- **SDF-Based Track** - Track defined as a spine of points, with capsule SDFs and smooth-minimum blending
- **Gradient-Based Normals** - Surface normals computed via central differences for accurate collision response
- **Steel Ball Bearing Physics** - Tuned for realistic metal-on-metal feel with proper restitution and friction
- **Gizmos Visualization** - See the track spine and tube cross-sections rendered with Bevy Gizmos

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

- The marble starts automatically and rolls down the track
- Camera follows the marble
- If the marble falls off, it resets to the start

## Dependencies

- [Bevy 0.15](https://bevyengine.org/) - Game engine
