# Claude Code Context

## Project Overview
This is a Bevy game implementing a marble rolling through a tube track using custom SDF-based physics. No third-party physics engine is used.

## Key Concepts

### SDF (Signed Distance Field)
- `capsule_sdf()` - Distance from point to a line segment with radius
- `smooth_min()` - Polynomial smooth minimum for blending SDFs (avoids harsh transitions)
- `track_sdf()` - Returns **negated** distance so collisions happen on the **inside** of the tube
- `track_sdf_gradient()` - Computes surface normal via central differences

### Track Generation
- `create_track_spine()` - Generates a `Vec<Vec3>` defining the track centerline
- Currently a descending helix with 3 loops and small cosine bumps
- Track uses cosine (not sine) for bumps so it starts downhill

### Physics System
The `marble_physics` system runs each frame:
1. Apply gravity to velocity
2. Predict next position
3. Check SDF penetration (if `marble_radius - sdf_dist > 0`)
4. Compute gradient-based normal
5. Resolve collision: push out + decompose velocity + apply restitution/friction

### Important Constants (top of main.rs)
- `GRAVITY` - Vec3, affects how snappy the ball feels
- `MARBLE_RADIUS` - Ball size, must be smaller than track radius
- `TRACK_RADIUS` - Tube inner radius
- `RESTITUTION` - Bounce factor (0-1, higher = more bounce)
- `FRICTION` - Tangent velocity retention (0.995 = very slippery)

## Common Tasks

### Adjusting Track Shape
Edit `create_track_spine()`. Key parameters:
- `segments` - Number of points (smoothness)
- `angle` multiplier - Number of loops
- `descent` - Total height drop
- `bumps` amplitude - Hill size (keep small to avoid stopping)

### Tuning Physics Feel
Edit constants at top of file:
- Sluggish? Increase `GRAVITY`, increase `FRICTION` toward 1.0
- Too bouncy? Decrease `RESTITUTION`
- Ball stuck? Reduce bump amplitude or increase descent slope

### Adding Track Features
The SDF system supports any geometry. To add obstacles:
1. Create new SDF function (sphere, box, etc.)
2. Blend with `smooth_min()` or `smooth_max()` (for subtractive)
3. Gradient will automatically adjust

## File Structure
```
src/main.rs    # Everything - components, systems, SDF functions, track generation
Cargo.toml     # Bevy 0.15 dependency
```

## Build & Run
```bash
cargo run          # Debug build
cargo run -r       # Release build (faster physics)
```
