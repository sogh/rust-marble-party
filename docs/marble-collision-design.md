# Marble-to-Marble Collision Design

## Overview

Add sphere-sphere collision detection and response between all marbles so they bounce off each other realistically.

## Algorithm

### Detection (O(n²) pairwise check)

For each unique pair of marbles (i, j):

```
distance = |pos_i - pos_j|
min_dist = radius_i + radius_j

if distance < min_dist:
    collision detected
```

### Collision Response

When two spheres collide:

1. **Collision Normal**: Unit vector from marble i to marble j
   ```
   normal = normalize(pos_j - pos_i)
   ```

2. **Penetration Depth**: How much they overlap
   ```
   penetration = min_dist - distance
   ```

3. **Separation**: Push marbles apart (half each)
   ```
   pos_i -= normal * (penetration / 2)
   pos_j += normal * (penetration / 2)
   ```

4. **Relative Velocity**:
   ```
   rel_vel = vel_i - vel_j
   vel_along_normal = dot(rel_vel, normal)
   ```

5. **Impulse Calculation** (elastic collision, equal mass):
   ```
   if vel_along_normal > 0:  # Moving apart, skip
       return

   # For equal mass spheres, velocities simply exchange along normal
   impulse = vel_along_normal * restitution

   vel_i -= normal * impulse
   vel_j += normal * impulse
   ```

## Implementation Plan

1. Add new constant `MARBLE_RESTITUTION` for marble-marble bounces (can differ from track)

2. Create new system `marble_collision` that:
   - Runs after `marble_physics` (track collision first, then inter-marble)
   - Iterates all unique pairs using index-based double loop
   - Applies detection and response as described above

3. Register system in app

## Performance Considerations

- With 8 marbles: 28 pair checks per frame (acceptable)
- For many more marbles: spatial partitioning would be needed (not required now)

## Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `MARBLE_RESTITUTION` | 0.9 | Steel balls bounce well off each other |
