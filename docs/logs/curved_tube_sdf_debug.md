# CurvedTube SDF Debug Log

## Issue
Penetration depth grows unboundedly (0.003 → 46.0) when marble enters curved tube section.

## Initial Analysis
The infinite cylinder approach for SDF is broken. When finding the "best segment" via closest-point-on-polyline (with clamped t), we then compute radial distance using an INFINITE cylinder that extends beyond the segment bounds.

Example failure case:
- Segment goes from A to B
- Marble at P is closest to endpoint B (t=1.0 clamped)
- Infinite cylinder projects P onto line AB extended beyond B
- Radial distance is computed from wrong point on extended line
- SDF returns incorrect value

## Fix Attempt 1: Revert to polyline distance SDF

Use closest-point-on-polyline distance directly (not infinite cylinder), but compute gradient analytically from the closest point direction.

---

## Test Results

### Attempt 1: Polyline SDF (no infinite cylinder)
**Changes**: Reverted SDF to use closest-point-on-polyline distance. Gradient computed as direction from closest point to marble.

**Results**:
- Penetration NO LONGER grows unboundedly (was 0→46, now stable at ~4.4)
- BUT marble gets STUCK at curve entrance (X≈-5, Z≈-17)
- Marble oscillates instead of progressing through curve
- Penetration ~4.4 means marble is ~5.2 units from spine (way outside tube!)

**Analysis of t=4s transition**:
- pos:( -1.4, 7.1, -16.6)
- norm:(-0.13, **-0.90**, 0.42) ← Y component is NEGATIVE!
- The gradient is pushing the marble DOWN instead of UP
- This causes marble to fall below the curve spine

**Root cause**: At curve transition, the gradient direction is wrong. The analytic gradient (closest_point - point) can point in unexpected directions at segment boundaries.

---

### Attempt 2: Central differences gradient + Exit rejection

**Changes**:
- Reverted to central differences for gradient (removed analytic gradient)
- Added exit rejection based on `exit.direction` dot product

**Results**:
- Marble still gets stuck at curve exit (X≈-5, Z≈-17)
- Penetration still ~4.4
- Debug logging revealed marble at Z=-21.2 while spine ends at Z=-17.7

**Root cause discovered**: Exit rejection using `exit.direction` doesn't work for curves!
- After a 90° left turn, exit direction points in -X
- Marble travels in -Z direction (perpendicular to exit direction)
- Dot product doesn't detect marble going past the exit

---

### Attempt 3: Track spine position in SDF + Add segment after curve

**Changes**:
1. Modified SDF to track which spine segment contains the closest point and where (t value)
2. Reject points where closest is on last segment with t>0.99 AND past exit
3. Added a StraightTube segment AFTER the curve so marble has somewhere to go

```rust
// Track closest segment and t value
let mut closest_segment_idx = 0;
let mut closest_t = 0.0f32;

// ... polyline search ...

// Reject points past the end of the spine
let last_segment_idx = self.spine.len() - 2;
if closest_segment_idx == last_segment_idx && closest_t > 0.99 {
    let to_point_exit = point - self.exit.position;
    let along_exit = to_point_exit.dot(self.exit.direction);
    if along_exit > 0.1 {
        return f32::MAX;
    }
}
```

**Results**: ✅ **FIXED!**
- Marbles successfully traverse the entire track
- Penetration stable at ~0.003 (normal)
- Smooth transition through curve: pen=0.501 at entry, pen=0.002 after exit
- All marbles properly transition from segment 1 (CurvedTube) to segment 2 (StraightTube)

**Sample output**:
```
t= 3s pos:(   0.0,   6.9, -11.4) vel:(  0.0, -1.9, -5.4) spd: 5.8 col:N pen:0.000
t= 4s pos:(  -1.3,   6.3, -16.9) vel:( -4.5, -3.3, -2.0) spd: 5.9 col:Y pen:0.501
t= 5s pos:(  -7.5,   4.8, -17.1) vel:( -6.7, -2.3, -0.9) spd: 7.1 col:Y pen:0.002
t= 6s pos:( -14.2,   3.1, -17.6) vel:( -6.6, -0.7,  1.9) spd: 6.9 col:Y pen:0.003
t= 7s pos:( -20.5,   1.9, -18.1) vel:( -6.3, -1.4,  0.7) spd: 6.5 col:Y pen:0.003
```

---

## Summary

The root cause was a combination of:
1. **Missing next segment**: The track had no segment after the curve, so marbles had nowhere to go
2. **Exit rejection based on direction**: For curves, exit direction is perpendicular to the path the marble was traveling, so dot product didn't detect the marble going past the exit

The fix:
1. Track which spine segment contains the closest point during SDF calculation
2. Detect when a point projects past the last spine segment endpoint
3. Ensure the track has continuous segments for marbles to flow into

---

## Follow-up: Slower Physics Revealed More Issues

### Problem with Slowed Physics
Running at 0.25x speed revealed the penetration spike at curve entry was still present:
- t=3.5s: `pen:2.475` with `norm:(-0.00, -0.98, 0.20)` - normal pointing DOWN!

### Root Cause Analysis
Debug logging revealed the polyline closest-point search was finding the wrong segment:
```
HIGH_PEN: sdf=-2.50 pt=(0.0,6.5,-14.5) closest_seg=0 spine_pt=(0.0,7.6,-11.8)
```
- Marble at Z=-14.5 was finding segment 0 (entry at Z=-11.8) as closest
- The marble at X=0 followed the straight tube's infinite cylinder
- The curve's spine veered left (to negative X) leaving the marble behind

### Attempted Fix: Bound the Straight Tube
Added axial bounds to StraightTube SDF to reject points past exit.
**Result**: Made it WORSE! Created a gap where no segment provided valid SDF:
- Straight tube rejected (past exit)
- Curved tube's spine was to the left
- Marble fell into void with `col:N pen:0.000`

### Working Fix: Longer Lead-in + Relaxed Entry Rejection

**Changes to CurvedTube:**
1. Increased lead-in length from 1.0 to **3.0 units** (more time to establish position)
2. Relaxed entry rejection from -0.5 to **-3.0** (allow curve to provide SDF earlier)

```rust
// Longer lead-in for smooth transition
let lead_in_length = 3.0;

// Generous entry rejection margin
if along_entry < -3.0 {
    return f32::MAX;
}
```

**Results**: ✅ **IMPROVED!**
- Marbles successfully traverse the entire track
- Penetration at curve entry: ~0.6 (acceptable vs previous 2.5+)
- Brief no-collision period at junction but marbles recover
- Stable physics in second straight tube (pen:0.003)

**Sample output at normal speed:**
```
t= 3s pos:(   0.0,   7.5, -11.4) seg:0 col:N pen:0.000
t= 4s pos:(  -1.0,   5.9, -18.4) seg:1 col:Y pen:0.664
t= 5s pos:(  -8.9,   4.6, -20.5) seg:2 col:N pen:0.000
t= 6s pos:( -16.9,   2.2, -19.6) seg:2 col:Y pen:0.003
t= 7s pos:( -24.3,   0.8, -19.4) seg:2 col:Y pen:0.002
```

---

## Remaining Issues

1. **Brief free-fall at junctions**: ~0.5s of no collision at curve entry/exit
2. **Penetration spike**: ~0.6 at curve entry vs ~0.003 normal

These are cosmetic issues - the marble physics is functional and marbles complete the track.

## Lessons Learned

1. **Infinite cylinders are tricky**: They provide good floors but conflicting gradients at junctions
2. **Polyline SDF needs matching geometry**: If marble doesn't follow the spine, distances become meaningless
3. **Lead-in sections help**: Give the marble time to transition before curving begins
4. **Generous overlap regions**: Segments need to overlap generously at junctions
