use bevy::prelude::*;

// ============================================================================
// CONSTANTS
// ============================================================================

// Steel ball bearing physics
const GRAVITY: Vec3 = Vec3::new(0.0, -20.0, 0.0); // Stronger gravity for snappier feel
const MARBLE_RADIUS: f32 = 0.2; // Smaller ball bearing
const TRACK_RADIUS: f32 = 1.0; // Tighter tube
const SMOOTH_K: f32 = 0.5;
const RESTITUTION: f32 = 0.6; // Steel bounces well
const FRICTION: f32 = 0.995; // Steel on metal is slippery
const GRADIENT_EPSILON: f32 = 0.01;

// ============================================================================
// COMPONENTS
// ============================================================================

#[derive(Component)]
struct Marble {
    velocity: Vec3,
    radius: f32,
}

#[derive(Resource)]
struct TrackSpine {
    points: Vec<Vec3>,
    radius: f32,
}

impl Default for TrackSpine {
    fn default() -> Self {
        Self {
            points: create_track_spine(),
            radius: TRACK_RADIUS,
        }
    }
}

// ============================================================================
// SDF FUNCTIONS
// ============================================================================

/// Signed distance from point `p` to a capsule defined by endpoints `a` and `b` with radius `r`
fn capsule_sdf(p: Vec3, a: Vec3, b: Vec3, r: f32) -> f32 {
    let pa = p - a;
    let ba = b - a;
    let h = (pa.dot(ba) / ba.dot(ba)).clamp(0.0, 1.0);
    (pa - ba * h).length() - r
}

/// Smooth minimum for blending SDFs
/// k controls the smoothness of the blend (larger = smoother)
fn smooth_min(a: f32, b: f32, k: f32) -> f32 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    b.lerp(a, h) - k * h * (1.0 - h)
}

/// Calculate the distance from a point to the INSIDE of the track tube
/// Returns negative when inside the tube (no collision), positive when penetrating the wall
fn track_sdf(p: Vec3, spine: &[Vec3], radius: f32) -> f32 {
    if spine.len() < 2 {
        return f32::MAX;
    }

    let mut dist = capsule_sdf(p, spine[0], spine[1], radius);

    for i in 1..spine.len() - 1 {
        let segment_dist = capsule_sdf(p, spine[i], spine[i + 1], radius);
        dist = smooth_min(dist, segment_dist, SMOOTH_K);
    }

    // Negate to make collision happen on the INSIDE of the tube
    // Now: negative = inside tube (free space), positive = outside/in wall
    -dist
}

/// Calculate the gradient (normal) of the SDF at point `p` using central differences
fn track_sdf_gradient(p: Vec3, spine: &[Vec3], radius: f32) -> Vec3 {
    let eps = GRADIENT_EPSILON;

    let dx = track_sdf(p + Vec3::X * eps, spine, radius)
        - track_sdf(p - Vec3::X * eps, spine, radius);
    let dy = track_sdf(p + Vec3::Y * eps, spine, radius)
        - track_sdf(p - Vec3::Y * eps, spine, radius);
    let dz = track_sdf(p + Vec3::Z * eps, spine, radius)
        - track_sdf(p - Vec3::Z * eps, spine, radius);

    Vec3::new(dx, dy, dz).normalize_or_zero()
}

// ============================================================================
// TRACK GENERATION
// ============================================================================

/// Create a fun roller-coaster-like track spine with consistent downward slope
fn create_track_spine() -> Vec<Vec3> {
    let mut points = Vec::new();
    let segments = 150;

    for i in 0..=segments {
        let t = i as f32 / segments as f32;
        let angle = t * std::f32::consts::TAU * 3.0; // 3 full loops

        // Spiral with varying radius
        let radius = 5.0 + 1.5 * (angle * 0.5).sin();
        let x = radius * angle.cos();
        let z = radius * angle.sin();

        // Descending helix with some bumps for fun
        let descent = 30.0 * (1.0 - t); // Steeper slope
        let bumps = 0.5 * (angle * 2.0).cos(); // Small ripples, won't stop the marble
        let y = descent + bumps + 2.0;

        points.push(Vec3::new(x, y, z));
    }

    points
}

// ============================================================================
// SYSTEMS
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Initialize track resource
    commands.insert_resource(TrackSpine::default());

    // Spawn marble at the start of the track spine (inside the tube)
    let start_pos = create_track_spine()[0];
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(MARBLE_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.2, 0.2),
            metallic: 0.7,
            perceptual_roughness: 0.3,
            ..default()
        })),
        Transform::from_translation(start_pos),
        Marble {
            velocity: Vec3::ZERO,
            radius: MARBLE_RADIUS,
        },
    ));

    // Spawn camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(15.0, 12.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Spawn directional light
    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ambient light
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 300.0,
    });
}

/// Physics system: apply gravity, check SDF penetration, resolve collision
fn marble_physics(
    time: Res<Time>,
    track: Res<TrackSpine>,
    mut query: Query<(&mut Transform, &mut Marble)>,
) {
    let dt = time.delta_secs();

    for (mut transform, mut marble) in query.iter_mut() {
        // Apply gravity
        marble.velocity += GRAVITY * dt;

        // Predict next position
        let next_pos = transform.translation + marble.velocity * dt;

        // Check SDF penetration
        let dist = track_sdf(next_pos, &track.points, track.radius);
        let penetration = marble.radius - dist;

        if penetration > 0.0 {
            // Calculate normal via gradient
            let normal = track_sdf_gradient(next_pos, &track.points, track.radius);

            // Resolve penetration by pushing marble out
            let corrected_pos = next_pos + normal * penetration;

            // Decompose velocity into normal and tangent components
            let vel_normal = normal * marble.velocity.dot(normal);
            let vel_tangent = marble.velocity - vel_normal;

            // Apply restitution (bounce) to normal component
            // Apply friction to tangent component
            marble.velocity = vel_tangent * FRICTION - vel_normal * RESTITUTION;

            transform.translation = corrected_pos;
        } else {
            transform.translation = next_pos;
        }

        // Rolling rotation based on velocity
        if marble.velocity.length() > 0.01 {
            let speed = marble.velocity.length();
            let angular_speed = speed / marble.radius;
            let rotation_axis = Vec3::Y.cross(marble.velocity.normalize()).normalize_or_zero();

            if rotation_axis.length() > 0.1 {
                transform.rotate(Quat::from_axis_angle(rotation_axis, angular_speed * dt));
            }
        }
    }
}

/// Visualize the track spine using Gizmos
fn draw_track_gizmos(mut gizmos: Gizmos, track: Res<TrackSpine>) {
    let points = &track.points;

    // Draw the spine as a polyline
    for i in 0..points.len() - 1 {
        gizmos.line(points[i], points[i + 1], Color::srgb(0.2, 0.8, 0.2));
    }

    // Draw circles at each point to show the track radius
    for (i, &point) in points.iter().enumerate() {
        if i % 5 == 0 {
            // Draw direction indicator
            if i + 1 < points.len() {
                let dir = (points[i + 1] - point).normalize_or_zero();
                let right = dir.cross(Vec3::Y).normalize_or_zero();
                let up = right.cross(dir).normalize_or_zero();

                // Draw a circle approximation for the track cross-section
                let segments = 16;
                for j in 0..segments {
                    let angle1 = (j as f32 / segments as f32) * std::f32::consts::TAU;
                    let angle2 = ((j + 1) as f32 / segments as f32) * std::f32::consts::TAU;

                    let p1 = point + (right * angle1.cos() + up * angle1.sin()) * track.radius;
                    let p2 = point + (right * angle2.cos() + up * angle2.sin()) * track.radius;

                    gizmos.line(p1, p2, Color::srgb(0.5, 0.5, 1.0));
                }
            }

            // Draw point markers
            gizmos.sphere(
                Isometry3d::from_translation(point),
                0.1,
                Color::srgb(1.0, 1.0, 0.0),
            );
        }
    }
}

/// Camera follow system
fn camera_follow(
    marble_query: Query<&Transform, With<Marble>>,
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<Marble>)>,
    time: Res<Time>,
) {
    if let (Ok(marble_transform), Ok(mut camera_transform)) =
        (marble_query.get_single(), camera_query.get_single_mut())
    {
        let target_pos = marble_transform.translation + Vec3::new(10.0, 8.0, 10.0);
        let lerp_speed = 2.0 * time.delta_secs();

        camera_transform.translation = camera_transform.translation.lerp(target_pos, lerp_speed);
        camera_transform.look_at(marble_transform.translation, Vec3::Y);
    }
}

/// Reset marble if it falls too far
fn reset_marble(mut query: Query<(&mut Transform, &mut Marble)>, track: Res<TrackSpine>) {
    for (mut transform, mut marble) in query.iter_mut() {
        if transform.translation.y < -20.0 {
            transform.translation = track.points[0];
            marble.velocity = Vec3::ZERO;
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                marble_physics,
                draw_track_gizmos,
                camera_follow,
                reset_marble,
            ),
        )
        .run();
}
