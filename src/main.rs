use bevy::prelude::*;

mod track;
use track::{Track, SpiralTube, Segment};

// ============================================================================
// CONSTANTS
// ============================================================================

// Steel ball bearing physics
const GRAVITY: Vec3 = Vec3::new(0.0, -20.0, 0.0);
const MARBLE_RADIUS: f32 = 0.2;
const TRACK_RADIUS: f32 = 1.0;
const RESTITUTION: f32 = 0.6;
const MARBLE_RESTITUTION: f32 = 0.9;
const FRICTION: f32 = 0.995;

// ============================================================================
// COMPONENTS
// ============================================================================

#[derive(Component)]
struct Marble {
    velocity: Vec3,
    radius: f32,
}

// ============================================================================
// SYSTEMS
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create track with spiral tube segment
    let mut track = Track::new();
    let spiral = SpiralTube::at_origin(3.0, 30.0, 5.0, TRACK_RADIUS);

    // Get starting positions before moving spiral into track
    let spine = spiral.spine().to_vec();

    track.add_segment(Box::new(spiral));
    commands.insert_resource(track);

    // Spawn 8 marbles with different colors
    let marble_colors = [
        Color::srgb(0.9, 0.1, 0.1), // Red
        Color::srgb(0.1, 0.7, 0.1), // Green
        Color::srgb(0.1, 0.3, 0.9), // Blue
        Color::srgb(0.9, 0.9, 0.1), // Yellow
        Color::srgb(0.9, 0.1, 0.9), // Magenta
        Color::srgb(0.1, 0.9, 0.9), // Cyan
        Color::srgb(1.0, 0.5, 0.0), // Orange
        Color::srgb(0.8, 0.8, 0.8), // Silver
    ];

    let mesh = meshes.add(Sphere::new(MARBLE_RADIUS));
    for (i, color) in marble_colors.iter().enumerate() {
        let start_pos = spine[i * 2];
        commands.spawn((
            Mesh3d(mesh.clone()),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: *color,
                metallic: 0.9,
                perceptual_roughness: 0.1,
                ..default()
            })),
            Transform::from_translation(start_pos),
            Marble {
                velocity: Vec3::ZERO,
                radius: MARBLE_RADIUS,
            },
        ));
    }

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(15.0, 12.0, 15.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Directional light
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
    track: Res<Track>,
    mut query: Query<(&mut Transform, &mut Marble)>,
) {
    let dt = time.delta_secs();

    for (mut transform, mut marble) in query.iter_mut() {
        // Apply gravity
        marble.velocity += GRAVITY * dt;

        // Predict next position
        let next_pos = transform.translation + marble.velocity * dt;

        // Check SDF penetration using new Track system
        let dist = track.sdf(next_pos);
        let penetration = marble.radius - dist;

        if penetration > 0.0 {
            // Calculate normal via gradient
            let normal = track.sdf_gradient(next_pos);

            // Resolve penetration
            let corrected_pos = next_pos + normal * penetration;

            // Decompose velocity
            let vel_normal = normal * marble.velocity.dot(normal);
            let vel_tangent = marble.velocity - vel_normal;

            // Apply restitution and friction
            marble.velocity = vel_tangent * FRICTION - vel_normal * RESTITUTION;
            transform.translation = corrected_pos;
        } else {
            transform.translation = next_pos;
        }

        // Rolling rotation
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

/// Marble-to-marble collision
fn marble_collision(mut query: Query<(Entity, &mut Transform, &mut Marble)>) {
    let mut marbles: Vec<(Entity, Vec3, Vec3, f32)> = query
        .iter()
        .map(|(e, t, m)| (e, t.translation, m.velocity, m.radius))
        .collect();

    let count = marbles.len();

    for i in 0..count {
        for j in (i + 1)..count {
            let (_, pos_i, vel_i, radius_i) = marbles[i];
            let (_, pos_j, vel_j, radius_j) = marbles[j];

            let diff = pos_j - pos_i;
            let distance = diff.length();
            let min_dist = radius_i + radius_j;

            if distance < min_dist && distance > 0.0 {
                let normal = diff / distance;
                let penetration = min_dist - distance;

                let separation = normal * (penetration / 2.0);
                marbles[i].1 -= separation;
                marbles[j].1 += separation;

                let rel_vel = vel_i - vel_j;
                let vel_along_normal = rel_vel.dot(normal);

                if vel_along_normal > 0.0 {
                    let impulse = normal * (vel_along_normal * MARBLE_RESTITUTION);
                    marbles[i].2 -= impulse;
                    marbles[j].2 += impulse;
                }
            }
        }
    }

    for (entity, new_pos, new_vel, _) in marbles {
        if let Ok((_, mut transform, mut marble)) = query.get_mut(entity) {
            transform.translation = new_pos;
            marble.velocity = new_vel;
        }
    }
}

/// Draw track using segment gizmos
fn draw_track_gizmos(mut gizmos: Gizmos, track: Res<Track>) {
    for segment in track.segments() {
        segment.draw_debug_gizmos(&mut gizmos, Color::srgb(0.2, 0.8, 0.2));
    }
}

/// Camera follows center of all marbles
fn camera_follow(
    marble_query: Query<&Transform, With<Marble>>,
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<Marble>)>,
    time: Res<Time>,
) {
    let Ok(mut camera_transform) = camera_query.get_single_mut() else {
        return;
    };

    let mut center = Vec3::ZERO;
    let mut count = 0;
    for marble_transform in marble_query.iter() {
        center += marble_transform.translation;
        count += 1;
    }

    if count > 0 {
        center /= count as f32;
        let target_pos = center + Vec3::new(12.0, 10.0, 12.0);
        let lerp_speed = 2.0 * time.delta_secs();

        camera_transform.translation = camera_transform.translation.lerp(target_pos, lerp_speed);
        camera_transform.look_at(center, Vec3::Y);
    }
}

/// Reset marble if it falls too far
fn reset_marble(mut query: Query<(&mut Transform, &mut Marble)>, track: Res<Track>) {
    if let Some(start_port) = track.first_port() {
        for (mut transform, mut marble) in query.iter_mut() {
            if transform.translation.y < -20.0 {
                transform.translation = start_port.position;
                marble.velocity = Vec3::ZERO;
            }
        }
    }
}

/// Press R to restart all marbles
fn restart_on_keypress(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut query: Query<(&mut Transform, &mut Marble)>,
    track: Res<Track>,
) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        if let Some(start_port) = track.first_port() {
            // Get direction for staggering
            let dir = start_port.direction;

            for (i, (mut transform, mut marble)) in query.iter_mut().enumerate() {
                // Stagger along track direction
                transform.translation = start_port.position + dir * (i as f32 * 0.5);
                marble.velocity = Vec3::ZERO;
            }
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
                marble_collision,
                draw_track_gizmos,
                camera_follow,
                reset_marble,
                restart_on_keypress,
            ),
        )
        .run();
}
