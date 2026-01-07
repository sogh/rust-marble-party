use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};

mod track;
use track::{Track, Port, SpiralTube, StraightTube, CurvedTube, FlatSlope, NarrowingTube, WideningTube, Funnel, Segment};

// ============================================================================
// CONSTANTS
// ============================================================================

// Steel ball bearing physics
const GRAVITY: Vec3 = Vec3::new(0.0, -20.0, 0.0);
const MARBLE_RADIUS: f32 = 0.2;
const TRACK_RADIUS: f32 = 1.0;
const RESTITUTION: f32 = 0.6;
const MARBLE_RESTITUTION: f32 = 0.9;
const FRICTION: f32 = 0.99;

// ============================================================================
// RESOURCES
// ============================================================================

#[derive(Resource)]
struct PhysicsTimeScale(f32);

impl Default for PhysicsTimeScale {
    fn default() -> Self {
        Self(1.0)
    }
}

// ============================================================================
// COMPONENTS
// ============================================================================

#[derive(Component)]
struct Marble {
    velocity: Vec3,
    radius: f32,
    /// Index of the segment the marble is currently in (-1 if none)
    current_segment: i32,
}

#[derive(Component)]
struct MarbleId(usize);

#[derive(Component)]
struct DebugText;

/// Stores debug info about the red marble's last collision
#[derive(Resource, Default)]
struct RedMarbleDebug {
    is_colliding: bool,
    collision_normal: Vec3,
    sdf_distance: f32,
    penetration: f32,
}

// ============================================================================
// SYSTEMS
// ============================================================================

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Create track with multiple segments - testing overlap fix
    let mut track = Track::new();

    // Start with spiral tube
    let spiral = SpiralTube::at_origin(2.0, 20.0, 5.0, TRACK_RADIUS);
    let spine = spiral.spine().to_vec();
    let exit_port = spiral.exit_ports()[0].clone();
    info!("Segment 0 (SpiralTube): exit={:?} dir={:?}",
        exit_port.position, exit_port.direction);
    track.add_segment(Box::new(spiral));

    // Continue with straight tube
    let straight = StraightTube::new(8.0, TRACK_RADIUS, exit_port.clone());
    let exit_port = straight.exit_ports()[0].clone();
    info!("Segment 1 (StraightTube): exit={:?} dir={:?}",
        exit_port.position, exit_port.direction);
    track.add_segment(Box::new(straight));

    // Add narrowing tube transition (very gentle narrowing)
    let narrowing = NarrowingTube::new(8.0, TRACK_RADIUS, TRACK_RADIUS * 0.9, exit_port.clone());
    let exit_port = narrowing.exit_ports()[0].clone();
    info!("Segment 2 (NarrowingTube): exit={:?} dir={:?}",
        exit_port.position, exit_port.direction);
    track.add_segment(Box::new(narrowing));

    // Add widening tube back to normal
    let widening = WideningTube::new(8.0, TRACK_RADIUS * 0.9, TRACK_RADIUS, exit_port.clone());
    let exit_port = widening.exit_ports()[0].clone();
    info!("Segment 3 (WideningTube): exit={:?} dir={:?}",
        exit_port.position, exit_port.direction);
    track.add_segment(Box::new(widening));

    // End with flat slope
    let slope = FlatSlope::new(10.0, 2.5, 1.0, 0.3, exit_port.clone());
    info!("Segment 4 (FlatSlope): exit={:?} dir={:?}",
        slope.exit_ports()[0].position, slope.exit_ports()[0].direction);
    track.add_segment(Box::new(slope));

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
                current_segment: 0, // Start in first segment
            },
            MarbleId(i),
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
    time_scale: Res<PhysicsTimeScale>,
    track: Res<Track>,
    mut red_marble_debug: ResMut<RedMarbleDebug>,
    mut query: Query<(&mut Transform, &mut Marble, &MarbleId)>,
) {
    let dt = time.delta_secs() * time_scale.0;

    for (mut transform, mut marble, marble_id) in query.iter_mut() {
        // Apply gravity
        marble.velocity += GRAVITY * dt;

        // Predict next position
        let next_pos = transform.translation + marble.velocity * dt;

        // Check SDF penetration using new Track system
        let dist = track.sdf(next_pos);
        let penetration = marble.radius - dist;

        // Track debug info for red marble (marble 0)
        if marble_id.0 == 0 {
            red_marble_debug.sdf_distance = dist;
            red_marble_debug.penetration = penetration.max(0.0);
        }

        if penetration > 0.0 {
            // Use current segment for gradient (prevents junction flip-flopping)
            // Fall back to nearest segment if current is invalid
            let normal = if marble.current_segment >= 0 {
                track.segment_gradient(marble.current_segment as usize, next_pos)
            } else {
                track.sdf_gradient(next_pos)
            };

            // Track collision info for red marble
            if marble_id.0 == 0 {
                red_marble_debug.is_colliding = true;
                red_marble_debug.collision_normal = normal;
            }

            // Resolve penetration
            let corrected_pos = next_pos + normal * penetration;

            // Decompose velocity
            let vel_normal = normal * marble.velocity.dot(normal);
            let vel_tangent = marble.velocity - vel_normal;

            // Apply friction and restitution
            marble.velocity = vel_tangent * FRICTION - vel_normal * RESTITUTION;
            transform.translation = corrected_pos;
        } else {
            transform.translation = next_pos;

            // Track no collision for red marble
            if marble_id.0 == 0 {
                red_marble_debug.is_colliding = false;
                red_marble_debug.collision_normal = Vec3::ZERO;
            }
        }

        // Track which segment the marble is in - only allow forward transitions
        // This prevents oscillation at junctions
        let detected_segment = track.find_segment(transform.translation);

        // Only transition if:
        // 1. Moving forward (to next segment), OR
        // 2. Currently in no segment (-1) and found one, OR
        // 3. Current segment is invalid
        let should_transition = if marble.current_segment < 0 {
            detected_segment >= 0
        } else if detected_segment < 0 {
            // Don't go to "no segment" unless truly outside
            false
        } else {
            // Only allow forward movement (or staying in same segment)
            detected_segment >= marble.current_segment
        };

        if should_transition && detected_segment != marble.current_segment {
            let segment_name = if detected_segment >= 0 {
                track.segments().get(detected_segment as usize)
                    .map(|s| s.type_name())
                    .unwrap_or("Unknown")
            } else {
                "None"
            };
            info!("Marble {} transitioned from segment {} to {} ({})",
                marble_id.0, marble.current_segment, detected_segment, segment_name);
            marble.current_segment = detected_segment;
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

/// Camera follows a single marble (marble 0)
fn camera_follow(
    marble_query: Query<(&Transform, &MarbleId), With<Marble>>,
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<Marble>)>,
    time: Res<Time>,
) {
    let Ok(mut camera_transform) = camera_query.get_single_mut() else {
        return;
    };

    // Find marble 0
    for (marble_transform, marble_id) in marble_query.iter() {
        if marble_id.0 == 0 {
            let target_pos = marble_transform.translation + Vec3::new(8.0, 6.0, 8.0);
            let lerp_speed = 3.0 * time.delta_secs();

            camera_transform.translation = camera_transform.translation.lerp(target_pos, lerp_speed);
            camera_transform.look_at(marble_transform.translation, Vec3::Y);
            break;
        }
    }
}

/// Reset marble if it falls too far
fn reset_marble(mut query: Query<(&mut Transform, &mut Marble)>, track: Res<Track>) {
    if let Some(start_port) = track.first_port() {
        for (mut transform, mut marble) in query.iter_mut() {
            if transform.translation.y < -20.0 {
                transform.translation = start_port.position;
                marble.velocity = Vec3::ZERO;
                marble.current_segment = 0;
            }
        }
    }
}

/// Press [ to slow down, ] to speed up, \ to reset speed
fn time_scale_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut time_scale: ResMut<PhysicsTimeScale>,
) {
    if keyboard.just_pressed(KeyCode::BracketLeft) {
        time_scale.0 = (time_scale.0 * 0.5).max(0.0625);
        info!("Physics speed: {}x", time_scale.0);
    }
    if keyboard.just_pressed(KeyCode::BracketRight) {
        time_scale.0 = (time_scale.0 * 2.0).min(4.0);
        info!("Physics speed: {}x", time_scale.0);
    }
    if keyboard.just_pressed(KeyCode::Backslash) {
        time_scale.0 = 1.0;
        info!("Physics speed: {}x", time_scale.0);
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
                marble.current_segment = 0;
            }
            info!("All marbles reset to start");
        }
    }
}

/// Setup debug UI overlay
fn setup_debug_ui(mut commands: Commands) {
    commands.spawn((
        Text::new(""),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        DebugText,
    ));
}

/// Update debug UI with current info
fn update_debug_ui(
    time_scale: Res<PhysicsTimeScale>,
    diagnostics: Res<DiagnosticsStore>,
    track: Res<Track>,
    red_marble_debug: Res<RedMarbleDebug>,
    marble_query: Query<(&Transform, &Marble, &MarbleId)>,
    mut text_query: Query<&mut Text, With<DebugText>>,
) {
    let Ok(mut text) = text_query.get_single_mut() else {
        return;
    };

    // Get FPS
    let fps = diagnostics
        .get(&bevy::diagnostic::FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    // Find red marble (marble 0)
    let mut red_marble_info = String::new();
    for (transform, marble, marble_id) in marble_query.iter() {
        if marble_id.0 == 0 {
            let segment_name = if marble.current_segment >= 0 {
                track.segments()
                    .get(marble.current_segment as usize)
                    .map(|s| s.type_name())
                    .unwrap_or("Unknown")
            } else {
                "None"
            };

            let speed = marble.velocity.length();

            red_marble_info = format!(
                "\n\n[Red Marble]\n\
                 Position: ({:.1}, {:.1}, {:.1})\n\
                 Velocity: ({:.1}, {:.1}, {:.1})\n\
                 Speed: {:.2} m/s\n\
                 Segment: {} ({})\n\
                 \n\
                 [Collision]\n\
                 Colliding: {}\n\
                 SDF Dist: {:.3}\n\
                 Penetration: {:.3}\n\
                 Normal: ({:.2}, {:.2}, {:.2})",
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                marble.velocity.x,
                marble.velocity.y,
                marble.velocity.z,
                speed,
                marble.current_segment,
                segment_name,
                if red_marble_debug.is_colliding { "YES" } else { "no" },
                red_marble_debug.sdf_distance,
                red_marble_debug.penetration,
                red_marble_debug.collision_normal.x,
                red_marble_debug.collision_normal.y,
                red_marble_debug.collision_normal.z,
            );
            break;
        }
    }

    **text = format!(
        "[Debug Info]\n\
         FPS: {:.0}\n\
         Physics Speed: {:.2}x\n\
         Controls: [/] speed, R restart{}",
        fps,
        time_scale.0,
        red_marble_info
    );
}

// ============================================================================
// MAIN
// ============================================================================

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .init_resource::<PhysicsTimeScale>()
        .init_resource::<RedMarbleDebug>()
        .add_systems(Startup, (setup, setup_debug_ui))
        .add_systems(
            Update,
            (
                time_scale_controls,
                marble_physics,
                marble_collision,
                draw_track_gizmos,
                camera_follow,
                reset_marble,
                restart_on_keypress,
                update_debug_ui,
            ),
        )
        .run();
}
