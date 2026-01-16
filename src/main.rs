use bevy::prelude::*;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::time::{Fixed, Virtual};

mod track;
use track::{Track, Port, PortProfile, StraightTube, CurvedTube, FlatSlope, NarrowingTube, WideningTube, HalfPipe, SpiralTube, Funnel, StartingGate, Segment, TrackGenerator, GeneratorConfig, TubeAdapter};

// ============================================================================
// CONSTANTS
// ============================================================================

// Steel ball bearing physics
const GRAVITY: Vec3 = Vec3::new(0.0, -20.0, 0.0);
const MARBLE_RADIUS: f32 = 0.2;
const TRACK_RADIUS: f32 = 1.0;
const RESTITUTION: f32 = 0.3;  // Lower bounce for smoother rolling
const MARBLE_RESTITUTION: f32 = 0.9;
const FRICTION: f32 = 0.998;  // Higher friction for smoother curves

// ============================================================================
// RESOURCES
// ============================================================================

#[derive(Resource)]
struct PhysicsTimeScale(f32);

impl Default for PhysicsTimeScale {
    fn default() -> Self {
        Self(1.0)  // Normal speed
    }
}

/// Command-line track specification
#[derive(Resource, Default)]
struct TrackSpec {
    /// Comma-separated segment names, e.g. "StartingGate,HalfPipe,StraightTube"
    segments: Option<String>,
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
    /// Previous segment index (for gradient blending during transitions)
    previous_segment: i32,
    /// Distance traveled since last segment transition
    transition_distance: f32,
    /// Position at start of physics step (for render interpolation)
    previous_position: Vec3,
    /// Position at end of physics step (for render interpolation)
    physics_position: Vec3,
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

/// Timer for logging red marble position once per second
#[derive(Resource)]
struct PositionLogger {
    timer: Timer,
    elapsed_seconds: u32,
}

impl Default for PositionLogger {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(0.25, TimerMode::Repeating),  // More frequent for junction debug
            elapsed_seconds: 0,
        }
    }
}

/// Debug settings
#[derive(Resource)]
struct DebugSettings {
    show_gizmos: bool,
}

impl Default for DebugSettings {
    fn default() -> Self {
        Self { show_gizmos: true }
    }
}

/// Debug track builder state
#[derive(Resource)]
struct TrackBuilder {
    /// Currently selected segment type to add
    selected_type: usize,
    /// Available segment type names
    segment_types: Vec<&'static str>,
    /// Is build mode active
    build_mode: bool,
}

impl Default for TrackBuilder {
    fn default() -> Self {
        Self {
            selected_type: 0,
            segment_types: vec![
                "StraightTube",
                "CurvedLeft",
                "CurvedRight",
                "NarrowingTube",
                "WideningTube",
                "FlatSlope",
                "HalfPipe",
            ],
            build_mode: false,
        }
    }
}

// ============================================================================
// SYSTEMS
// ============================================================================

/// Build a track from comma-separated segment names
/// Example: "StartingGate,HalfPipe,StraightTube,CurvedLeft"
fn build_track_from_spec(spec: &str, num_marbles: usize, marble_radius: f32) -> (Track, Vec<Vec3>) {
    let mut track = Track::new();

    // Parse segment names
    let segment_names: Vec<&str> = spec.split(',').map(|s| s.trim()).collect();

    if segment_names.is_empty() {
        panic!("Track spec cannot be empty");
    }

    // Calculate starting gate dimensions
    let marble_spacing = marble_radius * 2.0 + 0.1;
    let gate_width = marble_spacing * num_marbles as f32 + 0.2;
    let gate_radius = gate_width / 2.0;
    let gate_length = 3.5;
    let gate_slope: f32 = 0.25;

    let start_y = 12.0;
    let gate_entry_y = start_y + gate_length * gate_slope.sin();
    let gate_entry_z = gate_length * gate_slope.cos();

    let gate_entry = Port::new(
        Vec3::new(0.0, gate_entry_y, gate_entry_z),
        Vec3::NEG_Z,
        Vec3::Y,
        gate_radius,
    );

    // Always start with StartingGate
    let starting_gate = StartingGate::new(gate_width, gate_length, 1.0, gate_slope, gate_entry);
    let spawn_positions = starting_gate.get_spawn_positions(num_marbles, marble_radius);
    let mut last_exit = starting_gate.exit_ports()[0].clone();
    track.add_segment(Box::new(starting_gate));

    // Current tube radius for narrowing/widening
    let mut current_radius = TRACK_RADIUS;

    // Process remaining segments (skip first if it's StartingGate)
    let start_idx = if segment_names[0].eq_ignore_ascii_case("StartingGate") { 1 } else { 0 };

    // Helper to check if segment type requires tube profile
    fn needs_tube_profile(name: &str) -> bool {
        matches!(name.to_lowercase().as_str(),
            "straighttube" | "straight" |
            "curvedleft" | "left" |
            "curvedright" | "right" |
            "curvedtube" | "curved" |
            "halfpipe" | "half" |
            "narrowingtube" | "narrowing" | "narrow" |
            "wideningtube" | "widening" | "wide" |
            "spiraltube" | "spiral"
        )
    }

    for name in &segment_names[start_idx..] {
        // Auto-insert TubeAdapter when transitioning from FlatFloor to Tube profile
        if matches!(last_exit.profile, PortProfile::FlatFloor { .. }) && needs_tube_profile(name) {
            let adapter = TubeAdapter::new(4.0, gate_width, current_radius, last_exit.clone());
            last_exit = adapter.exit_ports()[0].clone();
            track.add_segment(Box::new(adapter));
        }

        let segment: Box<dyn Segment> = match name.to_lowercase().as_str() {
            "straighttube" | "straight" => {
                Box::new(StraightTube::new(8.0, current_radius, last_exit.clone()))
            }
            "curvedleft" | "left" => {
                Box::new(CurvedTube::new(0.8, 5.0, current_radius, last_exit.clone()))
            }
            "curvedright" | "right" => {
                Box::new(CurvedTube::new(-0.8, 5.0, current_radius, last_exit.clone()))
            }
            "curvedtube" | "curved" => {
                Box::new(CurvedTube::new(0.8, 5.0, current_radius, last_exit.clone()))
            }
            "halfpipe" | "half" => {
                // Use wider radius for half-pipe to accommodate marbles
                Box::new(HalfPipe::new(10.0, gate_radius, last_exit.clone()))
            }
            "flatslope" | "flat" => {
                let floor_y = last_exit.floor_y();
                let adjusted_port = Port::with_profile(
                    last_exit.position,
                    last_exit.direction,
                    last_exit.up,
                    gate_radius,
                    track::PortProfile::flat_floor(gate_width, floor_y),
                );
                Box::new(FlatSlope::new(8.0, gate_width, 1.0, 0.25, adjusted_port))
            }
            "narrowingtube" | "narrowing" | "narrow" => {
                let new_radius = current_radius * 0.7;
                let seg = NarrowingTube::new(6.0, current_radius, new_radius.max(TRACK_RADIUS * 0.5), last_exit.clone());
                current_radius = new_radius.max(TRACK_RADIUS * 0.5);
                Box::new(seg)
            }
            "wideningtube" | "widening" | "wide" => {
                let new_radius = (current_radius * 1.3).min(gate_radius);
                let seg = WideningTube::new(6.0, current_radius, new_radius, last_exit.clone());
                current_radius = new_radius;
                Box::new(seg)
            }
            "spiraltube" | "spiral" => {
                Box::new(SpiralTube::new(1.0, 8.0, 4.0, current_radius, last_exit.clone()))
            }
            "funnel" => {
                let funnel_entry = Port::new(
                    last_exit.position - Vec3::Y * 0.3,
                    Vec3::NEG_Y,
                    Vec3::NEG_Z,
                    gate_radius,
                );
                Box::new(Funnel::new(10.0, gate_radius, TRACK_RADIUS, funnel_entry))
            }
            other => {
                eprintln!("Unknown segment type: '{}', using StraightTube", other);
                Box::new(StraightTube::new(8.0, current_radius, last_exit.clone()))
            }
        };

        last_exit = segment.exit_ports()[0].clone();
        track.add_segment(segment);
    }

    // Log the built track
    let names: Vec<&str> = track.segments().iter().map(|s| s.type_name()).collect();
    info!("Built custom track: {}", names.join(" -> "));

    (track, spawn_positions)
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    track_spec: Res<TrackSpec>,
) {
    let num_marbles = 8;

    // Either build from spec or generate procedurally
    let (track, spawn_positions) = if let Some(ref spec) = track_spec.segments {
        build_track_from_spec(spec, num_marbles, MARBLE_RADIUS)
    } else {
        // Generate procedural track with starting gate
        let seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        let config = GeneratorConfig {
            seed,
            num_marbles,
            marble_radius: MARBLE_RADIUS,
            tube_radius: TRACK_RADIUS,
            ..Default::default()
        };

        let mut generator = TrackGenerator::new(config);
        generator.generate_with_spawns(10) // 10 segments total
    };

    commands.insert_resource(track);

    // Spawn 8 marbles with different colors - use positions from starting gate
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
        let start_pos = spawn_positions[i];
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
                velocity: Vec3::ZERO,  // Gravity will accelerate on sloped gate
                radius: MARBLE_RADIUS,
                current_segment: 0,
                previous_segment: 0,
                transition_distance: f32::MAX, // Start fully transitioned
                previous_position: start_pos,
                physics_position: start_pos,
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
    time: Res<Time<Fixed>>,
    track: Res<Track>,
    mut red_marble_debug: ResMut<RedMarbleDebug>,
    mut query: Query<(&mut Transform, &mut Marble, &MarbleId)>,
) {
    // Use fixed delta time directly - time scaling is handled by Virtual time
    // which controls how many fixed steps run per frame
    let dt = time.delta_secs();

    for (mut transform, mut marble, marble_id) in query.iter_mut() {
        // Store previous position for render interpolation
        marble.previous_position = marble.physics_position;

        // === ANTI-TUNNELING: Adaptive substepping ===
        // Calculate how far the marble will move this frame
        // Subdivide into substeps to prevent tunneling through thin geometry
        let speed = marble.velocity.length();
        let expected_displacement = speed * dt;

        // Maximum distance per substep = 50% of marble radius
        // This ensures we can't skip over walls thinner than the marble
        let max_step_distance = marble.radius * 0.5;

        // Calculate number of substeps needed (minimum 1)
        let num_substeps = if expected_displacement > max_step_distance {
            ((expected_displacement / max_step_distance).ceil() as usize).max(1).min(16)
        } else {
            1
        };

        let sub_dt = dt / num_substeps as f32;

        // Run physics substeps
        for substep in 0..num_substeps {
            // Apply gravity (distributed across substeps)
            marble.velocity += GRAVITY * sub_dt;

            // Predict next position
            let next_pos = marble.physics_position + marble.velocity * sub_dt;

            // Check SDF penetration using fast local check
            let dist = track.sdf_near_segment(next_pos, marble.current_segment);
            let penetration = marble.radius - dist;

            // Track debug info for red marble (marble 0) - only on first substep
            if marble_id.0 == 0 && substep == 0 {
                red_marble_debug.sdf_distance = dist;
                red_marble_debug.penetration = penetration.max(0.0);
            }

            // Cap penetration to prevent physics explosions from buggy SDF values
            // Max penetration is one marble diameter
            let max_penetration = marble.radius * 2.0;
            let penetration = penetration.min(max_penetration);

            if penetration > 0.0 {
                // Use previous segment's gradient during transition buffer period
                // This prevents the "lip" effect at segment junctions
                const TRANSITION_BUFFER: f32 = 2.0;

                let gradient_segment = if marble.transition_distance < TRANSITION_BUFFER
                    && marble.previous_segment >= 0
                    && marble.previous_segment != marble.current_segment
                {
                    marble.previous_segment
                } else {
                    marble.current_segment
                };

                let normal = if gradient_segment >= 0 {
                    track.segment_gradient(gradient_segment as usize, next_pos)
                } else {
                    track.sdf_gradient(next_pos)
                };

                // Track collision info for red marble
                if marble_id.0 == 0 && substep == 0 {
                    red_marble_debug.is_colliding = true;
                    red_marble_debug.collision_normal = normal;
                }

                // Resolve penetration (capped to prevent explosions)
                let corrected_pos = next_pos + normal * penetration;

                // Decompose velocity
                let vel_normal = normal * marble.velocity.dot(normal);
                let vel_tangent = marble.velocity - vel_normal;

                // Apply friction and restitution
                marble.velocity = vel_tangent * FRICTION - vel_normal * RESTITUTION;
                marble.physics_position = corrected_pos;
            } else {
                marble.physics_position = next_pos;

                // Track no collision for red marble
                if marble_id.0 == 0 && substep == 0 {
                    red_marble_debug.is_colliding = false;
                    red_marble_debug.collision_normal = Vec3::ZERO;
                }
            }

            // Update transition distance per substep
            let distance_traveled = marble.velocity.length() * sub_dt;
            marble.transition_distance += distance_traveled;
        }

        // Track which segment the marble is in - only allow forward transitions
        // This prevents oscillation at junctions (uses fast local search)
        let detected_segment = track.find_segment_near(marble.physics_position, marble.current_segment);

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

            // Store previous segment for gradient blending
            marble.previous_segment = marble.current_segment;
            marble.current_segment = detected_segment;
            marble.transition_distance = 0.0; // Reset buffer
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
    // Use physics_position as source of truth, not transform.translation
    let mut marbles: Vec<(Entity, Vec3, Vec3, f32)> = query
        .iter()
        .map(|(e, _t, m)| (e, m.physics_position, m.velocity, m.radius))
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
        if let Ok((_, _transform, mut marble)) = query.get_mut(entity) {
            // Only update physics state - interpolation handles transform
            marble.physics_position = new_pos;
            marble.velocity = new_vel;
        }
    }
}

/// Interpolate marble positions for smooth rendering between fixed timesteps
fn marble_interpolation(
    fixed_time: Res<Time<Fixed>>,
    mut query: Query<(&mut Transform, &Marble)>,
) {
    // How far we are between the last physics step and the next one (0.0 to 1.0)
    let alpha = fixed_time.overstep_fraction();

    for (mut transform, marble) in query.iter_mut() {
        // Lerp between previous physics position and current physics position
        transform.translation = marble.previous_position.lerp(marble.physics_position, alpha);
    }
}

/// Draw track using segment gizmos (toggleable with G key)
fn draw_track_gizmos(mut gizmos: Gizmos, track: Res<Track>, debug: Res<DebugSettings>) {
    if !debug.show_gizmos {
        return;
    }
    for segment in track.segments() {
        segment.draw_debug_gizmos(&mut gizmos, Color::srgb(0.2, 0.8, 0.2));
    }
}

/// Toggle debug gizmos with G key
fn toggle_gizmos(keyboard: Res<ButtonInput<KeyCode>>, mut debug_settings: ResMut<DebugSettings>) {
    if keyboard.just_pressed(KeyCode::KeyG) {
        debug_settings.show_gizmos = !debug_settings.show_gizmos;
        let status = if debug_settings.show_gizmos { "ON" } else { "OFF" };
        info!("Gizmos: {}", status);
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

/// Reset marble if it falls below the track's kill plane
fn reset_marble(mut query: Query<(&mut Transform, &mut Marble)>, track: Res<Track>) {
    let kill_y = track.kill_plane_y();
    if let Some(start_port) = track.first_port() {
        for (mut transform, mut marble) in query.iter_mut() {
            if marble.physics_position.y < kill_y {
                let pos = start_port.position;
                transform.translation = pos;
                marble.velocity = Vec3::ZERO;
                marble.current_segment = 0;
                marble.previous_position = pos;
                marble.physics_position = pos;
                info!("Marble reset (fell below kill plane at y={:.1})", kill_y);
            }
        }
    }
}

/// Press [ to slow down, ] to speed up, \ to reset speed
/// Uses Virtual time to control simulation rate without changing physics step size
fn time_scale_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut time_scale: ResMut<PhysicsTimeScale>,
    mut virtual_time: ResMut<Time<Virtual>>,
) {
    if keyboard.just_pressed(KeyCode::BracketLeft) {
        time_scale.0 = (time_scale.0 * 0.5).max(0.0625);
        virtual_time.set_relative_speed(time_scale.0);
        info!("Physics speed: {}x", time_scale.0);
    }
    if keyboard.just_pressed(KeyCode::BracketRight) {
        time_scale.0 = (time_scale.0 * 2.0).min(4.0);
        virtual_time.set_relative_speed(time_scale.0);
        info!("Physics speed: {}x", time_scale.0);
    }
    if keyboard.just_pressed(KeyCode::Backslash) {
        time_scale.0 = 1.0;
        virtual_time.set_relative_speed(time_scale.0);
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
                let pos = start_port.position + dir * (i as f32 * 0.5);
                transform.translation = pos;
                marble.velocity = Vec3::ZERO;
                marble.current_segment = 0;
                marble.previous_segment = 0;
                marble.transition_distance = f32::MAX;
                marble.previous_position = pos;
                marble.physics_position = pos;
            }
            info!("All marbles reset to start");
        }
    }
}

/// N - Generate new procedural track
fn procedural_generation_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut track: ResMut<Track>,
    mut marble_query: Query<(&mut Transform, &mut Marble)>,
    mut seed_counter: Local<u64>,
) {
    if keyboard.just_pressed(KeyCode::KeyN) {
        // Increment seed for variety
        *seed_counter += 1;

        let config = GeneratorConfig {
            seed: *seed_counter,
            ..default()
        };
        let mut generator = TrackGenerator::new(config);

        // Generate new track
        *track = generator.generate(12);
        info!("Generated procedural track with seed {} ({} segments)",
            *seed_counter, track.segment_count());

        // Reset marbles to start
        if let Some(start_port) = track.first_port() {
            let dir = start_port.direction;
            for (i, (mut transform, mut marble)) in marble_query.iter_mut().enumerate() {
                let pos = start_port.position + dir * (i as f32 * 0.5);
                transform.translation = pos;
                marble.velocity = Vec3::ZERO;
                marble.current_segment = 0;
                marble.previous_segment = 0;
                marble.transition_distance = f32::MAX;
                marble.previous_position = pos;
                marble.physics_position = pos;
            }
        }
    }
}

/// Track builder controls:
/// B - Toggle build mode
/// 1-7 - Select segment type
/// Enter - Add selected segment
/// Backspace - Remove last segment
fn track_builder_controls(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut track: ResMut<Track>,
    mut builder: ResMut<TrackBuilder>,
) {
    // Toggle build mode
    if keyboard.just_pressed(KeyCode::KeyB) {
        builder.build_mode = !builder.build_mode;
        info!("Build mode: {}", if builder.build_mode { "ON" } else { "OFF" });
    }

    if !builder.build_mode {
        return;
    }

    // Select segment type with number keys
    let type_keys = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
    ];

    for (i, key) in type_keys.iter().enumerate() {
        if keyboard.just_pressed(*key) && i < builder.segment_types.len() {
            builder.selected_type = i;
            info!("Selected: {}", builder.segment_types[i]);
        }
    }

    // Add segment with Enter
    if keyboard.just_pressed(KeyCode::Enter) {
        if let Some(exit_port) = track.last_port() {
            let segment_type = builder.segment_types[builder.selected_type];

            let new_segment: Box<dyn Segment> = match segment_type {
                "StraightTube" => Box::new(StraightTube::new(8.0, TRACK_RADIUS, exit_port)),
                "CurvedLeft" => Box::new(CurvedTube::new(
                    std::f32::consts::FRAC_PI_2,
                    5.0,
                    TRACK_RADIUS,
                    exit_port,
                )),
                "CurvedRight" => Box::new(CurvedTube::new(
                    -std::f32::consts::FRAC_PI_2,
                    5.0,
                    TRACK_RADIUS,
                    exit_port,
                )),
                "NarrowingTube" => Box::new(NarrowingTube::new(
                    6.0,
                    TRACK_RADIUS,
                    TRACK_RADIUS * 0.8,
                    exit_port,
                )),
                "WideningTube" => Box::new(WideningTube::new(
                    6.0,
                    TRACK_RADIUS * 0.8,
                    TRACK_RADIUS,
                    exit_port,
                )),
                "FlatSlope" => Box::new(FlatSlope::new(10.0, 2.5, 1.0, 0.3, exit_port)),
                "HalfPipe" => Box::new(HalfPipe::new(10.0, TRACK_RADIUS * 1.5, exit_port)),
                _ => return,
            };

            info!("Added {} (total segments: {})", segment_type, track.segment_count() + 1);
            track.add_segment(new_segment);
        } else {
            info!("No track to extend from!");
        }
    }

    // Remove last segment with Backspace
    if keyboard.just_pressed(KeyCode::Backspace) {
        if let Some(removed) = track.remove_last() {
            info!("Removed {} (remaining: {})", removed.type_name(), track.segment_count());
        } else {
            info!("No segments to remove!");
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
    builder: Res<TrackBuilder>,
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

    // Build mode info
    let build_info = if builder.build_mode {
        format!(
            "\n\n[BUILD MODE]\n\
             Selected: {} {}\n\
             Segments: {}\n\
             1-7: Select type\n\
             Enter: Add segment\n\
             Backspace: Remove last",
            builder.selected_type + 1,
            builder.segment_types[builder.selected_type],
            track.segment_count()
        )
    } else {
        String::new()
    };

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
         Segments: {}\n\
         Controls: [/] speed, R restart, B build, G gizmos, N new track{}{}",
        fps,
        time_scale.0,
        track.segment_count(),
        red_marble_info,
        build_info
    );
}

// ============================================================================
// POSITION LOGGING
// ============================================================================

fn log_red_marble_position(
    time: Res<Time>,
    mut logger: ResMut<PositionLogger>,
    red_debug: Res<RedMarbleDebug>,
    query: Query<(&Transform, &Marble, &MarbleId)>,
) {
    logger.timer.tick(time.delta());

    if logger.timer.just_finished() {
        logger.elapsed_seconds += 1;

        for (transform, marble, marble_id) in query.iter() {
            if marble_id.0 == 0 {
                let pos = transform.translation;
                let vel = marble.velocity;
                let speed = vel.length();
                let n = red_debug.collision_normal;
                info!(
                    "t={:.2}s pos:({:6.1},{:6.1},{:6.1}) vel:({:5.1},{:5.1},{:5.1}) spd:{:4.1} seg:{} col:{} pen:{:.3} norm:({:5.2},{:5.2},{:5.2})",
                    logger.elapsed_seconds as f32 * 0.25,
                    pos.x, pos.y, pos.z,
                    vel.x, vel.y, vel.z,
                    speed,
                    marble.current_segment,
                    if red_debug.is_colliding { "Y" } else { "N" },
                    red_debug.penetration,
                    n.x, n.y, n.z
                );
                break;
            }
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

fn parse_args() -> TrackSpec {
    let args: Vec<String> = std::env::args().collect();

    let mut track_spec = TrackSpec::default();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--track" | "-t" => {
                if i + 1 < args.len() {
                    track_spec.segments = Some(args[i + 1].clone());
                    i += 2;
                } else {
                    eprintln!("--track requires a segment list (e.g., 'HalfPipe,StraightTube,CurvedLeft')");
                    std::process::exit(1);
                }
            }
            "--help" | "-h" => {
                println!("Marble Party - A marble racing game\n");
                println!("Usage: marble_party [OPTIONS]\n");
                println!("Options:");
                println!("  -t, --track <SEGMENTS>  Specify track segments (comma-separated)");
                println!("  -h, --help              Show this help message\n");
                println!("Segment types:");
                println!("  StartingGate (auto-added), HalfPipe, StraightTube, CurvedLeft,");
                println!("  CurvedRight, FlatSlope, NarrowingTube, WideningTube, SpiralTube, Funnel\n");
                println!("Example:");
                println!("  marble_party --track \"HalfPipe,StraightTube,CurvedLeft,CurvedRight\"");
                std::process::exit(0);
            }
            _ => {
                i += 1;
            }
        }
    }

    track_spec
}

fn main() {
    let track_spec = parse_args();

    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(track_spec)
        .init_resource::<PhysicsTimeScale>()
        .init_resource::<RedMarbleDebug>()
        .init_resource::<TrackBuilder>()
        .init_resource::<DebugSettings>()
        .init_resource::<PositionLogger>()
        .add_systems(Startup, (setup, setup_debug_ui))
        .add_systems(
            FixedUpdate,
            (
                marble_physics,
                marble_collision,
            ),
        )
        .add_systems(
            Update,
            (
                marble_interpolation,
                time_scale_controls,
                track_builder_controls,
                procedural_generation_controls,
                toggle_gizmos,
                draw_track_gizmos,
                camera_follow,
                reset_marble,
                restart_on_keypress,
                update_debug_ui,
                log_red_marble_position,
            ),
        )
        .run();
}
