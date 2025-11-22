use avian3d::prelude::*;
use bevy::{input::mouse::MouseMotion, math::DVec3, prelude::*};

fn main() {
    App::new()
        // 1. TUNE THE SOLVER
        // Substeps help with fast motion/tunneling.
        // PositionalIters help with STIFFNESS (making joints look like steel).
        .insert_resource(SubstepCount(12))
        .insert_resource(TimeToSleep(0.2))
        .insert_resource(Gravity(DVec3::new(0.0, -9.81, 0.0)))
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, setup_scene)
        .add_systems(Update, orbit_camera)
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let floor_mesh = meshes.add(Cuboid::new(4.0, 0.5, 4.0)); // Thicker slabs look better
    let gray_mat = materials.add(Color::srgb(0.3, 0.3, 0.3));
    let blue_mat = materials.add(Color::srgb(0.2, 0.2, 0.8));

    // 2. THE ANCHOR (Ground)
    // We need the Entity ID of the ground to connect the first floor to it.
    let ground_entity = commands
        .spawn((
            Name::new("Ground"),
            RigidBody::Static,
            Collider::cuboid(50.0, 1.0, 50.0),
            Mesh3d(meshes.add(Cuboid::new(50.0, 1.0, 50.0))),
            MeshMaterial3d(gray_mat.clone()),
            Transform::from_xyz(0.0, -0.5, 0.0),
        ))
        .id();

    let story_height = 2.0;

    // Start the "previous" entity as the Ground.
    // This anchors the bottom of the stack to the world.
    let mut prev_floor_entity = ground_entity;

    for i in 0..50 {
        let y_pos = (i as f32 * story_height) + 1.0; // Adjust for ground height

        let floor_entity = commands
            .spawn((
                Name::new(format!("Floor {}", i + 1)),
                RigidBody::Dynamic,
                Collider::cuboid(2.0, 0.25, 2.0),
                Mesh3d(floor_mesh.clone()),
                MeshMaterial3d(blue_mat.clone()),
                Transform::from_xyz(0.0, y_pos, 0.0),
                // 3. MASS PROPERTIES
                // Making the building heavy helps stability against small impulses.
                Mass(500.0),
                // 4. DAMPING (Crucial for Concrete feel)
                // This kills the "ghostly" vibration.
                LinearDamping(0.9),
                AngularDamping(0.9),
                SleepThreshold {
                    linear: 0.25,
                    angular: 0.25,
                },
            ))
            .id();

        // 5. JOINT SETUP
        // We connect `prev` (which starts as Ground) to the current floor.
        let anchor_y = if i == 0 {
            // Distance from Ground center (-0.5) to Floor 0 center (1.0) is 1.5
            // But simpler: Anchor Ground at top (+0.5) and Floor at bottom (-height/2)
            DVec3::new(0.0, 0.5, 0.0)
        } else {
            DVec3::new(0.0, story_height as f64 / 2.0, 0.0)
        };

        commands.spawn(
            FixedJoint::new(prev_floor_entity, floor_entity)
                // Anchor 1 is on the 'previous' body (top of the block below)
                .with_local_anchor1(anchor_y)
                // Anchor 2 is on the 'current' body (bottom of this block)
                .with_local_anchor2(DVec3::new(0.0, -story_height as f64 / 2.0, 0.0)),
        );

        prev_floor_entity = floor_entity;
    }

    // Camera and Light (Same as before)
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(40.0, 30.0, 40.0).looking_at(Vec3::new(0.0, 25.0, 0.0), Vec3::Y),
        OrbitCamera {
            sensitivity: 0.005,
            yaw: 0.0,
            pitch: 0.0,
        },
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-0.5)),
    ));
}

// ... OrbitCamera code stays the same ...

// Simple orbit camera component and system
#[derive(Component)]
struct OrbitCamera {
    sensitivity: f32,
    yaw: f32,
    pitch: f32,
}

fn orbit_camera(
    mut query: Query<(&mut Transform, &mut OrbitCamera), With<Camera>>,
    mut motion_evr: MessageReader<MouseMotion>,
    buttons: Res<ButtonInput<MouseButton>>,
) {
    for ev in motion_evr.read() {
        if buttons.pressed(MouseButton::Right) {
            for (mut transform, mut camera) in query.iter_mut() {
                camera.yaw -= ev.delta.x * camera.sensitivity;
                camera.pitch -= ev.delta.y * camera.sensitivity;
                camera.pitch = camera.pitch.clamp(
                    -std::f32::consts::FRAC_PI_2 + 0.01,
                    std::f32::consts::FRAC_PI_2 - 0.01,
                );

                let rot = Quat::from_rotation_y(camera.yaw) * Quat::from_rotation_x(camera.pitch);
                transform.rotation = rot;
                transform.translation = rot * Vec3::new(0.0, 25.0, 15.0); // Orbit around origin
            }
        }
    }
}
