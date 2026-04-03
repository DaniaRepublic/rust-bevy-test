use avian3d::prelude::*;
use bevy::prelude::*;

#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            CameraControllerPlugin,
        ))
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (
                apply_gravity_to_kinematics,
                // Critical: This must run to fix positions
                resolve_kinematic_collisions,
            )
                .chain(),
        )
        .add_systems(Update, camera_movement)
        .run();
}

#[derive(Component)]
struct ManualKinematicActor;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // 1. STATIC GROUND
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(20.0, 1.0, 20.0))),
        MeshMaterial3d(materials.add(Color::WHITE)),
        Transform::from_xyz(0.0, -0.5, 0.0),
        RigidBody::Static,
        Collider::cuboid(20.0, 1.0, 20.0),
    ));

    // 2. KINEMATIC CUBE A (Bottom)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.0, 0.0))),
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody::Kinematic,
        Collider::cuboid(1.0, 1.0, 1.0),
        LinearVelocity(Vec3::ZERO),
        ManualKinematicActor,
    ));

    // 3. KINEMATIC CUBE B (Top - will hit A)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.0, 0.0, 1.0))),
        Transform::from_xyz(0.2, 8.0, 0.0), // Starts lower to hit sooner
        RigidBody::Kinematic,
        Collider::cuboid(1.0, 1.0, 1.0),
        LinearVelocity(Vec3::ZERO),
        ManualKinematicActor,
    ));

    // Camera & Light ...
    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, 10.0)));
    commands.spawn((
        Camera3d::default(),
        CameraController::default(),
        CameraMoveSpeed(10.0),
        Transform::from_xyz(0.0, 5.0, 20.0).looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
    ));
}

fn apply_gravity_to_kinematics(
    mut bodies: Query<&mut LinearVelocity, With<ManualKinematicActor>>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();
    let gravity = Vec3::new(0.0, -9.81, 0.0);
    for mut vel in &mut bodies {
        vel.0 += gravity * dt;
    }
}

fn resolve_kinematic_collisions(
    // Added Transform to this query so we can push them out!
    mut actors: Query<(&mut LinearVelocity, &mut Transform), With<ManualKinematicActor>>,
    collisions: Collisions,
    all_bodies: Query<&RigidBody>,
) {
    for contacts in collisions.iter() {
        let e1 = contacts.collider1;
        let e2 = contacts.collider2;

        let Some(manifold) = contacts.manifolds.first() else {
            continue;
        };
        let normal = manifold.normal;

        // Get penetration depth (how deep are we stuck?)
        // Avian 0.4 stores this in manifold points. We take the max depth.
        let penetration = manifold
            .points
            .iter()
            .map(|p| p.penetration)
            .fold(0.0f32, f32::max);

        // Ignore tiny touches
        if penetration < 1e-4 {
            continue;
        }

        // ---------------------------------------------------------------------
        // CASE 1: Actor vs Actor (Bounce off each other)
        // ---------------------------------------------------------------------
        if let Ok([mut actor1, mut actor2]) = actors.get_many_mut([e1, e2]) {
            // Distribute the push-out 50/50
            let push_vec = normal * (penetration * 0.5);

            // 1. Positional Correction (Depenetration)
            // e1 is pushed along normal (if normal points 2->1)
            actor1.1.translation += push_vec;
            actor2.1.translation -= push_vec;

            // 2. Velocity Correction (Bounce)
            // e1
            let v_normal_1 = actor1.0.0.dot(normal);
            if v_normal_1 < 0.0 {
                // Bounce with 0.3 restitution
                actor1.0.0 += normal * (-v_normal_1 * 1.3);
            }
            // e2
            let v_normal_2 = actor2.0.0.dot(-normal);
            if v_normal_2 < 0.0 {
                actor2.0.0 += (-normal) * (-v_normal_2 * 1.3);
            }
        }
        // ---------------------------------------------------------------------
        // CASE 2: Actor vs Static (Bounce off Wall)
        // ---------------------------------------------------------------------
        else if let Ok((mut vel1, mut tr1)) = actors.get_mut(e1) {
            let rb2_type = all_bodies.get(e2).unwrap_or(&RigidBody::Static);
            if *rb2_type == RigidBody::Static {
                // 1. Positional Correction
                // Push e1 fully out along normal
                tr1.translation += normal * penetration;

                // 2. Velocity Correction
                let v_normal = vel1.0.dot(normal);
                if v_normal < 0.0 {
                    // Bounce with 0.5 restitution
                    vel1.0 += normal * (-v_normal * 1.5);
                }
            }
        }
        // ---------------------------------------------------------------------
        // CASE 3: Static vs Actor
        // ---------------------------------------------------------------------
        else if let Ok((mut vel2, mut tr2)) = actors.get_mut(e2) {
            let rb1_type = all_bodies.get(e1).unwrap_or(&RigidBody::Static);
            if *rb1_type == RigidBody::Static {
                // 1. Positional Correction
                // Push e2 fully out opposite to normal
                tr2.translation -= normal * penetration;

                // 2. Velocity Correction
                let v_normal = vel2.0.dot(-normal);
                if v_normal < 0.0 {
                    vel2.0 += (-normal) * (-v_normal * 1.5);
                }
            }
        }
    }
}
