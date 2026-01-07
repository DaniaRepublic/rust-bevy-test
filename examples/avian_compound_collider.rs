use avian3d::prelude::*;
use bevy::prelude::*;

#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            CameraControllerPlugin,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .add_systems(Startup, setup)
        .add_systems(Update, camera_movement)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        CameraController::default(),
        Transform::from_xyz(0., 5., 15.),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(30., 1., 20.))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.8, 0.6))),
        RigidBody::Static,
        Collider::cuboid(30., 1., 20.),
        Transform::from_xyz(0., -0.5, 0.),
    ));

    // Ball
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.5))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.3, 0.9))),
        RigidBody::Dynamic,
        Collider::sphere(0.5),
        Restitution::new(0.9).with_combine_rule(CoefficientCombine::Max),
        Transform::from_xyz(1.5, 2.5, -1.0),
    ));

    // Compound collider
    let mut shapes = Vec::<(Position, Rotation, Collider)>::new();
    shapes.push((
        Position::from_xyz(2.0, 0.5, -2.0),
        Rotation::IDENTITY,
        Collider::cuboid(1., 1., 0.2),
    ));
    shapes.push((
        Position::from_xyz(2.0, 0.5, -1.4),
        Rotation::IDENTITY,
        Collider::cuboid(0.2, 1., 1.),
    ));
    commands.spawn((RigidBody::Static, Collider::compound(shapes)));

    // Light
    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, -1.0)));
}
