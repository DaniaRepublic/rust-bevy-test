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
            CameraControllerPlugin,
        ))
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                camera_movement,
                record_ray,
                draw_rayhit,
                spawn_node,
                color_nearby_nodes,
            ),
        )
        .run();
}

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default, // Layer 0 - the default layer that objects are assigned to
    SpacialNode, // Layer 1
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(CameraRayHitDataRecord(None));

    commands.spawn((
        Camera3d::default(),
        CameraController::default(),
        Transform::from_xyz(10.0, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        CameraMoveSpeedMult(5.0),
    ));

    commands.spawn((
        Transform::from_xyz(0.0, -0.5, 0.0),
        Mesh3d(meshes.add(Cuboid::new(50.0, 1.0, 50.0))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.1, 0.8, 0.2)))),
        RigidBody::Static,
        Collider::cuboid(50.0, 1.0, 50.0),
        CollisionLayers::new(
            [GameLayer::Default],
            [GameLayer::Default, GameLayer::SpacialNode],
        ),
    ));

    commands.spawn((
        Transform::from_xyz(0.0, 2.5, 0.0),
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.8, 0.1, 0.2)))),
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 1.0, 1.0),
        CollisionLayers::new(
            [GameLayer::SpacialNode],
            [GameLayer::Default, GameLayer::SpacialNode],
        ),
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, 5.0)));
}

#[derive(Resource)]
struct CameraRayHitDataRecord(Option<(RayHitData, Vec3)>);

fn record_ray(
    mut camera_ray_hit_data_record: ResMut<CameraRayHitDataRecord>,
    spatial_q: SpatialQuery,
    camera_q: Single<&GlobalTransform, With<CameraController>>,
) {
    let camera_origin = camera_q.translation();
    let camera_forward = camera_q.forward();

    if let Some(rayhit_data) = spatial_q.cast_ray(
        camera_origin,
        camera_forward,
        300.0,
        false,
        &SpatialQueryFilter::from_mask(LayerMask::ALL),
    ) {
        let impact_point = camera_origin + camera_forward * rayhit_data.distance;

        camera_ray_hit_data_record.0 = Some((rayhit_data, impact_point));
    }
}

const SPACIAL_SPHERE_RADIUS: f32 = 3.0;

fn draw_rayhit(mut gizmos: Gizmos, camera_ray_hit_data_record: Res<CameraRayHitDataRecord>) {
    if let Some((_, impact_point)) = camera_ray_hit_data_record.0 {
        gizmos.sphere(impact_point, 0.1, Color::srgb(0.1, 1.0, 0.1));
        gizmos.sphere(
            impact_point,
            SPACIAL_SPHERE_RADIUS,
            Color::srgb(0.1, 0.1, 1.0),
        );
    }
}

fn spawn_node(
    mut commands: Commands,
    buttons: Res<ButtonInput<MouseButton>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_ray_hit_data_record: Res<CameraRayHitDataRecord>,
) {
    if buttons.just_pressed(MouseButton::Left) {
        if let Some((_, impact_point)) = camera_ray_hit_data_record.0 {
            commands.spawn((
                Transform::from_xyz(impact_point.x, impact_point.y + 1.1, impact_point.z),
                Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
                MeshMaterial3d(
                    materials.add(StandardMaterial::from_color(Color::srgb(0.8, 0.1, 0.2))),
                ),
                RigidBody::Dynamic,
                Collider::cuboid(1.0, 1.0, 1.0),
                CollisionLayers::new(
                    [GameLayer::SpacialNode],
                    [GameLayer::Default, GameLayer::SpacialNode],
                ),
            ));
        }
    }
}

fn color_nearby_nodes(
    mut commands: Commands,
    mut materials: ResMut<Assets<StandardMaterial>>,
    buttons: Res<ButtonInput<MouseButton>>,
    spacial_q: SpatialQuery,
    camera_ray_hit_data_record: Res<CameraRayHitDataRecord>,
) {
    if buttons.just_pressed(MouseButton::Right) {
        if let Some((_, impact_point)) = camera_ray_hit_data_record.0 {
            let entities_in_region = spacial_q.shape_intersections(
                &Collider::sphere(SPACIAL_SPHERE_RADIUS),
                impact_point,
                Quat::IDENTITY,
                &SpatialQueryFilter::from_mask([GameLayer::SpacialNode]),
            );
            // color intersected entities white
            for entity in entities_in_region {
                commands.entity(entity).insert(MeshMaterial3d(
                    materials.add(StandardMaterial::from_color(Color::srgb(0.9, 0.9, 0.9))),
                ));
            }
        }
    }
}
