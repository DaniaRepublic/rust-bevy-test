use std::{fmt::Display, time::Duration};

use avian3d::{math::PI, prelude::*};
use bevy::{
    math::Vec3,
    mesh::VertexAttributeValues,
    prelude::*,
    window::{CursorGrabMode, CursorOptions},
};

mod camera_controller;
use camera_controller::*;

fn main() {
    App::new()
        // TUNE THE SOLVER
        // Substeps help with fast motion/tunneling.
        .insert_resource(SubstepCount(12))
        .insert_resource(TimeToSleep(0.2))
        .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            CameraControllerPlugin,
        ))
        .insert_gizmo_config(
            PhysicsGizmos {
                raycast_color: Some(Color::NONE),
                ..default()
            },
            GizmoConfig::default(),
        )
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (
                move_camera,
                reset_camera_rayhit_data,
                shoot_ray_from_camera_draw_intersection,
                modify_display_wall,
                position_and_draw_display_wall,
                place_display_wall,
            )
                .chain(),
        )
        .add_systems(Update, (push_last_floor, handle_ball_despawning).chain())
        .run();
}

#[derive(Component)]
struct LastTowerFloor;

#[derive(Component)]
struct FirstTowerFloor;

/// Marks placeable walls
#[derive(Component)]
struct Wall;

/// Marks placeable wall visual reference for placing
#[derive(Component)]
struct DisplayWall;

/// Marks placeable wall UI element
#[derive(Component)]
struct DisplayWallUI;

#[derive(Component, Clone, Copy)]
enum PlacementMode {
    Grounding,
    Connecting,
    Destroying,
}

impl PlacementMode {
    pub fn cycle_mode(&mut self) {
        *self = match self {
            PlacementMode::Grounding => PlacementMode::Connecting,
            PlacementMode::Connecting => PlacementMode::Destroying,
            PlacementMode::Destroying => PlacementMode::Grounding,
        }
    }
}

impl Display for PlacementMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mode_str = match self {
            PlacementMode::Grounding => "Grounding walls",
            PlacementMode::Connecting => "Connecting walls",
            PlacementMode::Destroying => "Destroying walls",
        };
        write!(f, "{mode_str}")
    }
}

#[derive(Component)]
struct AdditionalRotation(Quat);

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default,
    Wall,
    Ball,
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let gray_mat = materials.add(Color::srgb(0.3, 0.3, 0.3));

    // THE ANCHOR (Ground)
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

    // CONFIGURATION
    let story_height = 3.0;
    let floor_size = 4.0;
    let floor_rot_ang_rad = PI / 12.0;

    let joint_offset = Vec3::ZERO;

    let floor_mesh = meshes.add(Cuboid::new(floor_size, 0.5, floor_size)); // Thicker slabs look better
    let blue_mat = materials.add(Color::srgb(0.2, 0.2, 0.8));

    let mut prev_floor_entity = ground_entity;

    for i in 0..50 {
        let y_pos = (i as f32 * story_height) + 1.0;

        let curr_floor_global_rot_y = floor_rot_ang_rad * i as f32;
        let floor_entity = commands
            .spawn((
                Wall,
                RigidBody::Dynamic,
                Collider::cuboid(floor_size, 0.5, floor_size),
                Mesh3d(floor_mesh.clone()),
                MeshMaterial3d(blue_mat.clone()),
                Transform::from_xyz(0.0, y_pos, 0.0)
                    .with_rotation(Quat::from_rotation_y(curr_floor_global_rot_y)),
                Mass(1000.0),
                AngularInertia {
                    principal: Vec3::splat(1e8f32),
                    local_frame: Quat::IDENTITY,
                },
                LinearDamping(5.0),
                AngularDamping(5.0),
                SleepThreshold {
                    linear: 0.8,
                    angular: 0.8,
                },
            ))
            .id();

        let anchor_prev = if i == 0 {
            // Ground case: Anchor is on the top surface of the ground
            Vec3::new(joint_offset.x, 0.5, joint_offset.z)
        } else {
            Vec3::new(joint_offset.x, story_height / 2.0, joint_offset.z)
        };

        let anchor_curr = Vec3::new(joint_offset.x, -story_height / 2.0, joint_offset.z);

        let curr_joint_rot_y = if i == 0 {
            Quat::IDENTITY
        } else {
            Quat::from_rotation_y(floor_rot_ang_rad)
        };

        commands.spawn(
            FixedJoint::new(prev_floor_entity, floor_entity)
                .with_local_anchor1(anchor_prev)
                .with_local_anchor2(anchor_curr)
                .with_basis(curr_joint_rot_y),
        );

        prev_floor_entity = floor_entity;
        if i == 0 {
            commands.entity(floor_entity).insert(FirstTowerFloor);
        }
    }

    commands.entity(prev_floor_entity).insert(LastTowerFloor);

    commands.spawn((
        Wall,
        RigidBody::Dynamic,
        Collider::cuboid(floor_size, 0.5, floor_size),
        ColliderDensity(8.0),
        Mesh3d(floor_mesh.clone()),
        MeshMaterial3d(blue_mat.clone()),
        Transform::from_xyz(5.0, 5.0, -5.0).with_rotation(Quat::from_rotation_z(PI / 6.0)),
        //AngularInertia {
        //    principal: Vec3::splat(50_000.0),
        //    local_frame: Quat::IDENTITY,
        //},
        //LinearDamping(5.0),
        //AngularDamping(5.0),
        SleepThreshold {
            linear: 0.8,
            angular: 0.8,
        },
    ));

    // Display wall for visualizing placement
    let placement_mode = PlacementMode::Grounding;
    commands.spawn((
        DisplayWall,
        placement_mode,
        Mesh3d(meshes.add(Cuboid::new(0.5, 4.0, 2.0))),
        MeshMaterial3d(materials.add(Color::srgba(0.2, 0.2, 0.8, 0.7))),
        // Add rigid body and collider, but disable them
        RigidBodyComponents(RigidBody::Dynamic, Collider::cuboid(0.5, 4.0, 2.0)),
        RigidBodyDisabled,
        ColliderDisabled,
        Transform::from_xyz(0.0, 0.0, 0.0),
        AdditionalRotation(Quat::IDENTITY),
        Visibility::Hidden,
    ));

    // A state representation for display wall
    commands.spawn((
        DisplayWallUI,
        Text::new(format!(
            "Press T to switch grounding mode: {placement_mode}"
        )),
        TextLayout::new_with_justify(Justify::Center),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(20.0),
            top: Val::Px(20.0),
            ..Default::default()
        },
    ));

    // Create camera entity
    commands.spawn((
        Camera3d::default(),
        CameraController::default().with_sensitivity(8.0),
        Transform::from_xyz(15.0, 5.0, 25.0),
        RayCaster::new(Vec3::ZERO, Dir3::NEG_Z)
            .with_solidness(false)
            .with_max_hits(1)
            .with_max_distance(300.0)
            .with_query_filter(SpatialQueryFilter::from_mask([GameLayer::Default])),
        CameraRayHitData(None),
        MovementSpeed(20.0),
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

#[derive(Component)]
struct RigidBodyComponents(RigidBody, Collider);

#[derive(Component)]
struct MovementSpeed(f32);

fn move_camera(
    time: Res<Time>,
    mut camera_query: Single<(&mut Transform, &MovementSpeed), With<CameraController>>,
    key_inputs: Res<ButtonInput<KeyCode>>,
) {
    let delta_secs = time.delta_secs();

    // if shift is pressed increase speed
    let speed_up = if key_inputs.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]) {
        3.0
    } else {
        1.0
    };

    let mut dir = Vec2::new(
        key_inputs.pressed(KeyCode::KeyD) as u32 as f32
            - key_inputs.pressed(KeyCode::KeyA) as u32 as f32,
        key_inputs.pressed(KeyCode::KeyW) as u32 as f32
            - key_inputs.pressed(KeyCode::KeyS) as u32 as f32,
    );

    dir = dir.normalize_or_zero();
    if dir != Vec2::ZERO {
        let global_dir3d = camera_query.0.forward() * dir.y + camera_query.0.right() * dir.x;
        let movement_speed = camera_query.1.0;
        camera_query.0.translation += global_dir3d * delta_secs * movement_speed * speed_up;
    }
}

#[derive(Component)]
struct CameraRayHitData(Option<RayHitData>);

/// Should run before shoot_ray_from_camera_draw_intersection
fn reset_camera_rayhit_data(
    mut raycaster_query: Single<&mut CameraRayHitData, With<CameraController>>,
) {
    raycaster_query.0 = None;
}

fn shoot_ray_from_camera_draw_intersection(
    mut raycaster_query: Query<
        (&RayCaster, &RayHits, &mut CameraRayHitData),
        With<CameraController>,
    >,
) {
    for (_ray, hits, mut camera_rayhit_data) in &mut raycaster_query {
        for hit in hits.iter() {
            camera_rayhit_data.0 = Some(hit.clone());
            //println!(
            //    "Hit entity {} at {} with normal {}",
            //    hit.entity,
            //    ray.origin + *ray.direction * hit.distance,
            //    hit.normal,
            //);
        }
    }
}

fn modify_display_wall(
    keys: Res<ButtonInput<KeyCode>>,
    mut placeable_wall_query: Single<
        (&mut AdditionalRotation, &mut PlacementMode),
        With<DisplayWall>,
    >,
    mut placeable_wall_ui_query: Single<&mut Text, With<DisplayWallUI>>,
) {
    // Rotate around Y-axis by PI/2 on 'R'
    if keys.just_pressed(KeyCode::KeyR) {
        placeable_wall_query.0.0 *= Quat::from_rotation_y(PI / 2.0);
    }

    // Flip around X-axis by PI/2 on 'F'
    if keys.just_pressed(KeyCode::KeyF) {
        placeable_wall_query.0.0 *= Quat::from_rotation_z(PI / 2.0);
    }

    // Switch between absolute elements and joined
    if keys.just_pressed(KeyCode::KeyT) {
        placeable_wall_query.1.cycle_mode();
        let ground_walls = *placeable_wall_query.1;
        placeable_wall_ui_query.0 = format!("Press T to switch grounding mode: {ground_walls}")
    }
}

/// Smoothly transfers object to its position
/// When LMB is pressed, transfers to end position immediately since in this case,
/// the next system [place_display_wall] will place the wall down
fn position_and_draw_display_wall(
    mut commands: Commands,
    keys: Res<ButtonInput<MouseButton>>,
    time: Res<Time>,
    meshes: Res<Assets<Mesh>>,
    camera_rayhit_query: Single<(&RayCaster, &CameraRayHitData), With<CameraController>>,
    placeable_wall_query: Single<
        (
            Entity,
            &RigidBodyComponents,
            &mut Transform,
            &AdditionalRotation,
            &mut Visibility,
            &Mesh3d,
        ),
        With<DisplayWall>,
    >,
    rigid_bodies: Query<&RigidBody>,
    colliders: Query<&Collider>,
) {
    let lmb_pressed = keys.just_pressed(MouseButton::Left);

    let (ray, camera_rayhit_data) = camera_rayhit_query.into_inner();
    let (entity, rb_components, mut wall_transform, additional_rotation, mut visibility, mesh3d) =
        placeable_wall_query.into_inner();

    if let Some(rayhit_data) = camera_rayhit_data.0 {
        // If entity doesn't have rigid body and collider, add them
        if let Err(_) = rigid_bodies.get(entity) {
            commands.entity(entity).insert(rb_components.0.clone());
        }
        if let Err(_) = colliders.get(entity) {
            commands.entity(entity).insert(rb_components.1.clone());
        }

        let delta_secs = time.delta_secs();

        // Rotate mesh to rotation of hit normal, assuming 0 rotation is positive Y-axis
        // this doesn't change the local rotation introduced by modify_display_wall system
        // it only adds on top of it a global rotation that hit-normal vector makes with a surface
        // it hits.

        let up = rayhit_data.normal;
        let mut forward = Vec3::Z - Vec3::Z.project_onto(up); // try to keep world +Z as forward
        if forward.length_squared() < 1e-6 {
            forward = Vec3::X; // fallback
        }
        let forward = forward.normalize();
        let right = up.cross(forward).normalize();
        let corrected_forward = right.cross(up);

        let align_rotation = Quat::from_mat3(&Mat3::from_cols(right, up, corrected_forward));

        // First align with normal and then add additional rotation.
        let target_rotation = align_rotation * additional_rotation.0;

        let t = if lmb_pressed {
            1.0
        } else {
            1.0 - (-32.0 * delta_secs).exp()
        };
        wall_transform.rotation = wall_transform.rotation.slerp(target_rotation, t);

        // Strategy to snap to ground:
        // find the smallest distance of local point in -normal direction from hit_point of all vertices in a mesh
        // move_by = origin + normal * distance
        // move mesh by move_by in direction of normal
        //
        // anvancement might include adding a collider for intersection detection and not
        // letting shape get in places it doesn't fit

        let mut smallest_displacement: Option<f32> = None;

        if let Some(mesh) = meshes.get(mesh3d) {
            if let Some(vertex_attribute) = mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
                match vertex_attribute {
                    VertexAttributeValues::Float32x3(vertices) => {
                        for vertex in vertices {
                            // Respect rotation of mesh
                            let rotated_vertex =
                                wall_transform.rotation * Vec3::from_array(*vertex);

                            // We are pushing in the direction of rayhit normal
                            let distance = rayhit_data.normal.dot(rotated_vertex);

                            if let Some(smallest) = smallest_displacement {
                                if distance < smallest {
                                    smallest_displacement = Some(distance);
                                }
                            } else {
                                smallest_displacement = Some(distance);
                            }
                        }
                    }
                    _ => {
                        warn!("Found vertex_attribute but values aren't of type Float32x3!");
                    }
                }
            }
        }

        let snapping_displacement = if let Some(smallest) = smallest_displacement {
            rayhit_data.normal * smallest
        } else {
            Vec3::ZERO
        };

        // Determine point where ray hit
        let global_hit_point = ray.global_origin() + ray.global_direction() * rayhit_data.distance;
        // Move shape out of the surface (to remove intersections)
        let snapped_point = global_hit_point - snapping_displacement;

        let transform_lerp_factor = if lmb_pressed { 1.0 } else { 25.0 * delta_secs };
        wall_transform.translation = if wall_transform.translation.distance(snapped_point) < 8.0 {
            wall_transform
                .translation
                .lerp(snapped_point, transform_lerp_factor)
        } else {
            snapped_point
        };

        *visibility = Visibility::Visible;
    } else {
        // If entity has rigid body and collider, remove them
        if let Ok(_) = rigid_bodies.get(entity) {
            commands.entity(entity).remove::<RigidBody>();
        }
        if let Ok(_) = colliders.get(entity) {
            commands.entity(entity).remove::<Collider>();
        }

        // Hide wall visual
        *visibility = Visibility::Hidden;
    }
}

/// When placing an element, joints need to connect it to related bodies.
/// It should be joined to ground when placing on ground (perhaps switch modes
/// between grounding and default mode); to other structural elements when connecting to them.
/// Also, there needs to be some sort of magnet to 'snap' elements together.
/// This magnet might be a calculation on the edge of element to which connecting by offsetting
/// away from it by the half-depth of connected element.
fn place_display_wall(
    mut commands: Commands,
    keys: Res<ButtonInput<MouseButton>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_rayhit_query: Single<&CameraRayHitData, With<CameraController>>,
    placeable_wall_query: Single<
        (
            &Transform,
            &Mesh3d,
            &MeshMaterial3d<StandardMaterial>,
            &RigidBodyComponents,
            &PlacementMode,
            &Visibility,
        ),
        With<DisplayWall>,
    >,
    window_query: Single<(&Window, &CursorOptions)>,
    transforms_query: Query<&Transform>,
) {
    // In order to place a wall we need to know which direction is ground
    let Some(rayhit_data) = camera_rayhit_query.0 else {
        return;
    };
    let hit_entity = rayhit_data.entity;
    let hit_normal = rayhit_data.normal;

    let window_focused = window_query.0.focused;
    let cursor_locked = window_query.1.grab_mode == CursorGrabMode::Locked;
    let wall_visible = placeable_wall_query.5 == Visibility::Visible;

    if keys.just_pressed(MouseButton::Left) && window_focused && cursor_locked && wall_visible {
        let (wall_transform, mesh3d, material3d, rb_components, placement_mode, _) =
            placeable_wall_query.into_inner();

        // Clone the mesh handle directly (meshes are shared, no need to clone the actual Mesh data)
        let mesh_handle = mesh3d.0.clone();

        // Get the original material, clone it, modify alpha, and add to assets for a new handle
        let Some(original_material) = materials.get(&material3d.0) else {
            warn!("Couldn't find material in place_display_wall.");
            return;
        };
        let mut new_material = original_material.clone();
        new_material.base_color = new_material.base_color.with_alpha(1.0);
        let new_material_handle = materials.add(new_material);

        match placement_mode {
            PlacementMode::Grounding => {
                println!("grounding wall");

                // To place on ground, create an invisible static body for the sole purpose of
                // connecting new piece to it.
                // It should be placed in the direction opposite to hit normal and some distance away
                // from new object's center.

                let other_translation = (wall_transform.translation - hit_normal * 2.0).into();
                let other_rotation = Quat::IDENTITY;
                let other_transform = Transform::from_isometry(Isometry3d {
                    rotation: other_rotation,
                    translation: other_translation,
                });

                let other_entity = commands.spawn((RigidBody::Static, other_transform)).id();

                let new_entity = commands
                    .spawn((
                        Wall,
                        Mesh3d(mesh_handle),
                        MeshMaterial3d(new_material_handle),
                        wall_transform.clone(),
                        rb_components.0.clone(),
                        rb_components.1.clone(),
                        ColliderDensity(8.0),
                        SleepThreshold {
                            linear: 0.6,
                            angular: 0.6,
                        },
                    ))
                    .id();

                // 1. Define the joint frame to be exactly at the center of the NEW entity.
                // Relative to the new entity, its own center is Vec3::ZERO.
                let anchor_on_new = Vec3::ZERO;
                let basis_on_new = Quat::IDENTITY;

                // 2. Calculate where the new entity is relative to the HIT entity.
                // We convert the new wall's world position into the hit object's local space.
                let anchor_on_hit = other_transform.rotation.inverse()
                    * (wall_transform.translation - other_transform.translation);
                // 3. Calculate the relative rotation.
                let basis_on_hit = other_transform.rotation.inverse() * wall_transform.rotation;

                commands.spawn(
                    FixedJoint::new(new_entity, other_entity)
                        // Body 1 (New Wall): Connect at its center
                        .with_local_anchor1(anchor_on_new)
                        .with_local_basis1(basis_on_new)
                        // Body 2 (Hit Object): Connect at the calculated relative offset
                        .with_local_anchor2(anchor_on_hit)
                        .with_local_basis2(basis_on_hit),
                );
            }
            PlacementMode::Connecting => {
                println!("connecting wall");

                // just try connecting it to center of hit body
                if let Ok(other_transform) = transforms_query.get(hit_entity) {
                    let new_entity = commands
                        .spawn((
                            Wall,
                            Mesh3d(mesh_handle),
                            MeshMaterial3d(new_material_handle),
                            wall_transform.clone(),
                            rb_components.0.clone(),
                            rb_components.1.clone(),
                            ColliderDensity(8.0),
                            SleepThreshold {
                                linear: 0.6,
                                angular: 0.6,
                            },
                        ))
                        .id();

                    // 1. Define the joint frame to be exactly at the center of the NEW entity.
                    // Relative to the new entity, its own center is Vec3::ZERO.
                    let anchor_on_new = Vec3::ZERO;
                    let basis_on_new = Quat::IDENTITY;

                    // 2. Calculate where the new entity is relative to the HIT entity.
                    // We convert the new wall's world position into the hit object's local space.
                    let anchor_on_hit = other_transform.rotation.inverse()
                        * (wall_transform.translation - other_transform.translation);

                    // 3. Calculate the relative rotation.
                    let basis_on_hit = other_transform.rotation.inverse() * wall_transform.rotation;

                    commands.spawn(
                        FixedJoint::new(new_entity, hit_entity)
                            // Body 1 (New Wall): Connect at its center
                            .with_local_anchor1(anchor_on_new)
                            .with_local_basis1(basis_on_new)
                            // Body 2 (Hit Object): Connect at the calculated relative offset
                            .with_local_anchor2(anchor_on_hit)
                            .with_local_basis2(basis_on_hit),
                    );
                } else {
                    // NOTE can it not have transform if ray hit it?
                    // If it doesn't have transform, create some distance away in -normal direction
                    // from wall center.
                }
            }
            PlacementMode::Destroying => {
                println!("destroying wall");
            }
        }
    }
}

const BALL_VEL: f32 = 30.0;

#[derive(Component)]
struct BallDestructionTimer(Timer);

fn push_last_floor(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    keypresses: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&Transform, With<CameraController>>,
) {
    if keypresses.just_pressed(KeyCode::KeyB) {
        let shoot_vector = camera_query.forward();
        let ball_spawn_p = camera_query.translation;
        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            CollisionLayers::new([GameLayer::Ball], [GameLayer::Default]),
            ColliderDensity(8.0),
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.3, 0.4, 0.9)))),
            Transform::from_translation(ball_spawn_p),
            LinearVelocity(shoot_vector * BALL_VEL),
            BallDestructionTimer(Timer::new(Duration::from_secs_f32(5.0), TimerMode::Once)),
        ));
    }
}

fn handle_ball_despawning(
    mut commands: Commands,
    time: Res<Time>,
    mut ball_destruction_timers_query: Query<(Entity, &mut BallDestructionTimer)>,
) {
    for (entity, mut timer) in &mut ball_destruction_timers_query {
        timer.0.tick(time.delta());
        if timer.0.is_finished() {
            commands.entity(entity).despawn();
        }
    }
}
