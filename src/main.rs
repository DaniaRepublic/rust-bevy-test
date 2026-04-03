//! The app in this file demoes some destruction capabilities of physics scenes.

use std::{collections::HashMap, fmt::Display, time::Duration};

use avian_rerecast::AvianBackendPlugin;
use avian3d::{
    math::{FRAC_PI_2, PI},
    prelude::*,
};
use bevy::{
    asset::RenderAssetUsages,
    audio::Volume,
    camera::primitives::Aabb,
    color::palettes,
    input::common_conditions::input_just_pressed,
    math::Vec3,
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
    scene::SceneInstance,
    window::{CursorGrabMode, CursorOptions},
};
use vleue_navigator::prelude::*;

use bevy::remote::{RemotePlugin, http::RemoteHttpPlugin};
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use bevy_rerecast::{debug::DetailNavmeshGizmo, prelude::*, rerecast::DetailNavmesh};

mod camera_controller;
use camera_controller::*;

mod sourcelike_kinematic_controller;
use sourcelike_kinematic_controller::*;

mod helpers;
use helpers::*;

/// NavmeshUpdaterPlugin obstacle (add to collider to be used by plugin in navmesh construction)
//#[derive(Component)]
//struct Obstacle;

fn main() {
    App::new()
        // ====== RESOURCES ======
        .insert_resource(SubstepCount(12))
        .insert_resource(TimeToSleep(0.2))
        .insert_resource(Gravity(Vec3::new(0.0, -9.81, 0.0)))
        // ====== MESSAGES ======
        .add_message::<NavmeshRegeneration>()
        // ====== PLUGINS ======
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            SourceControllerPlugin,
            VleueNavigatorPlugin,
        ))
        .add_plugins((RemotePlugin::default(), RemoteHttpPlugin::default()))
        .add_plugins((NavmeshPlugins::default(), AvianBackendPlugin::default()))
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(CameraControllerPlugin)
        .insert_gizmo_config(
            PhysicsGizmos {
                raycast_color: Some(Color::NONE),
                ..default()
            },
            GizmoConfig::default(),
        )
        // ====== SYSTEMS ======
        .add_systems(
            Startup,
            (load_sounds, load_placeables, setup_scene, generate_navmesh).chain(),
        )
        .add_systems(Update, player_input.after(run_camera_controller))
        .add_systems(
            Update,
            (
                reset_camera_rayhit_data,
                shoot_ray_from_camera,
                modify_display_object,
                position_and_draw_display_object,
                place_display_object,
                add_colliders_to_new_meshes,
            )
                .chain(),
        )
        .add_systems(Update, (shoot_ball, handle_ball_despawning).chain())
        .add_systems(
            Update,
            (
                find_path.run_if(input_just_pressed(KeyCode::KeyN)),
                display_navigator_path,
            ),
        )
        .add_systems(
            PostUpdate,
            (remove_destroyed_entities, regenerate_navmesh).chain(),
        )
        // ====== OBSERVERS ======
        .add_observer(on_navmesh_ready)
        // =======================
        .run();
}

// ===============================================
// ========== COMPONENTS and RESOURCES  ==========
// ===============================================

/// Collision layers for the game
#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default,
    Player,
    Ball,
}

/// Stores all game sounds.
/// Storing them all at once shouldn't be a huge problem:
/// 1s of ogg file is around 20 Kb, so an hour is around 72 Mb, which is fine to store in RAM.
#[derive(Resource)]
struct GameSounds {
    metal_ball_concrete_collision: Handle<AudioSource>,
    concrete_crumbling: Handle<AudioSource>,
    steampunk_weapon_shot: Handle<AudioSource>,
}

#[derive(Component)]
struct CameraRayHitData(Option<RayHitData>);

/// Component that destructible objects have and it holds usefull values for such objects
/// NOTE: if more destruction scenarios arrise, it makes sense to use enum here instead of struct.
#[derive(Component, Clone, Copy)]
struct DestructibleParams {
    /// Max impulse a destructible object can withstand
    impulse_resistance: f32,
}

impl DestructibleParams {
    pub fn new(impulse_resistance: f32) -> Self {
        Self { impulse_resistance }
    }
}

/// Marks destroyed object. See [DestructibleParams] for specifying destruction conditions.
#[derive(Component)]
struct DestroyedObject;

/// Marks placeable walls
#[derive(Component)]
struct Wall;

/// Marks placeable element's visual reference for placing
#[derive(Component)]
struct PlaceableDisplay;

/// Marks placeable wall UI element
#[derive(Component)]
struct DisplayWallUI;

#[derive(Component)]
struct PlacementModeUI;

#[derive(Component, Clone, Copy)]
enum PlacementMode {
    Static,
    Grounding,
    Destroying,
}

impl PlacementMode {
    pub fn cycle_mode(&mut self) {
        *self = match self {
            PlacementMode::Static => PlacementMode::Grounding,
            PlacementMode::Grounding => PlacementMode::Destroying,
            PlacementMode::Destroying => PlacementMode::Static,
        }
    }
}

impl Display for PlacementMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mode_str = match self {
            PlacementMode::Static => "Placing static object",
            PlacementMode::Grounding => "Grounding object",
            PlacementMode::Destroying => "Destroying object",
        };
        write!(f, "{mode_str}")
    }
}

#[derive(Component)]
struct AdditionalRotation(Quat);

/// Marks projectiles
#[derive(Component)]
struct Projectile;

#[derive(Component)]
struct DestructionTimer(Timer);

/// Marks ball projectile
#[derive(Component)]
struct Ball;

/// Enum with all entities
#[derive(Component, Clone, Copy)]
enum PursuitProperty {
    Wall,
    Target,
    Mob,
}

impl PursuitProperty {
    fn add_appropriate_tag_to_entity(&self, mut commands: Commands, entity: Entity) {
        let mut entity_commands = commands.entity(entity);
        match self {
            PursuitProperty::Wall => {
                entity_commands.insert(Wall);
            }
            PursuitProperty::Target => {
                entity_commands.insert(Target);
            }
            PursuitProperty::Mob => {
                entity_commands.insert(Mob);
            }
        }
    }

    fn cycle_mode(&mut self) {
        *self = match self {
            PursuitProperty::Wall => PursuitProperty::Target,
            PursuitProperty::Target => PursuitProperty::Mob,
            PursuitProperty::Mob => PursuitProperty::Wall,
        }
    }
}

impl Display for PursuitProperty {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let property_str = match self {
            PursuitProperty::Wall => "PursuitProperty::Wall",
            PursuitProperty::Target => "PursuitProperty::Target",
            PursuitProperty::Mob => "PursuitProperty::Mob",
        };
        write!(f, "{property_str}")
    }
}

#[derive(Component)]
#[relationship(relationship_target = FollowedBy)]
struct Following(Entity);

#[derive(Component)]
#[relationship_target(relationship = Following, linked_spawn)]
struct FollowedBy(Vec<Entity>);

#[derive(Component)]
struct Navigator;

/// Target that mobs persue
#[derive(Component)]
struct Target;

#[derive(Component)]
struct Mob;

#[derive(Component)]
struct NavPath {
    current: Vec3,
    next: Vec<Vec3>,
}

type PlaceableObjectType = Handle<Scene>;

/// Objects that can be placed
#[derive(Resource)]
struct PlaceableObjects {
    wall: PlaceableObjectType,
    target: PlaceableObjectType,
    mob: PlaceableObjectType,
}

#[derive(Component)]
struct SelectedPlaceableObject(PlaceableObjectType);

// =============================
// ========== SYSTEMS ==========
// =============================

fn load_placeables(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.insert_resource(PlaceableObjects {
        wall: asset_server.load(GltfAssetLabel::Scene(0).from_asset("models/Primitives/Cube.glb")),
        target: asset_server
            .load(GltfAssetLabel::Scene(0).from_asset("models/Primitives/Cylinder.glb")),
        mob: asset_server
            .load(GltfAssetLabel::Scene(0).from_asset("models/Primitives/Icosphere.glb")),
    });
}

fn load_sounds(mut commands: Commands, asset_server: Res<AssetServer>) {
    commands.insert_resource(GameSounds {
        metal_ball_concrete_collision: asset_server
            .load("sounds/StoneImpact/Bluezone_BC0297_stone_impact_015.ogg"),
        concrete_crumbling: asset_server
            .load("sounds/StoneImpact/Bluezone_BC0297_stone_impact_041.ogg"),
        steampunk_weapon_shot: asset_server.load("sounds/BluezoneCorp - Steampunk Weapon And Textures/Bluezone_BC0296_steampunk_weapon_gun_shot_026_02.ogg"),
    });
}

fn detail_navmesh_to_bevy_mesh(dmesh: &DetailNavmesh) -> Mesh {
    let mut indices = Vec::new();
    let mut vertices = Vec::new();
    let mut vertex_map = HashMap::new();

    for mesh in &dmesh.meshes {
        let verts =
            &dmesh.vertices[mesh.base_vertex_index as usize..][..mesh.vertex_count as usize];
        let tris =
            &dmesh.triangles[mesh.base_triangle_index as usize..][..mesh.triangle_count as usize];

        for tri in tris {
            for i in 0..3 {
                let p = verts[tri[i] as usize];
                // Weld vertices by position to ensure connectivity
                let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
                let index = *vertex_map.entry(key).or_insert_with(|| {
                    vertices.push(p);
                    (vertices.len() - 1) as u32
                });
                indices.push(index);
            }
        }
    }

    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vertices)
    .with_inserted_indices(Indices::U32(indices))
    .with_computed_normals()
}

#[derive(Resource)]
struct NavmeshHandle(Handle<Navmesh>);

const AGENT_RADIUS: f32 = 0.3;
const AGENT_HEIGHT: f32 = 0.6;

fn get_navmesh_settings() -> NavmeshSettings {
    let mut settings = NavmeshSettings::from_agent_3d(AGENT_RADIUS, AGENT_HEIGHT);
    settings.walkable_slope_angle = 65.0_f32.to_radians();
    settings.walkable_climb = 0.5;
    settings
}

fn generate_navmesh(mut commands: Commands, mut generator: NavmeshGenerator) {
    let navmesh_handle = generator.generate(get_navmesh_settings());
    // Save navmesh handle so it doesn't get dropped
    commands.spawn(DetailNavmeshGizmo::new(&navmesh_handle));
    commands.insert_resource(NavmeshHandle(navmesh_handle));
}

#[derive(Message)]
struct NavmeshRegeneration;

fn regenerate_navmesh(
    mut message_reader: MessageReader<NavmeshRegeneration>,
    mut generator: NavmeshGenerator,
    navmesh_handle_res: Res<NavmeshHandle>,
) {
    if !message_reader.is_empty() {
        message_reader.clear();

        if generator.regenerate(&navmesh_handle_res.0, get_navmesh_settings()) {
            info!("Regeneration queued successfully.");
        } else {
            warn!("Regeneration queuing failed. Already queued.");
        }
    }
}

#[derive(Resource)]
pub struct NavMeshHandle(Handle<NavMesh>);

#[derive(Component)]
struct NavMeshRepr;

fn on_navmesh_ready(
    trigger: On<NavmeshReady>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    navmeshes: Res<Assets<Navmesh>>,
    mut vleue_navmeshes: ResMut<Assets<NavMesh>>,
    // Query returns Option<Single<...>> to handle 0 or 1 entity safely
    vleue_navmesh_repr_q: Option<Single<Entity, With<NavMeshRepr>>>,
) {
    let asset_id = trigger.event().0;

    if let Some(navmesh) = navmeshes.get(asset_id) {
        let dmesh = &navmesh.detail;

        let mesh = detail_navmesh_to_bevy_mesh(&dmesh);

        if let Some(vleue_navmesh) = NavMesh::from_bevy_mesh(&mesh) {
            let vleue_mesh_repr = vleue_navmesh.to_mesh().with_computed_normals();

            // 2. Debug Check: Ensure the converted visual mesh is valid
            let v_count = vleue_mesh_repr.count_vertices();
            info!("Regenerated NavMesh visual with {} vertices.", v_count);

            let new_mesh_repr = Mesh3d(meshes.add(vleue_mesh_repr));

            if let Some(vleue_navmesh_repr) = vleue_navmesh_repr_q {
                let vleue_navmesh_entity = vleue_navmesh_repr.into_inner();

                // CRITICAL FIX: Remove Aabb to force recalculation of culling bounds
                commands
                    .entity(vleue_navmesh_entity)
                    .insert(new_mesh_repr)
                    .remove::<Aabb>();
            } else {
                // Initial spawn
                commands.spawn((
                    NavMeshRepr,
                    new_mesh_repr,
                    MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::Srgba(
                        Srgba {
                            red: 0.8,
                            green: 0.1,
                            blue: 0.1,
                            alpha: 0.5,
                        },
                    )))),
                    Transform::from_xyz(0.0, 1.0, 0.0),
                ));
            }

            let vleue_navmesh_handle = vleue_navmeshes.add(vleue_navmesh);
            commands.insert_resource(NavMeshHandle(vleue_navmesh_handle));
            info!("Navmesh converted and resource inserted!");
        } else {
            warn!("Failed to convert bevy_rerecast mesh to vleue_navigator mesh.");
        }
    }
}

fn find_path(
    mut commands: Commands,
    navmeshes: Res<Assets<NavMesh>>,
    navmesh: Res<NavMeshHandle>,
    navigator_s: Single<(Entity, &Transform, &Following), With<Navigator>>,
    transform_q: Query<&Transform>,
) {
    if let Some(navmesh) = navmeshes.get(&navmesh.0) {
        let (entity, transform, following) = navigator_s.into_inner();

        let Ok(following_transform) = transform_q.get(following.0) else {
            return;
        };

        info!("target transform: {}", following_transform.translation);

        if let Some(path) =
            navmesh.transformed_path(transform.translation, following_transform.translation)
        {
            if let Some((first, remaining)) = path.path.split_first() {
                let mut remaining = remaining.to_vec();
                remaining.reverse();
                commands.entity(entity).insert(NavPath {
                    current: *first,
                    next: remaining.clone(),
                });
                info!(
                    "found path from {:?} to {:?}: {:?}",
                    first,
                    remaining.first(),
                    path
                );
            }
        } else {
            info!(
                "no path found from {:?} to {:?}",
                transform.translation, following_transform.translation
            );
        }
    }
}

fn display_navigator_path(navigator: Query<(&Transform, &NavPath)>, mut gizmos: Gizmos) {
    for (transform, path) in &navigator {
        let mut to_display = path.next.clone();
        to_display.push(path.current);
        to_display.push(transform.translation);
        to_display.reverse();
        if !to_display.is_empty() {
            gizmos.linestrip(
                to_display.iter().map(|xz| Vec3::new(xz.x, xz.y, xz.z)),
                Color::Srgba(palettes::tailwind::AMBER_400),
            );
        }
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
    placeable_objects: Res<PlaceableObjects>,
) {
    let gray_mat = materials.add(Color::srgb(0.3, 0.3, 0.3));

    // THE ANCHOR (Ground)
    commands.spawn((
        Name::new("Ground"),
        RigidBody::Static,
        Collider::cuboid(90.0, 1.0, 90.0),
        Mesh3d(meshes.add(Cuboid::new(90.0, 1.0, 90.0))),
        MeshMaterial3d(gray_mat.clone()),
        Transform::from_xyz(0.0, -0.5, 0.0),
    ));

    let floor_size = 4.0;
    let floor_mesh = meshes.add(Cuboid::new(floor_size, 0.5, floor_size));
    let blue_mat = materials.add(Color::srgb(0.2, 0.2, 0.8));

    commands.spawn((
        Wall,
        RigidBody::Dynamic,
        Collider::cuboid(floor_size, 0.5, floor_size),
        ColliderDensity(8.0),
        Mesh3d(floor_mesh.clone()),
        MeshMaterial3d(blue_mat.clone()),
        Transform::from_xyz(5.0, 2.5, -5.0).with_rotation(Quat::from_rotation_z(PI / 2.0)),
        // Add audio player to the block
        AudioPlayer::new(asset_server.load("sounds/Windless Slopes.ogg")),
        PlaybackSettings::LOOP.with_spatial(true),
    ));

    // Display wall for visualizing placement
    let placement_mode = PlacementMode::Static;
    let pursuit_property = PursuitProperty::Wall;
    commands.spawn((
        PlaceableDisplay,
        placement_mode,
        pursuit_property,
        SelectedPlaceableObject(placeable_objects.wall.clone()),
        SceneRoot(placeable_objects.wall.clone()),
        // Add rigid body and collider, but disable them
        RigidBody::Static,
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

    commands.spawn((
        PlacementModeUI,
        Text::new(format!(
            "Press C to switch pursuit mode: {pursuit_property}"
        )),
        TextLayout::new_with_justify(Justify::Center),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(20.0),
            top: Val::Px(40.0),
            ..Default::default()
        },
    ));

    // Create camera entity
    let player_entity = commands
        .spawn((
            Name::new("Player"),
            Transform::from_xyz(15.0, 2.0, 25.0),
            (
                RigidBody::Kinematic,
                LockedAxes::ROTATION_LOCKED,
                Collider::capsule(0.4, 1.0),
                ColliderDensity(5.0),
                CollisionLayers::new([GameLayer::Player], [GameLayer::Default]),
                CollisionEventsEnabled,
                // Debug rendering interferes with camera view
                DebugRender::none(),
            ),
            // Controller bundle
            (
                SourceController::default(),
                CharacterInput::default(),
                CharacterState::default(),
            ),
            // Audio listener
            SpatialListener::new(0.2),
        ))
        .with_child((
            Camera3d::default(),
            CameraController::default(),
            Transform::from_xyz(0.0, 0.0, 0.0),
            (
                RayCaster::new(Vec3::ZERO, Dir3::NEG_Z)
                    .with_solidness(false)
                    .with_max_hits(1)
                    .with_max_distance(300.0)
                    .with_query_filter(SpatialQueryFilter::from_mask([GameLayer::Default])),
                CameraRayHitData(None),
            ),
        ))
        .observe(player_collision_mark_destroyed_entities)
        .id();

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            illuminance: 10000.0,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_x(-0.5)),
    ));

    // Add navmesh
    commands.spawn((
        NavMeshSettings {
            fixed: Triangulation::from_outer_edges(&[
                vec2(-100.0, -100.0),
                vec2(100.0, -100.0),
                vec2(100.0, 100.0),
                vec2(-100.0, 100.0),
            ]),
            simplify: 0.005,
            merge_steps: 0,
            upward_shift: 1.0,
            build_timeout: Some(1.0),
            ..Default::default()
        },
        NavMeshUpdateMode::Direct,
        Transform::from_xyz(0.0, 0.1, 0.0).with_rotation(Quat::from_rotation_x(FRAC_PI_2)),
        NavMeshDebug(palettes::tailwind::RED_600.into()),
    ));

    // Add navigator
    commands.spawn((
        Navigator,
        Following(player_entity),
        Mesh3d(meshes.add(Capsule3d::new(0.3, 1.0))),
        MeshMaterial3d(blue_mat.clone()),
        Transform::from_xyz(-30.0, 0.0, -30.0),
    ));
}

pub fn player_input(
    keys: Res<ButtonInput<KeyCode>>,
    character_controller_q: Single<&mut CharacterInput, With<SourceController>>,
    camera_controller_q: Single<&Transform, With<CameraController>>,
) {
    let mut input = character_controller_q.into_inner();
    let camera_transform = camera_controller_q.into_inner();

    // 1. Get Forward/Right vectors from the Transform
    // Note: transform.forward() returns a Dir3, we convert to Vec3
    let raw_forward = camera_transform.forward().as_vec3();
    let raw_right = camera_transform.right().as_vec3();

    // 2. Flatten them to the XZ plane (Ground)
    // This ensures that looking Up/Down doesn't affect movement speed or direction
    let flat_forward = (raw_forward * Vec3::new(1.0, 0.0, 1.0)).normalize_or_zero();
    let flat_right = (raw_right * Vec3::new(1.0, 0.0, 1.0)).normalize_or_zero();

    // 3. Accumulate movement vector
    let mut wish_dir = Vec3::ZERO;

    if keys.pressed(KeyCode::KeyW) {
        wish_dir += flat_forward;
    }
    if keys.pressed(KeyCode::KeyS) {
        wish_dir -= flat_forward;
    }
    if keys.pressed(KeyCode::KeyD) {
        wish_dir += flat_right;
    }
    if keys.pressed(KeyCode::KeyA) {
        wish_dir -= flat_right;
    }

    // 4. Update the source controller input
    input.wish_dir = wish_dir.normalize_or_zero();
    input.jump = keys.pressed(KeyCode::Space);
    input.duck = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::KeyC);
}

/// Should run before shoot_ray_from_camera
fn reset_camera_rayhit_data(
    mut raycaster_query: Single<&mut CameraRayHitData, With<CameraController>>,
) {
    raycaster_query.0 = None;
}

fn shoot_ray_from_camera(
    mut raycaster_query: Query<
        (&RayCaster, &RayHits, &mut CameraRayHitData),
        With<CameraController>,
    >,
) {
    for (_ray, hits, mut camera_rayhit_data) in &mut raycaster_query {
        for hit in hits.iter() {
            camera_rayhit_data.0 = Some(hit.clone());
        }
    }
}

fn modify_display_object(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    placeable_objects: Res<PlaceableObjects>,
    mut placeable_object_query: Single<
        (
            Entity,
            &mut AdditionalRotation,
            &mut PlacementMode,
            &mut SelectedPlaceableObject,
            &mut PursuitProperty,
        ),
        With<PlaceableDisplay>,
    >,
    mut placeable_ui_query: Single<&mut Text, (With<DisplayWallUI>, Without<PlacementMode>)>,
    mut pursuit_ui_query: Single<&mut Text, (With<PlacementModeUI>, Without<DisplayWallUI>)>,
) {
    let mut changed = false;

    // Rotate around Y-axis by PI/4 on 'R'
    if keys.just_pressed(KeyCode::KeyR) {
        placeable_object_query.1.0 *= Quat::from_rotation_y(PI / 4.0);
        changed = true;
    }

    // Flip around X-axis by PI/2 on 'F'
    if keys.just_pressed(KeyCode::KeyF) {
        placeable_object_query.1.0 *= Quat::from_rotation_z(PI / 2.0);
        changed = true;
    }

    // Normalizing prevents rotational drift
    if changed {
        placeable_object_query.1.0 = placeable_object_query.1.0.normalize();
    }

    // Switch between absolute elements and joined
    if keys.just_pressed(KeyCode::KeyT) {
        placeable_object_query.2.cycle_mode();
        placeable_ui_query.0 = format!(
            "Press T to switch grounding mode: {}",
            *placeable_object_query.2
        );
    }

    // Switch pursuit mode
    if keys.just_pressed(KeyCode::KeyC) {
        placeable_object_query.4.cycle_mode();
        pursuit_ui_query.0 = format!(
            "Press C to switch pursuit mode: {}",
            *placeable_object_query.4
        );
    }

    // Change placeable object
    if keys.just_pressed(KeyCode::Digit1) {
        let object = placeable_objects.wall.clone();
        placeable_object_query.3.0 = object.clone();
        commands
            .entity(placeable_object_query.0)
            .insert(SceneRoot(object.clone()));
    }

    if keys.just_pressed(KeyCode::Digit2) {
        let object = placeable_objects.target.clone();
        placeable_object_query.3.0 = object.clone();
        commands
            .entity(placeable_object_query.0)
            .insert(SceneRoot(object.clone()));
    }

    if keys.just_pressed(KeyCode::Digit3) {
        let object = placeable_objects.mob.clone();
        placeable_object_query.3.0 = object.clone();
        commands
            .entity(placeable_object_query.0)
            .insert(SceneRoot(object.clone()));
    }
}

/// Smoothly transfers object to its position
/// When LMB is pressed, transfers to end position immediately since in this case,
/// the next system [place_display_object] will place the wall down
fn position_and_draw_display_object(
    keys: Res<ButtonInput<MouseButton>>,
    time: Res<Time>,
    // Query for Aabb and GlobalTransform
    child_query: Query<(&Aabb, &GlobalTransform)>,
    scene_spawner: Res<SceneSpawner>,
    camera_rayhit_query: Single<(&RayCaster, &CameraRayHitData), With<CameraController>>,
    placeable_wall_query: Single<
        (
            &mut Transform,
            &AdditionalRotation,
            &mut Visibility,
            &SceneInstance,
            &GlobalTransform,
            &PlacementMode,
        ),
        With<PlaceableDisplay>,
    >,
) {
    let (ray, hit) = camera_rayhit_query.into_inner();
    let (mut transform, add_rot, mut vis, scene, root_global, placement_mode) =
        placeable_wall_query.into_inner();

    let Some(hit_data) = hit.0 else {
        *vis = Visibility::Hidden;
        return;
    };

    // If destroying, don't draw anything
    if matches!(placement_mode, PlacementMode::Destroying) {
        *vis = Visibility::Hidden;
        return;
    }

    // Safety Check: Ensure Normal is valid
    let up = hit_data.normal.normalize_or_zero();
    if up == Vec3::ZERO {
        return;
    }

    *vis = Visibility::Visible;

    // Use a threshold to pick the helper vector.
    // If 'up' is close to Z, use X. Otherwise use Z.
    let helper = if up.dot(Vec3::Z).abs() > 0.99 {
        Vec3::X
    } else {
        Vec3::Z
    };

    // Create the basis vectors
    let right = up.cross(helper).normalize(); // We know this is safe because of the check above
    let fwd = right.cross(up); // Orthogonal by definition

    let align_rot = Quat::from_mat3(&Mat3::from_cols(right, up, fwd));
    let target_rot = (align_rot * add_rot.0).normalize();

    // Smooth rotation
    let t_rot = if keys.just_pressed(MouseButton::Left) {
        1.0
    } else {
        1.0 - (-32.0 * time.delta_secs()).exp()
    };
    if !target_rot.is_nan() {
        transform.rotation = transform.rotation.slerp(target_rot, t_rot);
    }

    // --- 3. Snap to Surface (NaN Protection) ---
    // Check if scale is valid before inverting matrix
    let (scale, _, _) = root_global.to_scale_rotation_translation();
    if scale.min_element() < 1e-4 {
        // Scale is too small or zero, cannot compute inverse safely.
        // Skip snapping this frame to prevent panic.
        return;
    }

    let root_inv = root_global.affine().inverse();
    let mut min_dist = f32::MAX;

    for entity in scene_spawner.iter_instance_entities(**scene) {
        if let Ok((aabb, child_global)) = child_query.get(entity) {
            let to_root = root_inv * child_global.affine();

            for i in 0..8 {
                let corner = aabb.center.to_vec3()
                    + aabb.half_extents.to_vec3()
                        * Vec3::new(
                            if i & 1 == 0 { -1.0 } else { 1.0 },
                            if i & 2 == 0 { -1.0 } else { 1.0 },
                            if i & 4 == 0 { -1.0 } else { 1.0 },
                        );

                // Transform corner to Root Space -> Apply *Current* Transform -> Project to Normal
                let root_local = to_root.transform_point3(corner);

                // Note: We use transform.scale here (Vec3), assuming the object scales uniformly or simple axial.
                // If transform.rotation is NaN, this calculation becomes NaN.
                let dynamic_pos = transform.rotation * (root_local * transform.scale);

                let dist = up.dot(dynamic_pos);
                if dist < min_dist {
                    min_dist = dist;
                }
            }
        }
    }

    // Position Application
    let global_hit_point = ray.global_origin() + ray.global_direction() * hit_data.distance;

    // Prevent bad values if no children found or math failed
    let snap_offset = if min_dist != f32::MAX && !min_dist.is_nan() {
        up * min_dist
    } else {
        Vec3::ZERO
    };

    let target_pos = global_hit_point - snap_offset;

    // Additional Safety: Do not set transform if target is NaN
    if target_pos.is_nan() {
        return;
    }

    let t_pos = if keys.just_pressed(MouseButton::Left) {
        1.0
    } else {
        25.0 * time.delta_secs()
    };

    // Snap instantly if far away (prevent trailing visual artifacts)
    if transform.translation.distance_squared(target_pos) > 64.0 {
        transform.translation = target_pos;
    } else {
        transform.translation = transform.translation.lerp(target_pos, t_pos);
    }
}

/// If placed or removed entity, triggers regeneration of navmesh
fn place_display_object(
    mut message_writer: MessageWriter<NavmeshRegeneration>,
    mut commands: Commands,
    window_query: Single<(&Window, &CursorOptions)>,
    keys: Res<ButtonInput<MouseButton>>,
    hierarchy_query: Query<&ChildOf>,
    camera_rayhit_query: Single<&CameraRayHitData, With<CameraController>>,
    placeable_wall_query: Single<
        (
            &Transform,
            &PlacementMode,
            &SelectedPlaceableObject,
            &PursuitProperty,
            &Visibility,
        ),
        With<PlaceableDisplay>,
    >,
    destructible_query: Query<Entity, With<DestructibleParams>>,
) {
    // In order to place a wall we need to know which direction is ground
    let Some(rayhit_data) = camera_rayhit_query.0 else {
        return;
    };
    let hit_normal = rayhit_data.normal;
    let hit_entity = rayhit_data.entity;

    let window_focused = window_query.0.focused;
    let cursor_locked = window_query.1.grab_mode == CursorGrabMode::Locked;
    let wall_visible = placeable_wall_query.4 == Visibility::Visible;
    let placement_mode_destroy = matches!(placeable_wall_query.1, PlacementMode::Destroying);

    if keys.just_pressed(MouseButton::Left)
        && window_focused
        && cursor_locked
        && (wall_visible || placement_mode_destroy)
    {
        let (self_transform, placement_mode, selected_placeable_object, pursuit_property, _) =
            placeable_wall_query.into_inner();

        let mut added_entity: Option<Entity> = None;

        let mut regenerate_navmesh = false;

        match placement_mode {
            PlacementMode::Static => {
                let entity = commands
                    .spawn((
                        DestructibleParams::new(100.0),
                        SceneRoot(selected_placeable_object.0.clone()),
                        self_transform.clone(),
                        RigidBody::Static,
                    ))
                    .id();

                added_entity = Some(entity);
                regenerate_navmesh = true;
            }
            PlacementMode::Grounding => {
                // To place on ground, create an invisible static body for the sole purpose of
                // connecting new piece to it.
                // It should be placed in the direction opposite to hit normal and some distance away
                // from new object's center.

                let other_translation = (self_transform.translation - hit_normal * 2.0).into();
                let other_rotation = Quat::IDENTITY;
                let other_transform = Transform::from_isometry(Isometry3d {
                    rotation: other_rotation,
                    translation: other_translation,
                });

                let other_entity = commands.spawn((RigidBody::Static, other_transform)).id();

                let new_entity = commands
                    .spawn((
                        DestructibleParams::new(100.0),
                        SceneRoot(selected_placeable_object.0.clone()),
                        self_transform.clone(),
                        RigidBody::Dynamic,
                        ColliderDensity(8.0),
                        LinearDamping(2.0),
                        AngularDamping(2.0),
                        SleepThreshold {
                            linear: 0.6,
                            angular: 0.6,
                        },
                    ))
                    .id();

                // Define the joint frame to be exactly at the center of the NEW entity.
                // Relative to the new entity, its own center is Vec3 ZERO.
                let anchor_on_new = Vec3::ZERO;
                let basis_on_new = Quat::IDENTITY;

                // Calculate where the new entity is relative to the HIT entity.
                // We convert the new wall's world position into the hit object's local space.
                let anchor_on_hit = other_transform.rotation.inverse()
                    * (self_transform.translation - other_transform.translation);
                // Calculate the relative rotation.
                let basis_on_hit = other_transform.rotation.inverse() * self_transform.rotation;

                commands.spawn(
                    FixedJoint::new(new_entity, other_entity)
                        .with_local_anchor1(anchor_on_new)
                        .with_local_basis1(basis_on_new)
                        .with_local_anchor2(anchor_on_hit)
                        .with_local_basis2(basis_on_hit),
                );

                added_entity = Some(new_entity);
                regenerate_navmesh = true;
            }
            PlacementMode::Destroying => {
                // Go through all ansestors and check if any have DestructibleParams
                let destructible_parent_opt = hierarchy_query
                    .iter_ancestors(hit_entity)
                    .find(|a| destructible_query.get(*a).is_ok());

                if let Some(destructible_parent) = destructible_parent_opt {
                    commands.entity(destructible_parent).insert(DestroyedObject);
                    regenerate_navmesh = true;
                }
            }
        }

        if let Some(entity) = added_entity {
            pursuit_property.add_appropriate_tag_to_entity(commands, entity);
        }

        if regenerate_navmesh == true {
            message_writer.write(NavmeshRegeneration);
        }
    }
}

fn add_colliders_to_new_meshes(
    mut commands: Commands,
    hierarchy_query: Query<&ChildOf>,
    placeable_display_single: Single<Entity, With<PlaceableDisplay>>,
    names_query: Query<&Name>,
    new_mesh_query: Query<(Entity, &ChildOf), Added<Mesh3d>>,
) {
    let placeable_display_entity = placeable_display_single.into_inner();
    for (entity, parent) in new_mesh_query {
        if let Ok(parent_name) = names_query.get(parent.0) {
            if parent_name.starts_with("collider_") {
                if is_descendant_of(placeable_display_entity, entity, hierarchy_query) {
                    commands.entity(entity).insert((
                        ColliderConstructor::ConvexHullFromMesh,
                        ColliderDisabled,
                        Visibility::Hidden,
                    ));
                } else {
                    commands
                        .entity(entity)
                        .insert((ColliderConstructor::ConvexHullFromMesh, Visibility::Hidden));
                }
            }
        }
    }
}

fn player_collision_mark_destroyed_entities(
    observation: On<CollisionStart>,
    mut message_writer: MessageWriter<NavmeshRegeneration>,
    mut commands: Commands,
    game_sounds: Res<GameSounds>,
    collisions: Collisions,
    destructible_query: Query<&DestructibleParams>,
    player_query: Single<(&LinearVelocity, &ComputedMass), With<CameraController>>,
    transforms_query: Query<&Transform>,
) {
    let observer_collider = observation.event().event_target();

    // Find other body as collider might be attached to a child entity
    let other_body = if observation.collider1 == observer_collider {
        observation.body2.unwrap_or(observation.collider2)
    } else {
        observation.body1.unwrap_or(observation.collider1)
    };

    // If body we hit isn't destructible, then simply return
    // NOTE: Or maybe damage the player on impact
    let Ok(destructible_params) = destructible_query.get(other_body).cloned() else {
        return;
    };

    let (lin_vel, mass) = *player_query;

    if let Some(contacts) = collisions.get(observation.collider1, observation.collider2) {
        if let Some(manifold) = contacts.manifolds.first() {
            // Kinematic bodies don't have impulse calculated by engine, so do it manually.
            // NOTE: This implementation doesn't take the other body's velocity or mass.
            let contact_normal = manifold.normal;
            // We don't care which way the normal points
            let collision_speed = lin_vel.dot(contact_normal).abs();
            let collision_impulse = collision_speed * mass.value();

            // Mark entity for destruction if impulse is sufficient
            if collision_impulse > destructible_params.impulse_resistance {
                commands.entity(other_body).insert(DestroyedObject);
                message_writer.write(NavmeshRegeneration);

                // Spawn wall destruction sound at destructible's position
                if let Ok(other_transform) = transforms_query.get(other_body) {
                    commands.spawn((
                        other_transform.clone(),
                        AudioPlayer::new(game_sounds.concrete_crumbling.clone()),
                        PlaybackSettings::DESPAWN
                            .with_spatial(true)
                            .with_spatial_scale(bevy::audio::SpatialScale::new(0.25))
                            .with_volume(Volume::Linear(2.0)),
                    ));
                }
            }
        }
    }
}

fn remove_destroyed_entities(
    mut commands: Commands,
    destroyed_query: Query<Entity, With<DestroyedObject>>,
) {
    for destroyed_entity in destroyed_query {
        commands.entity(destroyed_entity).despawn();
    }
}

fn shoot_ball(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    game_sounds: Res<GameSounds>,
    keypresses: Res<ButtonInput<KeyCode>>,
    camera_query: Single<(Entity, &GlobalTransform), With<CameraController>>,
) {
    if keypresses.just_pressed(KeyCode::KeyB) {
        let (cam_entity, cam_transform) = *camera_query;
        let shoot_vector = cam_transform.forward();
        let ball_spawn_p = cam_transform.translation();
        let ball_spawn_transform = Transform::from_translation(ball_spawn_p);
        commands
            .spawn((
                RigidBody::Dynamic,
                Collider::sphere(0.5),
                CollisionLayers::new([GameLayer::Ball], [GameLayer::Default, GameLayer::Ball]),
                ColliderDensity(8.0),
                CollisionEventsEnabled,
                Mesh3d(meshes.add(Sphere::new(0.5))),
                MeshMaterial3d(
                    materials.add(StandardMaterial::from_color(Color::srgb(0.3, 0.4, 0.9))),
                ),
                ball_spawn_transform.clone(),
                Projectile,
                Ball,
                LinearVelocity(shoot_vector * 30.0),
                DestructionTimer(Timer::new(Duration::from_secs_f32(10.0), TimerMode::Once)),
            ))
            .observe(ball_collision_mark_destroyed_entities);
        // Spawn sound
        commands.entity(cam_entity).with_child((
            Transform::IDENTITY,
            AudioPlayer::new(game_sounds.steampunk_weapon_shot.clone()),
            PlaybackSettings::DESPAWN
                .with_spatial(true)
                .with_volume(bevy::audio::Volume::Linear(0.35)),
        ));
    }
}

fn handle_ball_despawning(
    mut commands: Commands,
    time: Res<Time>,
    mut ball_destruction_timers_query: Query<(Entity, &mut DestructionTimer)>,
) {
    for (entity, mut timer) in &mut ball_destruction_timers_query {
        timer.0.tick(time.delta());
        if timer.0.is_finished() {
            commands.entity(entity).despawn();
        }
    }
}

fn ball_collision_mark_destroyed_entities(
    observation: On<CollisionStart>,
    mut message_writer: MessageWriter<NavmeshRegeneration>,
    mut commands: Commands,
    game_sounds: Res<GameSounds>,
    collisions: Collisions,
    destructible_query: Query<&DestructibleParams>,
    transforms_query: Query<&Transform>,
) {
    let observer_collider = observation.event().event_target();

    let observer_body = if observation.collider1 == observer_collider {
        observation.body1.unwrap_or(observation.collider1)
    } else {
        observation.body2.unwrap_or(observation.collider2)
    };

    // Find other body as collider might be attached to a child entity
    let other_body = if observation.collider1 == observer_collider {
        observation.body2.unwrap_or(observation.collider2)
    } else {
        observation.body1.unwrap_or(observation.collider1)
    };

    // If body we hit isn't destructible, then simply return
    // NOTE: Or maybe damage the ball (projectile) on impact
    let Ok(destructible_params) = destructible_query.get(other_body).cloned() else {
        return;
    };

    if let Some(contacts) = collisions.get(observation.collider1, observation.collider2) {
        if let Some(manifold) = contacts.manifolds.first() {
            let total_impulse = manifold.total_normal_impulse();

            // Spawn collision sound at ball's position if impulse is sufficient
            if total_impulse > 2.0 {
                if let Ok(ball_transform) = transforms_query.get(observer_body) {
                    commands.spawn((
                        ball_transform.clone(),
                        AudioPlayer::new(game_sounds.metal_ball_concrete_collision.clone()),
                        PlaybackSettings::DESPAWN
                            .with_spatial(true)
                            .with_spatial_scale(bevy::audio::SpatialScale::new(0.25))
                            .with_volume(Volume::Linear(2.0)),
                    ));
                }
            }

            // Mark entity for destruction if impulse is sufficient
            if total_impulse > destructible_params.impulse_resistance {
                commands.entity(other_body).insert(DestroyedObject);
                message_writer.write(NavmeshRegeneration);

                // Spawn wall destruction sound at destructible's position
                if let Ok(other_transform) = transforms_query.get(other_body) {
                    commands.spawn((
                        other_transform.clone(),
                        AudioPlayer::new(game_sounds.concrete_crumbling.clone()),
                        PlaybackSettings::DESPAWN
                            .with_spatial(true)
                            .with_spatial_scale(bevy::audio::SpatialScale::new(0.25))
                            .with_volume(Volume::Linear(2.0)),
                    ));
                }
            }
        }
    }
}
