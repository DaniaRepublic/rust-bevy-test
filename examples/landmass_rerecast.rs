use avian_rerecast::AvianBackendPlugin; // <--- NEW BACKEND
use avian3d::prelude::*;
use bevy::{color::palettes::css::*, prelude::*};
use bevy_landmass::{
    Agent, AgentDesiredVelocity3d, AgentTarget3d, PointSampleDistance3d,
    debug::LandmassDebugPlugin, prelude::*,
};
use bevy_rerecast::prelude::*;
use landmass_rerecast::{Island3dBundle, LandmassRerecastPlugin, NavMeshHandle3d};

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(), // Avian Physics
            // 1. Navigation Setup
            NavmeshPlugins::default(),
            AvianBackendPlugin::default(), // <--- Generate from Colliders, not Meshes
            Landmass3dPlugin::default(),
            LandmassRerecastPlugin::default(),
            LandmassDebugPlugin::<ThreeD>::default(),
        ))
        .init_resource::<NavMeshManager>()
        .add_systems(Startup, setup_scene)
        .add_systems(
            Update,
            (generate_navmesh_delayed, move_agents, debug_gizmos),
        )
        .run();
}

#[derive(Resource)]
struct NavMeshManager {
    timer: Timer,
    generated: bool,
    archipelago_entity: Option<Entity>,
}

impl Default for NavMeshManager {
    fn default() -> Self {
        Self {
            // Wait 1s to ensure physics world is initialized
            timer: Timer::from_seconds(1.0, TimerMode::Once),
            generated: false,
            archipelago_entity: None,
        }
    }
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut nav_manager: ResMut<NavMeshManager>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 40.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 4000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(-10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // --- PHYSICS GROUND ---
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(40.0, 2.0, 40.0))),
        MeshMaterial3d(materials.add(Color::from(GRAY))),
        Transform::from_xyz(0.0, -1.0, 0.0), // Surface at Y=0
        // Avian Components (NavMesh will read these)
        RigidBody::Static,
        Collider::cuboid(40.0, 2.0, 40.0),
    ));

    // --- ARCHIPELAGO ---
    let archipelago = commands
        .spawn(Archipelago3d::new(ArchipelagoOptions {
            point_sample_distance: PointSampleDistance3d {
                distance_above: 2.0,
                distance_below: 2.0,
                ..PointSampleDistance3d::from_agent_radius(0.5)
            },
            ..ArchipelagoOptions::from_agent_radius(0.5)
        }))
        .id();

    nav_manager.archipelago_entity = Some(archipelago);

    // --- AGENT ---
    commands.spawn((
        Mesh3d(meshes.add(Capsule3d::new(0.5, 1.0))),
        MeshMaterial3d(materials.add(Color::from(RED))),
        // START HIGH: Y=1.5 ensures feet are clearly above the floor (Y=0.0)
        // If spawned inside the floor, pathfinding often fails to snap.
        Transform::from_xyz(-10.0, 1.5, -10.0),
        Agent3dBundle {
            agent: Agent::default(),
            archipelago_ref: ArchipelagoRef3d::new(archipelago),
            settings: AgentSettings {
                desired_speed: 10.0,
                max_speed: 12.0,
                radius: 0.5,
            },
        },
        AgentTarget3d::Point(Vec3::new(10.0, 0.5, 10.0)),
    ));
}

fn generate_navmesh_delayed(
    mut commands: Commands,
    time: Res<Time>,
    mut nav_manager: ResMut<NavMeshManager>,
    mut generator: NavmeshGenerator,
) {
    if nav_manager.generated {
        return;
    }

    nav_manager.timer.tick(time.delta());

    if nav_manager.timer.is_finished() {
        println!("Generating NavMesh from Physics Colliders...");

        // Explicit settings to guarantee valid voxelization
        let settings = NavmeshSettings {
            cell_size_fraction: 0.1, // Coarser mesh = safer generation
            cell_height_fraction: 0.1,
            agent_height: 2.0,
            agent_radius: 0.5,
            walkable_climb: 0.5,
            walkable_slope_angle: 45.0_f32.to_radians(),
            ..Default::default()
        };

        let nav_mesh_handle = generator.generate(settings);

        if let Some(archipelago_entity) = nav_manager.archipelago_entity {
            commands.spawn(Island3dBundle {
                island: Island,
                archipelago_ref: ArchipelagoRef3d::new(archipelago_entity),
                nav_mesh: NavMeshHandle3d(nav_mesh_handle),
            });
            println!("NavMesh Attached!");
        }
        nav_manager.generated = true;
    }
}

fn move_agents(
    mut query: Query<(Entity, &mut Transform, &AgentDesiredVelocity3d)>,
    time: Res<Time>,
) {
    for (_entity, mut transform, desired_velocity) in query.iter_mut() {
        let vel = desired_velocity.velocity();

        if vel.length() > 0.05 {
            transform.translation += vel * time.delta_secs();

            // Look forward
            let look_target = transform.translation + vel;
            let target_rot = transform.looking_at(look_target, Vec3::Y).rotation;
            transform.rotation = transform
                .rotation
                .slerp(target_rot, 10.0 * time.delta_secs());
        }
    }
}

fn debug_gizmos(
    mut gizmos: Gizmos,
    query: Query<(&Transform, &AgentDesiredVelocity3d, &AgentTarget3d)>,
) {
    for (t, vel, target) in query.iter() {
        // Yellow = Velocity (Success)
        gizmos.line(
            t.translation,
            t.translation + vel.velocity(),
            Color::from(YELLOW),
        );

        // Green = Target
        if let AgentTarget3d::Point(pos) = target {
            gizmos.line(t.translation, *pos, Color::from(GREEN.with_alpha(0.3)));
            gizmos.sphere(*pos, 0.3, Color::from(GREEN));
        }
    }
}
