use avian3d::prelude::*;
#[allow(unused_imports)]
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{DQuat, DVec3}, // Import Double Precision types
    prelude::*,
};
use faer::{get_global_parallelism, prelude::*};
#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;
#[path = "./helpers/rigid_stress_sim.rs"]
mod rigid_stress_sim;
use rigid_stress_sim::*;

// =========================================================================
// SYSTEMS
// =========================================================================

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LogDiagnosticsPlugin::default(),
            PhysicsPlugins::default(),
            CameraControllerPlugin,
        ))
        .insert_resource(PhysicsSim::new())
        .insert_resource(SpaceRemoveableNodes::default())
        .add_systems(Startup, (print_faer_info, setup_doorframe, setup))
        .add_systems(FixedUpdate, solve_system)
        .add_systems(Update, (render, input_force, camera_movement))
        .insert_resource(Time::<Fixed>::from_seconds(DT)) // Bevy accepts f64 for Time
        .run();
}

fn print_faer_info() {
    match get_global_parallelism() {
        Par::Seq => {
            info!("faer runs sequentially.");
        }
        Par::Rayon(nz) => {
            info!("faer runs in parallel with {} threads.", nz);
        }
    }
}

#[allow(dead_code)]
fn setup_sqare_tower(mut sim: ResMut<PhysicsSim>) {
    let w = 6;
    let d = 6;
    let h = 20;

    // Spacing between nodes
    let s = 4.0;

    info!("Building a tower with {} nodes...", w * d * h);

    // Store indices in a 3D grid for easy connection
    let mut grid = vec![vec![vec![0; d]; w]; h];

    // 1. Create Nodes
    for y in 0..h {
        for x in 0..w {
            for z in 0..d {
                // Center the tower on X and Z, start Y at 0
                let pos = vec3(
                    (x as f32 - (w as f32 - 1.0) / 2.0) * s,
                    y as f32 * s,
                    (z as f32 - (d as f32 - 1.0) / 2.0) * s,
                );

                // Fix the bottom layer to the ground
                let fixed = y == 0;

                let idx = sim.add_node(fixed, pos, Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO);
                grid[y][x][z] = idx;
            }
        }
    }

    // 2. Create Connections (Structural Frame)
    for y in 0..h {
        for x in 0..w {
            for z in 0..d {
                let me = grid[y][x][z];

                // Vertical Column (Up)
                if y + 1 < h {
                    sim.add_conn(me, grid[y + 1][x][z]);
                }

                // Horizontal Beam (Right/X)
                if x + 1 < w {
                    sim.add_conn(me, grid[y][x + 1][z]);
                }

                // Horizontal Beam (Forward/Z)
                if z + 1 < d {
                    sim.add_conn(me, grid[y][x][z + 1]);
                }
            }
        }
    }

    sim.create_island();
}

#[derive(Resource, Default)]
struct SpaceRemoveableNodes(Vec<usize>);

fn setup_doorframe(mut sim: ResMut<PhysicsSim>, mut removable_nodes: ResMut<SpaceRemoveableNodes>) {
    // =========================================================================
    // CONTROLS
    // =========================================================================
    const SUPPORT_HEIGHT: f32 = 3.0;
    const CHAIN_LENGTH: f32 = 4.0; // Defines the width of the doorframe

    // Resolution of the simulation (distance between nodes)
    const SPACING: f32 = 1.0;

    info!(
        "Generating Doorframe: Height={}, Width={}",
        SUPPORT_HEIGHT, CHAIN_LENGTH
    );

    // =========================================================================
    // BUILD
    // =========================================================================

    let half_width = CHAIN_LENGTH / 2.0;
    let left_x = -half_width;
    let right_x = half_width;
    let top_y = SUPPORT_HEIGHT;

    // Helper to build a vertical pillar
    // Returns the index of the top node
    let mut build_pillar = |x_pos: f32| -> usize {
        let mut prev_node = None;
        let steps = (SUPPORT_HEIGHT / SPACING).ceil() as usize;

        for i in 0..=steps {
            let t = i as f32 / steps as f32;
            let y = t * SUPPORT_HEIGHT;

            // Fix the bottom node (i=0)
            let fixed = i == 0;

            let node_idx = sim.add_node(
                fixed,
                Vec3::new(x_pos, y, 0.0),
                Vec3::ZERO,
                Quat::IDENTITY,
                Vec3::ZERO,
            );

            if let Some(prev) = prev_node {
                sim.add_conn(prev, node_idx);
            }
            prev_node = Some(node_idx);
        }
        prev_node.unwrap()
    };

    // 1. Build Left and Right Supports
    let top_left_idx = build_pillar(left_x);
    let top_right_idx = build_pillar(right_x);

    // 2. Build Top Beam ("The Chain")
    // We connect the existing top-left node to the existing top-right node
    // by filling the space in between.

    let beam_steps = (CHAIN_LENGTH / SPACING).ceil() as usize;
    let mut prev_node = top_left_idx;

    // Iterate from 1 to steps-1 (skipping 0 and max, because those are the pillars)
    for i in 1..beam_steps {
        let t = i as f32 / beam_steps as f32;
        let x = left_x + (right_x - left_x) * t;

        let node_idx = sim.add_node(
            false,
            Vec3::new(x, top_y, 0.0),
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
        );

        sim.add_conn(prev_node, node_idx);
        prev_node = node_idx;
    }

    removable_nodes.0.push(0);

    // Connect the last piece of the chain to the Right Support
    sim.add_conn(prev_node, top_right_idx);

    // Initialize Solver
    sim.create_island();
}

// Setup functions must now handle Vector casting (Bevy Vec3 -> DVec3 happens inside add_node)
#[allow(dead_code)]
fn setup_tshape(mut sim: ResMut<PhysicsSim>) {
    // Node 0: Base
    let n0 = sim.add_node(
        true,
        vec3(0.0, 0.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    // Node 1: Pillar Top
    let n1 = sim.add_node(
        false,
        vec3(0.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    // Arms
    let n2 = sim.add_node(
        false,
        vec3(-4.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );
    let n3 = sim.add_node(
        false,
        vec3(4.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );

    sim.add_conn(n0, n1);
    sim.add_conn(n1, n2);
    sim.add_conn(n1, n3);

    // Add heavy chain to test cantilever shear
    let mut prev = n3;
    for i in 2..20 {
        let new = sim.add_node(
            false,
            vec3(i as f32 * 4.0, 6.0, 0.0),
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
        );
        sim.add_conn(prev, new);
        prev = new;
    }

    sim.create_island();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(20.0, 15.0, 30.0).looking_at(Vec3::new(5.0, 5.0, 0.0), Vec3::Y),
        CameraMoveSpeed(15.0),
        CameraMoveSpeedMult(5.0),
        CameraController::default(),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(50.0, 1.0, 50.0))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.2, 0.2, 0.2)))),
        RigidBody::Static,
        Collider::cuboid(50.0, 1.0, 50.0),
        Transform::from_xyz(0.0, -2.0, 0.0),
    ));

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.6))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.9, 0.9, 0.9)))),
        RigidBody::Dynamic,
        Collider::sphere(0.6),
        Transform::from_xyz(0.0, 2.0, 0.0),
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, 10.0)));
}

fn solve_system(mut sim: ResMut<PhysicsSim>) {
    sim.solve_for_x();
    sim.check_breakage();
    if sim.island.as_ref().unwrap().needs_refactor {
        sim.update_stiffness_matrix();
    }
}

fn input_force(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&GlobalTransform, With<Camera>>,
    mut sim: ResMut<PhysicsSim>,
    mut gizmos: Gizmos,
    mut removable_nodes: ResMut<SpaceRemoveableNodes>,
) {
    if keys.just_pressed(KeyCode::Space) {
        for idx in removable_nodes.0.iter() {
            sim.nodes[*idx].fixed = false;
        }

        removable_nodes.0.clear();

        sim.update_stiffness_matrix();

        if let Some(island) = sim.island.as_mut() {
            island.needs_refactor = true;
        }
    }

    let tform = *camera;
    let pole_len = tform.forward() * 8.0;
    let center_f32 = Vec3::new(
        tform.translation().x + pole_len.x,
        tform.translation().y + pole_len.y,
        tform.translation().z + pole_len.z,
    );
    let center = center_f32.as_dvec3(); // Convert to f64 for physics

    gizmos.sphere(center_f32, 0.1, Color::WHITE);

    if mouse.just_pressed(MouseButton::Left) {
        let impulse = 500_000.0;
        for n in &mut sim.nodes {
            let diff = n.pos - center;
            let dist = diff.length();
            if dist < 40.0 {
                let dist_clamped = dist.max(1.0);
                let force_mag = (impulse / NODE_MASS) / (dist_clamped * dist_clamped);
                n.vel += diff.normalize() * force_mag * DT;
            }
        }
    }
}

fn render(mut gizmos: Gizmos, sim: Res<PhysicsSim>) {
    for conn in &sim.connections {
        if !conn.alive {
            continue;
        }
        // Downcast f64 -> f32 for rendering
        let p_a = sim.nodes[conn.node_a].pos.as_vec3();
        let p_b = sim.nodes[conn.node_b].pos.as_vec3();
        let ratio = conn.last_stress_ratio;
        let color = if ratio > 1.0 {
            Color::WHITE
        } else {
            Color::srgb(ratio, 1.0 - ratio, 0.0)
        };
        gizmos.line(p_a, p_b, color);
    }
    for node in &sim.nodes {
        let pos = node.pos.as_vec3();
        let color = if node.fixed {
            Color::WHITE
        } else {
            Color::srgb(0.0, 0.0, 1.0)
        };
        gizmos.cuboid(
            Transform::from_translation(pos).with_scale(Vec3::splat(0.3)),
            color,
        );

        // Rotation axis
        let axis = (node.rot * DVec3::X).as_vec3() * 0.1;
        gizmos.line(pos, pos + axis, Color::srgb(1.0, 0.0, 0.0));
    }
}
