//! The Pipeline:
//! Import: Detect "Simulation_Graph" mesh -> Match Vertices to Sibling Meshes -> Build Kernel.
//! Update: Apply Avian Collisions -> Run Solver -> Sync Child Transforms.

use std::collections::VecDeque;

use avian3d::prelude::*;
use bevy::{
    math::{DQuat, DVec3},
    mesh::{Indices, VertexAttributeValues},
    platform::collections::HashMap,
    prelude::*,
    scene::{SceneInstance, SceneInstanceReady},
};

// Assuming these modules exist in your project structure
#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;
#[path = "./helpers/integrated_stress_sim.rs"]
mod integrated_stress_sim;
use integrated_stress_sim::*;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            CameraControllerPlugin,
        ))
        .add_systems(Startup, setup)
        // Physics Step: 1. Solve FEM, 2. Break, 3. Sync Visuals/Colliders
        .add_systems(
            FixedUpdate,
            (solve_system, sync_structural_transforms).chain(),
        )
        .add_systems(Update, camera_movement)
        //.add_systems(
        //    Update,
        //    (
        //        //add_colliders_to_new_meshes,
        //        //setup_structural_sim_from_scene,
        //        //handle_structure_collisions,
        //        //handle_structural_splitting,
        //    ),
        //)
        .add_systems(Update, (render_sims, handle_input_force))
        .add_observer(handle_scene_added)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Load the GLTF.
    // The Setup system will detect "Simulation_Graph" inside this scene.
    commands.spawn((
        SceneRoot(
            asset_server
                .load(GltfAssetLabel::Scene(0).from_asset("models/WoodenModularHuts/Hut.glb")),
        ),
        Transform::from_xyz(10., 0., 0.),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        CameraController::default(),
        CameraMoveSpeed(10.),
        Transform::from_xyz(0., 10., 30.).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(30., 1., 20.))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.8, 0.6))),
        Transform::from_xyz(0., -0.5, 0.),
        // Ground needs to be a RigidBody so Avian solves collisions against it
        RigidBody::Static,
        Collider::cuboid(30., 1., 20.),
    ));

    // Light
    commands.spawn((PointLight::default(), Transform::from_xyz(-12., 20., 0.)));
}

// =========================================================================
// SETUP & ARCHITECTURE
// =========================================================================

fn add_colliders_to_new_meshes(
    mut commands: Commands,
    names_query: Query<&Name>,
    new_mesh_query: Query<(Entity, &ChildOf), Added<Mesh3d>>,
) {
    for (entity, parent) in new_mesh_query {
        if let Ok(parent_name) = names_query.get(parent.0) {
            if parent_name.starts_with("collider_aabb") {
                println!("adding collider to {}", entity);
                commands
                    .entity(entity)
                    .insert((ColliderConstructor::ConvexHullFromMesh, Visibility::Hidden));
            }
        }
    }
}

/// Scene has global transform, its children only have transform.
/// If scene has Simulation_Graph_Obj, then its structure is as follows:
/// ```text
/// RootObj {
///     Simulation_Graph_Obj {
///         Simulation_Graph_Data
///     }
///     MeshObj.1 {
///         MeshData.1
///         collider_obj_MeshObj.1 {
///             collider_box_MeshObj.1
///         }
///     }
///     MeshObj.2 {
///         MeshData.2
///         collider_obj_MeshObj.2 {
///             collider_box_MeshObj.2
///         }
///     }
///     ...
/// }
/// ```
///
/// Otherwise, it's structure is as follows:
/// ```text
/// RootObj {
///     MeshObj.1 {
///         MeshData.1
///         collider_obj_MeshObj.1 {
///             collider_box_MeshObj.1
///         }
///     }
///     MeshObj.2 {
///         MeshData.2
///         collider_obj_MeshObj.2 {
///             collider_box_MeshObj.2
///         }
///     }
///     ...
/// }
/// ```
fn handle_scene_added(
    event: On<SceneInstanceReady>,
    mut commands: Commands,
    scene_spawner: Res<SceneSpawner>,
    names: Query<&Name>,
    meshes: Res<Assets<Mesh>>,
    mesh_handles: Query<&Mesh3d>,
    transforms: Query<&Transform>,
    global_transforms: Query<&GlobalTransform>,
) {
    info!("Scene instance ready.");

    let scene_entity = event.entity;
    commands
        .entity(scene_entity)
        .insert((RigidBody::Kinematic,)); //LinearVelocity(Vec3::new(0., 1., 0.))

    let scene_global_tr = global_transforms
        .get(scene_entity)
        .unwrap_or(&GlobalTransform::IDENTITY);
    info!(
        "scene global translation is: {}",
        scene_global_tr.translation()
    );

    let mut scene_meshes: Vec<Vec3> = Vec::new();

    let mut graph_obj_entity: Option<Entity> = None;
    let mut graph_data_entity: Option<Entity> = None;

    let instance_id = event.instance_id;
    let mut scene_children_cnt: usize = 0;
    for entity in scene_spawner.iter_instance_entities(instance_id) {
        scene_children_cnt += 1;
        if let Ok(child_name) = names.get(entity) {
            if child_name.starts_with("Simulation_Graph_Obj") {
                graph_obj_entity = Some(entity);
                //commands.entity(entity).insert(Visibility::Hidden);
            } else if child_name.starts_with("Simulation_Graph_Data") {
                graph_data_entity = Some(entity);
            } else if child_name.starts_with("collider_box_") {
                // Create collider from collider mesh
                commands
                    .entity(entity)
                    .insert((ColliderConstructor::ConvexHullFromMesh, Visibility::Hidden));
            }
        }
    }
    info!("Scene has {} descendants.", scene_children_cnt);

    // If there is a graph entity, setup structural sim
    let Some(graph_obj_entity) = graph_obj_entity else {
        return;
    };
    let Some(graph_data_entity) = graph_data_entity else {
        return;
    };
    info!("Found Simulation_Graph_Data.");

    let Ok(mesh_handle) = mesh_handles.get(graph_data_entity) else {
        warn!("Mesh handle for Simulation_Graph_Data not available.");
        return;
    };
    let Some(mesh) = meshes.get(mesh_handle.id()) else {
        warn!("Mesh for Simulation_Graph_Data not available.");
        return;
    };

    let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(p)) => p,
        _ => {
            warn!("Graph has no positions");
            return;
        }
    };
    let colors: Option<&Vec<[f32; 4]>> = match mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
        Some(VertexAttributeValues::Float32x4(c)) => Some(c),
        _ => None,
    };

    // Get Graph's Local Transform to convert Vertices -> Wrapper Space
    let graph_obj_local_tr = transforms
        .get(graph_obj_entity)
        .unwrap_or(&Transform::IDENTITY);
    info!(
        "graph object local translation is: {}",
        graph_obj_local_tr.translation
    );
}

// Define a marker to ensure we process each scene instance exactly once
#[derive(Component)]
pub struct StructuralSceneProcessed;

fn setup_structural_sim_from_scene(
    mut commands: Commands,
    // Query for scenes that are loaded but not yet processed
    scenes: Query<(Entity, &SceneInstance), Without<StructuralSceneProcessed>>,
    scene_spawner: Res<SceneSpawner>,

    // Data Access
    names: Query<&Name>,
    children: Query<&Children>,
    parents: Query<&ChildOf>,
    transforms: Query<&Transform>,
    meshes: Res<Assets<Mesh>>,
    mesh_handles: Query<&Mesh3d>,
) {
    for (scene_entity, instance) in scenes.iter() {
        // Critical Check: Is the GLTF fully spawned?
        if !scene_spawner.instance_is_ready(**instance) {
            continue;
        }

        // Mark as processed immediately
        commands
            .entity(scene_entity)
            .insert(StructuralSceneProcessed);
        info!(
            "Scene Instance {:?} is ready. Analyzing structure...",
            scene_entity
        );

        // 1. FIND THE SIMULATION GRAPH
        // We iterate ONLY the entities in this specific scene instance
        let mut graph_entity = None;
        for entity in scene_spawner.iter_instance_entities(**instance) {
            if let Ok(name) = names.get(entity) {
                if name.as_str().starts_with("Simulation_Graph") {
                    graph_entity = Some(entity);
                    break;
                }
            }
        }

        let Some(graph_entity) = graph_entity else {
            // This scene might be just environment/props, skip warning if not expected
            continue;
        };

        // 2. IDENTIFY THE STRUCTURE ROOT (Parent Wrapper)
        // The graph is a child of the object that holds the parts.
        let Ok(wrapper_entity) = parents.get(graph_entity).map(|p| p.parent()) else {
            warn!(
                "Simulation Graph {:?} has no parent! Structure invalid.",
                graph_entity
            );
            continue;
        };

        let root_name = names
            .get(wrapper_entity)
            .map(|n| n.as_str())
            .unwrap_or("Unknown");
        info!(
            "Identified Structure Root: '{}' ({:?})",
            root_name, wrapper_entity
        );

        // 3. COLLECT CANDIDATE PARTS (Siblings)
        // We assume all "Parts" are siblings of the "Simulation_Graph"
        let mut part_candidates = Vec::new();
        if let Ok(siblings) = children.get(wrapper_entity) {
            for &sibling in siblings {
                if sibling == graph_entity {
                    continue;
                }

                // We capture the Local Transform (Position relative to Wrapper)
                if let Ok(tr) = transforms.get(sibling) {
                    part_candidates.push((sibling, tr.translation));
                }
            }
        }

        if part_candidates.is_empty() {
            warn!("Structure Root has no children other than the graph!");
            continue;
        }

        // 4. EXTRACT TOPOLOGY FROM MESH
        let Ok(mesh_handle) = mesh_handles.get(graph_entity) else {
            continue;
        };
        let Some(mesh) = meshes.get(mesh_handle.id()) else {
            continue;
        };

        let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(p)) => p,
            _ => {
                warn!("Graph has no positions");
                continue;
            }
        };
        let colors: Option<&Vec<[f32; 4]>> = match mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
            Some(VertexAttributeValues::Float32x4(c)) => Some(c),
            _ => None,
        };

        // Get Graph's Local Transform to convert Vertices -> Wrapper Space
        let graph_local_tr = transforms.get(graph_entity).unwrap_or(&Transform::IDENTITY);

        // 5. MATCH NODES
        let mut nodes = Vec::with_capacity(positions.len());
        let mut node_to_entity = Vec::with_capacity(positions.len());

        info!(
            "Matching {} nodes against {} candidates...",
            positions.len(),
            part_candidates.len()
        );

        for (i, local_pos) in positions.iter().enumerate() {
            // Transform Vertex: Graph Space -> Wrapper Space
            let vertex_wrapper_space = graph_local_tr.transform_point(Vec3::from(*local_pos));

            let is_fixed = if let Some(c) = colors {
                c[i][0] > 0.5
            } else {
                false
            };

            let mut matched_entity = Entity::PLACEHOLDER;
            let mut best_dist_sq = 0.1 * 0.1; // 10cm tolerance

            for (part_entity, part_pos) in &part_candidates {
                let d2 = part_pos.distance_squared(vertex_wrapper_space);
                if d2 < best_dist_sq {
                    best_dist_sq = d2;
                    matched_entity = *part_entity;
                }
            }

            if matched_entity != Entity::PLACEHOLDER {
                // Attach SimPart to the Visual Part
                commands.entity(matched_entity).insert((
                    SimPart {
                        root_entity: wrapper_entity,
                        node_index: i,
                    },
                    CollisionEventsEnabled, // Ensure Avian reports hits on this part
                ));
            } else {
                warn!(
                    "Node {} FAIL. WrapperSpace Pos: {:?}",
                    i, vertex_wrapper_space
                );
            }

            nodes.push(PhysNode {
                fixed: is_fixed,
                pos: vertex_wrapper_space.as_dvec3(), // Solver runs in Wrapper Local Space
                vel: DVec3::ZERO,
                rot: DQuat::IDENTITY,
                ang_vel: DVec3::ZERO,
            });
            node_to_entity.push(matched_entity);
        }

        // 6. BUILD CONNECTIONS
        let mut connections = Vec::new();
        if let Some(indices) = mesh.indices() {
            let indices_iter: Box<dyn Iterator<Item = usize>> = match indices {
                Indices::U16(vec) => Box::new(vec.iter().map(|&i| i as usize)),
                Indices::U32(vec) => Box::new(vec.iter().map(|&i| i as usize)),
            };
            let idx_vec: Vec<usize> = indices_iter.collect();
            for edge in idx_vec.chunks_exact(2) {
                let a = edge[0];
                let b = edge[1];
                let diff = nodes[b].pos - nodes[a].pos;
                let len = diff.length();
                let dir = if len > 1e-6 { diff / len } else { DVec3::Y };

                connections.push(Connection {
                    alive: true,
                    node_a: a,
                    node_b: b,
                    rest_len: len,
                    rest_local_rot: DQuat::IDENTITY,
                    local_axis: dir,
                    material: CONCRETE,
                    baseline_impulse: 0.,
                    last_stress_ratio: 0.,
                });
            }
        }

        // 7. INITIALIZE SIMULATION ON WRAPPER
        let mut system = StructuralSim {
            nodes,
            connections,
            island: None,
            frame_count: 0,
            node_to_entity,
        };
        system.create_island();

        commands.entity(wrapper_entity).insert((
            system,
            RigidBody::Kinematic, // The Wrapper acts as the monolithic body
                                  // Note: Transform/GlobalTransform already exist on the GLTF node
        ));

        // Hide the graph
        commands.entity(graph_entity).insert(Visibility::Hidden);
    }
}

// =========================================================================
// COLLISION HANDLING
// =========================================================================

fn handle_structure_collisions(
    collisions: Collisions,
    sim_parts: Query<&SimPart>, // Component is on Visual Parent
    parents: Query<&ChildOf>,   // To walk up from Collider -> Visual
    mut structures: Query<(&mut StructuralSim, &GlobalTransform)>,
    bodies: Query<(&LinearVelocity, &ComputedMass)>,
) {
    let mut impulse_buffer: HashMap<Entity, Vec<(usize, DVec3)>> = HashMap::new();

    // Helper to find SimPart on entity OR its parent
    let find_sim_part = |start: Entity| -> Option<&SimPart> {
        if let Ok(p) = sim_parts.get(start) {
            return Some(p);
        }
        if let Ok(parent) = parents.get(start) {
            if let Ok(p) = sim_parts.get(parent.parent()) {
                return Some(p);
            }
        }
        None
    };

    for contacts in collisions.iter() {
        let e1 = contacts.collider1;
        let e2 = contacts.collider2;

        let p1 = find_sim_part(e1);
        let p2 = find_sim_part(e2);

        // If neither is a structural part, ignore this collision
        if p1.is_none() && p2.is_none() {
            continue;
        }

        // Get the collision normal (World Space, points from 2 to 1)
        let Some(manifold) = contacts.manifolds.first() else {
            continue;
        };
        let normal_world = manifold.normal.as_dvec3();

        // Helper to calculate manual impulse for Kinematic vs Kinematic collisions
        // (e.g. Player Character hitting the wall)
        let get_manual_world_impulse = |other_entity: Entity, hit_normal: DVec3| -> DVec3 {
            if let Ok((vel, mass)) = bodies.get(other_entity) {
                // Calculate impact speed along the normal
                // If velocity opposes normal, dot is negative. We want the magnitude.
                let speed = vel.as_dvec3().dot(-hit_normal).max(0.0);

                // J = p = mv
                // Push along the normal
                -hit_normal * (speed * mass.value() as f64)
            } else {
                DVec3::ZERO
            }
        };

        // --- Logic for Entity 1 (Structure Part) ---
        if let Some(part) = p1 {
            // 1. Get Root Transform for Coordinate Conversion
            if let Ok((_, root_tr)) = structures.get(part.root_entity) {
                let world_impulse: DVec3;
                let solved_mag = contacts.total_normal_impulse_magnitude();

                if solved_mag > 1e-6 {
                    // Case A: Avian solved it (Dynamic Body hit us).
                    // Avian applies impulse to e1. Normal points 2->1.
                    // Impulse on 1 is usually in direction of normal.
                    world_impulse = normal_world * (solved_mag as f64);
                } else {
                    // Case B: Kinematic vs Kinematic (Player hit us).
                    // Avian applied 0 impulse. Calculate manually.
                    let other_body = contacts.body2.unwrap_or(e2);
                    world_impulse = get_manual_world_impulse(other_body, normal_world);
                }

                if world_impulse.length_squared() > 1e-6 {
                    // 2. Transform World Impulse -> Local Impulse
                    // We rotate the vector by the inverse of the Root's rotation.
                    let root_rot_inv = root_tr.rotation().inverse().as_dquat();
                    let local_impulse = root_rot_inv * world_impulse;

                    impulse_buffer
                        .entry(part.root_entity)
                        .or_default()
                        .push((part.node_index, local_impulse));
                }
            }
        }

        // --- Logic for Entity 2 (Structure Part) ---
        if let Some(part) = p2 {
            if let Ok((_, root_tr)) = structures.get(part.root_entity) {
                let world_impulse: DVec3;
                let solved_mag = contacts.total_normal_impulse_magnitude();

                if solved_mag > 1e-6 {
                    // Impulse on 2 is opposite to 1
                    world_impulse = -normal_world * (solved_mag as f64);
                } else {
                    let other_body = contacts.body1.unwrap_or(e1);
                    // Pass -normal because we are calculating impact on E2
                    world_impulse = get_manual_world_impulse(other_body, -normal_world);
                }

                if world_impulse.length_squared() > 1e-6 {
                    let root_rot_inv = root_tr.rotation().inverse().as_dquat();
                    let local_impulse = root_rot_inv * world_impulse;

                    impulse_buffer
                        .entry(part.root_entity)
                        .or_default()
                        .push((part.node_index, local_impulse));
                }
            }
        }
    }

    // Apply Batched Impulses to Solvers
    for (root, impulses) in impulse_buffer {
        if let Ok((mut system, _)) = structures.get_mut(root) {
            for (idx, impulse_vec) in impulses {
                if idx < system.nodes.len() {
                    // F = ma -> dv = J / m
                    // We apply this to the node's Local Velocity
                    system.nodes[idx].vel += impulse_vec / NODE_MASS;
                }
            }
        }
    }
}

// =========================================================================
// SPLITTING & TOPOLOGY
// =========================================================================

fn handle_structural_splitting(
    mut commands: Commands,
    mut structures: Query<(Entity, &mut StructuralSim)>,
    mut sim_parts: Query<&mut SimPart>,
) {
    for (_original_root_entity, mut system) in structures.iter_mut() {
        if !system.island.as_ref().map_or(false, |i| i.needs_refactor) {
            continue;
        }

        let n_nodes = system.nodes.len();
        if n_nodes == 0 {
            continue;
        }

        let mut adj = vec![Vec::new(); n_nodes];
        let mut active_nodes = vec![false; n_nodes];

        for conn in &system.connections {
            if conn.alive {
                adj[conn.node_a].push(conn.node_b);
                adj[conn.node_b].push(conn.node_a);
                active_nodes[conn.node_a] = true;
                active_nodes[conn.node_b] = true;
            }
        }

        let mut visited = vec![false; n_nodes];
        let mut components: Vec<Vec<usize>> = Vec::new();

        for i in 0..n_nodes {
            let is_visual = system.node_to_entity[i] != Entity::PLACEHOLDER;
            if !visited[i] && (active_nodes[i] || is_visual) {
                let mut component = Vec::new();
                let mut queue = VecDeque::new();
                queue.push_back(i);
                visited[i] = true;

                while let Some(curr) = queue.pop_front() {
                    component.push(curr);
                    for &neighbor in &adj[curr] {
                        if !visited[neighbor] {
                            visited[neighbor] = true;
                            queue.push_back(neighbor);
                        }
                    }
                }
                components.push(component);
            }
        }

        if components.len() <= 1 {
            continue;
        }

        components.sort_by_key(|c| std::cmp::Reverse(c.len()));

        for debris_indices in &components[1..] {
            let (mut new_sys, child_map) = extract_system_subset(&system, debris_indices);
            new_sys.create_island();

            let new_root = commands
                .spawn((
                    new_sys,
                    Transform::default(),
                    GlobalTransform::default(),
                    Visibility::default(),
                    RigidBody::Kinematic,
                    Name::new("Debris_Chunk"),
                ))
                .id();

            for (new_idx, child_entity) in child_map {
                if child_entity == Entity::PLACEHOLDER {
                    continue;
                }

                commands.entity(child_entity).set_parent_in_place(new_root);

                commands
                    .entity(child_entity)
                    .insert((CollisionEventsEnabled,)); // RigidBody not needed on child

                if let Ok(mut handle) = sim_parts.get_mut(child_entity) {
                    handle.root_entity = new_root;
                    handle.node_index = new_idx;
                }
            }
        }

        let (mut compacted_sys, child_map) = extract_system_subset(&system, &components[0]);
        compacted_sys.create_island();

        for (new_idx, child_entity) in child_map {
            if child_entity == Entity::PLACEHOLDER {
                continue;
            }
            if let Ok(mut handle) = sim_parts.get_mut(child_entity) {
                handle.node_index = new_idx;
            }
        }
        *system = compacted_sys;
    }
}

// helper
fn extract_system_subset(
    source: &StructuralSim,
    indices: &[usize],
) -> (StructuralSim, Vec<(usize, Entity)>) {
    let mut new_nodes = Vec::with_capacity(indices.len());
    let mut new_node_to_entity = Vec::with_capacity(indices.len());
    let mut old_to_new_map = HashMap::with_capacity(indices.len());
    let mut child_map_result = Vec::with_capacity(indices.len());

    for (new_idx, &old_idx) in indices.iter().enumerate() {
        new_nodes.push(source.nodes[old_idx].clone());
        let entity = source.node_to_entity[old_idx];
        new_node_to_entity.push(entity);
        old_to_new_map.insert(old_idx, new_idx);
        child_map_result.push((new_idx, entity));
    }

    let mut new_connections = Vec::new();
    for conn in &source.connections {
        if conn.alive
            && old_to_new_map.contains_key(&conn.node_a)
            && old_to_new_map.contains_key(&conn.node_b)
        {
            let mut c = conn.clone();
            c.node_a = *old_to_new_map.get(&conn.node_a).unwrap();
            c.node_b = *old_to_new_map.get(&conn.node_b).unwrap();
            new_connections.push(c);
        }
    }

    (
        StructuralSim {
            nodes: new_nodes,
            connections: new_connections,
            island: None,
            frame_count: 0,
            node_to_entity: new_node_to_entity,
        },
        child_map_result,
    )
}

// =========================================================================
// SOLVER & SYNC
// =========================================================================

fn solve_system(mut sims: Query<&mut StructuralSim>) {
    // Parallelize if possible: sims.par_iter_mut()
    sims.par_iter_mut().for_each(|mut sim| {
        sim.solve_for_x();
        sim.check_breakage();
        if sim.island.as_ref().unwrap().needs_refactor {
            sim.update_stiffness_matrix();
        }
    });
}

/// Crucial System: Syncs the Visual/Collider Entities to the FEM Nodes.
/// Since the children are parts of a Compound RigidBody, updating their
/// Transform updates the actual shape of the Physics Body in Avian.
fn sync_structural_transforms(sims: Query<&StructuralSim>, mut transforms: Query<&mut Transform>) {
    for sim in sims.iter() {
        for (i, &entity) in sim.node_to_entity.iter().enumerate() {
            if entity == Entity::PLACEHOLDER {
                continue;
            }

            // Nodes are in World Space.
            // If the Root RigidBody is at (0,0,0), then Child Local == World.
            // If Root moves, we'd need to inverse transform, but for this specific
            // Architecture, keeping Root at Identity and moving children is easiest.
            if let Ok(mut t) = transforms.get_mut(entity) {
                let node = &sim.nodes[i];
                t.translation = node.pos.as_vec3();
                t.rotation = node.rot.as_quat();
            }
        }
    }
}

// =========================================================================
// DEBUG & INPUT
// =========================================================================

fn render_sims(mut gizmos: Gizmos, sims: Query<&StructuralSim>) {
    for sim in sims.iter() {
        for conn in &sim.connections {
            if !conn.alive {
                continue;
            }
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
}

fn handle_input_force(
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&GlobalTransform, With<Camera>>,
    mut sims: Query<&mut StructuralSim>,
    mut gizmos: Gizmos,
) {
    let tform = *camera;
    let pole_len = tform.forward() * 8.0;
    let center_f32 = tform.translation() + pole_len;
    let center = center_f32.as_dvec3();

    gizmos.sphere(center_f32, 0.1, Color::WHITE);

    if mouse.just_pressed(MouseButton::Left) {
        let impulse = 500_000.0;
        for mut sim in sims.iter_mut() {
            for n in &mut sim.nodes {
                let diff = n.pos - center;
                let dist = diff.length();
                if dist < 10.0 {
                    let force = diff.normalize() * (impulse / dist.max(1.0));
                    // Apply as velocity change
                    n.vel += force * DT;
                }
            }
        }
    }
}
