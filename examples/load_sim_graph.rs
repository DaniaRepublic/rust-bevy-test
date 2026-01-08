//! The Pipeline:
//! Import: Detect "Simulation_Graph" mesh -> Match Vertices to Sibling Meshes -> Build Kernel.
//! Update: Apply Avian Collisions -> Run Solver -> Sync Child Transforms.

use std::{collections::VecDeque, time::Duration};

use avian3d::prelude::*;
use bevy::{
    math::DVec3,
    mesh::{Indices, VertexAttributeValues},
    platform::collections::HashMap,
    prelude::*,
    scene::SceneInstanceReady,
};

// Assuming these modules exist in your project structure
#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;
#[path = "./helpers/integrated_stress_sim.rs"]
mod integrated_stress_sim;
use integrated_stress_sim::*;
use kiddo::{KdTree, SquaredEuclidean};

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
        //        //handle_structure_collisions,
        //        //handle_structural_splitting,
        //    ),
        //)
        .add_systems(Update, (render_sims, handle_input_force))
        .add_systems(Update, (shoot_ball, handle_ball_despawning))
        .add_observer(handle_scene_added)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    // Load the glb scene.
    commands.spawn((
        SceneRoot(
            asset_server
                .load(GltfAssetLabel::Scene(0).from_asset("models/WoodenModularHuts/Hut.001.glb")),
        ),
        Transform::from_xyz(10., 0., 0.),
    ));

    // Load another copy at a different position for testing.
    commands.spawn((
        SceneRoot(
            asset_server
                .load(GltfAssetLabel::Scene(0).from_asset("models/WoodenModularHuts/Hut.001.glb")),
        ),
        Transform::from_xyz(-10., 0., 0.),
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

/// Scene has global transform, its children only have transform.
/// If scene has Simulation_Graph_Obj, then its structure is as follows:
/// ```text
/// RootObj {
///     Simulation_Graph_Obj {
///         Simulation_Graph_Data
///     }
///     visual_MeshObj.1 {
///         MeshData.1
///         collider_obj_MeshObj.1 {
///             collider_box_MeshObj.1
///         }
///     }
///     visual_MeshObj.2 {
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
///     visual_MeshObj.1 {
///         MeshData.1
///         collider_obj_MeshObj.1 {
///             collider_box_MeshObj.1
///         }
///     }
///     visual_MeshObj.2 {
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
    children_of: Query<&ChildOf>,
) {
    info!("Processing new scene instance.");

    let scene_entity = event.entity;
    commands
        .entity(scene_entity)
        .insert((RigidBody::Kinematic,));

    let scene_global_tr = global_transforms
        .get(scene_entity)
        .unwrap_or(&GlobalTransform::IDENTITY);

    // Use parallel arrays for colliders
    let mut colliders_pos: Vec<Vec3> = Vec::new();
    let mut colliders_entity: Vec<Entity> = Vec::new();

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
                // Create collider from collider mesh.
                // Colliders will be connected with simulation graph nodes so collect transforms
                // along with entities to use them for matching later. This isn't very expensive,
                // so do here even if scene isn't simulated.
                commands.entity(entity).insert((
                    ColliderConstructor::ConvexHullFromMesh,
                    CollisionEventsEnabled,
                    Visibility::Hidden,
                ));
                // Parent has translation relative to visual mesh
                let Ok(child_of) = children_of.get(entity) else {
                    error!("Collider doesn't have a parent.");
                    return;
                };
                let parent_entity = child_of.parent();
                // Grandparent is a visual mesh and has translation relative to scene
                let Ok(grandchild_of) = children_of.get(child_of.parent()) else {
                    error!("Collider doesn't have a grandparent.");
                    return;
                };
                let grandparent_entity = grandchild_of.parent();

                let parent_tf = transforms
                    .get(parent_entity)
                    .unwrap_or(&Transform::IDENTITY);
                let grandparent_tf = transforms
                    .get(grandparent_entity)
                    .unwrap_or(&Transform::IDENTITY);

                // 1. We assume the 'collider_box' entity itself is at 0,0,0 relative to its parent (collider_obj).
                //    So the point in "Parent Space" is just Vec3::ZERO.
                // 2. Convert to "Grandparent Space" (Visual Mesh Space):
                let pos_in_visual_mesh = parent_tf.transform_point(Vec3::ZERO);

                // 3. Convert to "Scene Space" (Root Space):
                //    We apply the Grandparent's transform (including its rotation!) to the point.
                let collider_local_pos = grandparent_tf.transform_point(pos_in_visual_mesh);

                colliders_pos.push(collider_local_pos);
                colliders_entity.push(entity);
            } else if child_name.starts_with("visual_") {
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

    // Get Graph's Local Transform to convert translation of vertices to local space relative to scene
    // root
    let graph_obj_transform = transforms
        .get(graph_obj_entity)
        .unwrap_or(&Transform::IDENTITY);

    // to match graph nodes against colliders
    let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(p)) => p,
        _ => {
            warn!("Graph has no positions");
            return;
        }
    };
    let graph_local_positions: Vec<Vec3> = positions
        .iter()
        .map(|e| graph_obj_transform.transform_point(Vec3::from(*e)))
        .collect();

    // Red value determines if node is fixed
    let colors: &Vec<[f32; 4]> = match mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
        Some(VertexAttributeValues::Float32x4(c)) => c,
        _ => &vec![[0., 0., 0., 1.]; positions.len()],
    };

    let pairings = pair_vectors_optimized(&colliders_pos, &graph_local_positions);

    assert_eq!(pairings.len(), positions.len());

    let cm_squared = 0.01 * 0.01;
    for (idx_a, idx_b) in pairings.iter() {
        assert!(colliders_pos[*idx_a].distance_squared(graph_local_positions[*idx_b]) < cm_squared);
    }

    // Now we can create StructuralSim.
    let mut structural_sim = StructuralSim::new();

    let mut graph_idx_to_node = HashMap::<usize, usize>::new();

    // First, create nodes and attach SimPart component to respective collider.
    for (idx_collider, idx_graph) in pairings {
        // Red value threashold is 0.5
        let fixed = colors[idx_graph][0] > 0.5;
        // Use global pos of graph node
        let pos = scene_global_tr.transform_point(graph_local_positions[idx_graph]);
        let node_index =
            structural_sim.add_node(fixed, pos, Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO);

        commands
            .entity(colliders_entity[idx_collider])
            .insert(SimPart {
                root_entity: scene_entity,
                node_index,
            });

        graph_idx_to_node.insert(idx_graph, node_index);
    }

    // Then, create connections, build island, etc. to finish StructuralSim setup.
    if let Some(indices) = mesh.indices() {
        let indices_iter: Box<dyn Iterator<Item = usize>> = match indices {
            Indices::U16(idxs) => Box::new(idxs.iter().map(|v| *v as usize)),
            Indices::U32(idxs) => Box::new(idxs.iter().map(|v| *v as usize)),
        };
        let indices: Vec<usize> = indices_iter.collect();
        for edge in indices.chunks_exact(2) {
            let idx_a = edge[0];
            let idx_b = edge[1];

            let Some(node_a) = graph_idx_to_node.get(&idx_a) else {
                error!("Node for vertex {} wasn't created.", idx_a);
                continue;
            };
            let Some(node_b) = graph_idx_to_node.get(&idx_b) else {
                error!("Node for vertex {} wasn't created.", idx_b);
                continue;
            };

            structural_sim.add_conn(*node_a, *node_b);
        }
    }

    structural_sim.create_island();

    commands.entity(scene_entity).insert(structural_sim);

    info!("StructuralSim was set up and added to scene entity successfully!");
}

// Returns a vector of pairs of indices of spacially matching elements.
pub fn pair_vectors_optimized(list_a: &Vec<Vec3>, list_b: &Vec<Vec3>) -> Vec<(usize, usize)> {
    assert_eq!(list_a.len(), list_b.len());

    // 1. Build k-d tree
    let mut tree: KdTree<f32, 3> = KdTree::with_capacity(list_b.len());

    // NOTE: If list_b is already sorted, shuffle it before this loop
    // to avoid O(N) worst case.
    for (i, vec) in list_b.iter().enumerate() {
        tree.add(&vec.to_array(), i as u64);
    }

    // 2. Query
    let mut pairings = Vec::with_capacity(list_a.len());
    for (i, vec_a) in list_a.iter().enumerate() {
        let neighbor = tree.nearest_one::<SquaredEuclidean>(&vec_a.to_array());
        // neighbor.item is the index we stored in .add()
        pairings.push((i, neighbor.item as usize));
    }
    pairings
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
    sims.par_iter_mut().for_each(|mut sim| {
        sim.solve_for_x();
        sim.check_breakage();
        if sim.island.as_ref().map_or(false, |i| i.needs_refactor) {
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
        let impulse = 1_000.0;
        for mut sim in sims.iter_mut() {
            for n in &mut sim.nodes {
                let diff = n.pos - center;
                let dist = diff.length();
                if dist < 20.0 {
                    let force = diff.normalize() * (impulse / dist.max(1.0));
                    // Apply as velocity change
                    n.vel += force * DT;
                }
            }
        }
    }
}

#[derive(Component)]
struct DestructionTimer(Timer);

fn shoot_ball(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    keypresses: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&GlobalTransform, With<CameraController>>,
) {
    if keypresses.just_pressed(KeyCode::KeyB) {
        let cam_transform = *camera_query;
        let shoot_vector = cam_transform.forward();
        let ball_spawn_p = cam_transform.translation();
        let ball_spawn_transform = Transform::from_translation(ball_spawn_p);
        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            ColliderDensity(8.0),
            CollisionEventsEnabled,
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.3, 0.4, 0.9)))),
            ball_spawn_transform.clone(),
            LinearVelocity(shoot_vector * 30.0),
            DestructionTimer(Timer::new(Duration::from_secs_f32(10.0), TimerMode::Once)),
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
