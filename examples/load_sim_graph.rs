//! The Pipeline:
//! Import: Detect "Simulation_Graph" mesh -> Match Vertices to Sibling Meshes -> Build Kernel.
//! Update: Apply Avian Collisions -> Run Solver -> Sync Child Transforms.

use std::{collections::VecDeque, time::Duration};

use avian3d::prelude::*;
use bevy::{
    ecs::schedule::Stepping,
    math::DVec3,
    mesh::{Indices, VertexAttributeValues},
    platform::collections::{HashMap, HashSet},
    prelude::*,
    scene::SceneInstanceReady,
};
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use kiddo::{KdTree, SquaredEuclidean};

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
            EguiPlugin::default(),
            WorldInspectorPlugin::new(),
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
            CameraControllerPlugin,
        ))
        .add_systems(Startup, setup)
        // Stepping
        //.add_systems(Startup, setup_stepping)
        //.add_systems(Update, controll_stepping)
        // Physics Step: 1. Solve FEM, 2. Break, 3. Sync Visuals/Colliders
        // Run collision handler *after* physics to catch impacts for next frame
        .add_systems(
            FixedPostUpdate,
            (
                handle_structure_collision_shock,
                solve_system,
                handle_structural_splitting,
            )
                .chain(),
        )
        .add_systems(Update, camera_movement)
        .add_systems(Update, (render_sims, handle_input_impulse))
        .add_systems(Update, (shoot_ball, handle_ball_despawning))
        .add_observer(handle_scene_added)
        .run();
}

fn setup_stepping(mut commands: Commands) {
    // Add stepping
    let mut stepping = Stepping::new();
    stepping.add_schedule(FixedPostUpdate);
    stepping.enable();

    commands.insert_resource(stepping);
}

fn controll_stepping(mut stepping: ResMut<Stepping>, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::KeyN) || keyboard.pressed(KeyCode::Space) {
        stepping.step_frame();
    }
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
            asset_server.load(
                GltfAssetLabel::Scene(0)
                    .from_asset("models/destructible/wrapper_Soviet_Apartment_Unit.glb"),
            ),
        ),
        Transform::from_xyz(10., 0., 0.),
    ));

    // Load another copy at a different position for testing.
    commands.spawn((
        SceneRoot(
            asset_server.load(
                GltfAssetLabel::Scene(0)
                    .from_asset("models/destructible/wrapper_Soviet_Apartment_Unit.001.glb"),
            ),
        ),
        Transform::from_xyz(-10., 2.1, 0.),
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
        RigidBody::Kinematic,
        Collider::cuboid(30., 1., 20.),
    ));

    // spawn a stack of 10 cubes
    //let height = 30;
    //for i in 0..(height - 1) {
    //    commands.spawn((
    //        Mesh3d(meshes.add(Cuboid::new(2.0, 0.3, 2.0))),
    //        MeshMaterial3d(materials.add(Color::srgb(
    //            1.0 - 1.0 / height as f32 * i as f32,
    //            0.1,
    //            0.1,
    //        ))),
    //        Transform::from_xyz(0.0, 0.3 * i as f32, 0.0),
    //        RigidBody::Dynamic,
    //        Collider::cuboid(2.0, 0.3, 2.0),
    //    ));
    //}

    // Spawn a stack of 10 convex cuboids
    //let height = 30;
    //for i in 0..(height - 1) {
    //    commands.spawn((
    //        SceneRoot(
    //            asset_server.load(
    //                GltfAssetLabel::Scene(0)
    //                    .from_asset("models/destructible/visual_chunk_Ceiling_Slab_0.002.glb"),
    //            ),
    //        ),
    //        RigidBody::Dynamic,
    //        Transform::from_xyz(0.0, 0.2 * i as f32, 0.0),
    //    ));
    //}

    // Light
    commands.spawn((PointLight::default(), Transform::from_xyz(-12., 20., 0.)));
}

// =========================================================================
// SETUP & ARCHITECTURE
// =========================================================================

fn handle_scene_added(
    event: On<SceneInstanceReady>,
    mut commands: Commands,
    scene_spawner: Res<SceneSpawner>,
    names: Query<&Name>,
    meshes: Res<Assets<Mesh>>,
    mesh_handles: Query<&Mesh3d>,
    transforms: Query<&Transform>,
    children_of: Query<&ChildOf>,
) {
    info!("Processing new scene instance.");

    let scene_entity = event.entity;

    // Storage for extraction
    let mut colliders_pos: Vec<Vec3> = Vec::new();
    let mut colliders_entity: Vec<Entity> = Vec::new();

    let mut graph_mesh_holder_entity: Option<Entity> = None; // sim_graph_combined_mesh_[Name]
    let mut graph_data_entity: Option<Entity> = None; // sim_graph_combined_data_[Name]

    let instance_id = event.instance_id;
    let mut scene_children_cnt: usize = 0;

    for entity in scene_spawner.iter_instance_entities(instance_id) {
        scene_children_cnt += 1;

        // Safety check: ensure entity still exists/has name
        let Ok(child_name) = names.get(entity) else {
            continue;
        };

        if child_name.starts_with("sim_graph_combined_mesh_") {
            graph_mesh_holder_entity = Some(entity);
        } else if child_name.starts_with("sim_graph_combined_data_") {
            graph_data_entity = Some(entity);
        } else if child_name.starts_with("collider_mesh_") {
            // New Hierarchy: Wrapper -> visual_chunk -> collider_chunk -> collider_mesh (entity)

            commands.entity(entity).insert((
                ColliderConstructor::ConvexHullFromMesh,
                CollisionEventsEnabled,
                CollidingEntities::default(),
                Visibility::Hidden, // Wireframe physics mesh should be hidden
            ));

            // Traverse hierarchy to calculate position relative to Scene Root (Wrapper)
            // 1. Get Parent (Collider Chunk)
            let Ok(child_of_mesh) = children_of.get(entity) else {
                continue;
            };
            let collider_chunk_entity = child_of_mesh.parent();

            // 2. Get Grandparent (Visual Chunk)
            let Ok(child_of_chunk) = children_of.get(collider_chunk_entity) else {
                continue;
            };
            let visual_chunk_entity = child_of_chunk.parent();

            // 3. Get Transforms
            let mesh_tf = transforms.get(entity).unwrap_or(&Transform::IDENTITY);
            let collider_chunk_tf = transforms
                .get(collider_chunk_entity)
                .unwrap_or(&Transform::IDENTITY);
            let visual_chunk_tf = transforms
                .get(visual_chunk_entity)
                .unwrap_or(&Transform::IDENTITY);

            // Calculate Composite Transform relative to Wrapper
            // Pos = T_visual * T_collider_chunk * T_collider_mesh * Zero
            let pos_in_collider_chunk = mesh_tf.transform_point(Vec3::ZERO);
            let pos_in_visual_chunk = collider_chunk_tf.transform_point(pos_in_collider_chunk);
            let pos_in_wrapper = visual_chunk_tf.transform_point(pos_in_visual_chunk);

            colliders_pos.push(pos_in_wrapper);
            colliders_entity.push(entity);
        } else if child_name.starts_with("visual_mesh_") {
            // Tag the renderable mesh
            commands.entity(entity).insert(VisualSimPart);
        }
    }
    info!("Scene has {} descendants.", scene_children_cnt);

    // --- Process Graph ---
    let Some(graph_data_entity) = graph_data_entity else {
        warn!("Simulation Graph Data mesh not found in scene.");
        return;
    };

    // We need the parent object (holder) to get the full transform of the graph vertices
    // Hierarchy: Wrapper -> sim_graph_combined_mesh -> sim_graph_combined_data
    let graph_holder_tf = graph_mesh_holder_entity
        .and_then(|e| transforms.get(e).ok())
        .copied()
        .unwrap_or(Transform::IDENTITY);

    let graph_data_tf = transforms
        .get(graph_data_entity)
        .copied()
        .unwrap_or(Transform::IDENTITY);

    // Combine transforms to convert Vertex Data Space -> Wrapper Space
    let graph_to_wrapper_tf = graph_holder_tf * graph_data_tf;

    let Ok(mesh_handle) = mesh_handles.get(graph_data_entity) else {
        return;
    };
    let Some(mesh) = meshes.get(mesh_handle.id()) else {
        return;
    };

    let positions = match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
        Some(VertexAttributeValues::Float32x3(p)) => p,
        _ => return,
    };

    // Graph Vertices transformed to Local Space of the Scene Root (Wrapper)
    let graph_local_positions: Vec<Vec3> = positions
        .iter()
        .map(|e| graph_to_wrapper_tf.transform_point(Vec3::from(*e)))
        .collect();

    let colors: &Vec<[f32; 4]> = match mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
        Some(VertexAttributeValues::Float32x4(c)) => c,
        _ => &vec![[0., 0., 0., 1.]; positions.len()],
    };

    // --- Core Logic (Unchanged) ---
    let pairings = pair_vectors_optimized(&colliders_pos, &graph_local_positions);

    let mut structural_sim = StructuralSim::new();
    let mut graph_idx_to_node = HashMap::<usize, usize>::new();

    let mut has_fixed_node = false;

    for (idx_collider, idx_graph) in pairings {
        let fixed = colors[idx_graph][0] > 0.5;

        has_fixed_node |= fixed;

        let local_pos = graph_local_positions[idx_graph];

        let node_index = structural_sim.add_node(
            fixed,
            local_pos,
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
            colliders_entity[idx_collider],
        );

        commands
            .entity(colliders_entity[idx_collider])
            .insert(SimPart {
                root_entity: scene_entity,
                node_index,
            });

        graph_idx_to_node.insert(idx_graph, node_index);
    }

    if let Some(indices) = mesh.indices() {
        let indices_iter: Box<dyn Iterator<Item = usize>> = match indices {
            Indices::U16(idxs) => Box::new(idxs.iter().map(|v| *v as usize)),
            Indices::U32(idxs) => Box::new(idxs.iter().map(|v| *v as usize)),
        };
        for edge in indices_iter.collect::<Vec<usize>>().chunks_exact(2) {
            let (a, b) = (edge[0], edge[1]);
            if let (Some(&na), Some(&nb)) = (graph_idx_to_node.get(&a), graph_idx_to_node.get(&b)) {
                structural_sim.add_conn(na, nb);
            }
        }
    }

    structural_sim.create_island();

    // The Wrapper becomes the RigidBody root.
    let rigid_body = if has_fixed_node {
        RigidBody::Kinematic
    } else {
        RigidBody::Dynamic
    };
    commands.entity(scene_entity).insert((
        rigid_body,
        SleepThreshold {
            linear: 1.0,
            angular: 1.0,
        },
        structural_sim,
    ));

    info!("StructuralSim set up as Kinematic Anchor.");
}

// =========================================================================
// COLLISION HANDLING (SHOCK INJECTION)
// =========================================================================

/// Handles "Shock": When a Dynamic structure hits something, the impact stops the Root.
fn handle_structure_collision_shock(
    sim_parts: Query<(&SimPart, &CollidingEntities), With<CollisionEventsEnabled>>,
    collisions: Collisions,
    mut structures: Query<(&mut StructuralSim, &GlobalTransform)>,
    roots: Query<&RigidBody>,
) {
    let mut impact_buffer: HashMap<Entity, Vec<(usize, DVec3, DVec3)>> = HashMap::new();

    for contacts in collisions.iter() {
        let Some(manifold) = contacts.manifolds.first() else {
            continue;
        };

        // Avian: Manifold Normal points from Entity 2 -> Entity 1
        let normal_world = manifold.normal.as_dvec3();
        let impulse_mag = contacts.total_normal_impulse_magnitude() as f64;

        if impulse_mag < 1.0 {
            continue;
        }

        // Calculate world contact position
        let contact_count = manifold.points.len() as f64;
        let contact_pos_world: DVec3 = manifold
            .points
            .iter()
            .map(|p| p.point.as_dvec3())
            .sum::<DVec3>()
            / contact_count;

        let mut process_impact = |entity: Entity, is_entity_1: bool| {
            // 1. Is it a SimPart?
            let Ok((part, _)) = sim_parts.get(entity) else {
                return;
            };

            // 2. Does it have a valid physics root? (Kinematic OR Dynamic)
            if roots.contains(part.root_entity) {
                // E1 gets +Normal, E2 gets -Normal
                let impulse_dir = if is_entity_1 {
                    normal_world
                } else {
                    -normal_world
                };
                let impulse_vector_world = impulse_dir * impulse_mag;

                if let Ok((_, root_tf)) = structures.get(part.root_entity) {
                    // Transform World Impulse -> Local Space
                    let root_rot_inv = root_tf.compute_transform().rotation.inverse().as_dquat();
                    let impulse_local = root_rot_inv * impulse_vector_world;

                    // Transform World Contact Pos -> Local Space
                    let contact_pos_local = root_tf
                        .affine()
                        .inverse()
                        .transform_point3(contact_pos_world.as_vec3())
                        .as_dvec3();

                    impact_buffer.entry(part.root_entity).or_default().push((
                        part.node_index,
                        impulse_local,
                        contact_pos_local,
                    ));
                }
            }
        };

        process_impact(contacts.collider1, true);
        process_impact(contacts.collider2, false);
    }

    if !impact_buffer.is_empty() {
        info!("Processed {} structural impacts", impact_buffer.len());
    }

    // Apply Impacts
    for (root_entity, impacts) in impact_buffer {
        if let Ok((mut sim, _)) = structures.get_mut(root_entity) {
            for (node_idx, impulse_local, contact_pos_local) in impacts {
                if node_idx >= sim.nodes.len() {
                    continue;
                }

                let node = &mut sim.nodes[node_idx];

                // Calculate Arm from Node Center to Contact Point
                let arm = contact_pos_local - node.pos;

                // 1. Linear Shock (Force/Mass)
                node.vel += impulse_local / NODE_MASS;

                // 2. Angular Shock (Torque/Inertia)
                let torque_impulse = arm.cross(impulse_local);
                node.ang_vel += torque_impulse / NODE_MOMENT_OF_INERTIA;
            }
        }
    }
}

// =========================================================================
// SPLITTING LOGIC
// =========================================================================

fn handle_structural_splitting(
    mut commands: Commands,
    mut structures: Query<(
        Entity,
        &mut StructuralSim,
        &GlobalTransform,
        Option<&LinearVelocity>,
    )>,
    mut sim_parts: Query<&mut SimPart>,
    parents: Query<&ChildOf>,
) {
    for (original_root_entity, mut system, root_global_tf, root_lin_vel) in structures.iter_mut() {
        // 1. Trigger Check
        if !system.island.as_ref().map_or(false, |i| i.needs_refactor) {
            continue;
        }

        let n_nodes = system.nodes.len();
        if n_nodes == 0 {
            continue;
        }

        // 2. Build Adjacency Graph
        let mut adj = vec![Vec::new(); n_nodes];
        let mut active_node_indices = HashSet::new();
        for conn in &system.connections {
            if conn.alive {
                adj[conn.node_a].push(conn.node_b);
                adj[conn.node_b].push(conn.node_a);
                active_node_indices.insert(conn.node_a);
                active_node_indices.insert(conn.node_b);
            }
        }
        for i in 0..n_nodes {
            if system.node_to_entity[i] != Entity::PLACEHOLDER {
                active_node_indices.insert(i);
            }
        }

        // 3. Find Connected Components
        let mut visited = vec![false; n_nodes];
        let mut components: Vec<Vec<usize>> = Vec::new();
        for &i in &active_node_indices {
            if visited[i] {
                continue;
            }
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

        if components.len() <= 1 {
            continue;
        }
        components.sort_by_key(|c| std::cmp::Reverse(c.len()));

        info!("Structure split into {} islands.", components.len());

        // 4. Spawn New Entities for ALL Components (Survivor + Debris)
        // We do this for everyone to ensure clean Physics State (Mass, Colliders, Solver Registry).
        for (i, indices) in components.iter().enumerate() {
            // Check if this chunk has any fixed nodes (Anchors)
            let has_anchor = indices.iter().any(|&idx| system.nodes[idx].fixed);
            let is_dynamic = !has_anchor;

            spawn_island(
                &mut commands,
                &system,
                indices,
                root_global_tf,
                root_lin_vel.cloned().unwrap_or(LinearVelocity::ZERO),
                &mut sim_parts,
                &parents,
                is_dynamic,
                // Name them for easier debugging
                if i == 0 {
                    "Survivor_Chunk"
                } else {
                    "Debris_Chunk"
                },
            );
        }

        // 5. Despawn the Old Root
        // Since we migrated all visuals and physics to the new islands, the old wrapper is empty/stale.
        commands.entity(original_root_entity).despawn();
    }
}

fn spawn_island(
    commands: &mut Commands,
    original_system: &StructuralSim,
    indices: &[usize],
    old_root_tf: &GlobalTransform,
    inherited_velocity: LinearVelocity,
    sim_parts: &mut Query<&mut SimPart>,
    parents: &Query<&ChildOf>,
    is_dynamic: bool,
    name_str: &str,
) {
    // 1. Calculate Centroid (New Origin)
    let mut center_acc = DVec3::ZERO;
    let mut count = 0.0;
    for &idx in indices {
        let local_pos = original_system.nodes[idx].pos;
        let world_pos = old_root_tf.transform_point(local_pos.as_vec3()).as_dvec3();
        center_acc += world_pos;
        count += 1.0;
    }
    if count == 0.0 {
        return;
    }
    let new_origin_world = center_acc / count;

    // 2. Create System Subset
    // force_unfix = true if the chunk is dynamic (no anchors)
    let (mut new_sys, child_map) = extract_system_subset(
        original_system,
        indices,
        old_root_tf,
        Some(new_origin_world),
        is_dynamic,
    );
    new_sys.is_dynamic = is_dynamic;
    new_sys.create_island();

    // 3. Spawn RigidBody
    let mut cmd = commands.spawn((
        new_sys,
        Transform::from_translation(new_origin_world.as_vec3())
            .with_rotation(old_root_tf.compute_transform().rotation),
        GlobalTransform::from_translation(new_origin_world.as_vec3()),
        CollisionEventsEnabled, // Keep user's custom event flag
        Name::new(name_str.to_string()),
    ));

    if is_dynamic {
        // Explicitly calculate Mass/Inertia to satisfy Solver on Frame 1
        let total_mass = count as f32 * NODE_MASS as f32;
        let inertia_val = (total_mass / 6.0) * 10.0; // Box approximation

        cmd.insert((
            RigidBody::Dynamic,
            inherited_velocity,
            Mass(total_mass),
            // AngularInertia is a struct in Avian 0.4+, use ::new()
            AngularInertia::new(Vec3::splat(inertia_val)),
            CenterOfMass::default(),
        ));
    } else {
        cmd.insert(RigidBody::Kinematic);
    }

    let new_root_entity = cmd.id();

    // 4. Migrate Children
    for (new_idx, collider_entity) in child_map {
        if collider_entity == Entity::PLACEHOLDER {
            continue;
        }

        // Update Mapping
        if let Ok(mut handle) = sim_parts.get_mut(collider_entity) {
            handle.root_entity = new_root_entity;
            handle.node_index = new_idx;
        }

        // Force Contact Reset: Remove Collider so Avian drops old contacts
        commands
            .entity(collider_entity)
            .remove::<(Collider, CollidingEntities)>();

        // Reparent Hierarchy: ColliderMesh -> ColliderChunk -> VisualChunk -> [NewRoot]
        if let Ok(child_of_mesh) = parents.get(collider_entity) {
            let collider_chunk = child_of_mesh.parent();
            if let Ok(child_of_chunk) = parents.get(collider_chunk) {
                let visual_entity = child_of_chunk.parent();
                commands
                    .entity(visual_entity)
                    .set_parent_in_place(new_root_entity);
            }
        }

        // Re-add Physics: Avian will generate new Colliders in the next frame
        commands.entity(collider_entity).insert((
            ColliderConstructor::ConvexHullFromMesh,
            CollidingEntities::default(),
        ));
    }
}

// Helper (Unchanged from previous valid version)
fn extract_system_subset(
    source: &StructuralSim,
    indices: &[usize],
    old_root_tf: &GlobalTransform,
    new_origin_world: Option<DVec3>,
    force_unfix: bool,
) -> (StructuralSim, Vec<(usize, Entity)>) {
    let mut new_nodes = Vec::with_capacity(indices.len());
    let mut new_node_to_entity = Vec::with_capacity(indices.len());
    let mut old_to_new_map = HashMap::with_capacity(indices.len());
    let mut child_map_result = Vec::with_capacity(indices.len());

    let old_root_affine = old_root_tf.affine();

    for (new_idx, &old_idx) in indices.iter().enumerate() {
        let mut node = source.nodes[old_idx].clone();
        if force_unfix {
            node.fixed = false;
        }

        if let Some(origin) = new_origin_world {
            let pos_world = old_root_affine.transform_point3(node.pos.as_vec3());
            let offset_world = pos_world - origin.as_vec3();
            let rot_inv = old_root_tf.compute_transform().rotation.inverse();
            node.pos = (rot_inv * offset_world).as_dvec3();
            node.vel = DVec3::ZERO;
            node.ang_vel = DVec3::ZERO;
        }

        new_nodes.push(node);
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
            c.last_stress_ratio = 0.0;
            c.baseline_impulse = 0.0;
            new_connections.push(c);
        }
    }

    let new_sys = StructuralSim {
        nodes: new_nodes,
        connections: new_connections,
        island: None,
        frame_count: 0,
        node_to_entity: new_node_to_entity,
        is_dynamic: if force_unfix { true } else { source.is_dynamic },
    };

    (new_sys, child_map_result)
}

// Helpers
fn pair_vectors_optimized(list_a: &Vec<Vec3>, list_b: &Vec<Vec3>) -> Vec<(usize, usize)> {
    assert_eq!(list_a.len(), list_b.len());
    let mut tree: KdTree<f32, 3> = KdTree::with_capacity(list_b.len());
    for (i, vec) in list_b.iter().enumerate() {
        tree.add(&vec.to_array(), i as u64);
    }
    let mut pairings = Vec::with_capacity(list_a.len());
    for (i, vec_a) in list_a.iter().enumerate() {
        let neighbor = tree.nearest_one::<SquaredEuclidean>(&vec_a.to_array());
        pairings.push((i, neighbor.item as usize));
    }
    pairings
}

fn solve_system(mut sims: Query<&mut StructuralSim>) {
    sims.par_iter_mut().for_each(|mut sim| {
        sim.solve_for_x();
        sim.check_breakage();
        if sim.island.as_ref().map_or(false, |i| i.needs_refactor) {
            sim.update_stiffness_matrix();
        }
    });
}

fn render_sims(mut gizmos: Gizmos, sims: Query<(&StructuralSim, &GlobalTransform)>) {
    for (sim, root_tr) in sims.iter() {
        let root_mat = Mat4::from(root_tr.affine());

        for conn in &sim.connections {
            if !conn.alive {
                continue;
            }
            let p_a = root_mat.transform_point3(sim.nodes[conn.node_a].pos.as_vec3());
            let p_b = root_mat.transform_point3(sim.nodes[conn.node_b].pos.as_vec3());

            let ratio = conn.last_stress_ratio;
            let color = if ratio > 1.0 {
                Color::WHITE
            } else {
                Color::srgb(ratio, 1.0 - ratio, 0.0)
            };
            gizmos.line(p_a, p_b, color);
        }

        // Debug draw nodes
        for node in &sim.nodes {
            let p_world = root_mat.transform_point3(node.pos.as_vec3());
            let color = if node.fixed {
                Color::WHITE
            } else {
                Color::srgb(0., 0., 1.)
            };
            gizmos.cube(
                Transform::from_translation(p_world).with_scale(Vec3::splat(0.2)),
                color,
            );
        }
    }
}

// =========================================================================
// INPUT & GAMEPLAY
// =========================================================================

fn handle_input_impulse(
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&GlobalTransform, With<Camera>>,
    mut sims: Query<(&mut StructuralSim, &GlobalTransform)>,
) {
    let tform = *camera;
    let pole_len = tform.forward() * 8.0;
    let center_f32 = tform.translation() + pole_len;

    if mouse.just_pressed(MouseButton::Left) {
        let impulse = 5_000.0;

        for (mut sim, root_tr) in sims.iter_mut() {
            // Need to transform World Input -> Local Node
            let root_inv = root_tr.affine().inverse();
            let center_local = (root_inv.transform_point3(center_f32)).as_dvec3();

            for n in &mut sim.nodes {
                let diff = n.pos - center_local;
                let dist = diff.length();
                if dist < 10.0 {
                    let force = diff.normalize() * (impulse / dist.max(1.0));
                    n.vel += force * DT;
                    break;
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
        let ball_spawn_p = cam_transform.translation() + shoot_vector * 2.0;

        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(0.5),
            ColliderDensity(100.0),
            // Enable events so the collision system picks it up
            CollisionEventsEnabled,
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.3, 0.4, 0.9)))),
            Transform::from_translation(ball_spawn_p),
            LinearVelocity(shoot_vector * 40.0),
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
