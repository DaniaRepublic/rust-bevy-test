//! Implementation of a realtime structural stress simulation (6-DOF Frame Solver).
//! Uses faer for sparse Cholesky decomposition.

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    math::{DQuat, DVec3, dvec3},
    prelude::*,
};
use faer::{
    prelude::*,
    sparse::{SparseColMat, Triplet},
};
#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;

// =========================================================================
// CONFIGURATION
// =========================================================================

const DT: f64 = 0.01667;
const DOF_PER_NODE: usize = 6;

// --- UNITS ---
// Mass: kg
// Distance: meters
// Time: seconds

/// Gravity (Y-axis)
const GRAVITY: DVec3 = DVec3::new(0.0, -9.81, 0.0);

/// Standard Node Mass
const NODE_MASS: f64 = 100.0;
/// "Infinite" mass for fixed nodes (Penalty Method)
const ANCHOR_MASS: f64 = 1e12;

/// Moment of Inertia (Approx Sphere: 2/5 * M * R^2)
const NODE_MOMENT_OF_INERTIA: f64 = 1_000.0;

// --- STIFFNESS TUNING ---
// Axial: Resists stretching (pulling apart)
const JOINT_LIN_STIFFNESS: f64 = 1e6;
// Shear: Resists sliding sideways (parallelograming)
const JOINT_SHEAR_STIFFNESS: f64 = 1e6;
// Rotational: Resists bending/twisting
const JOINT_ROT_STIFFNESS: f64 = 1e8;

/// Visualization: Scale for color gradient
const MAX_CONNECTION_DISP: f64 = 1.10;

// =========================================================================
// DATA STRUCTURES
// =========================================================================

struct PhysNode {
    fixed: bool,
    pos: DVec3,
    vel: DVec3,
    rot: DQuat,
    ang_vel: DVec3,
}

struct Connection {
    node_a: usize,
    node_b: usize,
    rest_len: f64,
    /// The relative rotation from A to B when connected.
    rest_local_rot: DQuat,
    /// The direction vector from A to B in A's local space.
    local_axis: DVec3,
}

struct Island {
    sleeping: bool,
    a: SparseColMat<usize, f64>,
    b: Col<f64>,
}

#[derive(Resource)]
struct PhysicsWorld {
    nodes: Vec<PhysNode>,
    connections: Vec<Connection>,
    island: Option<Island>,
}

// =========================================================================
// PHYSICS IMPLEMENTATION
// =========================================================================

impl PhysicsWorld {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            connections: Vec::new(),
            island: None,
        }
    }

    fn add_node(
        &mut self,
        fixed: bool,
        pos: DVec3,
        vel: DVec3,
        rot: DQuat,
        ang_vel: DVec3,
    ) -> usize {
        self.nodes.push(PhysNode {
            fixed,
            pos,
            vel,
            rot,
            ang_vel,
        });
        self.nodes.len() - 1
    }

    fn add_conn(&mut self, node_a: usize, node_b: usize) {
        let pos_a = self.nodes[node_a].pos;
        let pos_b = self.nodes[node_b].pos;
        let diff = pos_b - pos_a;
        let rest_len = diff.length();

        // Calculate Direction in World Space
        let dir_world = if rest_len > 1e-6 {
            diff / rest_len
        } else {
            DVec3::Y
        };

        let rot_a = self.nodes[node_a].rot;
        let rot_b = self.nodes[node_b].rot;

        // 1. Initial Relative Rotation (q_a^-1 * q_b)
        let rest_local_rot = rot_a.inverse() * rot_b;

        // 2. Initial Local Axis (The "Socket" direction relative to A)
        let local_axis = rot_a.inverse() * dir_world;

        self.connections.push(Connection {
            node_a,
            node_b,
            rest_len,
            rest_local_rot,
            local_axis,
        });
    }

    /// Constructs the System Matrix A = M + dt^2 * K
    fn create_island(&mut self) {
        let n_nodes = self.nodes.len();
        let dof: usize = n_nodes * DOF_PER_NODE;

        let mut triplets: Vec<Triplet<usize, usize, f64>> = Vec::new();

        // -----------------------------------------------------------------
        // 1. INERTIA (Mass Matrix M)
        // -----------------------------------------------------------------
        for i in 0..n_nodes {
            let offset = i * DOF_PER_NODE;

            // Penalty Method: Fixed nodes get massive inertia
            let mass = if self.nodes[i].fixed {
                ANCHOR_MASS
            } else {
                NODE_MASS
            };
            let inertia = if self.nodes[i].fixed {
                ANCHOR_MASS
            } else {
                NODE_MOMENT_OF_INERTIA
            };

            // Translation Mass (Diagonal)
            triplets.push(Triplet::new(offset, offset, mass));
            triplets.push(Triplet::new(offset + 1, offset + 1, mass));
            triplets.push(Triplet::new(offset + 2, offset + 2, mass));

            // Rotational Inertia (Diagonal)
            triplets.push(Triplet::new(offset + 3, offset + 3, inertia));
            triplets.push(Triplet::new(offset + 4, offset + 4, inertia));
            triplets.push(Triplet::new(offset + 5, offset + 5, inertia));
        }

        // -----------------------------------------------------------------
        // 2. STIFFNESS (K Matrix)
        // -----------------------------------------------------------------
        let dt2 = DT * DT;

        let k_axial = JOINT_LIN_STIFFNESS * dt2;
        let k_shear = JOINT_SHEAR_STIFFNESS * dt2;
        let k_rot = JOINT_ROT_STIFFNESS * dt2;

        for conn in self.connections.iter() {
            let node_a = conn.node_a;
            let node_b = conn.node_b;

            let diff = self.nodes[node_b].pos - self.nodes[node_a].pos;
            let len_sq = diff.length_squared();
            if len_sq < 1e-9 {
                continue;
            } // Protect against zero-length

            let norm_dir = diff / len_sq.sqrt();

            // --- A. LINEAR BLOCK (Axial + Shear) ---
            // Formula: K = (K_axial - K_shear) * (n * nT) + K_shear * I
            // This resists stretching AND sliding sideways.

            for r in 0..3 {
                for c in 0..3 {
                    // Kronecker Delta
                    let delta = if r == c { 1.0 } else { 0.0 };

                    // Combined Stiffness
                    let val = (k_axial - k_shear) * norm_dir[r] * norm_dir[c] + (k_shear * delta);

                    let row_a = node_a * 6 + r;
                    let col_a = node_a * 6 + c;
                    let row_b = node_b * 6 + r;
                    let col_b = node_b * 6 + c;

                    // Self-interaction (+K)
                    triplets.push(Triplet::new(row_a, col_a, val));
                    triplets.push(Triplet::new(row_b, col_b, val));
                    // Mutual interaction (-K)
                    triplets.push(Triplet::new(row_a, col_b, -val));
                    triplets.push(Triplet::new(row_b, col_a, -val));
                }
            }

            // --- B. ANGULAR BLOCK ---
            // Simplified Isotropic Bending
            for r in 0..3 {
                let val = k_rot;

                let row_a = node_a * 6 + 3 + r;
                let col_a = node_a * 6 + 3 + r;
                let row_b = node_b * 6 + 3 + r;
                let col_b = node_b * 6 + 3 + r;

                // Self (+K)
                triplets.push(Triplet::new(row_a, col_a, val));
                triplets.push(Triplet::new(row_b, col_b, val));

                // Mutual (-K) - FIX: This must be negative!
                let row_a_cross = node_a * 6 + 3 + r;
                let col_a_cross = node_b * 6 + 3 + r;
                let row_b_cross = node_b * 6 + 3 + r;
                let col_b_cross = node_a * 6 + 3 + r;

                triplets.push(Triplet::new(row_a_cross, col_a_cross, -val));
                triplets.push(Triplet::new(row_b_cross, col_b_cross, -val));
            }
        }

        // 3. Create Matrix
        let a = SparseColMat::<usize, f64>::try_new_from_triplets(dof, dof, &triplets)
            .expect("Failed to create matrix from triplets");

        let a_rows = a.nrows();
        let b = Col::zeros(a_rows);

        self.island = Some(Island {
            sleeping: false,
            a,
            b,
        });
    }

    /// Solves the linear system to integrate velocities
    fn solve_for_x(&mut self) {
        let Some(island) = self.island.as_mut() else {
            return;
        };
        if island.sleeping {
            return;
        }

        let n_nodes = self.nodes.len();

        // Reset RHS (b)
        island.b.fill(0.0);

        // -----------------------------------------------------------------
        // 1. PREDICTION (Momentum)
        // b = M * (v + g*dt)
        // -----------------------------------------------------------------
        for (i, node) in self.nodes.iter().enumerate() {
            if node.fixed {
                continue;
            }

            let offset = i * DOF_PER_NODE;

            // Linear Momentum
            let vel_pred = node.vel + GRAVITY * DT;
            let momentum = vel_pred * NODE_MASS;
            island.b[offset + 0] = momentum.x;
            island.b[offset + 1] = momentum.y;
            island.b[offset + 2] = momentum.z;

            // Angular Momentum (Approx conservation)
            let ang_momentum = node.ang_vel * NODE_MOMENT_OF_INERTIA;
            island.b[offset + 3] = ang_momentum.x;
            island.b[offset + 4] = ang_momentum.y;
            island.b[offset + 5] = ang_momentum.z;
        }

        // -----------------------------------------------------------------
        // 2. RESTORING FORCES
        // -----------------------------------------------------------------
        for conn in self.connections.iter() {
            let p_a = self.nodes[conn.node_a].pos;
            let p_b = self.nodes[conn.node_b].pos;

            // --- A. LINEAR RESTORATION (Drift Correction) ---
            // "Where should B be relative to A?"
            let q_a = self.nodes[conn.node_a].rot;
            let target_offset_world = q_a * conn.local_axis * conn.rest_len;
            let target_pos_b = p_a + target_offset_world;

            // Vector Error: Where B is vs Where it should be
            // (Using the vector error allows Shear stiffness to work correctly)
            let pos_error = p_b - target_pos_b;

            if pos_error.length_squared() > 1e-12 {
                // Apply Stiffness
                // NOTE: For simplicity, we apply LIN_STIFFNESS to the whole error vector.
                // A more precise version would project this onto the axis vs shear plane.
                let restoring_force = pos_error * JOINT_LIN_STIFFNESS;
                let impulse = restoring_force * DT;

                let idx_a = conn.node_a * DOF_PER_NODE;
                let idx_b = conn.node_b * DOF_PER_NODE;

                // Pull B towards target, Pull A towards B
                // Sign: If p_b is too far (positive error), we want negative force on B.

                // Force on A
                island.b[idx_a + 0] += impulse.x;
                island.b[idx_a + 1] += impulse.y;
                island.b[idx_a + 2] += impulse.z;
                // Force on B
                island.b[idx_b + 0] -= impulse.x;
                island.b[idx_b + 1] -= impulse.y;
                island.b[idx_b + 2] -= impulse.z;

                // (Torque from linear force leverage is omitted for stability,
                // but 'Shear' matrix term handles the resistance implicitely)
            }

            // --- B. ANGULAR RESTORATION (Bending/Torsion) ---
            let q_b = self.nodes[conn.node_b].rot;

            // Current relative rot: q_rel = q_a^-1 * q_b
            let q_rel_curr = q_a.inverse() * q_b;

            // Error: difference between current relative and rest relative
            let q_error = q_rel_curr * conn.rest_local_rot.inverse();

            let (axis, angle) = q_error.to_axis_angle();
            if angle > 1e-6 {
                // Torque magnitude
                let torque_mag = JOINT_ROT_STIFFNESS * angle;

                // Axis is in A's local space (due to q_a.inverse)
                // Transform to World
                let torque_world = q_a * axis * torque_mag;
                let ang_impulse = torque_world * DT;

                let idx_a = conn.node_a * DOF_PER_NODE;
                let idx_b = conn.node_b * DOF_PER_NODE;

                // Apply Torque (Opposite directions)
                // A pulls forward
                island.b[idx_a + 3] += ang_impulse.x;
                island.b[idx_a + 4] += ang_impulse.y;
                island.b[idx_a + 5] += ang_impulse.z;

                // B pulls back
                island.b[idx_b + 3] -= ang_impulse.x;
                island.b[idx_b + 4] -= ang_impulse.y;
                island.b[idx_b + 5] -= ang_impulse.z;
            }
        }

        // -----------------------------------------------------------------
        // 3. SOLVE & UPDATE
        // -----------------------------------------------------------------
        if let Ok(llt) = island.a.sp_cholesky(faer::Side::Lower) {
            let x = llt.solve(&island.b);

            for i in 0..n_nodes {
                // Skip fixed nodes
                if self.nodes[i].fixed {
                    continue;
                }

                let offset = i * DOF_PER_NODE;

                // Update Linear Velocity
                self.nodes[i].vel = DVec3::new(x[offset], x[offset + 1], x[offset + 2]);

                // Update Angular Velocity
                self.nodes[i].ang_vel = DVec3::new(x[offset + 3], x[offset + 4], x[offset + 5]);

                // Update Position
                let vel = self.nodes[i].vel;
                self.nodes[i].pos += vel * DT;

                // Update Rotation
                let w = self.nodes[i].ang_vel;
                let q = self.nodes[i].rot;
                // Quaternion derivative: dq/dt = 0.5 * w * q
                let dq = DQuat::from_xyzw(w.x, w.y, w.z, 0.0) * q * (0.5 * DT);
                self.nodes[i].rot = (q + dq).normalize();
            }
        } else {
            warn!("Cholesky failed! Matrix is indefinite.");
        }
    }
}

// =========================================================================
// BEVY BOILERPLATE
// =========================================================================

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
            CameraControllerPlugin,
        ))
        .insert_resource(PhysicsWorld::new())
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, solve_system)
        .add_systems(Update, (render, input, camera_movement))
        .insert_resource(Time::<Fixed>::from_seconds(DT))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut sim: ResMut<PhysicsWorld>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(20.0, 15.0, 30.0).looking_at(Vec3::new(5.0, 5.0, 0.0), Vec3::Y),
        CameraController::default(),
    ));

    // Floor
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(50.0, 1.0, 50.0))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(Color::srgb(0.2, 0.2, 0.2)))),
        Transform::from_xyz(0.0, -2.0, 0.0),
    ));

    // Light
    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, 10.0)));

    // --- SETUP T-SHAPE STRUCTURE ---
    // Using a T-shape is the ultimate test for Rotational/Shear stiffness.
    // If rotation is broken, the cross-bar will flop down.

    // Node 0: Base (Fixed)
    let n0 = sim.add_node(
        true,
        dvec3(0.0, 0.0, 0.0),
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ZERO,
    );

    // Node 1: Top of Pillar
    let n1 = sim.add_node(
        false,
        dvec3(0.0, 6.0, 0.0),
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ZERO,
    );

    // Node 2: Left Arm
    let n2 = sim.add_node(
        false,
        dvec3(-4.0, 6.0, 0.0),
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ZERO,
    );

    // Node 3: Right Arm
    let n3 = sim.add_node(
        false,
        dvec3(4.0, 6.0, 0.0),
        DVec3::ZERO,
        DQuat::IDENTITY,
        DVec3::ZERO,
    );

    // Connections
    sim.add_conn(n0, n1); // Pillar
    sim.add_conn(n1, n2); // Left Arm
    sim.add_conn(n1, n3); // Right Arm

    sim.create_island();

    commands.spawn((
        Text::new(
            "6-DOF Frame Solver\nStructure should remain rigid T-shape.\nClick to apply impulse.",
        ),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..Default::default()
        },
    ));
}

fn solve_system(mut sim: ResMut<PhysicsWorld>) {
    sim.solve_for_x();
}

fn input(mouse: Res<ButtonInput<MouseButton>>, mut sim: ResMut<PhysicsWorld>) {
    if mouse.just_pressed(MouseButton::Left) {
        // Apply impulse to Right Arm to make it swing/wobble
        if sim.nodes.len() > 3 {
            sim.nodes[3].vel.z += 5.0; // Push sideways
            sim.nodes[3].vel.y += 2.0; // Push up
            println!("Impulse applied!");
        }
    }
}

fn render(mut gizmos: Gizmos, sim: Res<PhysicsWorld>) {
    // Draw Connections
    for conn in &sim.connections {
        let p_a = sim.nodes[conn.node_a].pos.as_vec3();
        let p_b = sim.nodes[conn.node_b].pos.as_vec3();

        // Color based on strain is nice, but simple white is fine for testing rigidity
        gizmos.line(p_a, p_b, Color::WHITE);
    }

    // Draw Nodes
    for node in &sim.nodes {
        let pos = node.pos.as_vec3();
        let color = if node.fixed {
            Color::srgb(1.0, 0.0, 0.0)
        } else {
            Color::srgb(0.0, 0.0, 1.0)
        };
        gizmos.cuboid(
            Transform::from_translation(pos).with_scale(Vec3::splat(0.5)),
            color,
        );

        // Visualize rotation axis (Local X)
        let axis = (node.rot * DVec3::X).as_vec3();
        gizmos.line(pos, pos + axis, Color::srgb(1.0, 0.0, 0.0));
    }
}
