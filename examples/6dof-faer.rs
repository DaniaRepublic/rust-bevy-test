//! Implementation of a realtime structural stress simulation (6-DOF Frame Solver).
//! Uses faer for sparse (hopefully supernodal) Cholesky decomposition.

#[allow(unused_imports)] // for FrameTimeDiagnosticsPlugin as it floods console
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
};
use faer::{
    get_global_parallelism,
    prelude::*,
    sparse::{
        SparseColMat, Triplet,
        linalg::solvers::{Llt, SymbolicLlt},
    },
};
#[path = "../src/camera_controller.rs"]
mod camera_controller;
use camera_controller::*;

// =========================================================================
// CONFIGURATION
// =========================================================================

const DT: f32 = 0.01667;
const DOF_PER_NODE: usize = 6;

// Global Damping (Air Resistance)
const LIN_DAMPING: f32 = 0.999;
const ANG_DAMPING: f32 = 0.997;

// --- INTERNAL DAMPING (Shock Absorbers) ---
// MUST satisfy: D < 2 * Mass / dt
// Linear Limit: 2 * 100 / 0.016 = 12,000. Safe value: 5,000.
// Angular Limit: 2 * 20 / 0.016 = 2,400. Safe value: 1,000.
const INTERNAL_LIN_DAMPING: f32 = 3000.0;
const INTERNAL_ROT_DAMPING: f32 = 500.0; // Reduced drastically to stop explosions

const GRAVITY: Vec3 = Vec3::new(0.0, -9.81, 0.0);

// MASS
const NODE_MASS: f32 = 50.0; // Reduced slightly
const NODE_MOMENT_OF_INERTIA: f32 = 5.0; // Reduced to help tumbling
// Reduced Anchor Mass to fit within f32 precision (Mantissa is ~7 digits)
// 1e8 vs 100 (1e2) fits. 1e12 does not.
const ANCHOR_MASS: f32 = 1e8;

// --- STIFFNESS TUNING ---
// Stiffness needs to be high to prevent "sag", but Strength needs to be lower.
const JOINT_LIN_STIFFNESS: f32 = 1.0e8;
// Increase Rotational Stiffness so walls act like plates, not hinges
const JOINT_ROT_STIFFNESS: f32 = 1.0e9;

// STRENGTH CALCULATION
// Gravity Impulse per node = 50kg * 9.8 * 0.016 ~= 8.0 units.
// A base connection supporting a column of 20 nodes carries ~160 impulse.
// If we set Strength to 1000.0, it can support ~120 nodes.
// Set it lower to make it fragile.
const CONCRETE_STRENGTH: f32 = 2000.0;

// =========================================================================
// DATA STRUCTURES
// =========================================================================

struct MaterialProps {
    tension_mult: f32,
    compression_mult: f32,
    shear_mult: f32,
    bending_mult: f32,
    strength: f32,
}

struct PhysNode {
    fixed: bool,
    pos: Vec3,
    vel: Vec3,
    rot: Quat,
    ang_vel: Vec3,
}

struct Connection {
    alive: bool,
    node_a: usize,
    node_b: usize,

    // Topology
    rest_len: f32,
    rest_local_rot: Quat,
    local_axis: Vec3,

    // --- NEW: THE FINALS LOGIC ---
    // We accumulate these during the solver loop
    accumulated_lin_impulse: Vec3, // Fl
    accumulated_ang_impulse: Vec3, // Fa

    last_stress_ratio: f32, // 0.0 to 1.0+

    // Baseline logic (Slide 3)
    baseline_impulse: f32,
    material: MaterialProps,
}

struct Island {
    sleeping: bool,
    needs_refactor: bool,
    // We don't strictly need to keep 'a' if we have 'llt',
    // unless you plan to update values (breakage).
    a: SparseColMat<usize, f32>,
    b: Col<f32>,
    // CACHED FACTORIZATION:
    symbolic: SymbolicLlt<usize>,
    llt: Llt<usize, f32>,
}

#[derive(Resource)]
struct PhysicsWorld {
    nodes: Vec<PhysNode>,
    connections: Vec<Connection>,
    island: Option<Island>,
    frame_count: usize,
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
            frame_count: 0,
        }
    }

    fn add_node(&mut self, fixed: bool, pos: Vec3, vel: Vec3, rot: Quat, ang_vel: Vec3) -> usize {
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
            Vec3::Y
        };

        let rot_a = self.nodes[node_a].rot;
        let rot_b = self.nodes[node_b].rot;

        // 1. Initial Relative Rotation (q_a^-1 * q_b)
        let rest_local_rot = rot_a.inverse() * rot_b;

        // 2. Initial Local Axis (The "Socket" direction relative to A)
        let local_axis = rot_a.inverse() * dir_world;

        let concrete = MaterialProps {
            strength: CONCRETE_STRENGTH,

            tension_mult: 1.0,
            compression_mult: 0.1,

            // INCREASED: Make shear almost as dangerous as tension.
            // This forces the "Floor" to stick to the "Wall" during a fall.
            shear_mult: 0.6,

            bending_mult: 0.1,
        };

        self.connections.push(Connection {
            alive: true,
            node_a,
            node_b,
            rest_len,
            rest_local_rot,
            local_axis,
            accumulated_lin_impulse: Vec3::ZERO,
            accumulated_ang_impulse: Vec3::ZERO,
            last_stress_ratio: 0.0,
            baseline_impulse: 5000.0,
            material: concrete,
        });
    }

    /// Constructs the System Matrix A = M + dt^2 * K
    fn create_island(&mut self) {
        let n_nodes = self.nodes.len();
        let dof: usize = n_nodes * DOF_PER_NODE;

        let mut triplets: Vec<Triplet<usize, usize, f32>> = Vec::new();

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

        let k_rot = JOINT_ROT_STIFFNESS * dt2;

        for conn in self.connections.iter() {
            let node_a = conn.node_a;
            let node_b = conn.node_b;

            let diff = self.nodes[node_b].pos - self.nodes[node_a].pos;
            let len_sq = diff.length_squared();
            if len_sq < 1e-9 {
                continue;
            } // Protect against zero-length

            // --- A. LINEAR BLOCK (Isotropic "Welded" Joint) ---
            // Instead of calculating directions, we assume the joint resists
            // movement in X, Y, and Z equally. This prevents the "Shear" drift
            // completely and is cheaper to compute.

            for r in 0..3 {
                // Diagonal only (Identity Matrix * Stiffness)
                let val = JOINT_LIN_STIFFNESS * dt2;

                let row_a = node_a * 6 + r;
                let col_a = node_a * 6 + r; // Note: col = row (Diagonal)
                let row_b = node_b * 6 + r;
                let col_b = node_b * 6 + r;

                // Self-interaction (+K)
                triplets.push(Triplet::new(row_a, col_a, val));
                triplets.push(Triplet::new(row_b, col_b, val));

                // Mutual interaction (-K)
                // We connect (row_a, col_b) and (row_b, col_a)
                // Since it's diagonal, this connects X_a to X_b, Y_a to Y_b...
                let col_b_mutual = node_b * 6 + r;
                let col_a_mutual = node_a * 6 + r;

                triplets.push(Triplet::new(row_a, col_b_mutual, -val));
                triplets.push(Triplet::new(row_b, col_a_mutual, -val));
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
        let a = SparseColMat::<usize, f32>::try_new_from_triplets(dof, dof, &triplets)
            .expect("Failed to create matrix from triplets");

        let a_rows = a.nrows();
        let b = Col::zeros(a_rows);

        // --- OPTIMIZATION START ---

        // 1. Symbolic Analysis (Pattern)
        let symbolic = SymbolicLlt::try_new(a.symbolic(), faer::Side::Lower)
            .expect("Symbolic factorization failed");

        // 2. Numeric Factorization (Values) -> HEAVY OPERATION
        // We do this ONCE.
        let llt = Llt::try_new_with_symbolic(symbolic.clone(), a.as_ref(), faer::Side::Lower)
            .expect("Matrix is indefinite (System is unstable or floating)");

        // --- OPTIMIZATION END ---

        self.island = Some(Island {
            sleeping: false,
            needs_refactor: false,
            a,
            b,
            symbolic,
            llt, // Store the calculated factors
        });
    }

    fn update_stiffness_matrix(&mut self) {
        let Some(island) = self.island.as_mut() else {
            return;
        };

        let n_nodes = self.nodes.len();
        let dof: usize = n_nodes * DOF_PER_NODE;

        let mut triplets: Vec<Triplet<usize, usize, f32>> = Vec::new();

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

        let k_rot = JOINT_ROT_STIFFNESS * dt2;

        for conn in self.connections.iter() {
            if !conn.alive {
                continue;
            }

            let node_a = conn.node_a;
            let node_b = conn.node_b;

            let diff = self.nodes[node_b].pos - self.nodes[node_a].pos;
            let len_sq = diff.length_squared();
            if len_sq < 1e-9 {
                continue;
            } // Protect against zero-length

            // --- A. LINEAR BLOCK (Isotropic "Welded" Joint) ---
            // Instead of calculating directions, we assume the joint resists
            // movement in X, Y, and Z equally. This prevents the "Shear" drift
            // completely and is cheaper to compute.

            for r in 0..3 {
                // Diagonal only (Identity Matrix * Stiffness)
                let val = JOINT_LIN_STIFFNESS * dt2;

                let row_a = node_a * 6 + r;
                let col_a = node_a * 6 + r; // Note: col = row (Diagonal)
                let row_b = node_b * 6 + r;
                let col_b = node_b * 6 + r;

                // Self-interaction (+K)
                triplets.push(Triplet::new(row_a, col_a, val));
                triplets.push(Triplet::new(row_b, col_b, val));

                // Mutual interaction (-K)
                // We connect (row_a, col_b) and (row_b, col_a)
                // Since it's diagonal, this connects X_a to X_b, Y_a to Y_b...
                let col_b_mutual = node_b * 6 + r;
                let col_a_mutual = node_a * 6 + r;

                triplets.push(Triplet::new(row_a, col_b_mutual, -val));
                triplets.push(Triplet::new(row_b, col_a_mutual, -val));
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
        island.a = SparseColMat::<usize, f32>::try_new_from_triplets(dof, dof, &triplets)
            .expect("Failed to create matrix from triplets");
    }

    fn set_island_needs_refactor(&mut self, needs_refactor: bool) {
        if let Some(island) = &mut self.island {
            island.needs_refactor = needs_refactor;
        }
    }

    fn check_breakage(&mut self) {
        let Some(island) = self.island.as_mut() else {
            return;
        };

        let is_calibrating = self.frame_count < 120; // First 2 seconds are warmup

        for conn in self.connections.iter_mut() {
            if !conn.alive {
                continue;
            }

            // --- 1. GATHER DATA ---
            // Fl (Linear Force/Impulse)
            let fl = conn.accumulated_lin_impulse;
            // Fa (Angular Force/Torque)
            let fa = conn.accumulated_ang_impulse;

            let p_a = self.nodes[conn.node_a].pos;
            let p_b = self.nodes[conn.node_b].pos;

            // Normal(A, B)
            let dir = (p_b - p_a).normalize_or_zero();

            // --- 2. DECOMPOSE (Slide 2) ---

            // A. Tension/Compression
            // Project Fl onto the Normal
            let normal_magnitude = fl.dot(dir);

            // Determine if Tension or Compression
            // If the spring is pulling A towards B (Tension), Force vector points TO B.
            // Direction vector is A->B.
            // So if dot > 0, Force matches Direction => Tension.
            let is_tension = normal_magnitude > 0.0;

            let term_normal = if is_tension {
                normal_magnitude.abs() * conn.material.tension_mult
            } else {
                normal_magnitude.abs() * conn.material.compression_mult
            };

            // B. Shearing
            // Shear = Fl - (Normal * Dot)
            // It's the remaining force vector perpendicular to the beam
            let shear_vec = fl - (dir * normal_magnitude);
            let term_shear = shear_vec.length() * conn.material.shear_mult;

            // C. Bending
            // Fa is the torque applied. Its magnitude represents bending stress.
            let term_bending = fa.length() * conn.material.bending_mult;

            // --- 3. AGGREGATE ---
            let total_stress_impulse = term_normal + term_shear + term_bending;

            // --- 4. THE BASELINE (Slide 3) ---
            if is_calibrating {
                // During warmup, we learn what "normal" stress looks like (Gravity)
                // Use a peak-hold or simple max to capture the static load
                if total_stress_impulse > conn.baseline_impulse {
                    conn.baseline_impulse = total_stress_impulse;
                }
            } else {
                // Determine Threshold
                // BreakOffset is the "HP" of the connection
                let limit = conn.baseline_impulse + conn.material.strength;

                // Store ratio for rendering
                // If stress is baseline (gravity), ratio is 0.
                // If stress is baseline + strength, ratio is 1 (Break).
                let safe_zone = total_stress_impulse - conn.baseline_impulse;
                conn.last_stress_ratio = (safe_zone / conn.material.strength).clamp(0.0, 2.0);

                if total_stress_impulse > limit {
                    conn.alive = false;
                    println!(
                        "BREAK! Stress: {:.2} > Limit: {:.2}",
                        total_stress_impulse, limit
                    );
                    // Trigger island refactor
                    island.needs_refactor = true;
                }
            }
        }
    }

    /// Solves the linear system to integrate velocities
    fn solve_for_x(&mut self) {
        let Some(island) = self.island.as_mut() else {
            return;
        };

        self.frame_count += 1;

        if island.sleeping {
            return;
        }

        if island.needs_refactor {
            // Note: In production, handle the error gracefully (e.g. break the island apart)
            if let Ok(llt) = Llt::try_new_with_symbolic(
                island.symbolic.clone(),
                island.a.as_ref(),
                faer::Side::Lower,
            ) {
                island.llt = llt;
            } else {
                warn!("Matrix Factorization Failed! Resetting velocities.");
                for node in &mut self.nodes {
                    node.vel = Vec3::ZERO;
                }
            }
            island.needs_refactor = false;
        }

        let n_nodes = self.nodes.len();
        island.b.fill(0.0);

        // 1. PREDICTION
        for (i, node) in self.nodes.iter_mut().enumerate() {
            if node.fixed {
                continue;
            }

            node.vel *= LIN_DAMPING;
            node.ang_vel *= ANG_DAMPING;

            let offset = i * DOF_PER_NODE;
            let vel_pred = node.vel + GRAVITY * DT;
            let momentum = vel_pred * NODE_MASS;

            island.b[offset + 0] = momentum.x;
            island.b[offset + 1] = momentum.y;
            island.b[offset + 2] = momentum.z;

            let ang_momentum = node.ang_vel * NODE_MOMENT_OF_INERTIA;
            island.b[offset + 3] = ang_momentum.x;
            island.b[offset + 4] = ang_momentum.y;
            island.b[offset + 5] = ang_momentum.z;
        }

        // 2. RESTORING FORCES
        for conn in self.connections.iter_mut() {
            if !conn.alive {
                continue;
            }

            let p_a = self.nodes[conn.node_a].pos;
            let p_b = self.nodes[conn.node_b].pos;
            let v_a = self.nodes[conn.node_a].vel;
            let v_b = self.nodes[conn.node_b].vel;
            let w_a = self.nodes[conn.node_a].ang_vel;
            let w_b = self.nodes[conn.node_b].ang_vel;

            // --- A. LINEAR ---
            let q_a = self.nodes[conn.node_a].rot;
            let target_offset_world = q_a * conn.local_axis * conn.rest_len;
            let target_pos_b = p_a + target_offset_world;
            let pos_error = p_b - target_pos_b;

            // Calculate Linear Impulse
            let restoring_force = pos_error * JOINT_LIN_STIFFNESS;
            let lin_impulse = restoring_force * DT;
            // CAPTURE FOR ANALYSIS
            // We store the magnitude/vector applied to Node A
            conn.accumulated_lin_impulse = lin_impulse;

            // Apply stiffness only if error is significant
            if pos_error.length_squared() > 1e-12 {
                let spring_force = pos_error * JOINT_LIN_STIFFNESS;

                // Damping (Relative Velocity)
                let rel_vel = v_b - v_a;
                let damping_force = rel_vel * INTERNAL_LIN_DAMPING;

                let impulse = (spring_force + damping_force) * DT;

                let idx_a = conn.node_a * DOF_PER_NODE;
                let idx_b = conn.node_b * DOF_PER_NODE;

                island.b[idx_a + 0] += impulse.x;
                island.b[idx_a + 1] += impulse.y;
                island.b[idx_a + 2] += impulse.z;

                island.b[idx_b + 0] -= impulse.x;
                island.b[idx_b + 1] -= impulse.y;
                island.b[idx_b + 2] -= impulse.z;
            }

            // --- B. ANGULAR ---
            let q_b = self.nodes[conn.node_b].rot;
            let q_rel_curr = q_a.inverse() * q_b;
            let q_error = q_rel_curr * conn.rest_local_rot.inverse();

            let (axis, angle_sin) = q_error.to_axis_angle();

            // SAFETY: Clamp angle calculation to avoid NaN
            if angle_sin.abs() > 1e-6 {
                // For Quats, to_axis_angle returns the angle in [0, 2*PI].
                // We want the shortest path, so if > PI, invert axis and take 2PI - angle.
                // However, Bevy's impl usually handles this.
                // Just use the angle directly for torque.

                let torque_mag = JOINT_ROT_STIFFNESS * angle_sin;
                let torque_world = q_a * axis * torque_mag;

                // CAPTURE FOR ANALYSIS
                let ang_impulse = torque_world * DT;
                conn.accumulated_ang_impulse = ang_impulse;

                // Angular Damping
                let rel_w = w_b - w_a;
                let damping_torque = rel_w * INTERNAL_ROT_DAMPING;

                let ang_impulse = (torque_world + damping_torque) * DT;

                let idx_a = conn.node_a * DOF_PER_NODE;
                let idx_b = conn.node_b * DOF_PER_NODE;

                island.b[idx_a + 3] += ang_impulse.x;
                island.b[idx_a + 4] += ang_impulse.y;
                island.b[idx_a + 5] += ang_impulse.z;

                island.b[idx_b + 3] -= ang_impulse.x;
                island.b[idx_b + 4] -= ang_impulse.y;
                island.b[idx_b + 5] -= ang_impulse.z;
            }
        }

        // 3. SOLVE
        let x = island.llt.solve(&island.b);

        // 4. INTEGRATE
        for i in 0..n_nodes {
            if self.nodes[i].fixed {
                continue;
            }

            let offset = i * DOF_PER_NODE;

            let mut new_vel = Vec3::new(x[offset], x[offset + 1], x[offset + 2]);
            let mut new_ang_vel = Vec3::new(x[offset + 3], x[offset + 4], x[offset + 5]);

            // SAFETY: Cap maximum velocity to prevent explosion spirals
            // If the physics explodes, this stops it from turning into NaN
            let max_speed = 100.0;
            if new_vel.length_squared() > max_speed * max_speed {
                new_vel = new_vel.normalize() * max_speed;
            }
            if new_ang_vel.length_squared() > max_speed * max_speed {
                new_ang_vel = new_ang_vel.normalize() * max_speed;
            }

            self.nodes[i].vel = new_vel;
            self.nodes[i].ang_vel = new_ang_vel;

            self.nodes[i].pos += new_vel * DT;

            let q = self.nodes[i].rot;
            let dq =
                Quat::from_xyzw(new_ang_vel.x, new_ang_vel.y, new_ang_vel.z, 0.0) * q * (0.5 * DT);
            self.nodes[i].rot = (q + dq).normalize();
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
            //FrameTimeDiagnosticsPlugin::default(), // floods the terminal output
            CameraControllerPlugin,
        ))
        .insert_resource(PhysicsWorld::new())
        .add_systems(Startup, (print_faer_info, setup_tshape, setup))
        .add_systems(FixedUpdate, solve_system)
        .add_systems(Update, (render, input_force, camera_movement))
        .insert_resource(Time::<Fixed>::from_seconds(DT as f64))
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

const SKYSCRAPER_WIDTH_NODES: usize = 6 * 2;
const SKYSCRAPER_DEPTH_NODES: usize = 6;
const SKYSCRAPER_HEIGHT_NODES: usize = 28 * 2;

#[allow(dead_code)]
fn setup_sqare_tower(mut sim: ResMut<PhysicsWorld>) {
    let w = SKYSCRAPER_WIDTH_NODES;
    let d = SKYSCRAPER_DEPTH_NODES;
    let h = SKYSCRAPER_HEIGHT_NODES;

    info!("Building a 4 neigbor structure with {} nodes...", w * d * h);

    let s = 4.0; // Spacing between nodes

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

/// This example should test rotational stiffness of supporting connections
#[allow(dead_code)]
fn setup_wall_with_ceiling(mut sim: ResMut<PhysicsWorld>) {}

#[allow(dead_code)]
fn setup_tshape(mut sim: ResMut<PhysicsWorld>) {
    // --- SETUP T-SHAPE STRUCTURE ---
    // Using a T-shape is the ultimate test for Rotational/Shear stiffness.
    // If rotation is broken, the cross-bar will flop down.

    // Node 0: Base (Fixed)
    let n0 = sim.add_node(
        true,
        vec3(0.0, 0.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );

    // Node 1: Top of Pillar
    let n1 = sim.add_node(
        false,
        vec3(0.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );

    // Node 2: Left Arm
    let n2 = sim.add_node(
        false,
        vec3(-4.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );

    // Node 3: Right Arm
    let n3 = sim.add_node(
        false,
        vec3(4.0, 6.0, 0.0),
        Vec3::ZERO,
        Quat::IDENTITY,
        Vec3::ZERO,
    );

    // Connections
    sim.add_conn(n0, n1); // Pillar
    sim.add_conn(n1, n2); // Left Arm
    sim.add_conn(n1, n3); // Right Arm

    let mut prev = n3;
    for i in 2..300 {
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

    let mut prev = n2;
    for i in 2..200 {
        let new = sim.add_node(
            false,
            vec3(-i as f32 * 4.0, 6.0, 0.0),
            Vec3::ZERO,
            Quat::IDENTITY,
            Vec3::ZERO,
        );
        sim.add_conn(prev, new);
        prev = new;
    }

    let new_start_node = 10;
    prev = 10;
    for j in 1..6 {
        let new = sim.add_node(
            false,
            vec3((new_start_node - 2) as f32 * 4.0, 6.0, j as f32 * 4.0),
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
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(20.0, 15.0, 30.0).looking_at(Vec3::new(5.0, 5.0, 0.0), Vec3::Y),
        CameraMoveSpeed(15.0),
        CameraMoveSpeedMult(5.0),
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

    commands.spawn((
        Text::new("6-DOF Frame Solver\nStructure should remain rigid.\nClick to apply impulse."),
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

    sim.check_breakage();

    if sim.island.as_ref().unwrap().needs_refactor {
        sim.update_stiffness_matrix();
    }
}

fn input_force(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&GlobalTransform, With<Camera>>,
    mut sim: ResMut<PhysicsWorld>,
    mut gizmos: Gizmos,
) {
    if keys.just_pressed(KeyCode::Space) {
        let w = SKYSCRAPER_WIDTH_NODES;
        let d = SKYSCRAPER_DEPTH_NODES;
        for i in 1..w {
            for j in 1..d {
                let idx = j + d * i;
                sim.nodes[idx].fixed = false;
            }
        }

        sim.set_island_needs_refactor(true);
    }

    let tform = *camera;

    let pole_len = tform.forward() * 8.0;

    let center = Vec3::new(
        tform.translation().x + pole_len.x,
        tform.translation().y + pole_len.y,
        tform.translation().z + pole_len.z,
    );

    gizmos.sphere(center, 0.1, Color::WHITE);

    if mouse.just_pressed(MouseButton::Left) {
        let center = vec3(
            tform.translation().x + pole_len.x,
            tform.translation().y + pole_len.y,
            tform.translation().z + pole_len.z,
        );

        let impulse = 10_000_000.0;

        for n in &mut sim.nodes {
            let diff = n.pos - center;
            let dist = diff.length();

            if dist < 100.0 {
                // 1. Linear Push
                let dist_clamped = dist.max(1.0);
                let force_mag = (impulse / NODE_MASS) / (dist_clamped * dist_clamped);
                n.vel += diff.normalize() * force_mag * DT;
            }
        }
    }
}

fn render(mut gizmos: Gizmos, sim: Res<PhysicsWorld>) {
    // Draw Connections
    for conn in &sim.connections {
        if !conn.alive {
            continue;
        }

        let p_a = sim.nodes[conn.node_a].pos;
        let p_b = sim.nodes[conn.node_b].pos;

        // Use the actual physical stress ratio calculated in check_breakage
        let ratio = conn.last_stress_ratio;

        let color = if ratio > 1.0 {
            // It broke this frame (or is about to)
            Color::WHITE
        } else {
            // Green (Safe) -> Red (Danger)
            Color::srgb(ratio, 1.0 - ratio, 0.0)
        };

        gizmos.line(p_a, p_b, color);
    }

    // Draw Nodes
    for node in &sim.nodes {
        let pos = node.pos;
        let color = if node.fixed {
            Color::WHITE
        } else {
            Color::srgb(0.0, 0.0, 1.0)
        };
        gizmos.cuboid(
            Transform::from_translation(pos).with_scale(Vec3::splat(0.5)),
            color,
        );

        // Visualize rotation axis (Local X)
        let axis = node.rot * Vec3::X;
        gizmos.line(pos, pos + axis, Color::srgb(1.0, 0.0, 0.0));
    }
}
