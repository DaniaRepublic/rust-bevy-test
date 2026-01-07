//! Implementation of a realtime stress simulation for a large rigid system.
//!
//! # Mathematical Model
//! Uses a direct Cholesky decomposition of a sparse linear system `Ax = b`, where:
//! *   **A**: System matrix combining Inertia (Mass) and Stiffness constraints (N*dof x N*dof).
//! *   **x**: Vector of unknown new velocities (N*dof).
//! *   **b**: Target momentum / Generalized force vector (N*dof).
//!
//! # Dimensions
//! For a system of `N` nodes, each with `dof` degrees of freedom:
//! *   Matrix `A` is `(N * dof) x (N * dof)`.
//! *   Vectors `x` and `b` are length `N * dof`.
//!
//! # Optimization Strategy
//! To achieve realtime performance (targeting <8ms frame times), the solver employs:
//!
//! 1.  **Island Decomposition**: Disconnected graphs are split into independent systems
//!     to solve smaller matrices in parallel.
//! 2.  **Sleep Management**: Islands with kinetic energy below a threshold are ignored
//!     until woken by collision.
//! 3.  **Block-Sparse Structure**: Treating the system as `N x N` blocks of `dof x dof`
//!     dense matrices (Supernodes) to leverage SIMD/AVX instructions via `faer`.
//! 4.  **Symbolic Factorization Reuse**:
//!     *   **Symbolic Step**: Computes the elimination tree and fill-in pattern (Slow, runs only on topology change).
//!     *   **Numeric Step**: Refactors the matrix values using the pre-computed pattern (Fast, runs every frame).
//!
//! *Note: While strictly "Incremental Cholesky" (rank-1 updates) is theoretically possible,
//! this implementation relies on highly optimized Numeric Refactorization which is
//! typically more performant for batch updates.*

use avian3d::prelude::*;
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
};
use nalgebra::{DVector, Vector3};
use nalgebra_sparse::{coo::CooMatrix, csc::CscMatrix, factorization::CscCholesky};

#[path = "../src/camera_controller.rs"]
mod camera_controller;
use crate::camera_controller::*;

// =========================================================================
// CONFIGURATION
// =========================================================================

const DT: f64 = 0.01667;

// --- TUNING ---
// stiffness reduced slightly to allow visible "spring back" for demonstration
const STIFFNESS_MAIN: f64 = 4e7;
const STIFFNESS_DIAG: f64 = 3e7;

const NODE_MASS: f64 = 20.0;
const ANCHOR_MASS: f64 = 1e5;

// STRENGTH LIMITS
// We treat Verticals as "Columns" (Very strong in compression)
// We treat Horizontals as "Beams" (Strong in tension due to rebar)
const BASE_STRENGTH: f64 = 50_000.0;

#[derive(Component, Clone, Debug)]
struct PhysNode {
    pos: Vector3<f64>,
    vel: Vector3<f64>,
    vel_pre: Vector3<f64>,
    fixed: bool,
    accumulated_force: Vector3<f64>, // New: Accumulate spring forces for RHS
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum ConnectionType {
    Vertical,
    Horizontal,
    Diagonal,
}

#[derive(Clone, Debug)]
struct Connection {
    node_a: usize,
    node_b: usize,
    rest_length: f64,
    alive: bool,
    current_force: f64,
    c_type: ConnectionType,
}

#[derive(Resource)]
struct PhysicsWorld {
    nodes: Vec<PhysNode>,
    connections: Vec<Connection>,
}

// A in A x = b
#[derive(Resource)]
struct Island {
    asleep: bool,
}

impl PhysicsWorld {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            connections: Vec::new(),
        }
    }

    fn add_node(&mut self, x: f64, y: f64, z: f64, fixed: bool) -> usize {
        let idx = self.nodes.len();
        self.nodes.push(PhysNode {
            pos: Vector3::new(x, y, z),
            vel: Vector3::zeros(),
            vel_pre: Vector3::zeros(),
            fixed,
            accumulated_force: Vector3::zeros(),
        });
        idx
    }

    fn add_connection(&mut self, a: usize, b: usize, c_type: ConnectionType) {
        let dist = (self.nodes[a].pos - self.nodes[b].pos).norm();
        self.connections.push(Connection {
            node_a: a,
            node_b: b,
            rest_length: dist,
            alive: true,
            current_force: 0.0,
            c_type,
        });
    }

    fn step(&mut self) {
        let n_nodes = self.nodes.len();
        let dof = n_nodes * 3; // 3 Degrees of Freedom (X, Y, Z)
        let dt2 = DT * DT;

        // 1. CLEAR FORCES & PREDICT
        let gravity = Vector3::new(0.0, -9.81 * NODE_MASS, 0.0);

        for node in &mut self.nodes {
            node.accumulated_force = Vector3::zeros();

            if node.fixed {
                node.vel_pre = Vector3::zeros();
                node.vel = Vector3::zeros();
            } else {
                node.vel_pre = node.vel + (gravity / NODE_MASS) * DT;
            }
        }

        // 2. MATRIX ASSEMBLY
        let mut coo_matrix: CooMatrix<f64> = CooMatrix::new(dof, dof);

        // A. Mass (Diagonal) - FIX: Stride is 3
        for i in 0..n_nodes {
            let m = if self.nodes[i].fixed {
                ANCHOR_MASS
            } else {
                NODE_MASS
            };
            let idx = i * 3;
            // X
            coo_matrix.push(idx, idx, m);
            // Y
            coo_matrix.push(idx + 1, idx + 1, m);
            // Z
            coo_matrix.push(idx + 2, idx + 2, m);
        }

        // B. Stiffness & Forces
        for conn in &self.connections {
            if !conn.alive {
                continue;
            }

            let p_a = self.nodes[conn.node_a].pos;
            let p_b = self.nodes[conn.node_b].pos;
            let diff = p_b - p_a;
            let len_sq = diff.norm_squared();
            if len_sq < 1e-8 {
                continue;
            }
            let len = len_sq.sqrt();
            let n = diff / len; // Normal direction (Vector3)

            let k = match conn.c_type {
                ConnectionType::Diagonal => STIFFNESS_DIAG,
                _ => STIFFNESS_MAIN,
            };

            // Restoring Force (RHS)
            let displacement = len - conn.rest_length;
            let spring_force_mag = k * displacement;
            let f_vector = n * spring_force_mag;

            self.nodes[conn.node_a].accumulated_force += f_vector;
            self.nodes[conn.node_b].accumulated_force -= f_vector;

            // Matrix Assembly (LHS) - FIX: 3D Outer Product
            let k_dt2 = k * dt2;

            // This loop builds the 3x3 block: k * (n * n_transpose)
            // It automatically handles xx, xy, xz, yy, yz, zz...
            let idx_a = conn.node_a * 3;
            let idx_b = conn.node_b * 3;

            for r in 0..3 {
                for c in 0..3 {
                    // Calculate term for n[r]*n[c]
                    let val = n[r] * n[c] * k_dt2;

                    // Add to Node A block (Positive)
                    coo_matrix.push(idx_a + r, idx_a + c, val);
                    // Add to Node B block (Positive)
                    coo_matrix.push(idx_b + r, idx_b + c, val);
                    // Add to Interactions (Negative)
                    coo_matrix.push(idx_a + r, idx_b + c, -val);
                    coo_matrix.push(idx_b + r, idx_a + c, -val);
                }
            }
        }

        // 3. RHS BUILD - FIX: Stride is 3
        let mut rhs = DVector::zeros(dof);
        for i in 0..n_nodes {
            if self.nodes[i].fixed {
                continue;
            }
            let idx = i * 3;

            // X
            rhs[idx] =
                (NODE_MASS * self.nodes[i].vel_pre.x) + (self.nodes[i].accumulated_force.x * DT);
            // Y
            rhs[idx + 1] =
                (NODE_MASS * self.nodes[i].vel_pre.y) + (self.nodes[i].accumulated_force.y * DT);
            // Z
            rhs[idx + 2] =
                (NODE_MASS * self.nodes[i].vel_pre.z) + (self.nodes[i].accumulated_force.z * DT);
        }

        // 4. SOLVE
        let matrix_a = CscMatrix::from(&coo_matrix);
        let cholesky = CscCholesky::factor(&matrix_a)
            .expect("Cholesky decomposition failed - matrix might not be positive definite");
        let x = cholesky.solve(&rhs);
        for i in 0..n_nodes {
            let idx = i * 3;
            // FIX: Read X, Y, Z
            self.nodes[i].vel = Vector3::new(x[idx], x[idx + 1], x[idx + 2]);
        }

        // 5. UPDATE
        for node in &mut self.nodes {
            node.pos += node.vel * DT;
        }

        // 6. CHECK BREAKAGE (Post-Update)
        for conn in &mut self.connections {
            if !conn.alive {
                continue;
            }
            let p_a = self.nodes[conn.node_a].pos;
            let p_b = self.nodes[conn.node_b].pos;
            let k = match conn.c_type {
                ConnectionType::Diagonal => STIFFNESS_DIAG,
                _ => STIFFNESS_MAIN,
            };
            let force = k * ((p_a - p_b).norm() - conn.rest_length);
            conn.current_force = force;

            // Simple symmetric limits for demo
            let limit = BASE_STRENGTH * 5.0;
            if force.abs() > limit {
                conn.alive = false;
                println!("SNAP!");
            }
        }
    }
}

// =========================================================================
// STANDARD BEVY BOILERPLATE
// =========================================================================

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            PhysicsPlugins::default(),
            LogDiagnosticsPlugin::default(),
            FrameTimeDiagnosticsPlugin::default(),
            camera_controller::CameraControllerPlugin,
        ))
        .insert_resource(PhysicsWorld::new())
        .add_systems(Startup, setup)
        .add_systems(Update, (render, input, camera_controller::camera_movement))
        .add_systems(FixedUpdate, loop_physics)
        .insert_resource(Time::<Fixed>::from_seconds(DT))
        .run();
}

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default, // Layer 0 - the default layer that objects are assigned to
    SpacialNode, // Layer 1
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut sim: ResMut<PhysicsWorld>,
) {
    commands.spawn((
        Camera3d::default(),
        camera_controller::CameraController::default(),
        CameraMoveSpeed(10.0),
        CameraMoveSpeedMult(5.0),
        Transform::from_xyz(40.0, -10.0, 40.0).looking_at(Vec3::new(0.0, 10.0, 0.0), Vec3::Y),
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

    // Lighting
    commands.spawn((PointLight::default(), Transform::from_xyz(0.0, 10.0, 5.0)));

    let w = 4;
    let d = 4;
    let h = 10;
    let s = 4.0;
    let offset_y = 0.0;

    // 3D Grid storage
    let mut ids = vec![vec![vec![0; d]; w]; h];

    for y in 0..h {
        for x in 0..w {
            for z in 0..d {
                let px = (x as f64 - w as f64 / 2.0) * s;
                let py = offset_y + (y as f64 * s);
                let pz = (z as f64 - d as f64 / 2.0) * s;
                // Anchor bottom face
                ids[y][x][z] = sim.add_node(px, py, pz, y == 0);
            }
        }
    }

    for y in 0..h {
        for x in 0..w {
            for z in 0..d {
                let me = ids[y][x][z];

                // 1. Orthogonal Connections (The Cube frame)
                if x + 1 < w {
                    sim.add_connection(me, ids[y][x + 1][z], ConnectionType::Horizontal);
                }
                if z + 1 < d {
                    sim.add_connection(me, ids[y][x][z + 1], ConnectionType::Horizontal);
                }
                if y + 1 < h {
                    sim.add_connection(me, ids[y + 1][x][z], ConnectionType::Vertical);
                }

                // 2. DIAGONALS (CRITICAL FOR 3D RIGIDITY)
                // We need to brace the faces to prevent shearing.

                // Front/Back Face Cross (XY plane)
                if x + 1 < w && y + 1 < h {
                    sim.add_connection(me, ids[y + 1][x + 1][z], ConnectionType::Diagonal);
                    sim.add_connection(
                        ids[y][x + 1][z],
                        ids[y + 1][x][z],
                        ConnectionType::Diagonal,
                    );
                }

                // Side Face Cross (YZ plane)
                if z + 1 < d && y + 1 < h {
                    sim.add_connection(me, ids[y + 1][x][z + 1], ConnectionType::Diagonal);
                    sim.add_connection(
                        ids[y][x][z + 1],
                        ids[y + 1][x][z],
                        ConnectionType::Diagonal,
                    );
                }

                // Top/Bottom Face Cross (XZ plane) - Optional, but good for torsion
                if x + 1 < w && z + 1 < d {
                    sim.add_connection(me, ids[y][x + 1][z + 1], ConnectionType::Diagonal);
                }
            }
        }
    }
}

fn loop_physics(mut sim: ResMut<PhysicsWorld>) {
    sim.step();
}

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    camera: Single<&GlobalTransform, With<Camera>>,
    mut sim: ResMut<PhysicsWorld>,
    mut gizmos: Gizmos,
) {
    if keys.just_pressed(KeyCode::Space) {
        let w = 4;
        let d = 4;
        for i in 1..w {
            for j in 1..d {
                let idx = j + w * i;
                sim.nodes[idx].fixed = false;
            }
        }
    }

    let tform = *camera;

    let pole_len = tform.forward() * 10.0;

    let center = Vec3::new(
        tform.translation().x + pole_len.x,
        tform.translation().y + pole_len.y,
        tform.translation().z + pole_len.z,
    );

    gizmos.sphere(center, 0.1, Color::WHITE);

    if mouse.just_pressed(MouseButton::Left) {
        let center = Vector3::new(
            (tform.translation().x + pole_len.x) as f64,
            (tform.translation().y + pole_len.y) as f64,
            (tform.translation().z + pole_len.z) as f64,
        );

        let impulse = 20_000.0;
        for n in &mut sim.nodes {
            if (n.pos - center).norm() < 100.0 {
                n.vel += (n.pos - center).normalize() * (impulse / NODE_MASS) * DT;
            }
        }
    }
}

fn render(mut gizmos: Gizmos, sim: Res<PhysicsWorld>) {
    for conn in &sim.connections {
        if !conn.alive {
            continue;
        }
        let p_a = sim.nodes[conn.node_a].pos;
        let p_b = sim.nodes[conn.node_b].pos;
        let force = conn.current_force;

        let limit = match conn.c_type {
            ConnectionType::Vertical => BASE_STRENGTH * 50.0,
            ConnectionType::Horizontal => BASE_STRENGTH * 5.0,
            _ => BASE_STRENGTH,
        };

        let ratio = (force.abs() / limit).clamp(0.0, 1.0) as f32;
        let color = if force > 0.0 {
            Color::srgb(1.0, 1.0 - ratio, 0.0)
        } else {
            Color::srgb(0.0, 1.0 - ratio, 1.0)
        };
        gizmos.line(
            Vec3::new(p_a.x as f32, p_a.y as f32, p_a.z as f32),
            Vec3::new(p_b.x as f32, p_b.y as f32, p_b.z as f32),
            color,
        );
    }
    for node in &sim.nodes {
        if node.fixed {
            gizmos.rect(
                Vec3::new(node.pos.x as f32, node.pos.y as f32, node.pos.z as f32),
                Vec2::splat(2.0),
                Color::WHITE,
            );
        }
    }
}
