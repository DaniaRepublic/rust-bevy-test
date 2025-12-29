use avian3d::prelude::*;
use bevy::prelude::*;

pub struct SourceControllerPlugin;

impl Plugin for SourceControllerPlugin {
    fn build(&self, app: &mut App) {
        // Run in FixedUpdate to drive physics manually before the solver. Solver runs in
        // FixedPostUpdate
        app.add_systems(FixedUpdate, movement_pipeline);
    }
}

// --- Configuration & Components ---

#[derive(Component, Reflect)]
#[reflect(Component)]
pub struct SourceController {
    pub max_speed: f32,
    pub accelerate: f32,
    pub air_accelerate: f32,
    pub friction: f32,
    pub stop_speed: f32,
    pub gravity: f32,
    pub step_height: f32,
    pub jump_impulse: f32,
}

impl Default for SourceController {
    fn default() -> Self {
        Self {
            max_speed: 10.0,
            accelerate: 10.0,
            air_accelerate: 100.0,
            friction: 8.0,
            stop_speed: 4.0,
            gravity: 20.0,
            step_height: 0.5,
            jump_impulse: 8.0,
        }
    }
}

#[derive(Component, Default)]
pub struct CharacterInput {
    pub wish_dir: Vec3,
    pub jump: bool,
    pub duck: bool,
}

#[derive(Component, Default)]
pub struct CharacterState {
    pub is_grounded: bool,
    pub ground_normal: Vec3,
    pub velocity: Vec3,
}

// --- Pipeline ---

fn movement_pipeline(
    time: Res<Time>,
    spatial_query: SpatialQuery,
    mut query: Query<
        (
            Entity,
            &mut Transform,
            &mut CharacterState,
            &SourceController,
            &CharacterInput,
            &Collider,
        ),
        With<RigidBody>,
    >,
) {
    let dt = time.delta_secs().min(0.05); // Clamp dt for stability

    for (entity, mut transform, mut state, config, input, collider) in query.iter_mut() {
        // 1. Depenetrate
        // Pushes the character out of the floor/walls if they drifted in.
        resolve_penetration(entity, &mut transform, collider, &spatial_query);

        // 2. Ground Check
        check_ground(entity, &transform, &mut state, &spatial_query, collider);

        // 3. Physics (Gravity / Friction)
        if state.is_grounded {
            apply_friction(&mut state, config, dt);
        } else {
            state.velocity.y -= config.gravity * dt;
        }

        // 4. Input / Acceleration
        let wish_dir = input.wish_dir;
        let wish_speed = config.max_speed;

        if state.is_grounded {
            accelerate(&mut state, wish_dir, wish_speed, config.accelerate, dt);

            if input.jump {
                state.velocity.y = config.jump_impulse;
                state.is_grounded = false; // Detach immediately
            }
        } else {
            let air_cap = 2.0f32.min(wish_speed);
            accelerate(&mut state, wish_dir, air_cap, config.air_accelerate, dt);
        }

        // 5. Move & Collide
        resolve_collisions(
            entity,
            &mut transform,
            &mut state,
            collider,
            &spatial_query,
            config.step_height,
            dt,
        );

        // 6. Snapping
        // If we are moving down/flat and validly grounded, stick to the floor.
        if state.is_grounded && !input.jump && state.velocity.y <= 0.0 {
            try_snap_to_ground(
                entity,
                &mut transform,
                &mut state,
                collider,
                &spatial_query,
                config.step_height,
            );
        }
    }
}

// --- Algorithms ---

/// Pushes the character out of geometry if they are intersecting.
fn resolve_penetration(
    entity: Entity,
    transform: &mut Transform,
    collider: &Collider,
    spatial_query: &SpatialQuery,
) {
    let filter = SpatialQueryFilter::from_excluded_entities([entity]);

    // We want to detect if we are INSIDE.
    // We use a shape cast with distance 0.0 and compute_contact_on_penetration = true.
    // This allows detecting overlap.
    let mut config = ShapeCastConfig::from_max_distance(0.0);
    config.compute_contact_on_penetration = true;
    config.ignore_origin_penetration = false;

    // Iterative depenetration (up to 3 bumps)
    for _ in 0..3 {
        // Use shape_hits to find multiple contacts (floor + wall corner)
        // Correct signature: shape, origin, rotation, dir, max_hits, config, filter
        let hits = spatial_query.shape_hits(
            collider,
            transform.translation,
            transform.rotation,
            Dir3::Y, // Direction is dummy here since distance is 0.0
            4,       // Max hits to resolve
            &config,
            &filter,
        );

        if hits.is_empty() {
            break;
        }

        let mut moved = false;

        for hit in hits {
            // If distance is <= 0, we are penetrating or touching.
            if hit.distance <= 0.0 {
                // Nudge along the contact normal.
                // Since we don't have exact penetration depth from shape_hits in all backends,
                // we use a small fixed epsilon step to "walk" out.
                let nudge = hit.normal1 * 0.005;

                if nudge.length_squared() > 1e-6 {
                    transform.translation += nudge;
                    moved = true;
                }
            }
        }

        if !moved {
            break;
        }
    }
}

fn check_ground(
    entity: Entity,
    transform: &Transform,
    state: &mut CharacterState,
    spatial_query: &SpatialQuery,
    collider: &Collider,
) {
    // Cast slightly down to find ground
    let cast_dist = 0.05;
    let filter = SpatialQueryFilter::from_excluded_entities([entity]);

    // We allow origin penetration here so we don't lose ground contact
    // just because we are slightly embedded (which `resolve_penetration` handles).
    let mut config = ShapeCastConfig::from_max_distance(cast_dist);
    config.ignore_origin_penetration = true;

    if let Some(hit) = spatial_query.cast_shape(
        collider,
        transform.translation,
        transform.rotation,
        Dir3::NEG_Y,
        &config,
        &filter,
    ) {
        // Valid ground if normal is generally upwards
        if hit.normal1.y > 0.7 {
            state.is_grounded = true;
            state.ground_normal = hit.normal1;
            return;
        }
    }

    state.is_grounded = false;
    state.ground_normal = Vec3::Y;
}

fn apply_friction(state: &mut CharacterState, config: &SourceController, dt: f32) {
    let speed = state.velocity.length();
    if speed < 0.001 {
        state.velocity = Vec3::ZERO;
        return;
    }
    let control = if speed < config.stop_speed {
        config.stop_speed
    } else {
        speed
    };
    let drop = control * config.friction * dt;
    let new_speed = (speed - drop).max(0.0);
    if speed > 0.0 {
        state.velocity *= new_speed / speed;
    }
}

fn accelerate(state: &mut CharacterState, wish_dir: Vec3, wish_speed: f32, accel: f32, dt: f32) {
    let current_speed = state.velocity.dot(wish_dir);
    let add_speed = wish_speed - current_speed;
    if add_speed <= 0.0 {
        return;
    }

    let accel_speed = (accel * wish_speed * dt).min(add_speed);
    state.velocity += wish_dir * accel_speed;
}

fn resolve_collisions(
    entity: Entity,
    transform: &mut Transform,
    state: &mut CharacterState,
    collider: &Collider,
    spatial_query: &SpatialQuery,
    step_height: f32,
    dt: f32,
) {
    let original_pos = transform.translation;
    let original_vel = state.velocity;

    // 1. Try normal move
    let (pos_slide, vel_slide, hit_slide) = collide_and_slide(
        entity,
        original_pos,
        state.velocity,
        collider,
        spatial_query,
        dt,
    );

    if !hit_slide || !state.is_grounded {
        transform.translation = pos_slide;
        state.velocity = vel_slide;
        return;
    }

    // 2. Step Logic (Source Engine Style)
    let filter = SpatialQueryFilter::from_excluded_entities([entity]);

    // A. Move UP
    let mut step_up_config = ShapeCastConfig::from_max_distance(step_height);
    step_up_config.ignore_origin_penetration = true;

    // Check ceiling
    let up_dist = if let Some(hit) = spatial_query.cast_shape(
        collider,
        original_pos,
        transform.rotation,
        Dir3::Y,
        &step_up_config,
        &filter,
    ) {
        hit.distance
    } else {
        step_height
    };

    let step_up_pos = original_pos + Vec3::Y * up_dist;

    // B. Move Forward (at new height, using original velocity)
    let (pos_step_fwd, vel_step_fwd, _) = collide_and_slide(
        entity,
        step_up_pos,
        original_vel,
        collider,
        spatial_query,
        dt,
    );

    // C. Move Down
    let mut step_down_config = ShapeCastConfig::from_max_distance(up_dist * 2.0);
    step_down_config.ignore_origin_penetration = true;

    let mut final_step_pos = pos_step_fwd;
    let mut floor_found = false;

    if let Some(hit) = spatial_query.cast_shape(
        collider,
        pos_step_fwd,
        transform.rotation,
        Dir3::NEG_Y,
        &step_down_config,
        &filter,
    ) {
        final_step_pos = pos_step_fwd - Vec3::Y * hit.distance;
        floor_found = true;
    }

    // D. Compare Results
    let dist_slide = (pos_slide - original_pos).with_y(0.0).length_squared();
    let dist_step = (final_step_pos - original_pos).with_y(0.0).length_squared();

    // Only commit step if we found floor and went further
    if floor_found && dist_step > dist_slide {
        transform.translation = final_step_pos;
        state.velocity = vel_step_fwd;
    } else {
        transform.translation = pos_slide;
        state.velocity = vel_slide;
    }
}

fn try_snap_to_ground(
    entity: Entity,
    transform: &mut Transform,
    state: &mut CharacterState,
    collider: &Collider,
    spatial_query: &SpatialQuery,
    step_height: f32,
) {
    let filter = SpatialQueryFilter::from_excluded_entities([entity]);
    let mut config = ShapeCastConfig::from_max_distance(step_height);
    config.ignore_origin_penetration = true;

    if let Some(hit) = spatial_query.cast_shape(
        collider,
        transform.translation,
        transform.rotation,
        Dir3::NEG_Y,
        &config,
        &filter,
    ) {
        if hit.normal1.y > 0.7 {
            transform.translation -= Vec3::Y * hit.distance;
            state.is_grounded = true;
            state.ground_normal = hit.normal1;

            // Project velocity to avoid launching off slopes
            let dot = state.velocity.dot(hit.normal1);
            if dot > 0.0 {
                state.velocity -= hit.normal1 * dot;
            }
        }
    }
}

fn collide_and_slide(
    entity: Entity,
    start_pos: Vec3,
    mut velocity: Vec3,
    collider: &Collider,
    spatial_query: &SpatialQuery,
    dt: f32,
) -> (Vec3, Vec3, bool) {
    let mut current_pos = start_pos;
    let mut time_left = dt;
    let mut hit_something = false;
    let mut planes = Vec::<Vec3>::with_capacity(5);

    let filter = SpatialQueryFilter::from_excluded_entities([entity]);

    for _ in 0..4 {
        if time_left <= 0.0 || velocity.length_squared() < 0.001 {
            break;
        }

        let move_dist = velocity.length() * time_left;
        let dir = match Dir3::new(velocity) {
            Ok(d) => d,
            Err(_) => break,
        };

        let mut config = ShapeCastConfig::from_max_distance(move_dist);
        config.ignore_origin_penetration = true;

        let hit =
            spatial_query.cast_shape(collider, current_pos, Quat::IDENTITY, dir, &config, &filter);

        match hit {
            None => {
                current_pos += velocity * time_left;
                time_left = 0.0;
            }
            Some(hit_data) => {
                hit_something = true;
                let fraction = hit_data.distance / move_dist;

                let epsilon = 0.002;
                let safe_dist = (hit_data.distance - epsilon).max(0.0);

                current_pos += dir.as_vec3() * safe_dist;

                // Important: Nudge away from wall to prevent re-collision
                current_pos += hit_data.normal1 * 0.001;

                time_left -= time_left * fraction;

                // Unique plane check
                if !planes.iter().any(|p| p.dot(hit_data.normal1) > 0.99) {
                    planes.push(hit_data.normal1);
                }

                // Multi-Plane Resolution
                let mut new_vel = velocity;
                let mut blocked = false;

                for _ in 0..planes.len() {
                    blocked = false;
                    for plane in &planes {
                        if new_vel.dot(*plane) < 0.0 {
                            new_vel -= *plane * new_vel.dot(*plane);
                            blocked = true;
                        }
                    }
                    if !blocked {
                        break;
                    }
                }

                if blocked && planes.len() >= 2 {
                    let crease = planes[0].cross(planes[1]);
                    if crease.length_squared() > 0.0 {
                        let c_dir = crease.normalize();
                        new_vel = c_dir * new_vel.dot(c_dir);
                    } else {
                        new_vel = Vec3::ZERO;
                    }
                }

                velocity = new_vel;
            }
        }
    }

    (current_pos, velocity, hit_something)
}
