//! A freecam-style camera controller plugin.
//!
//! **This implementation only controls rotation, it assumes physics character controller controls
//! movement.**
//!
//! To use in your own application:
//! - Copy the code for the [`CameraControllerPlugin`] and add the plugin to your App.
//! - Attach the [`CameraController`] component to an entity with a [`Camera3d`].

use bevy::{
    input::mouse::AccumulatedMouseMotion,
    prelude::*,
    window::{CursorGrabMode, CursorOptions},
};
use std::{f32::consts::*, fmt};

/// A freecam-style camera controller plugin.
pub struct CameraControllerPlugin;

impl Plugin for CameraControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, run_camera_controller);
    }
}

pub const RADIANS_PER_DOT: f32 = 0.00025; // feels good

/// Camera controller [`Component`].
#[derive(Component)]
pub struct CameraController {
    /// Enables this [`CameraController`] when `true`.
    pub enabled: bool,
    /// Indicates if this controller has been initialized by the [`CameraControllerPlugin`].
    pub initialized: bool,
    /// Multiplier for pitch and yaw rotation speed.
    pub sensitivity: f32,
    /// [`MouseButton`] for grabbing the mouse focus.
    pub mouse_key_cursor_grab: MouseButton,
    /// [`KeyCode`] for grabbing the keyboard focus.
    pub keyboard_key_toggle_cursor_grab: KeyCode,
    /// This [`CameraController`]'s pitch rotation.
    pub pitch: f32,
    /// This [`CameraController`]'s yaw rotation.
    pub yaw: f32,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            enabled: true,
            initialized: false,
            sensitivity: 1.0,
            mouse_key_cursor_grab: MouseButton::Left,
            keyboard_key_toggle_cursor_grab: KeyCode::KeyM,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

impl CameraController {
    pub fn with_sensitivity(mut self: Self, sensitivity: f32) -> Self {
        self.sensitivity = sensitivity;
        self
    }
}

impl fmt::Display for CameraController {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "
Freecam Controls:
    Mouse\t- Move camera orientation
    {:?}\t- Hold to grab cursor
    {:?}\t- Toggle cursor grab",
            self.mouse_key_cursor_grab, self.keyboard_key_toggle_cursor_grab,
        )
    }
}

// limit pitch to 10 degrees away from y-axis
pub const MAX_PITCH: f32 = PI / 2.0 - 5.0 / 180.0 * PI;

pub fn run_camera_controller(
    time: Res<Time>,
    mut windows: Query<(&Window, &mut CursorOptions)>,
    accumulated_mouse_motion: Res<AccumulatedMouseMotion>,
    mouse_button_input: Res<ButtonInput<MouseButton>>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut toggle_cursor_grab: Local<bool>,
    mut mouse_cursor_grab: Local<bool>,
    query: Single<(&mut Transform, &mut CameraController), With<Camera3d>>,
) {
    let (mut transform, mut controller) = query.into_inner();

    if !controller.initialized {
        let (yaw, pitch, _roll) = transform.rotation.to_euler(EulerRot::YXZ);
        controller.yaw = yaw;
        controller.pitch = pitch.clamp(-MAX_PITCH, MAX_PITCH);
        controller.initialized = true;
        info!("{}", *controller);
    }

    if !controller.enabled {
        return;
    }

    // === Cursor grab handling ===
    let mut cursor_grab_change = false;

    if key_input.just_pressed(controller.keyboard_key_toggle_cursor_grab) {
        *toggle_cursor_grab = !*toggle_cursor_grab;
        cursor_grab_change = true;
    }
    if mouse_button_input.just_pressed(controller.mouse_key_cursor_grab) {
        *mouse_cursor_grab = true;
        cursor_grab_change = true;
    }
    if mouse_button_input.just_released(controller.mouse_key_cursor_grab) {
        *mouse_cursor_grab = false;
        cursor_grab_change = true;
    }

    let cursor_grabbed = *toggle_cursor_grab || *mouse_cursor_grab;

    if cursor_grab_change {
        for (window, mut cursor_options) in windows.iter_mut() {
            if !window.focused {
                continue;
            }
            if cursor_grabbed {
                cursor_options.grab_mode = CursorGrabMode::Locked;
                cursor_options.visible = false;
            } else {
                cursor_options.grab_mode = CursorGrabMode::None;
                cursor_options.visible = true;
            }
        }
    }

    let delta_secs = time.delta_secs();

    if !cursor_grabbed {
        // Smooth interpolation
        let t = 1.0 - (-32.0 * delta_secs).exp();
        transform.rotation = transform.rotation.slerp(
            Quat::from_euler(EulerRot::YXZ, controller.yaw, controller.pitch, 0.0),
            t,
        );
        return;
    }

    let delta = accumulated_mouse_motion.delta;

    if delta != Vec2::ZERO {
        let sensitivity = controller.sensitivity * RADIANS_PER_DOT;

        // Apply clamped deltas directly (small per-frame, no wrap issues)
        let pitch_delta = delta.y * sensitivity;
        let yaw_delta = delta.x * sensitivity;

        controller.pitch = (controller.pitch - pitch_delta).clamp(-MAX_PITCH, MAX_PITCH);
        controller.yaw = controller.yaw - yaw_delta;
    }

    // Smooth interpolation
    let t = 1.0 - (-32.0 * delta_secs).exp();
    transform.rotation = transform.rotation.slerp(
        Quat::from_euler(EulerRot::YXZ, controller.yaw, controller.pitch, 0.0),
        t,
    );
}
