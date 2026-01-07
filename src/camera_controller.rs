//! A freecam-style camera controller plugin.
//! Supports mouse and gamepad
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
use std::f32::consts::*;

/// A freecam-style camera controller plugin.
pub struct CameraControllerPlugin;

impl Plugin for CameraControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_message::<RotationAction>().add_systems(
            Update,
            (
                collect_gamepad_input,
                collect_mouse_input,
                run_camera_controller,
            )
                .chain(),
        );
    }
}

pub const RADIANS_PER_DOT: f32 = 0.00025; // feels good

/// Camera controller [`Component`].
/// If using a [camera_movement] system, you can add [CameraMoveSpeed] and [CameraMoveSpeedMult] to the
/// same entity that has [CameraController] to controll the speed and speed multiplier of movement.
/// Default values for them are: CameraMoveSpeed(5.0) and CameraMoveSpeedMult(3.0).
#[derive(Component)]
pub struct CameraController {
    /// Enables this [`CameraController`] when `true`.
    pub enabled: bool,
    /// Indicates if this controller has been initialized by the [`CameraControllerPlugin`].
    pub initialized: bool,
    /// Multiplier for pitch and yaw rotation speed when using mouse.
    pub sensitivity_mouse: f32,
    /// Multiplier for pitch and yaw rotation speed when using mouse.
    pub sensitivity_gamepad: f32,
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
            sensitivity_mouse: 8.0,
            sensitivity_gamepad: 50.0,
            keyboard_key_toggle_cursor_grab: KeyCode::KeyM,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

impl CameraController {
    #[allow(dead_code)]
    pub fn with_mouse_sensitivity(mut self: Self, sensitivity: f32) -> Self {
        self.sensitivity_mouse = sensitivity;
        self
    }

    #[allow(dead_code)]
    pub fn with_gamepad_sensitivity(mut self: Self, sensitivity: f32) -> Self {
        self.sensitivity_gamepad = sensitivity;
        self
    }
}

pub enum InputDevice {
    Mouse,
    Gamepad,
}

#[derive(Message)]
pub struct RotationAction {
    delta: Vec2,
    input_device: InputDevice,
}

impl RotationAction {
    fn new(x: f32, y: f32, input_device: InputDevice) -> Self {
        RotationAction {
            delta: vec2(x, y),
            input_device,
        }
    }
}

/// This threshold feels right
const GAMEPAD_NOISE_THRESHOLD: f32 = 0.085;

pub fn collect_gamepad_input(
    mut message_writer: MessageWriter<RotationAction>,
    gamepad_q: Query<&Gamepad>,
) {
    for gamepad in gamepad_q.iter() {
        if let (Some(right_x), Some(right_y)) = (
            gamepad.get(GamepadAxis::RightStickX),
            gamepad.get(GamepadAxis::RightStickY),
        ) {
            let denoised_right_x = if right_x.abs() > GAMEPAD_NOISE_THRESHOLD {
                right_x
            } else {
                0.0
            };
            let denoised_right_y = if right_y.abs() > GAMEPAD_NOISE_THRESHOLD {
                right_y
            } else {
                0.0
            };
            message_writer.write(RotationAction::new(
                denoised_right_x,
                -denoised_right_y,
                InputDevice::Gamepad,
            ));
        }
    }
}

pub fn collect_mouse_input(
    mut message_writer: MessageWriter<RotationAction>,
    accumulated_mouse_motion: Res<AccumulatedMouseMotion>,
) {
    let delta = accumulated_mouse_motion.delta;

    if delta != Vec2::ZERO {
        message_writer.write(RotationAction {
            delta: delta,
            input_device: InputDevice::Mouse,
        });
    }
}

// limit pitch to 5 degrees away from y-axis
pub const MAX_PITCH: f32 = PI / 2.0 - 5.0 / 180.0 * PI;

pub fn run_camera_controller(
    mut message_reader: MessageReader<RotationAction>,
    time: Res<Time>,
    mut windows: Query<(&Window, &mut CursorOptions)>,
    key_input: Res<ButtonInput<KeyCode>>,
    mut toggle_cursor_grab: Local<bool>,
    query: Single<(&mut Transform, &mut CameraController), With<Camera3d>>,
) {
    let (mut transform, mut controller) = query.into_inner();

    if !controller.initialized {
        let (yaw, pitch, _roll) = transform.rotation.to_euler(EulerRot::YXZ);
        controller.yaw = yaw;
        controller.pitch = pitch.clamp(-MAX_PITCH, MAX_PITCH);
        controller.initialized = true;
        info!(
            "{:?}\t- Toggle cursor grab",
            controller.keyboard_key_toggle_cursor_grab,
        );
    }

    if !controller.enabled {
        return;
    }

    if key_input.just_pressed(controller.keyboard_key_toggle_cursor_grab) {
        *toggle_cursor_grab = !*toggle_cursor_grab;

        for (window, mut cursor_options) in windows.iter_mut() {
            if !window.focused {
                continue;
            }
            if *toggle_cursor_grab {
                cursor_options.grab_mode = CursorGrabMode::Locked;
                cursor_options.visible = false;
            } else {
                cursor_options.grab_mode = CursorGrabMode::None;
                cursor_options.visible = true;
            }
        }
    }

    let delta_secs = time.delta_secs();

    // Smooth interpolation closure
    let mut smooth_interpolate = |controller: &Mut<'_, CameraController>| {
        let t = 1.0 - (-32.0 * delta_secs).exp();
        transform.rotation = transform.rotation.slerp(
            Quat::from_euler(EulerRot::YXZ, controller.yaw, controller.pitch, 0.0),
            t,
        );
    };

    let no_cursor_grab = !*toggle_cursor_grab;
    let no_input = message_reader.is_empty();

    // we still want to interpolate to prevent freezing mid-motion
    if no_cursor_grab || no_input {
        smooth_interpolate(&controller);
        return;
    }

    for message in message_reader.read() {
        let delta = message.delta;
        let sensitivity = match message.input_device {
            InputDevice::Mouse => controller.sensitivity_mouse,
            InputDevice::Gamepad => controller.sensitivity_gamepad,
        };

        if delta != Vec2::ZERO {
            let sensitivity = sensitivity * RADIANS_PER_DOT;

            // Apply clamped deltas directly (small per-frame, no wrap issues)
            let pitch_delta = delta.y * sensitivity;
            let yaw_delta = delta.x * sensitivity;

            controller.pitch = (controller.pitch - pitch_delta).clamp(-MAX_PITCH, MAX_PITCH);
            controller.yaw = controller.yaw - yaw_delta;
        }
    }

    smooth_interpolate(&controller);
}

#[derive(Component)]
pub struct CameraMoveSpeed(pub f32);

#[derive(Component)]
pub struct CameraMoveSpeedMult(pub f32);

/// Optional basic movement system for controller.
/// Usefull when don't want a separate character controller.
/// You can add [CameraMoveSpeed] and [CameraMoveSpeedMult] to the same entity that
/// has [CameraController] to controll the speed and speed multiplier of movement.
/// Default values for them are: CameraMoveSpeed(5.0) and CameraMoveSpeedMult(3.0).
#[allow(dead_code)]
pub fn camera_movement(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    camera_controller_q: Single<
        (
            &mut Transform,
            Option<&CameraMoveSpeed>,
            Option<&CameraMoveSpeedMult>,
        ),
        With<CameraController>,
    >,
) {
    let delta_t = time.delta_secs();

    let (mut camera_transform, move_speed_opt, move_speed_mult_opt) =
        camera_controller_q.into_inner();

    let raw_forward = camera_transform.forward().as_vec3();
    let raw_right = camera_transform.right().as_vec3();

    let mut wish_dir = Vec3::ZERO;

    if keys.pressed(KeyCode::KeyW) {
        wish_dir += raw_forward;
    }
    if keys.pressed(KeyCode::KeyS) {
        wish_dir -= raw_forward;
    }
    if keys.pressed(KeyCode::KeyD) {
        wish_dir += raw_right;
    }
    if keys.pressed(KeyCode::KeyA) {
        wish_dir -= raw_right;
    }

    let move_speed = move_speed_opt.unwrap_or(&CameraMoveSpeed(5.0)).0;
    let move_speed_mult = if keys.pressed(KeyCode::ShiftLeft) {
        move_speed_mult_opt.unwrap_or(&CameraMoveSpeedMult(3.0)).0
    } else {
        1.0
    };

    let wish_translation = wish_dir * delta_t * move_speed * move_speed_mult;

    camera_transform.translation += wish_translation;
}
