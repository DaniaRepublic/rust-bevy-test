use bevy::{prelude::*, text::FontSmoothing};
use bevy_dev_tools::fps_overlay::{FpsOverlayConfig, FpsOverlayPlugin, FrameTimeGraphConfig};

pub struct FPSOverlayPlugin;

impl Plugin for FPSOverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FpsOverlayPlugin {
            config: FpsOverlayConfig {
                text_config: TextFont {
                    // Here we define size of our overlay
                    font_size: 18.0,
                    // If we want, we can use a custom font
                    font: default(),
                    // We could also disable font smoothing,
                    font_smoothing: FontSmoothing::default(),
                    ..default()
                },
                // We can also change color of the overlay
                text_color: OverlayColor::GREEN,
                // We can also set the refresh interval for the FPS counter
                refresh_interval: core::time::Duration::from_millis(100),
                enabled: true,
                frame_time_graph_config: FrameTimeGraphConfig {
                    enabled: false,
                    // The minimum acceptable fps
                    min_fps: 30.0,
                    // The target fps
                    target_fps: 120.0,
                },
            },
        })
        .add_systems(Startup, setup_ui)
        .add_systems(Update, customize_config);
    }
}

struct OverlayColor;

#[allow(dead_code)]
impl OverlayColor {
    const RED: Color = Color::srgb(1.0, 0.0, 0.0);
    const GREEN: Color = Color::srgb(0.0, 1.0, 0.0);
}

fn setup_ui() {
    info!(
        "
FPS UI Controls:
    Press 1 to decrease the overlay size.
    Press 2 to increase the overlay size.
    Press 3 to toggle the text visibility.
    Press 4 to toggle the frame time graph."
    );
}

fn customize_config(input: Res<ButtonInput<KeyCode>>, mut overlay: ResMut<FpsOverlayConfig>) {
    if input.just_pressed(KeyCode::Digit1) {
        overlay.text_config.font_size -= 2.0;
    }
    if input.just_pressed(KeyCode::Digit2) {
        overlay.text_config.font_size += 2.0;
    }
    if input.just_pressed(KeyCode::Digit3) {
        overlay.enabled = !overlay.enabled;
    }
    if input.just_released(KeyCode::Digit4) {
        overlay.frame_time_graph_config.enabled = !overlay.frame_time_graph_config.enabled;
    }
}
