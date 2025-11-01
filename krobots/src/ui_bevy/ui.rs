use super::{ObjUID, PhysicsState, SimulatorTx};
use crate::{
    physics::{Tank, UICommand},
    ui_bevy::camera_controller::CameraController,
};
use bevy::prelude::*;
use bevy_egui::{
    EguiContexts,
    egui::{self},
};

#[derive(Default, Resource)]
pub(super) struct UiState {
    value: f32,
    is_window_open: bool,
    pub selected_tank_id: Option<ObjUID>,
}

pub(super) fn configure_ui_state_system(mut ui_state: ResMut<UiState>) {
    ui_state.is_window_open = true;
}

pub(super) fn ui_update(
    mut ui_state: ResMut<UiState>,
    mut is_initialized: Local<bool>,
    mut contexts: EguiContexts,
    physics_state: Res<PhysicsState>,
    simulator_tx: Res<SimulatorTx>,
    mut query: Query<&mut CameraController, With<Camera>>,
) {
    let mut load = false;
    let mut remove = false;
    let mut invert = false;

    if !*is_initialized {
        *is_initialized = true;
    }

    let ctx = contexts.ctx_mut().unwrap();

    // Sort tanks
    let mut sorted_tanks: Vec<&Tank> = physics_state.tanks.values().collect();
    sorted_tanks.sort_by_cached_key(|t| t.name.clone());
    let sorted_tanks = sorted_tanks;

    // If no tanks selected tanks is None
    // if there are tanks but no selection, select the first one
    if sorted_tanks.is_empty() {
        ui_state.selected_tank_id = None;
    } else if ui_state.selected_tank_id.is_none() {
        ui_state.selected_tank_id = Some(sorted_tanks[0].get_id());
    }
    let selected_tank = match ui_state.selected_tank_id {
        None => None,
        Some(objui) => physics_state.tanks.get(&objui),
    };
    let selected_tank_name = selected_tank.map_or("", |x| &x.name);

    egui::SidePanel::left("side_panel")
        .default_width(200.0)
        .show(ctx, |ui| {
            ui.heading("Tank parameters");
            egui::ComboBox::from_label("Select one!")
                .selected_text(selected_tank_name)
                .show_ui(ui, |ui| {
                    for tank in sorted_tanks {
                        ui.selectable_value(&mut ui_state.selected_tank_id, Some(tank.get_id()), {
                            if tank.is_dead() {
                                tank.name.clone().to_uppercase()
                            } else {
                                tank.name.clone()
                            }
                        });
                    }
                });
            if let Some(tank) = selected_tank {
                let position = (
                    tank.position().translation.vector.x,
                    tank.position().translation.vector.y,
                );
                let rotation = tank.position().rotation.angle();
                let power = tank.engine_power();
                let power_fraction = tank.engine_power_fraction();
                let turning_power = tank.turning_power();
                let turning_power_fraction = tank.turning_power_fraction();
                let damage = tank.damage();
                let damage_fraction = damage / tank.damage_max();
                let linear_velocity = tank.linear_velocity();
                let forward_velocity = tank.forward_velocity();
                let angular_velocity = tank.angular_velocity();
                egui::Grid::new("some_unique_id").show(ui, |ui| {
                    ui.label("Position");
                    ui.label(format!("{:.0},{:.0}", position.0, position.1));
                    ui.end_row();

                    ui.label("Angle");
                    ui.label(format!("{rotation:.3}"));
                    ui.end_row();

                    ui.label("Forward Vel");
                    ui.label(format!("{:.2e}", forward_velocity));
                    ui.end_row();

                    ui.label("Linear Vel");
                    ui.label(format!("{:.2e}", linear_velocity));
                    ui.end_row();

                    ui.label("Angular Vel");
                    ui.label(format!("{:.2e}", angular_velocity));
                    ui.end_row();
                });
                ui.horizontal(|ui| {
                    ui.add(
                        egui::ProgressBar::new(power_fraction)
                            .show_percentage()
                            .text("Engine Power")
                            .desired_width(150.0)
                            .fill(egui::Color32::DARK_BLUE),
                    );
                    ui.label(format!("{:.1}", power));
                });
                ui.horizontal(|ui| {
                    ui.add(
                        egui::ProgressBar::new(turning_power_fraction)
                            .show_percentage()
                            .text("Turning Power")
                            .desired_width(150.0)
                            .fill(egui::Color32::DARK_BLUE),
                    );
                    ui.label(format!("{:.1}", turning_power));
                });
                ui.horizontal(|ui| {
                    ui.add(
                        egui::ProgressBar::new(damage_fraction)
                            .show_percentage()
                            .text("Damage")
                            .desired_width(150.0)
                            .fill(egui::Color32::DARK_RED),
                    );
                    ui.label(format!("{:.1}", damage));
                });

                ui.separator();
                ui.label("Turret");
                let turret = tank.turret();
                let turret_angle = turret.angle();
                let cannon_temp = turret.cannon_temperature();
                egui::Grid::new("turret_parameters").show(ui, |ui| {
                    ui.label("Angle");
                    ui.label(format!("{:.3}", turret_angle));
                    ui.end_row();

                    ui.label("Temp");
                    ui.label(format!("{:.1}", cannon_temp));
                    ui.end_row();
                });

                ui.separator();
                ui.label("Radar");
                let radar_position = tank.radar_position();
                let radar_width = tank.radar_width();
                egui::Grid::new("radar_parameters").show(ui, |ui| {
                    ui.label("Angle");
                    ui.label(format!("{:.3}", radar_position));
                    ui.end_row();

                    ui.label("Width");
                    ui.label(format!("{:.3}", radar_width));
                    ui.end_row();
                });
            }
            ui.separator();
            if let Ok(mut camera_controller) = query.single_mut() {
                ui.checkbox(&mut camera_controller.track_tank_enabled, "Tank tracking");
            }
            ui.add(egui::Slider::new(&mut ui_state.value, 0.0..=10.0).text("value"));
            if ui.button("Increment").clicked() {
                ui_state.value += 1.0;
            }

            ui.allocate_space(egui::Vec2::new(1.0, 100.0));
            ui.horizontal(|ui| {
                load = ui.button("Load").clicked();
                invert = ui.button("Invert").clicked();
                remove = ui.button("Remove").clicked();
            });

            ui.allocate_space(egui::Vec2::new(1.0, 10.0));

            ui.with_layout(egui::Layout::bottom_up(egui::Align::Center), |ui| {
                ui.add(egui::Hyperlink::from_label_and_url(
                    "powered by egui",
                    "https://github.com/emilk/egui/",
                ));
            });
        });

    egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
        // The top panel is often a good place for a menu bar:
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("Quit").clicked() {
                    simulator_tx
                        .tx_ui_command
                        .lock()
                        .unwrap()
                        .send(UICommand::QUIT)
                        .expect("Failed to send quit command to simulator");
                }
            });
        });
    });
}
