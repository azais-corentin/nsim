use crate::nodes::{self, SimNode, SimViewer};
use crate::simulation::SimConfig;
use egui_snarl::Snarl;
use egui_snarl::ui::SnarlWidget;

/// Pane types for the tile tree.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
enum Pane {
    Left,
    Center,
    Right,
}

/// Creates the initial tile tree with a horizontal layout: Left (10%) | Center (80%) | Right (10%).
fn create_tree() -> egui_tiles::Tree<Pane> {
    let mut tiles = egui_tiles::Tiles::default();

    let left = tiles.insert_pane(Pane::Left);
    let center = tiles.insert_pane(Pane::Center);
    let right = tiles.insert_pane(Pane::Right);

    let mut linear =
        egui_tiles::Linear::new(egui_tiles::LinearDir::Horizontal, vec![left, center, right]);
    linear.shares.set_share(left, 1.0);
    linear.shares.set_share(center, 8.0);
    linear.shares.set_share(right, 1.0);

    let root = tiles.insert_container(egui_tiles::Container::Linear(linear));

    egui_tiles::Tree::new("nsim_tree", root, tiles)
}

/// Defines how panes are rendered and their behavior.
struct TreeBehavior<'a> {
    snarl: &'a mut Snarl<SimNode>,
    sim_status: &'a str,
}

impl egui_tiles::Behavior<Pane> for TreeBehavior<'_> {
    fn tab_title_for_pane(&mut self, pane: &Pane) -> egui::WidgetText {
        match pane {
            Pane::Left => "Node Library".into(),
            Pane::Center => "Editor".into(),
            Pane::Right => "Properties".into(),
        }
    }

    fn pane_ui(
        &mut self,
        ui: &mut egui::Ui,
        _tile_id: egui_tiles::TileId,
        pane: &mut Pane,
    ) -> egui_tiles::UiResponse {
        match pane {
            Pane::Center => {
                SnarlWidget::new()
                    .id(egui::Id::new("editor-snarl"))
                    .style(nodes::default_style())
                    .show(self.snarl, &mut SimViewer, ui);
            }
            Pane::Left => {
                show_node_library(ui, self.snarl);
            }
            Pane::Right => {
                ui.label(format!("Status: {}", self.sim_status));
            }
        }

        egui_tiles::UiResponse::None
    }

    fn simplification_options(&self) -> egui_tiles::SimplificationOptions {
        egui_tiles::SimplificationOptions {
            all_panes_must_have_tabs: true,
            ..Default::default()
        }
    }

    fn is_tab_closable(
        &self,
        tiles: &egui_tiles::Tiles<Pane>,
        tile_id: egui_tiles::TileId,
    ) -> bool {
        !matches!(
            tiles.get(tile_id),
            Some(egui_tiles::Tile::Pane(Pane::Center))
        )
    }
}

/// Main application state. Persisted across sessions.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)]
pub struct TemplateApp {
    tree: egui_tiles::Tree<Pane>,
    snarl: Snarl<SimNode>,
    sim_config: SimConfig,
    #[serde(skip)]
    sim_status: String,
}

impl Default for TemplateApp {
    fn default() -> Self {
        Self {
            tree: create_tree(),
            snarl: Snarl::new(),
            sim_config: SimConfig::default(),
            sim_status: "Ready".to_owned(),
        }
    }
}

impl TemplateApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        if let Some(storage) = cc.storage {
            eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default()
        } else {
            Self::default()
        }
    }
}

impl eframe::App for TemplateApp {
    /// Called by the framework to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    /// Called each time the UI needs repainting.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                let is_web = cfg!(target_arch = "wasm32");
                if !is_web {
                    ui.menu_button("File", |ui| {
                        if ui.button("Quit").clicked() {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                        }
                    });
                    ui.add_space(16.0);
                }

                // Simulation controls
                ui.separator();
                ui.label("t₀:");
                ui.add(
                    egui::DragValue::new(&mut self.sim_config.t_start)
                        .speed(0.01)
                        .range(0.0..=self.sim_config.t_end),
                );
                ui.label("t₁:");
                ui.add(
                    egui::DragValue::new(&mut self.sim_config.t_end)
                        .speed(0.01)
                        .range(self.sim_config.t_start..=100.0),
                );
                ui.label("Δt:");
                ui.add(
                    egui::DragValue::new(&mut self.sim_config.output_dt)
                        .speed(0.0001)
                        .range(0.0001..=1.0)
                        .suffix(" s")
                        .max_decimals(4),
                );

                if ui.button("▶ Simulate").clicked() {
                    match crate::simulation::solver::run_simulation(
                        &mut self.snarl,
                        &self.sim_config,
                    ) {
                        Ok(()) => {
                            self.sim_status = "Simulation complete".to_owned();
                        }
                        Err(e) => {
                            self.sim_status = format!("Error: {e}");
                            log::error!("Simulation failed: {e}");
                        }
                    }
                }

                ui.add_space(8.0);
                egui::widgets::global_theme_preference_buttons(ui);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let mut behavior = TreeBehavior {
                snarl: &mut self.snarl,
                sim_status: &self.sim_status,
            };
            self.tree.ui(&mut behavior, ui);
        });
    }
}

/// Renders the node library in the left pane.
fn show_node_library(ui: &mut egui::Ui, snarl: &mut Snarl<SimNode>) {
    ui.heading("Nodes");
    ui.separator();

    let center = egui::pos2(0.0, 0.0);
    let entries: &[(&str, SimNode)] = &[
        (
            "Constant",
            SimNode::Constant(nodes::constant::ConstantNode::default()),
        ),
        (
            "PMSM Electrical",
            SimNode::Electrical(nodes::electrical::ElectricalNode::default()),
        ),
        (
            "Torque Calculator",
            SimNode::Torque(nodes::torque::TorqueNode::default()),
        ),
        (
            "Mechanical Dynamics",
            SimNode::Mechanical(nodes::mechanical::MechanicalNode::default()),
        ),
        (
            "Inverse Park",
            SimNode::InversePark(nodes::park::InverseParkNode::default()),
        ),
        ("Park", SimNode::Park(nodes::park::ParkNode::default())),
        ("Plot", SimNode::Plot(nodes::plot::PlotNode::default())),
    ];

    for (label, template) in entries {
        if ui.button(*label).clicked() {
            snarl.insert_node(center, template.clone());
        }
    }
}
