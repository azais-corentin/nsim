use crate::nodes::{self, EditorNode, EditorViewer};
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

    // Horizontal layout with shares: 1 | 8 | 1 (10% | 80% | 10%)
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
    snarl: &'a mut Snarl<EditorNode>,
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
                    .show(self.snarl, &mut EditorViewer, ui);
            }
            Pane::Left | Pane::Right => {
                ui.label("hello, world");
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
    snarl: Snarl<EditorNode>,
}

impl Default for TemplateApp {
    fn default() -> Self {
        Self {
            tree: create_tree(),
            snarl: Snarl::new(),
        }
    }
}

impl TemplateApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
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
                egui::widgets::global_theme_preference_buttons(ui);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let mut behavior = TreeBehavior {
                snarl: &mut self.snarl,
            };
            self.tree.ui(&mut behavior, ui);
        });
    }
}
