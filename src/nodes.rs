use egui::{Color32, Ui};
use egui_snarl::ui::{
    AnyPins, NodeLayout, PinInfo, PinPlacement, SnarlStyle, SnarlViewer, WireStyle,
};
use egui_snarl::{InPin, InPinId, NodeId, OutPin, OutPinId, Snarl};

const STRING_COLOR: Color32 = Color32::from_rgb(0x00, 0xb0, 0x00);
const NUMBER_COLOR: Color32 = Color32::from_rgb(0xb0, 0x00, 0x00);
const UNTYPED_COLOR: Color32 = Color32::from_rgb(0xb0, 0xb0, 0xb0);

/// Node types for the editor graph.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub enum EditorNode {
    /// Displays the value of the connected input.
    Sink,
    /// Outputs a number value (editable via drag).
    Number(f64),
    /// Outputs a string value (editable via text input).
    String(String),
}

/// Returns the default style for the snarl widget.
pub const fn default_style() -> SnarlStyle {
    SnarlStyle {
        node_layout: Some(NodeLayout::coil()),
        pin_placement: Some(PinPlacement::Edge),
        pin_size: Some(7.0),
        node_frame: Some(egui::Frame {
            inner_margin: egui::Margin::same(8),
            outer_margin: egui::Margin {
                left: 0,
                right: 0,
                top: 0,
                bottom: 4,
            },
            corner_radius: egui::CornerRadius::same(8),
            fill: Color32::from_gray(30),
            stroke: egui::Stroke::NONE,
            shadow: egui::Shadow::NONE,
        }),
        bg_frame: Some(egui::Frame {
            inner_margin: egui::Margin::ZERO,
            outer_margin: egui::Margin::same(2),
            corner_radius: egui::CornerRadius::ZERO,
            fill: Color32::from_gray(40),
            stroke: egui::Stroke::NONE,
            shadow: egui::Shadow::NONE,
        }),
        ..SnarlStyle::new()
    }
}

/// Viewer implementation for rendering editor nodes.
pub struct EditorViewer;

impl SnarlViewer<EditorNode> for EditorViewer {
    fn connect(&mut self, from: &OutPin, to: &InPin, snarl: &mut Snarl<EditorNode>) {
        // Only Sink has inputs, and it accepts both Number and String
        match (&snarl[from.id.node], &snarl[to.id.node]) {
            (EditorNode::Sink, _) => {
                unreachable!("Sink node has no outputs")
            }
            (EditorNode::Number(_) | EditorNode::String(_), EditorNode::Sink) => {
                // Disconnect existing wires to this input (single connection only)
                for &remote in &to.remotes {
                    snarl.disconnect(remote, to.id);
                }
                snarl.connect(from.id, to.id);
            }
            (_, EditorNode::Number(_) | EditorNode::String(_)) => {
                // Number and String nodes have no inputs
            }
        }
    }

    fn title(&mut self, node: &EditorNode) -> String {
        match node {
            EditorNode::Sink => "Sink".to_owned(),
            EditorNode::Number(_) => "Number".to_owned(),
            EditorNode::String(_) => "String".to_owned(),
        }
    }

    fn inputs(&mut self, node: &EditorNode) -> usize {
        match node {
            EditorNode::Sink => 1,
            EditorNode::Number(_) | EditorNode::String(_) => 0,
        }
    }

    fn outputs(&mut self, node: &EditorNode) -> usize {
        match node {
            EditorNode::Sink => 0,
            EditorNode::Number(_) | EditorNode::String(_) => 1,
        }
    }

    #[expect(refining_impl_trait, reason = "egui-snarl demo pattern")]
    fn show_input(&mut self, pin: &InPin, ui: &mut Ui, snarl: &mut Snarl<EditorNode>) -> PinInfo {
        let wire_style = WireStyle::AxisAligned {
            corner_radius: 10.0,
        };

        match &snarl[pin.id.node] {
            EditorNode::Sink => match &*pin.remotes {
                [] => {
                    ui.label("None");
                    PinInfo::circle()
                        .with_fill(UNTYPED_COLOR)
                        .with_wire_style(wire_style)
                }
                [remote] => match &snarl[remote.node] {
                    EditorNode::Sink => unreachable!("Sink node has no outputs"),
                    EditorNode::Number(value) => {
                        ui.label(format_float(*value));
                        PinInfo::circle()
                            .with_fill(NUMBER_COLOR)
                            .with_wire_style(wire_style)
                    }
                    EditorNode::String(value) => {
                        ui.label(format!("{value:?}"));
                        PinInfo::circle()
                            .with_fill(STRING_COLOR)
                            .with_wire_style(wire_style)
                    }
                },
                _ => unreachable!("Sink input accepts only one connection"),
            },
            EditorNode::Number(_) | EditorNode::String(_) => {
                unreachable!("Number and String nodes have no inputs")
            }
        }
    }

    #[expect(refining_impl_trait, reason = "egui-snarl demo pattern")]
    fn show_output(&mut self, pin: &OutPin, ui: &mut Ui, snarl: &mut Snarl<EditorNode>) -> PinInfo {
        let wire_style = WireStyle::AxisAligned {
            corner_radius: 10.0,
        };

        match &mut snarl[pin.id.node] {
            EditorNode::Sink => {
                unreachable!("Sink node has no outputs")
            }
            EditorNode::Number(value) => {
                ui.add(egui::DragValue::new(value));
                PinInfo::circle()
                    .with_fill(NUMBER_COLOR)
                    .with_wire_style(wire_style)
            }
            EditorNode::String(value) => {
                let edit = egui::TextEdit::singleline(value)
                    .clip_text(false)
                    .desired_width(0.0)
                    .margin(ui.spacing().item_spacing);
                ui.add(edit);
                PinInfo::circle()
                    .with_fill(STRING_COLOR)
                    .with_wire_style(wire_style)
            }
        }
    }

    fn has_graph_menu(&mut self, _pos: egui::Pos2, _snarl: &mut Snarl<EditorNode>) -> bool {
        true
    }

    fn show_graph_menu(&mut self, pos: egui::Pos2, ui: &mut Ui, snarl: &mut Snarl<EditorNode>) {
        ui.label("Add node");
        if ui.button("Number").clicked() {
            snarl.insert_node(pos, EditorNode::Number(0.0));
            ui.close();
        }
        if ui.button("String").clicked() {
            snarl.insert_node(pos, EditorNode::String(String::new()));
            ui.close();
        }
        if ui.button("Sink").clicked() {
            snarl.insert_node(pos, EditorNode::Sink);
            ui.close();
        }
    }

    fn has_dropped_wire_menu(
        &mut self,
        _src_pins: AnyPins<'_>,
        _snarl: &mut Snarl<EditorNode>,
    ) -> bool {
        true
    }

    fn show_dropped_wire_menu(
        &mut self,
        pos: egui::Pos2,
        ui: &mut Ui,
        src_pins: AnyPins<'_>,
        snarl: &mut Snarl<EditorNode>,
    ) {
        // Pin compatibility flags
        type PinCompat = usize;
        const PIN_NUM: PinCompat = 1;
        const PIN_STR: PinCompat = 2;
        const PIN_SINK: PinCompat = PIN_NUM | PIN_STR; // Sink accepts both

        const fn pin_out_compat(node: &EditorNode) -> PinCompat {
            match node {
                EditorNode::Sink => 0,
                EditorNode::Number(_) => PIN_NUM,
                EditorNode::String(_) => PIN_STR,
            }
        }

        const fn pin_in_compat(node: &EditorNode) -> PinCompat {
            match node {
                EditorNode::Sink => PIN_SINK,
                EditorNode::Number(_) | EditorNode::String(_) => 0,
            }
        }

        ui.label("Add node");

        match src_pins {
            AnyPins::Out(src_pins) => {
                // Wire dragged from an output pin - show compatible input nodes
                let Some(&src_pin) = src_pins.first() else {
                    return;
                };
                if src_pins.len() != 1 {
                    ui.label("Multiple output pins not supported");
                    return;
                }

                let src_out_ty = pin_out_compat(
                    snarl
                        .get_node(src_pin.node)
                        .expect("source node should exist"),
                );

                // Only Sink has inputs
                if src_out_ty & PIN_SINK != 0 && ui.button("Sink").clicked() {
                    let new_node = snarl.insert_node(pos, EditorNode::Sink);
                    let dst_pin = InPinId {
                        node: new_node,
                        input: 0,
                    };
                    snarl.connect(src_pin, dst_pin);
                    ui.close();
                }
            }
            AnyPins::In(src_pins) => {
                // Wire dragged from an input pin - show compatible output nodes
                let all_src_types = src_pins.iter().fold(0, |acc, pin| {
                    acc | pin_in_compat(snarl.get_node(pin.node).expect("source node should exist"))
                });

                let dst_out_candidates = [
                    ("Number", EditorNode::Number(0.0), PIN_NUM),
                    ("String", EditorNode::String(String::new()), PIN_STR),
                ];

                for (name, node_template, out_ty) in dst_out_candidates {
                    if all_src_types & out_ty != 0 && ui.button(name).clicked() {
                        let new_node = snarl.insert_node(pos, node_template);
                        let dst_pin = OutPinId {
                            node: new_node,
                            output: 0,
                        };

                        for src_pin in src_pins {
                            let src_ty = pin_in_compat(
                                snarl
                                    .get_node(src_pin.node)
                                    .expect("source node should exist"),
                            );
                            if src_ty & out_ty != 0 {
                                snarl.drop_inputs(*src_pin);
                                snarl.connect(dst_pin, *src_pin);
                            }
                        }
                        ui.close();
                    }
                }
            }
        }
    }

    fn has_node_menu(&mut self, _node: &EditorNode) -> bool {
        true
    }

    fn show_node_menu(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut Ui,
        snarl: &mut Snarl<EditorNode>,
    ) {
        ui.label("Node menu");
        if ui.button("Remove").clicked() {
            snarl.remove_node(node);
            ui.close();
        }
    }

    fn header_frame(
        &mut self,
        frame: egui::Frame,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        snarl: &Snarl<EditorNode>,
    ) -> egui::Frame {
        match snarl[node] {
            EditorNode::Sink => frame.fill(Color32::from_rgb(70, 70, 80)),
            EditorNode::Number(_) => frame.fill(Color32::from_rgb(70, 40, 40)),
            EditorNode::String(_) => frame.fill(Color32::from_rgb(40, 70, 40)),
        }
    }
}

fn format_float(v: f64) -> String {
    let v = (v * 1000.0).round() / 1000.0;
    format!("{v}")
}
