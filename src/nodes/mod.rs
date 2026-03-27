//! Node types and graph viewer for the PMSM simulation.
//!
//! [`SimNode`] is the single enum carried by every slot in the
//! [`egui_snarl::Snarl`] graph.  [`SimViewer`] implements
//! [`SnarlViewer`](egui_snarl::ui::SnarlViewer) to render the graph
//! and enforce typed-port connection rules.

pub mod constant;
pub mod electrical;
pub mod mechanical;
pub mod park;
pub mod plot;
pub mod torque;

use egui::{Color32, Ui};
use egui_snarl::ui::{
    AnyPins, BackgroundPattern, Grid, NodeLayout, PinInfo, PinPlacement, SnarlStyle, SnarlViewer,
    WireStyle,
};
use egui_snarl::{InPin, InPinId, NodeId, OutPin, OutPinId, Snarl};

use crate::port::{PortType, PortValue};

use self::constant::ConstantNode;
use self::electrical::ElectricalNode;
use self::mechanical::MechanicalNode;
use self::park::InverseParkNode;
use self::plot::PlotNode;
use self::torque::TorqueNode;

/// All node types that can appear in the simulation graph.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub enum SimNode {
    /// Constant scalar source.
    Constant(ConstantNode),
    /// PMSM stator electrical dynamics (ODE).
    Electrical(ElectricalNode),
    /// Electromagnetic torque calculator (algebraic).
    Torque(TorqueNode),
    /// Rotor mechanical dynamics (ODE).
    Mechanical(MechanicalNode),
    /// Inverse Park transform (algebraic post-processing).
    InversePark(InverseParkNode),
    /// Time-series plot (sink).
    Plot(PlotNode),
}

impl SimNode {
    /// Human-readable title shown in the node header.
    pub fn title(&self) -> &'static str {
        match self {
            Self::Constant(_) => ConstantNode::title(),
            Self::Electrical(_) => ElectricalNode::title(),
            Self::Torque(_) => TorqueNode::title(),
            Self::Mechanical(_) => MechanicalNode::title(),
            Self::InversePark(_) => InverseParkNode::title(),
            Self::Plot(_) => PlotNode::title(),
        }
    }

    /// Number of input pins.
    pub fn num_inputs(&self) -> usize {
        self.input_port_types().len()
    }

    /// Number of output pins.
    pub fn num_outputs(&self) -> usize {
        self.output_port_types().len()
    }

    /// Declared port types for each input pin.
    fn input_port_types(&self) -> Vec<PortType> {
        match self {
            Self::Constant(_) => ConstantNode::input_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Electrical(_) => ElectricalNode::input_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Torque(_) => TorqueNode::input_ports().iter().map(|(_, t)| *t).collect(),
            Self::Mechanical(_) => MechanicalNode::input_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::InversePark(_) => InverseParkNode::input_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Plot(p) => p.input_ports().iter().map(|(_, t)| *t).collect(),
        }
    }

    /// Declared port types for each output pin.
    fn output_port_types(&self) -> Vec<PortType> {
        match self {
            Self::Constant(_) => ConstantNode::output_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Electrical(_) => ElectricalNode::output_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Torque(_) => TorqueNode::output_ports().iter().map(|(_, t)| *t).collect(),
            Self::Mechanical(_) => MechanicalNode::output_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::InversePark(_) => InverseParkNode::output_ports()
                .iter()
                .map(|(_, t)| *t)
                .collect(),
            Self::Plot(_) => PlotNode::output_ports().iter().map(|(_, t)| *t).collect(),
        }
    }

    /// The port type for a specific input pin, if it exists.
    fn input_port_type(&self, input: usize) -> Option<PortType> {
        self.input_port_types().get(input).copied()
    }

    /// The port type for a specific output pin, if it exists.
    fn output_port_type(&self, output: usize) -> Option<PortType> {
        self.output_port_types().get(output).copied()
    }

    /// Pin label for an input at the given index.
    fn input_label(&self, input: usize) -> &'static str {
        match self {
            Self::Constant(_) => ConstantNode::input_ports()
                .get(input)
                .map_or("?", |(n, _)| n),
            Self::Electrical(_) => ElectricalNode::input_ports()
                .get(input)
                .map_or("?", |(n, _)| n),
            Self::Torque(_) => TorqueNode::input_ports().get(input).map_or("?", |(n, _)| n),
            Self::Mechanical(_) => MechanicalNode::input_ports()
                .get(input)
                .map_or("?", |(n, _)| n),
            Self::InversePark(_) => InverseParkNode::input_ports()
                .get(input)
                .map_or("?", |(n, _)| n),
            Self::Plot(_) => "signal",
        }
    }

    /// Pin label for an output at the given index.
    fn output_label(&self, output: usize) -> &'static str {
        match self {
            Self::Constant(_) => ConstantNode::output_ports()
                .get(output)
                .map_or("?", |(n, _)| n),
            Self::Electrical(_) => ElectricalNode::output_ports()
                .get(output)
                .map_or("?", |(n, _)| n),
            Self::Torque(_) => TorqueNode::output_ports()
                .get(output)
                .map_or("?", |(n, _)| n),
            Self::Mechanical(_) => MechanicalNode::output_ports()
                .get(output)
                .map_or("?", |(n, _)| n),
            Self::InversePark(_) => InverseParkNode::output_ports()
                .get(output)
                .map_or("?", |(n, _)| n),
            Self::Plot(_) => PlotNode::output_ports().get(output).map_or("?", |(n, _)| n),
        }
    }

    /// Header background color for this node category.
    fn header_color(&self) -> Color32 {
        match self {
            Self::Constant(_) => ConstantNode::header_color(),
            Self::Electrical(_) => ElectricalNode::header_color(),
            Self::Torque(_) => TorqueNode::header_color(),
            Self::Mechanical(_) => MechanicalNode::header_color(),
            Self::InversePark(_) => InverseParkNode::header_color(),
            Self::Plot(_) => PlotNode::header_color(),
        }
    }

    /// Get the output [`PortValue`] at a given pin index, if simulation results exist.
    pub fn output_value(&self, output: usize) -> Option<&PortValue> {
        match (self, output) {
            (Self::Electrical(e), 0) => e.output_i_d.as_ref(),
            (Self::Electrical(e), 1) => e.output_i_q.as_ref(),
            (Self::Torque(t), 0) => t.output_t_e.as_ref(),
            (Self::Mechanical(m), 0) => m.output_omega_m.as_ref(),
            (Self::Mechanical(m), 1) => m.output_theta_e.as_ref(),
            (Self::InversePark(p), 0) => p.output_f_abc.as_ref(),
            _ => None,
        }
    }
}

/// Returns the default visual style for the snarl graph widget.
pub fn default_style() -> SnarlStyle {
    SnarlStyle {
        node_layout: Some(NodeLayout::coil()),
        pin_placement: Some(PinPlacement::Edge),
        pin_size: Some(7.0),
        bg_pattern: Some(BackgroundPattern::Grid(Grid::new(
            egui::Vec2::new(20.0, 20.0),
            0.0,
        ))),
        node_frame: Some(egui::Frame {
            inner_margin: egui::Margin::same(8),
            outer_margin: egui::Margin {
                left: 0,
                right: 0,
                top: 0,
                bottom: 4,
            },
            corner_radius: egui::CornerRadius::same(4),
            fill: Color32::from_gray(30),
            stroke: egui::Stroke::NONE,
            shadow: egui::Shadow::NONE,
        }),
        bg_frame: Some(egui::Frame {
            inner_margin: egui::Margin::ZERO,
            outer_margin: egui::Margin::ZERO,
            corner_radius: egui::CornerRadius::ZERO,
            fill: Color32::from_gray(40),
            stroke: egui::Stroke::NONE,
            shadow: egui::Shadow::NONE,
        }),
        ..SnarlStyle::new()
    }
}

/// Wire style shared by all connections.
fn wire_style() -> WireStyle {
    WireStyle::AxisAligned {
        corner_radius: 10.0,
    }
}

/// The [`SnarlViewer`] implementation that drives the node graph UI.
pub struct SimViewer;

impl SnarlViewer<SimNode> for SimViewer {
    fn connect(&mut self, from: &OutPin, to: &InPin, snarl: &mut Snarl<SimNode>) {
        let out_type = snarl[from.id.node].output_port_type(from.id.output);
        let in_type = snarl[to.id.node].input_port_type(to.id.input);

        // Only connect if port types are compatible.
        if let (Some(out_t), Some(in_t)) = (out_type, in_type)
            && out_t.is_compatible_with(in_t)
        {
            // Disconnect any existing wires to this input (single connection only).
            for &remote in &to.remotes {
                snarl.disconnect(remote, to.id);
            }
            snarl.connect(from.id, to.id);
        }
    }

    fn title(&mut self, node: &SimNode) -> String {
        node.title().to_owned()
    }

    fn inputs(&mut self, node: &SimNode) -> usize {
        node.num_inputs()
    }

    fn outputs(&mut self, node: &SimNode) -> usize {
        node.num_outputs()
    }

    #[expect(refining_impl_trait, reason = "egui-snarl requires concrete PinInfo")]
    fn show_input(&mut self, pin: &InPin, ui: &mut Ui, snarl: &mut Snarl<SimNode>) -> PinInfo {
        let node = &snarl[pin.id.node];
        let label = node.input_label(pin.id.input);
        let port_type = node
            .input_port_type(pin.id.input)
            .unwrap_or(PortType::Signal);

        ui.label(label);

        PinInfo::circle()
            .with_fill(port_type.color())
            .with_wire_style(wire_style())
    }

    #[expect(refining_impl_trait, reason = "egui-snarl requires concrete PinInfo")]
    fn show_output(&mut self, pin: &OutPin, ui: &mut Ui, snarl: &mut Snarl<SimNode>) -> PinInfo {
        let node = &mut snarl[pin.id.node];
        let label = node.output_label(pin.id.output);
        let port_type = node
            .output_port_type(pin.id.output)
            .unwrap_or(PortType::Signal);

        // For Constant nodes, show an editable drag value.
        if let SimNode::Constant(c) = node {
            ui.add(egui::DragValue::new(&mut c.value).speed(0.1));
        } else {
            ui.label(label);
        }

        PinInfo::circle()
            .with_fill(port_type.color())
            .with_wire_style(wire_style())
    }

    fn has_body(&mut self, node: &SimNode) -> bool {
        matches!(
            node,
            SimNode::Electrical(_) | SimNode::Mechanical(_) | SimNode::Torque(_) | SimNode::Plot(_)
        )
    }

    fn show_body(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut Ui,
        snarl: &mut Snarl<SimNode>,
    ) {
        // Plot nodes need immutable access to remote nodes for data, so handle
        // them before the mutable match on the node body.
        if matches!(&snarl[node], SimNode::Plot(_)) {
            show_plot_inline(node, _inputs, ui, snarl);
            return;
        }

        match &mut snarl[node] {
            SimNode::Electrical(e) => {
                egui::Grid::new(ui.id().with("elec_params"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        param_row(ui, "R_s (\u{03a9})", &mut e.r_s);
                        param_row(ui, "L_d (H)", &mut e.l_d);
                        param_row(ui, "L_q (H)", &mut e.l_q);
                        param_row(ui, "\u{03bb}_m (Wb)", &mut e.lambda_m);
                        param_row(ui, "N_p", &mut e.n_p);
                        ui.separator();
                        ui.end_row();
                        param_row(ui, "i_d\u{2080} (A)", &mut e.i_d_0);
                        param_row(ui, "i_q\u{2080} (A)", &mut e.i_q_0);
                    });
            }
            SimNode::Mechanical(m) => {
                egui::Grid::new(ui.id().with("mech_params"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        param_row(ui, "J (kg\u{00b7}m\u{00b2})", &mut m.j);
                        param_row(ui, "B (N\u{00b7}m\u{00b7}s/rad)", &mut m.b);
                        param_row(ui, "N_p", &mut m.n_p);
                        ui.separator();
                        ui.end_row();
                        param_row(ui, "\u{03c9}_m\u{2080} (rad/s)", &mut m.omega_m_0);
                        param_row(ui, "\u{03b8}_e\u{2080} (rad)", &mut m.theta_e_0);
                    });
            }
            SimNode::Torque(t) => {
                egui::Grid::new(ui.id().with("torque_params"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        param_row(ui, "N_p", &mut t.n_p);
                        param_row(ui, "\u{03bb}_m (Wb)", &mut t.lambda_m);
                        param_row(ui, "L_d (H)", &mut t.l_d);
                        param_row(ui, "L_q (H)", &mut t.l_q);
                    });
            }
            SimNode::Constant(_) | SimNode::InversePark(_) | SimNode::Plot(_) => {}
        }
    }

    fn has_graph_menu(&mut self, _pos: egui::Pos2, _snarl: &mut Snarl<SimNode>) -> bool {
        true
    }

    fn show_graph_menu(&mut self, pos: egui::Pos2, ui: &mut Ui, snarl: &mut Snarl<SimNode>) {
        ui.label("Add Node");
        for (label, node) in node_palette() {
            if ui.button(label).clicked() {
                snarl.insert_node(pos, node);
                ui.close();
            }
        }
    }

    fn has_dropped_wire_menu(
        &mut self,
        _src_pins: AnyPins<'_>,
        _snarl: &mut Snarl<SimNode>,
    ) -> bool {
        true
    }

    fn show_dropped_wire_menu(
        &mut self,
        pos: egui::Pos2,
        ui: &mut Ui,
        src_pins: AnyPins<'_>,
        snarl: &mut Snarl<SimNode>,
    ) {
        ui.label("Add Node");
        match src_pins {
            AnyPins::Out(src_pins) => {
                let Some(&src_pin) = src_pins.first() else {
                    return;
                };
                let out_type = snarl[src_pin.node].output_port_type(src_pin.output);

                for (label, template) in node_palette() {
                    // Check if any input of this template is compatible.
                    let compatible_input = template
                        .input_port_types()
                        .iter()
                        .enumerate()
                        .find(|(_, t)| out_type.is_some_and(|ot| ot.is_compatible_with(**t)))
                        .map(|(i, _)| i);

                    if let Some(input_idx) = compatible_input
                        && ui.button(label).clicked()
                    {
                        let new_node = snarl.insert_node(pos, template);
                        let dst_pin = InPinId {
                            node: new_node,
                            input: input_idx,
                        };
                        snarl.connect(src_pin, dst_pin);
                        ui.close();
                    }
                }
            }
            AnyPins::In(src_pins) => {
                let Some(&src_pin) = src_pins.first() else {
                    return;
                };
                let in_type = snarl[src_pin.node].input_port_type(src_pin.input);

                for (label, template) in node_palette() {
                    let compatible_output = template
                        .output_port_types()
                        .iter()
                        .enumerate()
                        .find(|(_, t)| in_type.is_some_and(|it| it.is_compatible_with(**t)))
                        .map(|(i, _)| i);

                    if let Some(output_idx) = compatible_output
                        && ui.button(label).clicked()
                    {
                        let new_node = snarl.insert_node(pos, template);
                        let dst_pin = OutPinId {
                            node: new_node,
                            output: output_idx,
                        };
                        snarl.drop_inputs(src_pin);
                        snarl.connect(dst_pin, src_pin);
                        ui.close();
                    }
                }
            }
        }
    }

    fn has_node_menu(&mut self, _node: &SimNode) -> bool {
        true
    }

    fn show_node_menu(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut Ui,
        snarl: &mut Snarl<SimNode>,
    ) {
        ui.label("Node");
        if let SimNode::Plot(p) = &mut snarl[node]
            && ui.button("Add Input").clicked()
        {
            p.add_input();
            ui.close();
        }
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
        snarl: &Snarl<SimNode>,
    ) -> egui::Frame {
        frame.fill(snarl[node].header_color())
    }
}

/// The palette of node types available for creation.
fn node_palette() -> Vec<(&'static str, SimNode)> {
    vec![
        ("Constant", SimNode::Constant(ConstantNode::default())),
        (
            "PMSM Electrical",
            SimNode::Electrical(ElectricalNode::default()),
        ),
        ("Torque Calculator", SimNode::Torque(TorqueNode::default())),
        (
            "Mechanical Dynamics",
            SimNode::Mechanical(MechanicalNode::default()),
        ),
        (
            "Inverse Park",
            SimNode::InversePark(InverseParkNode::default()),
        ),
        ("Plot", SimNode::Plot(PlotNode::default())),
    ]
}

/// Helper: renders a labelled `DragValue` row inside an `egui::Grid`.
fn param_row(ui: &mut Ui, label: &str, value: &mut f64) {
    ui.label(label);
    ui.add(egui::DragValue::new(value).speed(0.001));
    ui.end_row();
}

/// Renders the plot body for a `PlotNode` using `egui_plot`.
///
/// Collects signal data from connected input pins' remote output nodes and
/// plots each as a line series.
fn show_plot_inline(_node_id: NodeId, inputs: &[InPin], ui: &mut Ui, snarl: &Snarl<SimNode>) {
    use egui_plot::{Line, PlotPoints};

    // Collect plot data from connected remote nodes before rendering.
    // Each entry: (label, Vec of [x,y] points).
    let mut lines: Vec<(String, Vec<[f64; 2]>)> = Vec::new();

    for (i, input) in inputs.iter().enumerate() {
        if let Some(remote) = input.remotes.first() {
            let source_node = &snarl[remote.node];
            match source_node.output_value(remote.output) {
                Some(PortValue::Signal(data)) => {
                    lines.push((format!("signal_{i}"), data.clone()));
                }
                Some(PortValue::Vector(data)) => {
                    for (phase, &name) in ["a", "b", "c"].iter().enumerate() {
                        let pts: Vec<[f64; 2]> = data
                            .iter()
                            .map(|row| {
                                // row is [t, a, b, c] — index 0 is time, 1..3 are phases
                                [row[0], row.get(phase + 1).copied().unwrap_or(0.0)]
                            })
                            .collect();
                        lines.push((format!("{name}_{i}"), pts));
                    }
                }
                _ => {}
            }
        }
    }

    let plot = egui_plot::Plot::new(ui.id().with("plot_area"))
        .height(200.0)
        .allow_drag(false)
        .allow_scroll(false)
        .allow_zoom(false)
        .show_axes(true);

    plot.show(ui, |plot_ui| {
        for (name, data) in &lines {
            let points: PlotPoints<'_> = data.iter().map(|&[t, v]| [t, v]).collect();
            plot_ui.line(Line::new(name.as_str(), points));
        }
    });
}
