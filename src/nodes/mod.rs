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
use self::park::ParkNode;
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
    /// Forward Park transform (algebraic post-processing).
    Park(ParkNode),
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
            Self::Park(_) => ParkNode::title(),
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
            Self::Park(_) => ParkNode::input_ports().iter().map(|(_, t)| *t).collect(),
            Self::Plot(p) => p.input_ports().iter().map(|(_, t)| *t).collect(),
        }
    }

    /// Declared port types for each output pin.
    fn output_port_types(&self) -> Vec<PortType> {
        match self {
            Self::Constant(c) => c.output_ports().iter().map(|(_, t)| *t).collect(),
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
            Self::Park(_) => ParkNode::output_ports().iter().map(|(_, t)| *t).collect(),
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
            Self::Park(_) => ParkNode::input_ports().get(input).map_or("?", |(n, _)| n),
            Self::Plot(_) => "data",
        }
    }

    /// Pin label for an output at the given index.
    fn output_label(&self, output: usize) -> &'static str {
        match self {
            Self::Constant(c) => c.output_ports().get(output).map_or("?", |(n, _)| *n),
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
            Self::Park(_) => ParkNode::output_ports().get(output).map_or("?", |(n, _)| n),
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
            Self::Park(_) => ParkNode::header_color(),
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
            (Self::Park(p), 0) => p.output_f_d.as_ref(),
            (Self::Park(p), 1) => p.output_f_q.as_ref(),
            (Self::Constant(c), 0) => c.output_port_value.as_ref(),
            _ => None,
        }
    }

    /// Returns the user-defined node size override, if set.
    pub fn custom_size(&self) -> Option<egui::Vec2> {
        let raw = match self {
            Self::Constant(n) => n.custom_size,
            Self::Electrical(n) => n.custom_size,
            Self::Torque(n) => n.custom_size,
            Self::Mechanical(n) => n.custom_size,
            Self::InversePark(n) => n.custom_size,
            Self::Park(n) => n.custom_size,
            Self::Plot(n) => n.custom_size,
        };
        raw.map(|[w, h]| egui::vec2(w, h))
    }

    /// Sets or clears the user-defined node size override.
    pub fn set_custom_size(&mut self, size: egui::Vec2) {
        let val = Some([size.x, size.y]);
        match self {
            Self::Constant(n) => n.custom_size = val,
            Self::Electrical(n) => n.custom_size = val,
            Self::Torque(n) => n.custom_size = val,
            Self::Mechanical(n) => n.custom_size = val,
            Self::InversePark(n) => n.custom_size = val,
            Self::Park(n) => n.custom_size = val,
            Self::Plot(n) => n.custom_size = val,
        }
    }

    /// Clears the user-defined node size override, reverting to auto-layout.
    pub fn clear_custom_size(&mut self) {
        match self {
            Self::Constant(n) => n.custom_size = None,
            Self::Electrical(n) => n.custom_size = None,
            Self::Torque(n) => n.custom_size = None,
            Self::Mechanical(n) => n.custom_size = None,
            Self::InversePark(n) => n.custom_size = None,
            Self::Park(n) => n.custom_size = None,
            Self::Plot(n) => n.custom_size = None,
        }
    }
}

/// Returns the default visual style for the snarl graph widget.
pub fn default_style() -> SnarlStyle {
    SnarlStyle {
        node_layout: Some(NodeLayout::coil()),
        pin_placement: Some(PinPlacement::Edge),
        pin_size: Some(10.0),
        bg_pattern: Some(BackgroundPattern::Grid(Grid::new(
            egui::Vec2::new(20.0, 20.0),
            0.0,
        ))),
        node_frame: Some(egui::Frame {
            inner_margin: egui::Margin::same(10),
            outer_margin: egui::Margin {
                left: 0,
                right: 0,
                top: 0,
                bottom: 4,
            },
            corner_radius: egui::CornerRadius::same(4),
            fill: Color32::from_gray(25),
            stroke: egui::Stroke::new(1.0, Color32::from_gray(65)),
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
        wire_width: Some(3.0),
        bg_pattern_stroke: Some(egui::Stroke::new(0.5, Color32::from_gray(30))),
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
        let in_type = snarl[to.id.node].input_port_type(to.id.input);

        // If the source is a Constant, adapt its output type to match the destination.
        if let SimNode::Constant(c) = &mut snarl[from.id.node]
            && let Some(in_t) = in_type
        {
            c.adapt_output_type(in_t);
        }

        // Re-read output type after potential adaptation.
        let out_type = snarl[from.id.node].output_port_type(from.id.output);

        // Check standard type compatibility, with a special case for Plot
        // which accepts any plottable data (Signal or Vector).
        let compatible = match (out_type, in_type) {
            (Some(out_t), Some(in_t)) if out_t.is_compatible_with(in_t) => true,
            (Some(PortType::Signal | PortType::Vector), _)
                if matches!(snarl[to.id.node], SimNode::Plot(_)) =>
            {
                true
            }
            _ => false,
        };

        if compatible {
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

    fn show_header(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut Ui,
        snarl: &mut Snarl<SimNode>,
    ) {
        let title = self.title(&snarl[node]);
        let font = egui::FontId::proportional(14.0);
        let text_size = ui
            .painter()
            .layout_no_wrap(title.clone(), font.clone(), Color32::WHITE)
            .size();
        let (rect, _) = ui.allocate_exact_size(text_size, egui::Sense::hover());
        // Dark shadow behind text for readability on colored headers
        ui.painter().text(
            rect.min + egui::vec2(1.0, 1.0),
            egui::Align2::LEFT_TOP,
            &title,
            font.clone(),
            Color32::from_black_alpha(160),
        );
        // White foreground text
        ui.painter().text(
            rect.min,
            egui::Align2::LEFT_TOP,
            &title,
            font,
            Color32::WHITE,
        );
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

        // For Constant nodes, show an editable drag value inline with the pin.
        // Vector mode shows only a label here; the per-phase values go in the body.
        if let SimNode::Constant(c) = node {
            match c.output_type {
                PortType::Vector => {
                    ui.label("value");
                }
                PortType::Scalar | PortType::Signal => {
                    ui.add(egui::DragValue::new(&mut c.value).speed(0.1));
                }
            }
        } else {
            ui.label(label);
        }

        PinInfo::circle()
            .with_fill(port_type.color())
            .with_wire_style(wire_style())
    }

    fn has_body(&mut self, node: &SimNode) -> bool {
        match node {
            SimNode::Electrical(_)
            | SimNode::Mechanical(_)
            | SimNode::Torque(_)
            | SimNode::Plot(_) => true,
            // Vector-mode Constants need a body for the per-phase editors.
            SimNode::Constant(c) => c.output_type == PortType::Vector,
            _ => false,
        }
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
            let custom = snarl[node].custom_size();
            // Constrain width to custom size so the plot can shrink, not just grow.
            if let Some(size) = custom {
                ui.set_max_width(size.x);
            }
            let plot_height = custom.map_or(200.0, |s| s.y.max(80.0));
            show_plot_inline(node, _inputs, ui, snarl, plot_height);
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
            SimNode::Constant(c) if c.output_type == PortType::Vector => {
                egui::Grid::new(ui.id().with("const_vec"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        param_row(ui, "a", &mut c.value);
                        param_row(ui, "b", &mut c.value_b);
                        param_row(ui, "c", &mut c.value_c);
                    });
            }
            SimNode::InversePark(_)
            | SimNode::Park(_)
            | SimNode::Plot(_)
            | SimNode::Constant(_) => {}
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
                    // Plot accepts any plottable data (Signal or Vector).
                    let compatible_input = template
                        .input_port_types()
                        .iter()
                        .enumerate()
                        .find(|(_, t)| {
                            out_type.is_some_and(|ot| ot.is_compatible_with(**t))
                                || (matches!(out_type, Some(PortType::Signal | PortType::Vector))
                                    && matches!(template, SimNode::Plot(_)))
                        })
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
                    // Constants can adapt to any type, so always compatible.
                    let is_constant = matches!(template, SimNode::Constant(_));
                    let compatible_output = template
                        .output_port_types()
                        .iter()
                        .enumerate()
                        .find(|(_, t)| in_type.is_some_and(|it| it.is_compatible_with(**t)))
                        .map(|(i, _)| i)
                        .or(if is_constant { Some(0) } else { None });

                    if let Some(output_idx) = compatible_output
                        && ui.button(label).clicked()
                    {
                        let new_node = snarl.insert_node(pos, template);
                        // Adapt Constant output type to match the destination input.
                        if let Some(in_t) = in_type
                            && let SimNode::Constant(c) = &mut snarl[new_node]
                        {
                            c.adapt_output_type(in_t);
                        }
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
        if snarl[node].custom_size().is_some() && ui.button("Reset Size").clicked() {
            snarl[node].clear_custom_size();
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

    fn has_footer(&mut self, _node: &SimNode) -> bool {
        true
    }

    fn show_footer(
        &mut self,
        node: NodeId,
        _inputs: &[InPin],
        _outputs: &[OutPin],
        ui: &mut Ui,
        snarl: &mut Snarl<SimNode>,
    ) {
        let current_size = snarl[node].custom_size();

        // Force exact width when the user has resized. Setting both min and max
        // prevents the ratchet effect where content fills available width and
        // then prevents shrinking on the next frame.
        if let Some(size) = current_size {
            ui.set_min_width(size.x);
            ui.set_max_width(size.x);
        }

        // Capture the actual node width before entering the right-to-left
        // sub-layout, which has its own coordinate space.
        let node_width = ui.available_width();

        // Resize grip in the bottom-right corner.
        let grip_size = 12.0;
        ui.with_layout(egui::Layout::right_to_left(egui::Align::BOTTOM), |ui| {
            let (grip_rect, response) =
                ui.allocate_exact_size(egui::vec2(grip_size, grip_size), egui::Sense::drag());

            // Draw diagonal grip lines.
            let color = if response.hovered() || response.dragged() {
                Color32::from_gray(180)
            } else {
                Color32::from_gray(100)
            };
            let painter = ui.painter();
            for i in 0..3 {
                let offset = i as f32 * 3.5;
                painter.line_segment(
                    [
                        egui::pos2(grip_rect.right() - offset, grip_rect.bottom()),
                        egui::pos2(grip_rect.right(), grip_rect.bottom() - offset),
                    ],
                    egui::Stroke::new(1.0, color),
                );
            }

            if response.dragged() {
                let delta = response.drag_delta();
                // Use the real node width as the base, not the inner layout's
                // min_rect which only reflects the grip's own allocation.
                let base = current_size.unwrap_or_else(|| egui::vec2(node_width.max(80.0), 200.0));
                let new_size =
                    egui::vec2((base.x + delta.x).max(80.0), (base.y + delta.y).max(40.0));
                snarl[node].set_custom_size(new_size);
            }
        });
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
        ("Park", SimNode::Park(ParkNode::default())),
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
fn show_plot_inline(
    _node_id: NodeId,
    inputs: &[InPin],
    ui: &mut Ui,
    snarl: &Snarl<SimNode>,
    plot_height: f32,
) {
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
        .height(plot_height)
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
