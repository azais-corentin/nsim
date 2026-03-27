//! Plot sink node — accepts one or more Signal inputs and renders them as time-series data.

use egui::Color32;

use crate::port::PortType;

/// A sink node that collects Signal inputs for plotting.
///
/// The number of inputs is configurable at runtime (1–8).
/// It has no outputs; it is a pure sink in the simulation graph.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct PlotNode {
    /// Number of Signal input pins. Minimum 1, maximum 8.
    pub num_inputs: usize,
}

impl Default for PlotNode {
    fn default() -> Self {
        Self { num_inputs: 1 }
    }
}

impl PlotNode {
    /// Display title shown in the node header.
    pub fn title() -> &'static str {
        "Plot"
    }

    /// Dynamic input port list: `num_inputs` pins, all of type [`PortType::Signal`].
    ///
    /// All pins share the label `"signal"` because egui-snarl distinguishes
    /// them by index, not label alone.
    pub fn input_ports(&self) -> Vec<(&'static str, PortType)> {
        vec![("signal", PortType::Signal); self.num_inputs]
    }

    /// Output port list — empty; this is a sink node.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[]
    }

    /// Header background color for this node type.
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x40, 0xB0, 0x40)
    }

    /// Adds one more input pin, up to the maximum of 8.
    pub fn add_input(&mut self) {
        const MAX_INPUTS: usize = 8;
        if self.num_inputs < MAX_INPUTS {
            self.num_inputs += 1;
        }
    }
}
