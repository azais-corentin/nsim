//! Constant source node — emits a fixed scalar value with no inputs.

use egui::Color32;

use crate::port::PortType;

/// A source node that outputs a single constant scalar value.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ConstantNode {
    /// The constant value emitted on the output port.
    pub value: f64,
}

impl Default for ConstantNode {
    fn default() -> Self {
        Self { value: 0.0 }
    }
}

impl ConstantNode {
    /// Display title shown in the node header.
    pub fn title() -> &'static str {
        "Constant"
    }

    /// This node has no inputs.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        &[]
    }

    /// Outputs a single scalar port labelled `"value"`.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[("value", PortType::Scalar)]
    }

    /// Header background color for this node category (neutral gray).
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x5A, 0x5A, 0x5A)
    }
}
