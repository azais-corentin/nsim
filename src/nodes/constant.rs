//! Constant source node — emits a fixed value with no inputs.
//!
//! The output type adapts when connected to a typed input (Scalar, Signal,
//! or Vector). For Vector outputs the node exposes three editable phase
//! values (a, b, c).

use egui::Color32;

use crate::port::{PortType, PortValue};

/// A source node that outputs a constant value.
///
/// When connected to a Signal or Vector input the node adapts its output
/// type automatically. For Vector outputs, `value` is phase-a, `value_b`
/// is phase-b, and `value_c` is phase-c.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ConstantNode {
    /// Primary value: scalar, signal amplitude, or a-phase for Vector.
    pub value: f64,
    /// Phase-b value (Vector output only).
    #[serde(default)]
    pub value_b: f64,
    /// Phase-c value (Vector output only).
    #[serde(default)]
    pub value_c: f64,
    /// Current output type — adapts when connected to a typed input.
    #[serde(default)]
    pub output_type: PortType,
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
    /// Cached output value produced by the solver for Signal/Vector modes.
    #[serde(skip)]
    pub output_port_value: Option<PortValue>,
}

impl Default for ConstantNode {
    fn default() -> Self {
        Self {
            value: 0.0,
            value_b: 0.0,
            value_c: 0.0,
            output_type: PortType::Scalar,
            custom_size: None,
            output_port_value: None,
        }
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

    /// Output port descriptor — type adapts based on the connected input.
    pub fn output_ports(&self) -> Vec<(&'static str, PortType)> {
        vec![("value", self.output_type)]
    }

    /// Header background color for this node category (neutral gray).
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x5A, 0x5A, 0x5A)
    }

    /// Adapt the output type to match a target input's type.
    pub fn adapt_output_type(&mut self, target: PortType) {
        self.output_type = target;
    }
}
