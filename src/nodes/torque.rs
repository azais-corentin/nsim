//! Torque Calculator node — algebraic evaluation of electromagnetic torque.

use crate::port::{PortType, PortValue};
use egui::Color32;

/// Computes electromagnetic torque from d/q currents.
///
/// Equation: `T_e = (3/2) * N_p * (λ_m * i_q + (L_d - L_q) * i_d * i_q)`
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct TorqueNode {
    /// Number of pole pairs.
    pub n_p: f64,
    /// Permanent-magnet flux linkage (Wb).
    pub lambda_m: f64,
    /// d-axis inductance (H).
    pub l_d: f64,
    /// q-axis inductance (H).
    pub l_q: f64,
    /// Most-recently computed torque signal; skipped during serialization.
    #[serde(skip)]
    pub output_t_e: Option<PortValue>,
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
}

impl Default for TorqueNode {
    fn default() -> Self {
        Self {
            n_p: 4.0,
            lambda_m: 0.175,
            l_d: 0.008,
            l_q: 0.008,
            output_t_e: None,
            custom_size: None,
        }
    }
}

impl TorqueNode {
    /// Display name shown in the node graph header.
    pub fn title() -> &'static str {
        "Torque Calculator"
    }

    /// Input port descriptors: d-axis current and q-axis current.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        &[("i_d", PortType::Signal), ("i_q", PortType::Signal)]
    }

    /// Output port descriptors: electromagnetic torque signal.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[("T_e", PortType::Signal)]
    }

    /// Node header colour (yellow-ish, distinguishing algebraic torque nodes).
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0xB0, 0x80, 0x20)
    }

    /// Evaluate electromagnetic torque at a single time instant.
    ///
    /// # Arguments
    /// * `i_d` — d-axis stator current (A)
    /// * `i_q` — q-axis stator current (A)
    ///
    /// # Returns
    /// Electromagnetic torque `T_e` in N·m.
    pub fn compute(&self, i_d: f64, i_q: f64) -> f64 {
        // T_e = (3/2) * N_p * (λ_m * i_q + (L_d - L_q) * i_d * i_q)
        1.5 * self.n_p * (self.lambda_m * i_q + (self.l_d - self.l_q) * i_d * i_q)
    }
}
