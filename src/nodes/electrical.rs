//! PMSM electrical dynamics node — models stator current equations in the d/q reference frame.

use egui::Color32;

use crate::port::{PortType, PortValue};

/// Node representing the PMSM electrical subsystem.
///
/// Implements the d/q-axis stator voltage equations:
/// ```text
/// L_d * di_d/dt = v_d - R_s * i_d + N_p * ω_m * L_q * i_q
/// L_q * di_q/dt = v_q - R_s * i_q - N_p * ω_m * (L_d * i_d + λ_m)
/// ```
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ElectricalNode {
    /// Stator resistance (Ω)
    pub r_s: f64,
    /// d-axis inductance (H)
    pub l_d: f64,
    /// q-axis inductance (H)
    pub l_q: f64,
    /// Permanent magnet flux linkage (Wb)
    pub lambda_m: f64,
    /// Number of pole pairs (stored as f64 for use in ODE expressions)
    pub n_p: f64,
    /// Initial d-axis current (A)
    pub i_d_0: f64,
    /// Initial q-axis current (A)
    pub i_q_0: f64,
    /// Output signal for d-axis current, populated after simulation
    #[serde(skip)]
    pub output_i_d: Option<PortValue>,
    /// Output signal for q-axis current, populated after simulation
    #[serde(skip)]
    pub output_i_q: Option<PortValue>,
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
}

impl Default for ElectricalNode {
    fn default() -> Self {
        Self {
            r_s: 1.2,
            l_d: 0.008,
            l_q: 0.008,
            lambda_m: 0.175,
            n_p: 4.0,
            i_d_0: 0.0,
            i_q_0: 0.0,
            output_i_d: None,
            output_i_q: None,
            custom_size: None,
        }
    }
}

impl ElectricalNode {
    /// Display name shown in the node graph header.
    pub fn title() -> &'static str {
        "PMSM Electrical"
    }

    /// Input port descriptors: scalar voltages and mechanical speed signal.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        &[
            ("v_d", PortType::Scalar),
            ("v_q", PortType::Scalar),
            ("ω_m", PortType::Signal),
        ]
    }

    /// Output port descriptors: d/q current waveforms as time-series signals.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[("i_d", PortType::Signal), ("i_q", PortType::Signal)]
    }

    /// Header color for this node in the graph canvas.
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0xB0, 0x40, 0x40)
    }
}
