//! Mechanical dynamics node for the PMSM simulation graph.
//!
//! Models the rotor mechanical subsystem via two coupled ODEs:
//! - `dω_m/dt = (1/J) · (T_e − T_L − B·ω_m)`
//! - `dθ_e/dt = N_p · ω_m`

use egui::Color32;

use crate::port::{PortType, PortValue};

/// ODE node representing the mechanical dynamics of a PMSM rotor.
///
/// Inputs: electromagnetic torque `T_e` (Signal) and load torque `T_L` (Signal).\
/// Outputs: mechanical speed `ω_m` (Signal) and electrical angle `θ_e` (Signal).
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct MechanicalNode {
    /// Moment of inertia (kg·m²).
    pub j: f64,
    /// Viscous friction coefficient (N·m·s/rad).
    pub b: f64,
    /// Number of pole pairs.
    pub n_p: f64,
    /// Initial mechanical speed (rad/s).
    pub omega_m_0: f64,
    /// Initial electrical angle (rad).
    pub theta_e_0: f64,
    /// Mechanical speed time-series produced after simulation.
    #[serde(skip)]
    pub output_omega_m: Option<PortValue>,
    /// Electrical angle time-series produced after simulation.
    #[serde(skip)]
    pub output_theta_e: Option<PortValue>,
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
}

impl Default for MechanicalNode {
    fn default() -> Self {
        Self {
            j: 0.0008,
            b: 0.001,
            n_p: 4.0,
            omega_m_0: 0.0,
            theta_e_0: 0.0,
            output_omega_m: None,
            output_theta_e: None,
            custom_size: None,
        }
    }
}

/// Static input port descriptors: (name, type).
const INPUT_PORTS: &[(&str, PortType)] = &[("T_e", PortType::Signal), ("T_L", PortType::Signal)];

/// Static output port descriptors: (name, type).
const OUTPUT_PORTS: &[(&str, PortType)] = &[("ω_m", PortType::Signal), ("θ_e", PortType::Signal)];

impl MechanicalNode {
    /// Display title shown in the node header.
    pub fn title() -> &'static str {
        "Mechanical Dynamics"
    }

    /// Input port descriptors in connection order.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        INPUT_PORTS
    }

    /// Output port descriptors in connection order.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        OUTPUT_PORTS
    }

    /// Header background color distinguishing this node category.
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x40, 0x40, 0xB0)
    }
}
