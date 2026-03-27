//! Inverse Park Transform node — algebraic post-processing of d/q signals to three-phase ABC.

use crate::port::{PortType, PortValue};
use egui::Color32;

/// Transforms d/q rotating-frame signals into stationary ABC three-phase signals.
///
/// Equations evaluated pointwise over the time series:
/// - `f_a = f_d·cos(θ_e) − f_q·sin(θ_e)`
/// - `f_b = f_d·cos(θ_e − 2π/3) − f_q·sin(θ_e − 2π/3)`
/// - `f_c = f_d·cos(θ_e + 2π/3) − f_q·sin(θ_e + 2π/3)`
#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct InverseParkNode {
    /// Most-recently computed ABC output; skipped during serialization.
    #[serde(skip)]
    pub output_f_abc: Option<PortValue>,
}

impl InverseParkNode {
    /// Display name shown in the node graph header.
    pub fn title() -> &'static str {
        "Inverse Park"
    }

    /// Input port descriptors: d-axis component, q-axis component, and electrical angle.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        &[
            ("f_d", PortType::Signal),
            ("f_q", PortType::Signal),
            ("θ_e", PortType::Signal),
        ]
    }

    /// Output port descriptors: three-phase ABC vector signal.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[("f_abc", PortType::Vector)]
    }

    /// Node header colour (purple, distinguishing post-processing nodes).
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x80, 0x40, 0xB0)
    }

    /// Evaluate the inverse Park transform pointwise over the time series.
    ///
    /// All three input slices share the same time vector from the ODE solver;
    /// they are zipped by index. Each element is a `[t, value]` pair.
    ///
    /// # Arguments
    /// * `f_d`     — d-axis signal: `[t, value]` per time step
    /// * `f_q`     — q-axis signal: `[t, value]` per time step
    /// * `theta_e` — electrical angle signal: `[t, value]` in radians per time step
    ///
    /// # Returns
    /// A vector of `[t, f_a, f_b, f_c]` entries, one per time step.
    pub fn compute(f_d: &[[f64; 2]], f_q: &[[f64; 2]], theta_e: &[[f64; 2]]) -> Vec<[f64; 4]> {
        // 2π/3 offset between phases.
        let two_thirds_pi = std::f64::consts::TAU / 3.0;

        f_d.iter()
            .zip(f_q.iter())
            .zip(theta_e.iter())
            .map(|(([t, d], [_t_q, q]), [_t_th, th])| {
                let cos0 = th.cos();
                let sin0 = th.sin();
                let cos_b = (th - two_thirds_pi).cos();
                let sin_b = (th - two_thirds_pi).sin();
                let cos_c = (th + two_thirds_pi).cos();
                let sin_c = (th + two_thirds_pi).sin();

                let f_a = d * cos0 - q * sin0;
                let f_b = d * cos_b - q * sin_b;
                let f_c = d * cos_c - q * sin_c;

                [*t, f_a, f_b, f_c]
            })
            .collect()
    }
}
