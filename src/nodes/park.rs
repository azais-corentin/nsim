//! Park transform nodes — forward (ABC→dq) and inverse (dq→ABC) algebraic post-processing.

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
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
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


/// Transforms stationary ABC three-phase signals into d/q rotating-frame signals.
///
/// This is the forward Park transform, the inverse of [`InverseParkNode`].
/// Equations evaluated pointwise over the time series (amplitude-invariant form):
/// - `f_d =  (2/3) · [f_a·cos(θ_e) + f_b·cos(θ_e − 2π/3) + f_c·cos(θ_e + 2π/3)]`
/// - `f_q = −(2/3) · [f_a·sin(θ_e) + f_b·sin(θ_e − 2π/3) + f_c·sin(θ_e + 2π/3)]`
#[derive(Clone, Default, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct ParkNode {
    /// Most-recently computed d-axis output; skipped during serialization.
    #[serde(skip)]
    pub output_f_d: Option<PortValue>,
    /// Most-recently computed q-axis output; skipped during serialization.
    #[serde(skip)]
    pub output_f_q: Option<PortValue>,
    /// User-defined node size override (width, height).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub custom_size: Option<[f32; 2]>,
}

impl ParkNode {
    /// Display name shown in the node graph header.
    pub fn title() -> &'static str {
        "Park"
    }

    /// Input port descriptors: three-phase ABC vector and electrical angle.
    pub fn input_ports() -> &'static [(&'static str, PortType)] {
        &[("f_abc", PortType::Vector), ("θ_e", PortType::Signal)]
    }

    /// Output port descriptors: d-axis and q-axis signal components.
    pub fn output_ports() -> &'static [(&'static str, PortType)] {
        &[("f_d", PortType::Signal), ("f_q", PortType::Signal)]
    }

    /// Node header colour (purple, matching other transform nodes).
    pub fn header_color() -> Color32 {
        Color32::from_rgb(0x80, 0x40, 0xB0)
    }

    /// Evaluate the forward Park transform pointwise over the time series.
    ///
    /// # Arguments
    /// * `f_abc`   — three-phase vector signal: `[t, f_a, f_b, f_c]` per time step
    /// * `theta_e` — electrical angle signal: `[t, value]` in radians per time step
    ///
    /// # Returns
    /// A tuple `(f_d, f_q)` where each is a `Vec<[f64; 2]>` of `[t, value]` entries.
    pub fn compute(f_abc: &[[f64; 4]], theta_e: &[[f64; 2]]) -> (Vec<[f64; 2]>, Vec<[f64; 2]>) {
        let two_thirds_pi = std::f64::consts::TAU / 3.0;
        let two_thirds = 2.0 / 3.0;

        let (f_d, f_q): (Vec<_>, Vec<_>) = f_abc
            .iter()
            .zip(theta_e.iter())
            .map(|([t, a, b, c], [_t_th, th])| {
                let cos0 = th.cos();
                let sin0 = th.sin();
                let cos_b = (th - two_thirds_pi).cos();
                let sin_b = (th - two_thirds_pi).sin();
                let cos_c = (th + two_thirds_pi).cos();
                let sin_c = (th + two_thirds_pi).sin();

                let d = two_thirds * (a * cos0 + b * cos_b + c * cos_c);
                let q = -two_thirds * (a * sin0 + b * sin_b + c * sin_c);

                ([*t, d], [*t, q])
            })
            .unzip();

        (f_d, f_q)
    }
}

#[cfg(test)]
mod tests {
    use super::{InverseParkNode, ParkNode};

    /// Roundtrip: InversePark(d,q,θ) → ABC → Park(ABC,θ) should recover (d,q).
    #[expect(clippy::indexing_slicing, reason = "test assertions with known-length slices")]
    #[test]
    fn park_inverse_park_roundtrip() {
        let n = 50;
        let f_d_in: Vec<[f64; 2]> = (0..n)
            .map(|i| {
                let t = i as f64 * 0.001;
                [t, 10.0]
            })
            .collect();
        let f_q_in: Vec<[f64; 2]> = (0..n)
            .map(|i| {
                let t = i as f64 * 0.001;
                [t, -5.0]
            })
            .collect();
        let theta_e: Vec<[f64; 2]> = (0..n)
            .map(|i| {
                let t = i as f64 * 0.001;
                // Sweep angle over a full electrical revolution.
                [t, t * std::f64::consts::TAU / (n as f64 * 0.001)]
            })
            .collect();

        // dq → abc
        let f_abc = InverseParkNode::compute(&f_d_in, &f_q_in, &theta_e);

        // abc → dq
        let (f_d_out, f_q_out) = ParkNode::compute(&f_abc, &theta_e);

        let epsilon = 1e-10;
        for (i, (d_out, q_out)) in f_d_out.iter().zip(f_q_out.iter()).enumerate() {
            assert!(
                (d_out[1] - f_d_in[i][1]).abs() < epsilon,
                "f_d mismatch at step {i}: expected {}, got {}",
                f_d_in[i][1],
                d_out[1],
            );
            assert!(
                (q_out[1] - f_q_in[i][1]).abs() < epsilon,
                "f_q mismatch at step {i}: expected {}, got {}",
                f_q_in[i][1],
                q_out[1],
            );
        }
    }
}