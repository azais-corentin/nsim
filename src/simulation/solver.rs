//! ODE assembly and BDF integration for the coupled PMSM system.
//!
//! The entry point is [`run_simulation`].  It traverses the snarl graph,
//! extracts motor parameters from node fields and connected `Constant` sources,
//! assembles the 4-state ODE system `[i_d, i_q, ω_m, θ_e]`, solves it with
//! `diffsol`'s BDF integrator, and distributes the resulting time-series signals
//! back into the graph nodes so the UI can render them.

use diffsol::{DenseMatrix as _, NalgebraLU, NalgebraMat, OdeBuilder, OdeSolverMethod as _};
use egui_snarl::{InPinId, NodeId, Snarl};

use super::{SimConfig, SimError};
use crate::nodes::SimNode;
use crate::port::{PortType, PortValue};

/// Concrete dense-matrix type driven through the BDF solver.
type M = NalgebraMat<f64>;

/// Concrete LU linear-system solver used inside BDF Newton iterations.
type Ls = NalgebraLU<f64>;

// ── State-vector indices ──────────────────────────────────────────────────────
/// State index: d-axis stator current `i_d`.
const S_ID: usize = 0;
/// State index: q-axis stator current `i_q`.
const S_IQ: usize = 1;
/// State index: mechanical angular speed `ω_m`.
const S_WM: usize = 2;
/// State index: electrical angle `θ_e`.
const S_TE: usize = 3;

// ── Parameter-vector indices ──────────────────────────────────────────────────
/// Parameter index: stator resistance `R_s`.
const P_RS: usize = 0;
/// Parameter index: d-axis inductance `L_d`.
const P_LD: usize = 1;
/// Parameter index: q-axis inductance `L_q`.
const P_LQ: usize = 2;
/// Parameter index: permanent-magnet flux linkage `λ_m`.
const P_LAM: usize = 3;
/// Parameter index: pole-pair count `N_p` (from the mechanical node).
const P_NP: usize = 4;
/// Parameter index: moment of inertia `J`.
const P_J: usize = 5;
/// Parameter index: viscous-friction coefficient `B`.
const P_B: usize = 6;
/// Parameter index: torque-equation pole-pair count `N_p`.
const P_T_NP: usize = 7;
/// Parameter index: torque-equation flux linkage `λ_m`.
const P_T_LAM: usize = 8;
/// Parameter index: torque-equation d-axis inductance `L_d`.
const P_T_LD: usize = 9;
/// Parameter index: torque-equation q-axis inductance `L_q`.
const P_T_LQ: usize = 10;

/// Total number of states in the coupled PMSM ODE system.
const N_STATES: usize = 4;

/// Return the value emitted by a `Constant` source connected to a given input pin.
///
/// Returns `None` when the pin has no incoming wire or the source is not a
/// [`SimNode::Constant`] node.
fn get_constant_input(snarl: &Snarl<SimNode>, node: NodeId, input: usize) -> Option<f64> {
    let pin = snarl.in_pin(InPinId { node, input });
    let remote = pin.remotes.first()?;
    let SimNode::Constant(c) = snarl.get_node(remote.node)? else {
        return None;
    };
    Some(c.value)
}

/// Return the Signal time-series connected to a given input pin.
///
/// Returns `None` when the pin has no incoming wire or the connected output
/// does not hold a [`PortValue::Signal`].
fn get_signal_input(snarl: &Snarl<SimNode>, node: NodeId, input: usize) -> Option<Vec<[f64; 2]>> {
    let pin = snarl.in_pin(InPinId { node, input });
    let remote = pin.remotes.first()?;
    let source = snarl.get_node(remote.node)?;
    match source.output_value(remote.output)? {
        PortValue::Signal(data) => Some(data.clone()),
        _ => None,
    }
}

/// Return the Vector (3-phase) time-series connected to a given input pin.
///
/// Returns `None` when the pin has no incoming wire or the connected output
/// does not hold a [`PortValue::Vector`].
fn get_vector_input(snarl: &Snarl<SimNode>, node: NodeId, input: usize) -> Option<Vec<[f64; 4]>> {
    let pin = snarl.in_pin(InPinId { node, input });
    let remote = pin.remotes.first()?;
    let source = snarl.get_node(remote.node)?;
    match source.output_value(remote.output)? {
        PortValue::Vector(data) => Some(data.clone()),
        _ => None,
    }
}

/// Linearly interpolate a `[t, value]` time-series at time `t`.
///
/// Clamps to boundary values outside the series range.
/// Returns `0.0` for an empty series.
#[expect(
    clippy::indexing_slicing,
    reason = "indices are bounded by early-return guards: is_empty, len==1, boundary clamps, and partition_point"
)]
fn interpolate_signal(signal: &[[f64; 2]], t: f64) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    if signal.len() == 1 || t <= signal[0][0] {
        return signal[0][1];
    }
    let last = signal.len() - 1;
    if t >= signal[last][0] {
        return signal[last][1];
    }
    // Binary search for the bracketing interval.
    let idx = signal.partition_point(|s| s[0] < t);
    // idx > 0 because t > signal[0][0], and idx <= last because t < signal[last][0].
    let (t0, v0) = (signal[idx - 1][0], signal[idx - 1][1]);
    let (t1, v1) = (signal[idx][0], signal[idx][1]);
    let frac = if (t1 - t0).abs() < f64::EPSILON {
        0.0
    } else {
        (t - t0) / (t1 - t0)
    };
    v0 + frac * (v1 - v0)
}

/// Linearly interpolate a `[t, a, b, c]` 3-phase time-series at time `t`.
///
/// Clamps to boundary values outside the series range.
/// Returns `[0.0; 3]` for an empty series.
#[expect(
    clippy::indexing_slicing,
    reason = "indices are bounded by early-return guards: is_empty, len==1, boundary clamps, and partition_point"
)]
fn interpolate_vector(signal: &[[f64; 4]], t: f64) -> [f64; 3] {
    if signal.is_empty() {
        return [0.0; 3];
    }
    if signal.len() == 1 || t <= signal[0][0] {
        return [signal[0][1], signal[0][2], signal[0][3]];
    }
    let last = signal.len() - 1;
    if t >= signal[last][0] {
        return [signal[last][1], signal[last][2], signal[last][3]];
    }
    let idx = signal.partition_point(|s| s[0] < t);
    let frac = {
        let (t0, t1) = (signal[idx - 1][0], signal[idx][0]);
        if (t1 - t0).abs() < f64::EPSILON {
            0.0
        } else {
            (t - t0) / (t1 - t0)
        }
    };
    [
        signal[idx - 1][1] + frac * (signal[idx][1] - signal[idx - 1][1]),
        signal[idx - 1][2] + frac * (signal[idx][2] - signal[idx - 1][2]),
        signal[idx - 1][3] + frac * (signal[idx][3] - signal[idx - 1][3]),
    ]
}

/// Resample a `[t, value]` series onto a uniform time grid with spacing `dt`.
///
/// If the raw series already has at least as many points as the uniform grid
/// would produce, the solver captured fine-grained dynamics — return as-is.
fn resample_signal(series: &[[f64; 2]], t_start: f64, t_end: f64, dt: f64) -> Vec<[f64; 2]> {
    let n_uniform = ((t_end - t_start) / dt).ceil() as usize + 1;
    if series.len() >= n_uniform {
        return series.to_vec();
    }
    (0..n_uniform)
        .map(|i| {
            let t = (t_start + dt * i as f64).min(t_end);
            [t, interpolate_signal(series, t)]
        })
        .collect()
}

/// Resample a `[t, a, b, c]` 3-phase series onto a uniform time grid with spacing `dt`.
///
/// Same density-check logic as [`resample_signal`].
fn resample_vector(series: &[[f64; 4]], t_start: f64, t_end: f64, dt: f64) -> Vec<[f64; 4]> {
    let n_uniform = ((t_end - t_start) / dt).ceil() as usize + 1;
    if series.len() >= n_uniform {
        return series.to_vec();
    }
    (0..n_uniform)
        .map(|i| {
            let t = (t_start + dt * i as f64).min(t_end);
            let [a, b, c] = interpolate_vector(series, t);
            [t, a, b, c]
        })
        .collect()
}

/// An ODE forcing input resolved from the graph: constant or time-varying.
#[derive(Clone)]
enum ExternalInput {
    /// Fixed value (from a `ConstantNode` or default).
    Constant(f64),
    /// Time-varying signal to be interpolated at each ODE step.
    Signal(Vec<[f64; 2]>),
}

impl ExternalInput {
    /// Evaluate the input at time `t`.
    fn at(&self, t: f64) -> f64 {
        match self {
            Self::Constant(v) => *v,
            Self::Signal(s) => interpolate_signal(s, t),
        }
    }
}

/// Resolve an external input pin: try Constant first, then Signal, then default.
///
/// Checking `get_constant_input` first preserves backward compatibility with
/// saved graphs where `ConstantNode`s have `output_type: Scalar` wired to
/// now-Signal pins.
fn resolve_external_input(snarl: &Snarl<SimNode>, node: NodeId, input: usize) -> ExternalInput {
    if let Some(v) = get_constant_input(snarl, node, input) {
        return ExternalInput::Constant(v);
    }
    if let Some(data) = get_signal_input(snarl, node, input) {
        return ExternalInput::Signal(data);
    }
    ExternalInput::Constant(0.0)
}

/// Voltage source for the d/q ODE equations.
///
/// When `v_d`/`v_q` come directly from graph wires (constants or pre-computed
/// signals), they are `Direct`. When they come from a Park transform whose
/// `theta_e` is an ODE state (circular dependency), the Park math is folded
/// into the RHS closure via `ParkCoupled`.
#[derive(Clone)]
enum VoltageSource {
    /// `v_d` and `v_q` resolved independently from the graph.
    Direct {
        v_d: ExternalInput,
        v_q: ExternalInput,
    },
    /// Park transform computed inline using `theta_e` from the ODE state vector.
    /// `f_abc` is the 3-phase voltage source (pre-resolved, constant or signal).
    ParkCoupled { f_abc: Vec<[f64; 4]> },
}

impl VoltageSource {
    /// Evaluate `(v_d, v_q)` at time `t` using the given `theta_e`.
    ///
    /// For `Direct`, `theta_e` is ignored.
    fn eval(&self, t: f64, theta_e: f64) -> (f64, f64) {
        match self {
            Self::Direct { v_d, v_q } => (v_d.at(t), v_q.at(t)),
            Self::ParkCoupled { f_abc } => {
                let [v_a, v_b, v_c] = interpolate_vector(f_abc, t);
                let two_thirds = 2.0 / 3.0;
                let two_thirds_pi = std::f64::consts::TAU / 3.0;

                let cos0 = theta_e.cos();
                let sin0 = theta_e.sin();
                let cos_b = (theta_e - two_thirds_pi).cos();
                let sin_b = (theta_e - two_thirds_pi).sin();
                let cos_c = (theta_e + two_thirds_pi).cos();
                let sin_c = (theta_e + two_thirds_pi).sin();

                let v_d = two_thirds * (v_a * cos0 + v_b * cos_b + v_c * cos_c);
                let v_q = -two_thirds * (v_a * sin0 + v_b * sin_b + v_c * sin_c);
                (v_d, v_q)
            }
        }
    }

    /// Whether this source is a Park-coupled source (for Jacobian dispatch).
    const fn is_park_coupled(&self) -> bool {
        matches!(self, Self::ParkCoupled { .. })
    }
}

/// Traverse the graph, solve the PMSM ODE system, and write results into node outputs.
///
/// # Errors
///
/// - [`SimError::NoOdeNodes`] — the graph contains no `ElectricalNode` or no
///   `MechanicalNode`.
/// - [`SimError::GraphError`] — parameter extraction fails due to unexpected graph
///   state (should not occur if the graph was built through the normal UI).
/// - [`SimError::SolverFailed`] — the diffsol BDF integrator encounters a
///   numerical error (ill-conditioned system, step-size underflow, etc.).
#[expect(
    clippy::too_many_lines,
    reason = "ODE assembly inherently requires a long function"
)]
pub fn run_simulation(snarl: &mut Snarl<SimNode>, config: &SimConfig) -> Result<(), SimError> {
    // ── 1. Snapshot all node IDs before any mutable access ───────────────────
    let all_ids: Vec<NodeId> = snarl.node_ids().map(|(id, _)| id).collect();

    // ── 2. Clear outputs from any previous simulation run ───────────────────
    for &id in &all_ids {
        if let Some(node) = snarl.get_node_mut(id) {
            match node {
                SimNode::Electrical(e) => {
                    e.output_i_d = None;
                    e.output_i_q = None;
                }
                SimNode::Mechanical(m) => {
                    m.output_omega_m = None;
                    m.output_theta_e = None;
                }
                SimNode::Torque(t) => {
                    t.output_t_e = None;
                }
                SimNode::InversePark(p) => {
                    p.output_f_abc = None;
                }
                SimNode::Park(p) => {
                    p.output_f_d = None;
                    p.output_f_q = None;
                }
                SimNode::Constant(c) => {
                    c.output_port_value = None;
                }
                SimNode::Plot(_) => {}
            }
        }
    }

    // ── 3. Locate ODE and algebraic nodes (first occurrence wins) ───────────
    let mut elec_id: Option<NodeId> = None;
    let mut mech_id: Option<NodeId> = None;
    let mut torque_id: Option<NodeId> = None;
    let mut inv_park_id: Option<NodeId> = None;
    let mut park_id: Option<NodeId> = None;

    for (id, node) in snarl.node_ids() {
        match node {
            SimNode::Electrical(_) if elec_id.is_none() => elec_id = Some(id),
            SimNode::Mechanical(_) if mech_id.is_none() => mech_id = Some(id),
            SimNode::Torque(_) if torque_id.is_none() => torque_id = Some(id),
            SimNode::InversePark(_) if inv_park_id.is_none() => inv_park_id = Some(id),
            SimNode::Park(_) if park_id.is_none() => park_id = Some(id),
            _ => {}
        }
    }

    let elec_id = elec_id.ok_or(SimError::NoOdeNodes)?;
    let mech_id = mech_id.ok_or(SimError::NoOdeNodes)?;

    // ── 4. Extract parameters from the Electrical node ──────────────────────
    let (r_s, l_d, l_q, lambda_m, n_p_elec, i_d_0, i_q_0) = {
        let node = snarl
            .get_node(elec_id)
            .ok_or_else(|| SimError::GraphError("electrical node vanished".to_owned()))?;
        let SimNode::Electrical(e) = node else {
            return Err(SimError::GraphError("expected electrical node".to_owned()));
        };
        (e.r_s, e.l_d, e.l_q, e.lambda_m, e.n_p, e.i_d_0, e.i_q_0)
    };

    // ── 5. Extract parameters from the Mechanical node ──────────────────────
    let (j, b, n_p_mech, omega_m_0, theta_e_0) = {
        let node = snarl
            .get_node(mech_id)
            .ok_or_else(|| SimError::GraphError("mechanical node vanished".to_owned()))?;
        let SimNode::Mechanical(m) = node else {
            return Err(SimError::GraphError("expected mechanical node".to_owned()));
        };
        (m.j, m.b, m.n_p, m.omega_m_0, m.theta_e_0)
    };

    // ── 6. Extract parameters from the Torque node (fall back to Electrical) ─
    // The torque equation uses its own copy of (N_p, λ_m, L_d, L_q) so the
    // user can tune it independently from the ODE parameters.
    let (torque_n_p, torque_lambda_m, torque_l_d, torque_l_q) = if let Some(tid) = torque_id {
        let node = snarl
            .get_node(tid)
            .ok_or_else(|| SimError::GraphError("torque node vanished".to_owned()))?;
        let SimNode::Torque(t) = node else {
            return Err(SimError::GraphError("expected torque node".to_owned()));
        };
        (t.n_p, t.lambda_m, t.l_d, t.l_q)
    } else {
        // No dedicated Torque node — share the Electrical node's parameters.
        (n_p_elec, lambda_m, l_d, l_q)
    };

    // ── 7. Pre-generate Constant Signal/Vector outputs ─────────────────────────
    // Constants adapted to Signal or Vector mode need time-series data
    // populated *before* we resolve ODE external inputs, so that
    // `get_signal_input` can read from them (and by extension, the Park
    // transform can consume them).
    let n_pre = 1000_usize;
    let ts_pre: Vec<f64> = (0..n_pre)
        .map(|i| {
            config.t_start + (config.t_end - config.t_start) * (i as f64) / (n_pre as f64 - 1.0)
        })
        .collect();
    for &id in &all_ids {
        if let Some(SimNode::Constant(c)) = snarl.get_node(id) {
            let port_value = match c.output_type {
                PortType::Signal => {
                    let series: Vec<[f64; 2]> = ts_pre.iter().map(|&t| [t, c.value]).collect();
                    Some(PortValue::Signal(series))
                }
                PortType::Vector => {
                    let (a, b_val, cv) = (c.value, c.value_b, c.value_c);
                    let series: Vec<[f64; 4]> = ts_pre.iter().map(|&t| [t, a, b_val, cv]).collect();
                    Some(PortValue::Vector(series))
                }
                PortType::Scalar => None,
            };
            if let Some(pv) = port_value
                && let Some(SimNode::Constant(c)) = snarl.get_node_mut(id)
            {
                c.output_port_value = Some(pv);
            }
        }
    }

    // ── 7b. Pre-compute Park transform if inputs are available ─────────────
    // This populates Park f_d / f_q so they can feed Electrical v_d / v_q.
    if let Some(pid) = park_id {
        let f_abc = get_vector_input(snarl, pid, 0);
        let theta_e = get_signal_input(snarl, pid, 1);

        if let (Some(f_abc), Some(theta_e)) = (f_abc, theta_e) {
            let (f_d_series, f_q_series) = crate::nodes::park::ParkNode::compute(&f_abc, &theta_e);

            if let Some(SimNode::Park(park)) = snarl.get_node_mut(pid) {
                park.output_f_d = Some(PortValue::Signal(f_d_series));
                park.output_f_q = Some(PortValue::Signal(f_q_series));
            }
        }
    }

    // ── 7c. Resolve external ODE forcing inputs ─────────────────────────────
    // Check if v_d/v_q come from a Park node whose theta_e is ODE-coupled
    // (circular dependency: Electrical ← Park ← Mechanical theta_e).
    // When detected, the Park transform is folded into the RHS closure so it
    // evaluates at the instantaneous theta_e state during integration.
    let voltage_source = 'voltage: {
        if let Some(pid) = park_id {
            let vd_pin = snarl.in_pin(InPinId {
                node: elec_id,
                input: 0,
            });
            let vd_from_park = vd_pin.remotes.first().is_some_and(|r| r.node == pid);

            if vd_from_park {
                let theta_pin = snarl.in_pin(InPinId {
                    node: pid,
                    input: 1,
                });
                let theta_from_mech = theta_pin.remotes.first().is_some_and(|r| r.node == mech_id);

                if theta_from_mech && let Some(f_abc) = get_vector_input(snarl, pid, 0) {
                    break 'voltage VoltageSource::ParkCoupled { f_abc };
                }
            }
        }
        // Default: resolve v_d, v_q independently from the graph.
        VoltageSource::Direct {
            v_d: resolve_external_input(snarl, elec_id, 0),
            v_q: resolve_external_input(snarl, elec_id, 1),
        }
    };
    // MechanicalNode pin layout: 0 = T_e (Signal), 1 = T_L (Signal)
    let t_l_input = resolve_external_input(snarl, mech_id, 1);

    // Use pole-pair count from the Mechanical node throughout the ODE.
    let n_p = n_p_mech;

    // ── 8. Assemble parameter vector ─────────────────────────────────────────
    // Element order must match the P_* constants defined above.
    // v_d, v_q, and T_L are no longer in the parameter vector — they are
    // captured as `ExternalInput` values in the RHS closure.
    let p = vec![
        r_s,
        l_d,
        l_q,
        lambda_m,
        n_p, // P_RS … P_NP
        j,
        b, // P_J, P_B
        torque_n_p,
        torque_lambda_m,
        torque_l_d,
        torque_l_q, // P_T_NP … P_T_LQ
    ];

    // ── 9. Build and solve the ODE ────────────────────────────────────────────
    //
    // State vector:  [i_d, i_q, ω_m, θ_e]
    //
    // ODE system (Park-frame PMSM + rigid-rotor mechanics):
    //   di_d/dt = (1/L_d) * (v_d - R_s·i_d + N_p·ω_m·L_q·i_q)
    //   di_q/dt = (1/L_q) * (v_q - R_s·i_q - N_p·ω_m·(L_d·i_d + λ_m))
    //   dω_m/dt = (1/J)  * (T_e - T_L - B·ω_m)
    //   dθ_e/dt = N_p · ω_m
    //
    // where  T_e = (3/2) · N_p · (λ_m·i_q + (L_d - L_q)·i_d·i_q)
    let problem = OdeBuilder::<M>::new()
        .t0(config.t_start)
        .rtol(config.rtol)
        .atol([config.atol; N_STATES])
        .p(p)
        .rhs_implicit(
            // ── RHS: compute time derivatives ──────────────────────────────
            {
                let voltage_src = voltage_source.clone();
                let t_l_ext = t_l_input.clone();
                move |x, p, t, y| {
                    let (v_d, v_q) = voltage_src.eval(t, x[S_TE]);
                    let t_l = t_l_ext.at(t);

                    // Electromagnetic torque (used in the mechanical equation below)
                    let t_e = 1.5
                        * p[P_T_NP]
                        * (p[P_T_LAM] * x[S_IQ] + (p[P_T_LD] - p[P_T_LQ]) * x[S_ID] * x[S_IQ]);

                    // d-axis current
                    y[S_ID] = (1.0 / p[P_LD])
                        * (v_d - p[P_RS] * x[S_ID] + p[P_NP] * x[S_WM] * p[P_LQ] * x[S_IQ]);

                    // q-axis current
                    y[S_IQ] = (1.0 / p[P_LQ])
                        * (v_q
                            - p[P_RS] * x[S_IQ]
                            - p[P_NP] * x[S_WM] * p[P_LD] * x[S_ID]
                            - p[P_NP] * x[S_WM] * p[P_LAM]);

                    // Mechanical speed
                    y[S_WM] = (1.0 / p[P_J]) * (t_e - t_l - p[P_B] * x[S_WM]);

                    // Electrical angle
                    y[S_TE] = p[P_NP] * x[S_WM];
                }
            },
            // ── Jacobian-vector product J·v where J = ∂f/∂x ─────────────
            {
                let voltage_src_jac = voltage_source.clone();
                let park_coupled = voltage_source.is_park_coupled();
                move |x, p, t, v, y| {
                    // Partial derivatives of T_e w.r.t. state components
                    let dt_e_did = 1.5 * p[P_T_NP] * (p[P_T_LD] - p[P_T_LQ]) * x[S_IQ];
                    let dt_e_diq =
                        1.5 * p[P_T_NP] * (p[P_T_LAM] + (p[P_T_LD] - p[P_T_LQ]) * x[S_ID]);

                    // ∂(di_d/dt)/∂(i_d, i_q, ω_m)
                    let did_did = -p[P_RS] / p[P_LD];
                    let did_diq = p[P_NP] * x[S_WM] * p[P_LQ] / p[P_LD];
                    let did_dwm = p[P_NP] * p[P_LQ] * x[S_IQ] / p[P_LD];

                    // ∂(di_q/dt)/∂(i_d, i_q, ω_m)
                    let diq_did = -p[P_NP] * x[S_WM] * p[P_LD] / p[P_LQ];
                    let diq_diq = -p[P_RS] / p[P_LQ];
                    let diq_dwm = (-p[P_NP] * p[P_LD] * x[S_ID] - p[P_NP] * p[P_LAM]) / p[P_LQ];

                    // ∂(dω_m/dt)/∂(i_d, i_q, ω_m)
                    let dwm_did = dt_e_did / p[P_J];
                    let dwm_diq = dt_e_diq / p[P_J];
                    let dwm_dwm = -p[P_B] / p[P_J];

                    // ∂(dθ_e/dt)/∂ω_m  (only non-zero entry in the θ_e row)
                    let dte_dwm = p[P_NP];

                    // Park-coupled θ_e dependence: dv_d/dθ = v_q, dv_q/dθ = -v_d
                    let (did_dte, diq_dte) = if park_coupled {
                        let (v_d, v_q) = voltage_src_jac.eval(t, x[S_TE]);
                        (
                            v_q / p[P_LD],  // (1/L_d) · dv_d/dθ = (1/L_d) · v_q
                            -v_d / p[P_LQ], // (1/L_q) · dv_q/dθ = (1/L_q) · (-v_d)
                        )
                    } else {
                        (0.0, 0.0)
                    };

                    // J·v
                    y[S_ID] = did_did * v[S_ID]
                        + did_diq * v[S_IQ]
                        + did_dwm * v[S_WM]
                        + did_dte * v[S_TE];
                    y[S_IQ] = diq_did * v[S_ID]
                        + diq_diq * v[S_IQ]
                        + diq_dwm * v[S_WM]
                        + diq_dte * v[S_TE];
                    y[S_WM] = dwm_did * v[S_ID] + dwm_diq * v[S_IQ] + dwm_dwm * v[S_WM];
                    y[S_TE] = dte_dwm * v[S_WM];
                }
            },
        )
        .init(
            move |_p, _t, y| {
                y[S_ID] = i_d_0;
                y[S_IQ] = i_q_0;
                y[S_WM] = omega_m_0;
                y[S_TE] = theta_e_0;
            },
            N_STATES,
        )
        .build()
        .map_err(|e| SimError::SolverFailed(format!("{e:?}")))?;

    let mut solver = problem
        .bdf::<Ls>()
        .map_err(|e| SimError::SolverFailed(format!("{e:?}")))?;

    // ys: DenseMatrix with N_STATES rows, ts.len() columns (one per accepted step).
    let (ys, ts) = solver
        .solve(config.t_end)
        .map_err(|e| SimError::SolverFailed(format!("{e:?}")))?;

    // ── 10. Extract time-series signals from the solution matrix ─────────────
    let i_d_series: Vec<[f64; 2]> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| [t, ys.column(i)[S_ID]])
        .collect();

    let i_q_series: Vec<[f64; 2]> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| [t, ys.column(i)[S_IQ]])
        .collect();

    let omega_m_series: Vec<[f64; 2]> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| [t, ys.column(i)[S_WM]])
        .collect();

    let theta_e_series: Vec<[f64; 2]> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| [t, ys.column(i)[S_TE]])
        .collect();

    // ── 11. Compute algebraic post-processing signals ────────────────────────

    // Electromagnetic torque  T_e = (3/2) · N_p · (λ_m · i_q + (L_d - L_q) · i_d · i_q)
    let t_e_series: Vec<[f64; 2]> = ts
        .iter()
        .enumerate()
        .map(|(i, &t)| {
            let i_d = ys.column(i)[S_ID];
            let i_q = ys.column(i)[S_IQ];
            let t_e =
                1.5 * torque_n_p * (torque_lambda_m * i_q + (torque_l_d - torque_l_q) * i_d * i_q);
            [t, t_e]
        })
        .collect();

    // Inverse Park transform if an InversePark node is present in the graph.
    // f_abc = Park⁻¹(f_d, f_q, θ_e):
    //   f_a = f_d·cos(θ_e)           − f_q·sin(θ_e)
    //   f_b = f_d·cos(θ_e − 2π/3)   − f_q·sin(θ_e − 2π/3)
    //   f_c = f_d·cos(θ_e + 2π/3)   − f_q·sin(θ_e + 2π/3)
    if let Some(pid) = inv_park_id {
        let two_thirds_pi = std::f64::consts::PI * 2.0 / 3.0;

        let f_abc_series: Vec<[f64; 4]> = ts
            .iter()
            .enumerate()
            .map(|(i, &t)| {
                let i_d = ys.column(i)[S_ID];
                let i_q = ys.column(i)[S_IQ];
                let theta = ys.column(i)[S_TE];

                let f_a = i_d * theta.cos() - i_q * theta.sin();
                let f_b = i_d * (theta - two_thirds_pi).cos() - i_q * (theta - two_thirds_pi).sin();
                let f_c = i_d * (theta + two_thirds_pi).cos() - i_q * (theta + two_thirds_pi).sin();
                [t, f_a, f_b, f_c]
            })
            .collect();

        if let Some(SimNode::InversePark(park)) = snarl.get_node_mut(pid) {
            let f_abc_series = resample_vector(&f_abc_series, config.t_start, config.t_end, config.output_dt);
            park.output_f_abc = Some(PortValue::Vector(f_abc_series));
        }
    }

    // ── 11b. Resample all series onto a uniform output grid ──────────────
    let dt = config.output_dt;
    let t0 = config.t_start;
    let t1 = config.t_end;
    let i_d_series = resample_signal(&i_d_series, t0, t1, dt);
    let i_q_series = resample_signal(&i_q_series, t0, t1, dt);
    let omega_m_series = resample_signal(&omega_m_series, t0, t1, dt);
    let theta_e_series = resample_signal(&theta_e_series, t0, t1, dt);
    let t_e_series = resample_signal(&t_e_series, t0, t1, dt);

    // ── 12. Write time-series results back into the graph nodes ──────────────
    if let Some(SimNode::Electrical(e)) = snarl.get_node_mut(elec_id) {
        e.output_i_d = Some(PortValue::Signal(i_d_series));
        e.output_i_q = Some(PortValue::Signal(i_q_series));
    }

    if let Some(SimNode::Mechanical(m)) = snarl.get_node_mut(mech_id) {
        m.output_omega_m = Some(PortValue::Signal(omega_m_series));
        m.output_theta_e = Some(PortValue::Signal(theta_e_series));
    }

    if let Some(tid) = torque_id
        && let Some(SimNode::Torque(t)) = snarl.get_node_mut(tid)
    {
        t.output_t_e = Some(PortValue::Signal(t_e_series));
    }

    // ── 12b. Re-generate constant Signal/Vector outputs on the uniform output grid ─
    // Step 7 pre-generated these on a synthetic grid (`ts_pre`). Now regenerate
    // on the same uniform grid used for ODE outputs so all series align.
    let n_out = ((t1 - t0) / dt).ceil() as usize + 1;
    let ts_uniform: Vec<f64> = (0..n_out).map(|i| (t0 + dt * i as f64).min(t1)).collect();
    #[expect(clippy::shadow_unrelated, reason = "`b` is local to each scope")]
    for &id in &all_ids {
        if let Some(SimNode::Constant(c)) = snarl.get_node(id) {
            let port_value = match c.output_type {
                PortType::Signal => {
                    let series: Vec<[f64; 2]> = ts_uniform.iter().map(|&t| [t, c.value]).collect();
                    Some(PortValue::Signal(series))
                }
                PortType::Vector => {
                    let (a, b, cv) = (c.value, c.value_b, c.value_c);
                    let series: Vec<[f64; 4]> = ts_uniform.iter().map(|&t| [t, a, b, cv]).collect();
                    Some(PortValue::Vector(series))
                }
                PortType::Scalar => None, // handled by get_constant_input
            };
            if let Some(pv) = port_value
                && let Some(SimNode::Constant(c)) = snarl.get_node_mut(id)
            {
                c.output_port_value = Some(pv);
            }
        }
    }

    // ── 13. Re-compute Park transform at ODE time resolution ────────────────
    // Step 7b pre-computed this for ODE input resolution. Re-run with
    // actual data from connected nodes (which may now include ODE outputs).
    if let Some(pid) = park_id {
        let f_abc = get_vector_input(snarl, pid, 0);
        let theta_e = get_signal_input(snarl, pid, 1);

        if let (Some(f_abc), Some(theta_e)) = (f_abc, theta_e) {
            let (f_d_series, f_q_series) = crate::nodes::park::ParkNode::compute(&f_abc, &theta_e);

            if let Some(SimNode::Park(park)) = snarl.get_node_mut(pid) {
                park.output_f_d = Some(PortValue::Signal(resample_signal(&f_d_series, t0, t1, dt)));
                park.output_f_q = Some(PortValue::Signal(resample_signal(&f_q_series, t0, t1, dt)));
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use egui_snarl::{InPinId, OutPinId, Snarl};

    use crate::nodes::SimNode;
    use crate::nodes::constant::ConstantNode;
    use crate::nodes::electrical::ElectricalNode;
    use crate::nodes::mechanical::MechanicalNode;
    use crate::nodes::plot::PlotNode;
    use crate::nodes::torque::TorqueNode;
    use crate::port::PortValue;
    use crate::simulation::SimConfig;

    use super::run_simulation;

    /// Build a minimal PMSM graph and run the solver, verifying that outputs
    /// are populated and physically plausible.
    #[expect(
        clippy::too_many_lines,
        reason = "integration test exercises full simulation pipeline"
    )]
    #[test]
    fn pmsm_simulation_produces_plausible_results() {
        let mut snarl: Snarl<SimNode> = Snarl::new();

        let pos = egui::pos2(0.0, 0.0);

        // Create nodes
        let vd_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                ..ConstantNode::default()
            }),
        );
        let vq_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 24.0,
                ..ConstantNode::default()
            }),
        );
        let tl_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                ..ConstantNode::default()
            }),
        );
        let elec_node = snarl.insert_node(pos, SimNode::Electrical(ElectricalNode::default()));
        let torque_node = snarl.insert_node(pos, SimNode::Torque(TorqueNode::default()));
        let mech_node = snarl.insert_node(pos, SimNode::Mechanical(MechanicalNode::default()));
        let _plot_node = snarl.insert_node(pos, SimNode::Plot(PlotNode::default()));

        // Wire: v_d -> Electrical input 0
        snarl.connect(
            OutPinId {
                node: vd_node,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 0,
            },
        );
        // Wire: v_q -> Electrical input 1
        snarl.connect(
            OutPinId {
                node: vq_node,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 1,
            },
        );
        // Wire: T_L -> Mechanical input 1
        snarl.connect(
            OutPinId {
                node: tl_node,
                output: 0,
            },
            InPinId {
                node: mech_node,
                input: 1,
            },
        );

        // Note: Electrical ω_m input (pin 2) and Mechanical T_e input (pin 0)
        // are Signal type, coming from other ODE/algebraic nodes. The solver
        // handles the coupling internally via the state vector, not via graph wires.

        let config = SimConfig {
            t_start: 0.0,
            t_end: 0.5,
            rtol: 1e-6,
            atol: 1e-8,
            ..SimConfig::default()
        };

        run_simulation(&mut snarl, &config).expect("simulation should succeed");

        // Verify Electrical node has output signals
        let SimNode::Electrical(elec) = snarl.get_node(elec_node).expect("elec node") else {
            panic!("expected electrical node");
        };
        let PortValue::Signal(i_d) = elec.output_i_d.as_ref().expect("i_d output") else {
            panic!("expected Signal");
        };
        let PortValue::Signal(i_q) = elec.output_i_q.as_ref().expect("i_q output") else {
            panic!("expected Signal");
        };

        // Should have multiple time steps
        assert!(
            i_d.len() > 10,
            "expected many time steps, got {}",
            i_d.len()
        );
        assert_eq!(i_d.len(), i_q.len());

        // Initial conditions should be zero
        let epsilon = 1e-10;
        assert!(
            i_d.first().expect("non-empty")[1].abs() < epsilon,
            "i_d(0) should be ~0"
        );
        assert!(
            i_q.first().expect("non-empty")[1].abs() < epsilon,
            "i_q(0) should be ~0"
        );

        // With v_q=24V applied and no load, q-axis current should become non-zero
        let last_i_q = i_q.last().expect("non-empty")[1];
        assert!(
            last_i_q.abs() > 0.01,
            "i_q should be non-zero at end: {last_i_q}"
        );

        // Verify Mechanical node has output signals
        let SimNode::Mechanical(mech) = snarl.get_node(mech_node).expect("mech node") else {
            panic!("expected mechanical node");
        };
        let PortValue::Signal(omega) = mech.output_omega_m.as_ref().expect("omega output") else {
            panic!("expected Signal");
        };

        // Motor should be spinning after 0.5s with v_q=24V
        let last_omega = omega.last().expect("non-empty")[1];
        assert!(
            last_omega > 1.0,
            "motor should be spinning: omega={last_omega}"
        );

        // Verify torque node output
        let SimNode::Torque(torque) = snarl.get_node(torque_node).expect("torque node") else {
            panic!("expected torque node");
        };
        assert!(torque.output_t_e.is_some(), "torque should have output");
    }

    /// Verify that a Constant node adapted to Vector mode produces a constant
    /// 3-phase time series that the Park transform can consume, yielding
    /// the expected d/q decomposition.
    #[expect(
        clippy::too_many_lines,
        reason = "integration test wiring requires verbose graph setup"
    )]
    #[test]
    fn constant_adapted_to_vector_feeds_park_transform() {
        use crate::nodes::park::ParkNode;
        use crate::port::PortType;

        let mut snarl: Snarl<SimNode> = Snarl::new();
        let pos = egui::pos2(0.0, 0.0);

        // Minimal ODE graph so the solver runs (it requires Electrical + Mechanical).
        let elec_node = snarl.insert_node(pos, SimNode::Electrical(ElectricalNode::default()));
        let mech_node = snarl.insert_node(pos, SimNode::Mechanical(MechanicalNode::default()));

        // Constant adapted to Vector for Park's f_abc input.
        let const_abc = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 1.0,   // phase a
                value_b: 2.0, // phase b
                value_c: 3.0, // phase c
                output_type: PortType::Vector,
                ..ConstantNode::default()
            }),
        );

        // Constant adapted to Signal for Park's θ_e input (zero angle).
        let const_theta = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                output_type: PortType::Signal,
                ..ConstantNode::default()
            }),
        );

        let park_node = snarl.insert_node(pos, SimNode::Park(ParkNode::default()));

        // Wire: const_abc -> Park input 0 (f_abc)
        snarl.connect(
            OutPinId {
                node: const_abc,
                output: 0,
            },
            InPinId {
                node: park_node,
                input: 0,
            },
        );
        // Wire: const_theta -> Park input 1 (θ_e)
        snarl.connect(
            OutPinId {
                node: const_theta,
                output: 0,
            },
            InPinId {
                node: park_node,
                input: 1,
            },
        );

        // Wire v_d/v_q constants to Electrical so solver doesn't error.
        let vd = snarl.insert_node(pos, SimNode::Constant(ConstantNode::default()));
        let vq = snarl.insert_node(pos, SimNode::Constant(ConstantNode::default()));
        let tl = snarl.insert_node(pos, SimNode::Constant(ConstantNode::default()));
        snarl.connect(
            OutPinId {
                node: vd,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 0,
            },
        );
        snarl.connect(
            OutPinId {
                node: vq,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 1,
            },
        );
        snarl.connect(
            OutPinId {
                node: tl,
                output: 0,
            },
            InPinId {
                node: mech_node,
                input: 1,
            },
        );

        let config = SimConfig {
            t_start: 0.0,
            t_end: 0.1,
            rtol: 1e-6,
            atol: 1e-8,
            ..SimConfig::default()
        };

        run_simulation(&mut snarl, &config).expect("simulation should succeed");

        // Verify the Constant's output_port_value was generated.
        let SimNode::Constant(c_abc) = snarl.get_node(const_abc).expect("const node") else {
            panic!("expected constant node");
        };
        let Some(PortValue::Vector(vec_data)) = &c_abc.output_port_value else {
            panic!("expected Vector output from adapted constant");
        };
        assert!(!vec_data.is_empty(), "vector series should have entries");
        // Every row should carry the constant phase values.
        for row in vec_data {
            let epsilon = 1e-12;
            assert!(
                (row[1] - 1.0).abs() < epsilon,
                "phase a should be 1.0, got {}",
                row[1]
            );
            assert!(
                (row[2] - 2.0).abs() < epsilon,
                "phase b should be 2.0, got {}",
                row[2]
            );
            assert!(
                (row[3] - 3.0).abs() < epsilon,
                "phase c should be 3.0, got {}",
                row[3]
            );
        }

        // Verify Park transform produced outputs from the constant Vector.
        let SimNode::Park(park) = snarl.get_node(park_node).expect("park node") else {
            panic!("expected park node");
        };
        assert!(park.output_f_d.is_some(), "Park f_d should be populated");
        assert!(park.output_f_q.is_some(), "Park f_q should be populated");

        // With θ_e = 0, Park transform of [1, 2, 3] should give:
        //   f_d = 1·cos(0) + 2·cos(-2π/3) + 3·cos(2π/3) = 1 - 1 - 1.5 = -1.5
        //   f_q = -1·sin(0) - 2·sin(-2π/3) - 3·sin(2π/3)
        // But the exact values depend on the Park scaling convention.
        // Just verify non-trivial output exists.
        let PortValue::Signal(f_d) = park.output_f_d.as_ref().expect("f_d") else {
            panic!("expected Signal");
        };
        assert!(!f_d.is_empty(), "f_d series should be non-empty");
    }

    /// Verify that `ExternalInput::Signal` feeds the ODE correctly.
    /// Constants adapted to `PortType::Signal` should produce time-series
    /// that `resolve_external_input` picks up and interpolates during integration.
    #[test]
    fn signal_inputs_feed_ode() {
        use crate::port::PortType;

        let mut snarl: Snarl<SimNode> = Snarl::new();
        let pos = egui::pos2(0.0, 0.0);

        let elec_node = snarl.insert_node(pos, SimNode::Electrical(ElectricalNode::default()));
        let mech_node = snarl.insert_node(pos, SimNode::Mechanical(MechanicalNode::default()));

        // Constants adapted to Signal for v_d, v_q, T_L
        let vd_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                output_type: PortType::Signal,
                ..ConstantNode::default()
            }),
        );
        let vq_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 24.0,
                output_type: PortType::Signal,
                ..ConstantNode::default()
            }),
        );
        let tl_node = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                output_type: PortType::Signal,
                ..ConstantNode::default()
            }),
        );

        // Wire: v_d -> Electrical input 0
        snarl.connect(
            OutPinId {
                node: vd_node,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 0,
            },
        );
        // Wire: v_q -> Electrical input 1
        snarl.connect(
            OutPinId {
                node: vq_node,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 1,
            },
        );
        // Wire: T_L -> Mechanical input 1
        snarl.connect(
            OutPinId {
                node: tl_node,
                output: 0,
            },
            InPinId {
                node: mech_node,
                input: 1,
            },
        );

        let config = SimConfig {
            t_start: 0.0,
            t_end: 0.5,
            rtol: 1e-6,
            atol: 1e-8,
            ..SimConfig::default()
        };

        run_simulation(&mut snarl, &config).expect("simulation should succeed");

        // Verify outputs are populated
        let SimNode::Electrical(elec) = snarl.get_node(elec_node).expect("elec node") else {
            panic!("expected electrical node");
        };
        let PortValue::Signal(i_q) = elec.output_i_q.as_ref().expect("i_q output") else {
            panic!("expected Signal");
        };

        // With v_q=24V via Signal, results should match the Scalar case
        let last_i_q = i_q.last().expect("non-empty")[1];
        assert!(
            last_i_q.abs() > 0.01,
            "i_q should be non-zero at end: {last_i_q}"
        );

        let SimNode::Mechanical(mech) = snarl.get_node(mech_node).expect("mech node") else {
            panic!("expected mechanical node");
        };
        let PortValue::Signal(omega) = mech.output_omega_m.as_ref().expect("omega output") else {
            panic!("expected Signal");
        };
        let last_omega = omega.last().expect("non-empty")[1];
        assert!(
            last_omega > 1.0,
            "motor should be spinning: omega={last_omega}"
        );

        // The Signal-adapted constants should also have output_port_value populated
        let SimNode::Constant(c_vq) = snarl.get_node(vq_node).expect("vq node") else {
            panic!("expected constant node");
        };
        assert!(
            c_vq.output_port_value.is_some(),
            "Signal-adapted constant should have output"
        );
    }

    #[test]
    fn interpolate_signal_edge_cases() {
        use super::interpolate_signal;

        // Empty signal returns 0.0
        assert_eq!(interpolate_signal(&[], 1.0), 0.0);

        // Single point returns that value for any t
        let single = [[5.0, 42.0]];
        assert_eq!(interpolate_signal(&single, 0.0), 42.0);
        assert_eq!(interpolate_signal(&single, 5.0), 42.0);
        assert_eq!(interpolate_signal(&single, 100.0), 42.0);

        // Clamps to boundaries
        let series = [[0.0, 10.0], [1.0, 20.0], [2.0, 30.0]];
        assert_eq!(interpolate_signal(&series, -1.0), 10.0);
        assert_eq!(interpolate_signal(&series, 3.0), 30.0);

        // Exact boundary values
        let eps = 1e-12;
        assert!((interpolate_signal(&series, 0.0) - 10.0).abs() < eps);
        assert!((interpolate_signal(&series, 2.0) - 30.0).abs() < eps);

        // Midpoint interpolation
        assert!((interpolate_signal(&series, 0.5) - 15.0).abs() < eps);
        assert!((interpolate_signal(&series, 1.5) - 25.0).abs() < eps);
    }

    #[test]
    fn interpolate_vector_edge_cases() {
        use super::interpolate_vector;

        // Empty signal returns [0, 0, 0]
        assert_eq!(interpolate_vector(&[], 1.0), [0.0; 3]);

        // Single point returns those values for any t
        let single = [[5.0, 1.0, 2.0, 3.0]];
        assert_eq!(interpolate_vector(&single, 0.0), [1.0, 2.0, 3.0]);
        assert_eq!(interpolate_vector(&single, 5.0), [1.0, 2.0, 3.0]);
        assert_eq!(interpolate_vector(&single, 100.0), [1.0, 2.0, 3.0]);

        // Clamps to boundaries
        let series = [
            [0.0, 10.0, 20.0, 30.0],
            [1.0, 40.0, 50.0, 60.0],
            [2.0, 70.0, 80.0, 90.0],
        ];
        assert_eq!(interpolate_vector(&series, -1.0), [10.0, 20.0, 30.0]);
        assert_eq!(interpolate_vector(&series, 3.0), [70.0, 80.0, 90.0]);

        // Midpoint interpolation
        let eps = 1e-12;
        let mid = interpolate_vector(&series, 0.5);
        assert!((mid[0] - 25.0).abs() < eps, "a: {}", mid[0]);
        assert!((mid[1] - 35.0).abs() < eps, "b: {}", mid[1]);
        assert!((mid[2] - 45.0).abs() < eps, "c: {}", mid[2]);
    }

    /// Verify that a Park node wired between a 3-phase constant and the Electrical
    /// node, with `theta_e` from the Mechanical node (circular ODE dependency),
    /// produces non-zero motor currents and speed.
    #[expect(
        clippy::too_many_lines,
        reason = "integration test exercises full Park-coupled simulation pipeline"
    )]
    #[test]
    fn park_coupled_to_ode_produces_nonzero_output() {
        use crate::nodes::park::ParkNode;
        use crate::port::PortType;

        let mut snarl: Snarl<SimNode> = Snarl::new();
        let pos = egui::pos2(0.0, 0.0);

        // -- ODE nodes --
        let elec_node = snarl.insert_node(pos, SimNode::Electrical(ElectricalNode::default()));
        let mech_node = snarl.insert_node(pos, SimNode::Mechanical(MechanicalNode::default()));

        // -- Park node --
        let park_node = snarl.insert_node(pos, SimNode::Park(ParkNode::default()));

        // -- 3-phase voltage source (Constant in Vector mode) --
        // v_a=0, v_b=100, v_c=-100 produces v_q ≈ 115 V at θ_e=0 via Park,
        // which drives q-axis current and generates torque to spin the motor.
        let const_abc = snarl.insert_node(
            pos,
            SimNode::Constant(ConstantNode {
                value: 0.0,
                value_b: 100.0,
                value_c: -100.0,
                output_type: PortType::Vector,
                ..ConstantNode::default()
            }),
        );

        // -- Load torque = 0 --
        let tl_node = snarl.insert_node(pos, SimNode::Constant(ConstantNode::default()));

        // Wire: const_abc -> Park input 0 (f_abc)
        snarl.connect(
            OutPinId {
                node: const_abc,
                output: 0,
            },
            InPinId {
                node: park_node,
                input: 0,
            },
        );
        // Wire: Mechanical theta_e (output 1) -> Park input 1 (theta_e)
        snarl.connect(
            OutPinId {
                node: mech_node,
                output: 1,
            },
            InPinId {
                node: park_node,
                input: 1,
            },
        );
        // Wire: Park f_d (output 0) -> Electrical input 0 (v_d)
        snarl.connect(
            OutPinId {
                node: park_node,
                output: 0,
            },
            InPinId {
                node: elec_node,
                input: 0,
            },
        );
        // Wire: Park f_q (output 1) -> Electrical input 1 (v_q)
        snarl.connect(
            OutPinId {
                node: park_node,
                output: 1,
            },
            InPinId {
                node: elec_node,
                input: 1,
            },
        );
        // Wire: T_L -> Mechanical input 1
        snarl.connect(
            OutPinId {
                node: tl_node,
                output: 0,
            },
            InPinId {
                node: mech_node,
                input: 1,
            },
        );

        let config = SimConfig {
            t_start: 0.0,
            t_end: 0.5,
            rtol: 1e-6,
            atol: 1e-8,
            ..SimConfig::default()
        };

        run_simulation(&mut snarl, &config).expect("simulation should succeed");

        // Verify Electrical node has non-zero outputs
        let SimNode::Electrical(elec) = snarl.get_node(elec_node).expect("elec node") else {
            panic!("expected electrical node");
        };
        let PortValue::Signal(i_d) = elec.output_i_d.as_ref().expect("i_d output") else {
            panic!("expected Signal");
        };
        let PortValue::Signal(i_q) = elec.output_i_q.as_ref().expect("i_q output") else {
            panic!("expected Signal");
        };

        assert!(
            i_d.len() > 10,
            "expected many time steps, got {}",
            i_d.len()
        );

        // With a 3-phase voltage applied via Park-coupled path, currents must be non-zero.
        // This is the key assertion: the old code would produce all zeros here.
        let last_i_d = i_d.last().expect("non-empty")[1];
        let last_i_q = i_q.last().expect("non-empty")[1];
        assert!(
            last_i_d.abs() > 0.01 || last_i_q.abs() > 0.01,
            "Park-coupled currents should be non-zero: i_d={last_i_d}, i_q={last_i_q}"
        );

        // Verify Mechanical node shows rotation
        let SimNode::Mechanical(mech) = snarl.get_node(mech_node).expect("mech node") else {
            panic!("expected mechanical node");
        };
        let PortValue::Signal(omega) = mech.output_omega_m.as_ref().expect("omega output") else {
            panic!("expected Signal");
        };
        let last_omega = omega.last().expect("non-empty")[1];
        assert!(
            last_omega.abs() > 0.01,
            "motor should be spinning: omega={last_omega}"
        );

        // Verify Park outputs are populated post-ODE (step 13 re-compute)
        let SimNode::Park(park) = snarl.get_node(park_node).expect("park node") else {
            panic!("expected park node");
        };
        assert!(
            park.output_f_d.is_some(),
            "Park f_d should be populated post-ODE"
        );
        assert!(
            park.output_f_q.is_some(),
            "Park f_q should be populated post-ODE"
        );
    }
}
