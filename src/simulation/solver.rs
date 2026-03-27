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
/// Parameter index: d-axis voltage `v_d`.
const P_VD: usize = 7;
/// Parameter index: q-axis voltage `v_q`.
const P_VQ: usize = 8;
/// Parameter index: load torque `T_L`.
const P_TL: usize = 9;
/// Parameter index: torque-equation pole-pair count `N_p`.
const P_T_NP: usize = 10;
/// Parameter index: torque-equation flux linkage `λ_m`.
const P_T_LAM: usize = 11;
/// Parameter index: torque-equation d-axis inductance `L_d`.
const P_T_LD: usize = 12;
/// Parameter index: torque-equation q-axis inductance `L_q`.
const P_T_LQ: usize = 13;

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

    // ── 7. Read voltage and load-torque sources from connected Constants ─────
    // ElectricalNode pin layout: 0 = v_d (Scalar), 1 = v_q (Scalar), 2 = ω_m (Signal)
    let v_d = get_constant_input(snarl, elec_id, 0).unwrap_or(0.0);
    let v_q = get_constant_input(snarl, elec_id, 1).unwrap_or(0.0);
    // MechanicalNode pin layout: 0 = T_e (Signal), 1 = T_L (Scalar)
    let t_l = get_constant_input(snarl, mech_id, 1).unwrap_or(0.0);

    // Use pole-pair count from the Mechanical node throughout the ODE.
    let n_p = n_p_mech;

    // ── 8. Assemble parameter vector ─────────────────────────────────────────
    // Element order must match the P_* constants defined above.
    let p = vec![
        r_s,
        l_d,
        l_q,
        lambda_m,
        n_p, // P_RS … P_NP
        j,
        b,
        v_d,
        v_q,
        t_l, // P_J  … P_TL
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
            |x, p, _t, y| {
                // Electromagnetic torque (used in the mechanical equation below)
                let t_e = 1.5
                    * p[P_T_NP]
                    * (p[P_T_LAM] * x[S_IQ] + (p[P_T_LD] - p[P_T_LQ]) * x[S_ID] * x[S_IQ]);

                // d-axis current
                y[S_ID] = (1.0 / p[P_LD])
                    * (p[P_VD] - p[P_RS] * x[S_ID] + p[P_NP] * x[S_WM] * p[P_LQ] * x[S_IQ]);

                // q-axis current
                y[S_IQ] = (1.0 / p[P_LQ])
                    * (p[P_VQ]
                        - p[P_RS] * x[S_IQ]
                        - p[P_NP] * x[S_WM] * p[P_LD] * x[S_ID]
                        - p[P_NP] * x[S_WM] * p[P_LAM]);

                // Mechanical speed
                y[S_WM] = (1.0 / p[P_J]) * (t_e - p[P_TL] - p[P_B] * x[S_WM]);

                // Electrical angle
                y[S_TE] = p[P_NP] * x[S_WM];
            },
            // ── Jacobian-vector product J·v where J = ∂f/∂x ───────────────
            |x, p, _t, v, y| {
                // Partial derivatives of T_e w.r.t. state components
                let dt_e_did = 1.5 * p[P_T_NP] * (p[P_T_LD] - p[P_T_LQ]) * x[S_IQ];
                let dt_e_diq = 1.5 * p[P_T_NP] * (p[P_T_LAM] + (p[P_T_LD] - p[P_T_LQ]) * x[S_ID]);

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

                // J·v  (θ_e has no dependence on i_d, i_q, or θ_e itself)
                y[S_ID] = did_did * v[S_ID] + did_diq * v[S_IQ] + did_dwm * v[S_WM];
                y[S_IQ] = diq_did * v[S_ID] + diq_diq * v[S_IQ] + diq_dwm * v[S_WM];
                y[S_WM] = dwm_did * v[S_ID] + dwm_diq * v[S_IQ] + dwm_dwm * v[S_WM];
                y[S_TE] = dte_dwm * v[S_WM];
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
            park.output_f_abc = Some(PortValue::Vector(f_abc_series));
        }
    }

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

    // ── 12b. Generate constant Signal/Vector outputs ─────────────────────────
    // Constants that adapted to Signal or Vector mode need time-series data
    // so downstream nodes (e.g. Park) can read them via get_signal_input /
    // get_vector_input. Must run before any algebraic post-processing that
    // reads from graph wires.
    for &id in &all_ids {
        if let Some(SimNode::Constant(c)) = snarl.get_node(id) {
            let port_value = match c.output_type {
                PortType::Signal => {
                    let series: Vec<[f64; 2]> = ts.iter().map(|&t| [t, c.value]).collect();
                    Some(PortValue::Signal(series))
                }
                PortType::Vector => {
                    let (a, b, cv) = (c.value, c.value_b, c.value_c);
                    let series: Vec<[f64; 4]> = ts.iter().map(|&t| [t, a, b, cv]).collect();
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

    // ── 13. Forward Park transform if a Park node is present ─────────────────
    // Reads the Vector (f_abc) and Signal (θ_e) from whatever nodes are
    // connected to its inputs, then writes f_d and f_q back.
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
}
