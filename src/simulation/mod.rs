//! Simulation configuration and error types for the PMSM node graph solver.
//!
//! [`SimConfig`] carries runtime parameters (time span and solver tolerances)
//! for a single simulation run. [`SimError`] enumerates all failure modes that
//! [`solver::run_simulation`] can surface to the caller.

pub mod solver;

/// Configuration for a single PMSM simulation run.
///
/// All fields have physics-appropriate defaults; persist with [`serde`] for
/// round-trip compatibility across application versions.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct SimConfig {
    /// Simulation start time in seconds. Must satisfy `t_start < t_end`.
    pub t_start: f64,
    /// Simulation end time in seconds.
    pub t_end: f64,
    /// Relative tolerance for the adaptive BDF solver.
    pub rtol: f64,
    /// Absolute tolerance for the adaptive BDF solver.
    pub atol: f64,
    /// Maximum time between output points (seconds). Controls plot smoothness.
    /// The solver output is resampled onto a uniform grid with this spacing.
    pub output_dt: f64,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            t_start: 0.0,
            t_end: 2.0,
            rtol: 1e-6,
            atol: 1e-8,
            output_dt: 0.001,
        }
    }
}

/// Errors that can occur when running a graph simulation.
#[derive(Debug)]
pub enum SimError {
    /// No ODE nodes (both [`crate::nodes::electrical::ElectricalNode`] and
    /// [`crate::nodes::mechanical::MechanicalNode`]) were found in the graph.
    NoOdeNodes,
    /// A required connection between nodes is absent.
    MissingConnection(String),
    /// The `diffsol` BDF solver encountered a numerical failure.
    SolverFailed(String),
    /// The node graph has an inconsistent or unsupported topology.
    GraphError(String),
}

impl std::fmt::Display for SimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoOdeNodes => {
                write!(f, "no ODE nodes (Electrical + Mechanical) found in graph")
            }
            Self::MissingConnection(msg) => write!(f, "missing connection: {msg}"),
            Self::SolverFailed(msg) => write!(f, "solver failed: {msg}"),
            Self::GraphError(msg) => write!(f, "graph topology error: {msg}"),
        }
    }
}

impl std::error::Error for SimError {}
