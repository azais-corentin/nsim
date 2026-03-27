//! Typed port system for the node graph.
//!
//! Each pin on every node declares a [`PortType`]. Connections are only
//! allowed between compatible types. [`PortValue`] carries the actual
//! data flowing through a connection after simulation.

use egui::Color32;

/// The kind of data a port carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PortType {
    /// A constant scalar parameter (`f64`).
    Scalar,
    /// A time-series signal: `[(t, value), ...]`.
    Signal,
    /// A 3-phase time-series: `[(t, a, b, c), ...]`.
    Vector,
}

impl PortType {
    /// Whether an output of type `self` can connect to an input of type `other`.
    pub fn is_compatible_with(self, other: Self) -> bool {
        self == other
    }

    /// Distinctive color for visual differentiation in the graph.
    pub fn color(self) -> Color32 {
        match self {
            Self::Scalar => Color32::from_rgb(0x4E, 0xBA, 0x6F), // green
            Self::Signal => Color32::from_rgb(0x56, 0x9C, 0xD6), // blue
            Self::Vector => Color32::from_rgb(0xD6, 0x8C, 0x45), // orange
        }
    }
}

/// Concrete data flowing through a port after simulation.
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub enum PortValue {
    /// Constant parameter.
    Scalar(f64),
    /// Time-series: each entry is `[t, value]`.
    Signal(Vec<[f64; 2]>),
    /// 3-phase time-series: each entry is `[t, a, b, c]`.
    Vector(Vec<[f64; 4]>),
}

impl PortValue {
    /// The port type this value corresponds to.
    pub fn port_type(&self) -> PortType {
        match self {
            Self::Scalar(_) => PortType::Scalar,
            Self::Signal(_) => PortType::Signal,
            Self::Vector(_) => PortType::Vector,
        }
    }
}
