# Repository Guidelines

## Project Overview

nsim is an interactive node-graph simulator for Permanent Magnet Synchronous Motors (PMSMs). Users wire simulation nodes on an egui-snarl canvas, configure motor parameters inline, and run a batch ODE solve (diffsol BDF integrator) over a 4-state system (`i_d`, `i_q`, `omega_m`, `theta_e`). Results render as time-series plots inside the graph. Targets both native desktop and WebAssembly (PWA).

Mathematical reference: `pmsm.md` (all 18 equations, parameter defaults).

## Architecture & Data Flow

```
main.rs              Platform entry (native / WASM cfg split)
lib.rs               Crate root; re-exports TemplateApp
app.rs               egui_tiles 3-pane shell, simulation trigger
nodes/
  mod.rs             SimNode enum, SimViewer (SnarlViewer impl)
  constant.rs        Scalar source node
  electrical.rs      PMSM stator ODE node (i_d, i_q)
  mechanical.rs      Rotor dynamics ODE node (omega_m, theta_e)
  torque.rs          Algebraic torque calculator (optional)
  park.rs            Inverse Park transform dq->abc (optional)
  plot.rs            Variable-arity sink/visualization node
port.rs              PortType (Scalar/Signal/Vector) + PortValue
simulation/
  mod.rs             SimConfig, SimError
  solver.rs          Graph traversal, ODE assembly, BDF solve, result write-back
```

### Pattern: Data-Flow Node Graph

The central data structure is `Snarl<SimNode>` (persisted via serde). Simulation is a one-shot batch solve, not a real-time loop.

### Data flow

1. User places nodes and connects typed pins on the egui-snarl canvas.
2. Constant nodes provide scalar parameters (`v_d`, `v_q`, `T_L`) to ODE nodes via Scalar wires.
3. On "Simulate": `run_simulation` traverses the Snarl, reads parameters from node fields and connected Constants, assembles a 4-state ODE with a 14-element parameter vector, and solves with diffsol BDF.
4. Results (`Signal`/`Vector` `PortValue`s) are written back into node `Option<PortValue>` output fields (skipped by serde).
5. Plot nodes read remote `output_value()` and render egui_plot line series.

### Solver coupling

Electrical and Mechanical ODEs are **fully coupled in a single 4-state system** -- graph wires for `omega_m -> Electrical` and `T_e -> Mechanical` are UI-only annotations. The actual coupling is algebraic in the RHS and Jacobian closures. Torque and InversePark nodes are optional; the solver detects their presence.

### Port type system

Three types with strict compatibility: `Scalar` (constant `f64`), `Signal` (time series `[t, v]`), `Vector` (3-phase `[t, a, b, c]`). Plot is the sole exception: it accepts Signal or Vector.

## Development Commands

### Build & Run

```sh
cargo run --release          # Native desktop
trunk serve                  # WASM dev server at http://127.0.0.1:8080
trunk build --release        # WASM production build (output: dist/)
```

### Check & Lint

```sh
./check.sh                   # Full CI-equivalent local check (run before pushing)
cargo check --workspace --all-targets
cargo check --workspace --all-features --lib --target wasm32-unknown-unknown
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings -W clippy::all
typos                        # Spell check
```

### Test

```sh
cargo test --workspace --all-targets --all-features
cargo test --workspace --doc
```

### Interactive Development

```sh
bacon                        # File watcher, default job: check
bacon clippy-all             # Or press 'c' in bacon
bacon test                   # Run tests on change
```

## Code Conventions

### Rust Edition & Toolchain

- **Edition 2024**, MSRV 1.92, Rust stable channel
- Single crate (not a multi-crate workspace)
- `devenv` + `direnv` provides the reproducible Nix dev environment with all native GUI libs

### Strict Lint Posture

~150 clippy lints at warn level, promoted to errors in CI (`RUSTFLAGS=-D warnings`). Key enforced rules:

- `unsafe_code = "deny"` -- no unsafe code, period
- `unwrap_used`, `get_unwrap` -- use `.expect("reason")` or proper error handling
- `indexing_slicing` -- prefer `.get()` or iterators
- `print_stdout`, `print_stderr`, `dbg_macro` -- use `log` crate (`log::info!`, etc.)
- `wildcard_imports` -- explicit imports only (`use foo::bar`, not `use foo::*`)
- `todo`, `unimplemented` -- warn; don't leave these in
- `use_self` -- use `Self` in impl blocks
- Allowed exceptions: `manual_range_contains`, `map_unwrap_or`

Use `#[expect(clippy::lint_name, reason = "...")]` with a mandatory reason string when suppressing a lint.

### Formatting

- **dprint** is the canonical formatter for all file types (Rust, JSON, TOML, YAML, Markdown, shell, Dockerfile)
- Rust: `rustfmt --edition 2024` (via dprint exec plugin)
- Shell: `shfmt` (via mise/dprint exec plugin)
- Line width: 100, indent: 2 spaces
- Tools managed by mise (`.mise/config.toml`)

### Serde & Persistence

- All persistent app state round-trips through `eframe::Storage` via serde
- Use `#[serde(default)]` for backwards compatibility on new fields
- Simulation outputs are `#[serde(skip)]` -- recomputed on each run

### Platform-Specific Code

```rust
#[cfg(not(target_arch = "wasm32"))]  // Native-only
#[cfg(target_arch = "wasm32")]        // WASM-only
cfg!(target_arch = "wasm32")          // Runtime check (e.g., hiding Quit menu on web)
```

Native dependencies: `env_logger`. WASM dependencies: `wasm-bindgen-futures`, `web-sys`, `getrandom` (wasm_js).

### Error Handling

- `SimError` enum in `src/simulation/mod.rs` implements `std::error::Error`
- Variants: `NoOdeNodes`, `MissingConnection`, `SolverFailed`, `GraphError`
- Errors surface to UI via `sim_status` string in `TemplateApp`

### Key Types

| Type | Location | Purpose |
|------|----------|---------|
| `TemplateApp` | `src/app.rs` | Root app state, implements `eframe::App` |
| `SimNode` | `src/nodes/mod.rs` | Enum over all six node types, carried by `Snarl<SimNode>` |
| `SimViewer` | `src/nodes/mod.rs` | Unit struct implementing `SnarlViewer<SimNode>` |
| `PortType` / `PortValue` | `src/port.rs` | Typed port data carriers |
| `SimConfig` | `src/simulation/mod.rs` | Solver time span + tolerances (rtol=1e-6, atol=1e-8) |
| `SimError` | `src/simulation/mod.rs` | Typed error enum |
| `ElectricalNode` | `src/nodes/electrical.rs` | PMSM stator ODE state (R_s, L_d, L_q, lambda_m, N_p) |
| `MechanicalNode` | `src/nodes/mechanical.rs` | Rotor dynamics (J, B, N_p) |

## Important Files

| File | Purpose |
|------|---------|
| `src/main.rs` | Binary entry point (native/WASM cfg split) |
| `src/lib.rs` | Crate root, module declarations |
| `src/app.rs` | Application shell, pane layout, simulation trigger |
| `src/simulation/solver.rs` | Core simulation pipeline (ODE assembly + BDF solve) |
| `src/nodes/mod.rs` | Node enum, graph viewer, context menus, node rendering |
| `src/port.rs` | Port type system and compatibility |
| `pmsm.md` | Mathematical reference (all 18 equations) |
| `check.sh` | Local CI script (run before pushing) |
| `index.html` | WASM entry point consumed by Trunk (canvas id: `the_canvas_id`) |
| `Cargo.toml` | Dependencies, build profiles, and all lint configuration |
| `dprint.json` | Formatter config for all file types |
| `devenv.nix` | Nix dev environment (GUI libs, Rust toolchain, WASM target) |

## Build Profiles

- **release**: `opt-level = 2` (balanced speed/size for WASM)
- **dev**: all dependency packages at `opt-level = 2` (critical for numerical simulation performance in debug builds)

## CI Pipeline

GitHub Actions (`rust.yml`) runs on push and PRs:

| Job | What |
|-----|------|
| `check` | `cargo check --all-features` |
| `check_wasm` | `cargo check --all-features --lib --target wasm32-unknown-unknown` |
| `test` | `cargo test --lib` (Ubuntu, requires X11/Wayland dev libs) |
| `fmt` | `cargo fmt --all -- --check` |
| `clippy` | `cargo clippy -- -D warnings` |
| `trunk` | WASM build (pinned to Rust 1.88.0) |
| `build` | Cross-compile matrix: macOS (aarch64, x86_64), Linux (arm, armv7, x86_64-musl), Windows (x86_64-msvc) |

Additional workflows: `pages.yml` (GitHub Pages deploy on push to main), `typos.yml` (spell check on PRs).

## Testing

No test suite currently exists. CI test jobs pass vacuously. The `solver.rs` file contains a placeholder integration test `pmsm_simulation_produces_plausible_results`. All quality assurance relies on the strict lint configuration, type checking, and cross-compilation CI.

When adding tests:
- Inline unit tests via `#[cfg(test)]` modules preferred
- Run with `cargo test --workspace --all-targets --all-features`
- Doc tests with `cargo test --workspace --doc`
- `bacon test` or `bacon nextest` for interactive development
