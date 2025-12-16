# AGENTS.md

## Build Commands

- `cargo check --workspace --all-targets` - Quick compile check
- `cargo build --release` - Build release binary
- `trunk serve` - Dev server for web (WASM) at http://127.0.0.1:8080
- `./check.sh` - Run all CI checks locally

## Test Commands

- `cargo test --workspace --all-targets --all-features` - Run all tests
- `cargo test <test_name>` - Run single test by name
- `cargo test --workspace --doc` - Run doc tests

## Lint/Format

- `cargo fmt --all -- --check` - Check formatting
- `cargo clippy --workspace --all-targets --all-features -- -D warnings` - Run lints
- `typos` - Spell check (install: `cargo install typos-cli`)

## Code Style

- **Use commands over manual edits** - prefer CLI tools to modify config files (e.g., `cargo add package` instead of manually editing Cargo.toml)
- **Rust 2024 edition**, requires Rust 1.88+
- **No unsafe code** - `unsafe_code = "deny"`
- **No `.unwrap()`** - use `.expect("reason")` or proper error handling
- **No print macros** - `print!`/`println!`/`dbg!` are warned; use logging
- **Explicit imports** - no wildcard imports (`use foo::*`)
- **Use `Self`** in impl blocks where applicable
- **Serde**: use `#[serde(default)]` for backwards compatibility
- **Conditional compilation**: use `#[cfg(target_arch = "wasm32")]` for platform-specific code
