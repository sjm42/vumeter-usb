# Repository Guidelines

## Project Structure & Module Organization
`src/main.rs` contains the entire firmware entry point, RTIC tasks, USB command parser, PWM control, and startup animation. `build.rs` copies `memory.x` into Cargo's output directory so linker settings follow the crate. `.cargo/config.toml` pins the default target to `thumbv7em-none-eabihf` and sets embedded linker flags. The `build` helper script wraps common build, lint, and flash flows. Keep hardware-specific changes aligned with `memory.x` and the STM32F411/BlackPill assumptions documented in `README.md`.

## Build, Test, and Development Commands
Use the helper script for the standard workflow:

- `./build b`: release build for `thumbv7em-none-eabihf`.
- `./build c`: run `cargo clippy --release --target thumbv7em-none-eabihf`.
- `./build f`: flash with `cargo flash` for `STM32F411CEUx`.
- `./build d`: create `target/out.bin` and flash via `dfu-util`.

Direct Cargo commands also work, for example `cargo build --release --target thumbv7em-none-eabihf`. Install the target first with `rustup target add thumbv7em-none-eabihf`.

## Coding Style & Naming Conventions
Follow `rustfmt.toml`: 120-column width, grouped imports, and crate-granularity import merging. Use standard Rust naming: `snake_case` for functions and locals, `SCREAMING_SNAKE_CASE` for constants, and `CamelCase` for types and enums. This crate is `#![no_std]`; prefer predictable control flow, explicit hardware intent, and small helper types over allocation-heavy abstractions.

## Testing Guidelines
There is no host-side test suite yet. Before opening a PR, run `./build b` and `./build c`. For behavior changes, include hardware validation notes covering USB CDC enumeration, 4-byte command parsing, PWM output on PA0-PA3, and any watchdog or startup-animation impact.

## Commit & Pull Request Guidelines
Recent history uses short, imperative commit subjects such as `cargo update`. Keep commits narrowly scoped and describe one logical change each. Pull requests should include a short summary, affected hardware behavior, commands run, and any manual flashing or bench-test results. Link related issues when available and call out changes to `memory.x`, USB identifiers, or flashing steps explicitly.
