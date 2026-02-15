# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
cargo build            # debug build
cargo build --release  # release build (optimized for size with opt-level='z' and LTO)
cargo clippy           # lint
cargo fmt              # format (max_width=120, crate-level import granularity)
```

Cross-compiles to `thumbv7em-none-eabihf` (set in `.cargo/config.toml`). No tests or benchmarks (`test = false`, `bench = false`). This is a `#![no_std]` `#![no_main]` embedded target — standard test infrastructure does not apply.

## Architecture

Bare-metal Rust firmware for a **STM32F411 "Blackpill"** board that acts as a USB-controlled 4-channel PWM VU meter.

**Runtime**: RTIC v2 (Real-Time Interrupt-driven Concurrency) — all concurrency is defined via RTIC tasks with priority levels in `src/main.rs`. There is only one source file.

**How it works**: The device enumerates as a USB CDC serial device (VID `0x16c0`, PID `0x27dd`). A host sends 4-byte commands over serial: `[0xFD, 0x02, channel+0x30, duty_value]`. The `usb_fs` ISR (priority 5) parses this state machine and spawns `set_pwm` (priority 7) to update PWM duty on TIM5 channels (PA0–PA3).

**Key RTIC tasks and priorities**:
- `set_pwm` (7): Sets PWM duty cycle on a channel
- `usb_fs` (5): USB OTG_FS interrupt handler, parses incoming serial command protocol
- `periodic` (2): Feeds the hardware watchdog every 1s
- `hello` (1): Startup animation — ramps all PWM channels up then down
- `led_blink`/`led_off` (1): Blinks the onboard LED (PC13) briefly on PWM updates

**Memory layout**: Defined in `memory.x` (512K FLASH, 128K RAM for STM32F411). The `build.rs` copies it to the linker search path.
