# vumeter-usb

Bare-metal Rust firmware for a **STM32F411 "BlackPill"** board that acts as a USB-controlled
4-channel PWM VU meter.

The device enumerates as a USB CDC serial device. A host application sends simple 4-byte commands
over the virtual serial port to set PWM duty cycles on four independent output channels (PA0-PA3),
driving analog meter needle movements.

## Hardware

- **MCU**: STM32F411CEU6 (Cortex-M4F, 512K Flash, 128K RAM)
- **Board**: WeAct Blackpill F411
- **Crystal**: 25 MHz HSE, system clock configured to 84 MHz
- **PWM outputs**: TIM5 channels on PA0, PA1, PA2, PA3 (50 kHz)
- **Status LED**: PC13 (active low, blinks briefly on PWM updates)
- **USB**: OTG_FS on PA11/PA12

## USB Device Info

| Field        | Value              |
|--------------|--------------------|
| VID          | `0x16c0`           |
| PID          | `0x27dd`           |
| Class        | CDC (Virtual COM)  |
| Manufacturer | Siuro Hacklab      |
| Product      | PWM Controller     |
| Device release | Generated from the crate version (`major.minor.patch -> 0xMMmp`) |
| Serial       | MCU lot, wafer number, X/Y coordinates |

The serial number includes all MCU unique-ID fields. This changes serial numbers produced by
older firmware, so update any Linux `/dev/serial/by-id/` paths or device-selection rules in the host
application. The VID/PID are unchanged. Device release encoding requires major <= 99, minor <= 9
and patch <= 9; unsupported versions fail the build with an explanatory error. Prerelease/build
suffixes do not change the release descriptor.

## Serial Protocol

Commands are 4 bytes:

```
Byte 0: 0xFD  (start marker 1)
Byte 1: 0x02  (start marker 2)
Byte 2: 0x30 + channel  (channel 0-3)
Byte 3: duty value  (0-255)
```

The original duty scaling is preserved: `max(1, floor(value * timer_period / 256))` timer ticks.
A value of 0 sets the minimum duty (one timer tick, not fully off); 255 is slightly below 100% duty.

Commands may be split across reads or combined in one USB packet. An incomplete command expires
after **100 ms between bytes**. This is separate from the **10-second output timeout**: slow
applications can send complete commands infrequently. Every complete valid command refreshes
that timeout for all four channels; malformed traffic does not. If it expires, all outputs return
to minimum and the partial command is discarded. A subsequent valid command resumes control.

USB reset, deconfiguration or suspend returns all outputs to minimum as soon as the state change
is serviced, clearing buffered data and partial commands. Physical disconnect is handled through
the USB suspend state when the board remains powered. Closing the Linux serial port does not
necessarily disconnect USB; stopping updates still triggers the 10-second timeout.

The four-byte protocol has no checksum or escaping. A truncated command immediately followed by
a new header can still interpret the new header's first byte as duty data. The parser timeout and
USB reset handling reduce this ambiguity while keeping compatibility with existing host software.

### Example

Set channel 2 to half of max value:

```
0xFD 0x02 0x32 0x80
```

## Startup Behavior

On power-up, after a 2-second delay the firmware ramps all four channels up and back down,
finishing at minimum around 7.2 seconds after initialization. The first valid host command cancels
the entire animation, including during the initial delay. Disconnect also cancels it. Reconnecting
does not restart the animation. Software and watchdog resets suppress the sweep.
When a command takes over during the animation, channels the host has not set return to minimum.

One RTIC task services USB, command parsing, animation, timeouts, PWM and the status LED. It polls
USB at 1 ms intervals with up to four 64-byte reads per iteration, applying the latest value for
each channel. Remaining data is processed on the next iteration without requiring a new USB
interrupt. USB device/class resources are owned by that task, with no mutable application globals.

The independent hardware watchdog has a nominal 5-second timeout, starts before clock/USB setup,
and is fed only after the service task finishes an iteration. A quiet host remains healthy.
Unexpected receive errors, panics and unhandled faults trigger a reset; timer compare registers
are set to minimum first. The boot sequence forces a brief USB disconnect/reconnect for host
re-enumeration, including after a rapid reset with the cable attached.

## Building

Requires the `thumbv7em-none-eabihf` Rust target:

```bash
rustup target add thumbv7em-none-eabihf
```

Build, lint and run host regression tests:

```bash
./build b
./build c
./build t
```

Direct Cargo commands also work because `.cargo/config.toml` sets the default embedded target:

```bash
cargo build --release --target thumbv7em-none-eabihf
cargo clippy --release --target thumbv7em-none-eabihf
```

The release profile is optimized for size (`opt-level = 'z'`, LTO enabled). `build.rs` copies
`memory.x` into Cargo's output directory and generates the USB device release constant from the
package version.

## Flashing

Flash the resulting binary to the Blackpill using your preferred method (ST-Link, DFU, etc.)
or use the utility script:

```bash
# Example using probe-rs
./build f
# or
./build d
```

## Development Notes

- Run `./build c` before submitting changes to lint the release build.
- Use `cargo outdated --root-deps-only` to check whether direct dependencies can be updated.
- `./build t` runs the host test package against the production parser, output policy, USB service
  function and release encoding. Mock-bus tests use the real `usb-device`/`usbd-serial` stack and
  the same workspace lockfile as the firmware. They cover batched/buffered commands, all channel
  and duty values, every frame partition, receive-budget continuation, reset/suspend, malformed
  input, partial-frame expiry, 10-second inactivity, slow updates and startup cancellation.
- Run `cargo audit` to check the workspace lockfile for advisories. The current `bare-metal`
  deprecation warnings come through `cortex-m` and `stm32f4xx-hal`; there is no patched version
  for RUSTSEC-2026-0110.
- Contributor guidance lives in `AGENTS.md`. Agent-specific workflow notes live in `CLAUDE.md`.

### Hardware validation

Host tests do not exercise the STM32 USB controller, timer registers, reset flags or IWDG. Before
deploying this firmware, validate the following on a BlackPill (these bench checks remain pending):

1. Enumerate on Linux with the cable attached at boot, reconnect, and reset/reflash without
   unplugging. Confirm the new serial number and the existing VID/PID and CDC descriptors.
2. Send individual, fragmented and batched four-byte commands; verify the intended PA0-PA3
   channels at 50 kHz, including values 0, 128 and 255 with the original duty scaling.
3. Stop valid commands and confirm all channels return to minimum after 10 seconds. Send updates
   nine seconds apart and confirm they are held. Malformed bytes must not postpone the timeout.
4. Interrupt a partial frame, wait at least 100 ms, then send a fresh command. Also reset or suspend
   USB mid-frame and confirm a fresh session starts cleanly with outputs at minimum.
5. Send commands before and during startup; confirm the sweep never overwrites them. Disconnect
   during startup and confirm it stops. Exercise sustained traffic across the receive budget.
6. Force a panic/fault and a service-task stall using a debug build or debugger. Confirm reset,
   minimum outputs, USB re-enumeration, and sweep suppression after software/IWDG resets.

## Dependencies

- [`rtic`](https://rtic.rs/) and [`rtic-monotonics`](https://github.com/rtic-rs/rtic) provide the
  task runtime and TIM2 monotonic timer.
- [`stm32f4xx-hal`](https://github.com/stm32-rs/stm32f4xx-hal) provides the STM32F411 hardware
  abstraction layer.
- [`usb-device`](https://github.com/rust-embedded-community/usb-device) and
  [`usbd-serial`](https://github.com/rust-embedded-community/usbd-serial) provide the USB CDC serial
  stack.
- [`cortex-m`](https://github.com/rust-embedded/cortex-m),
  [`heapless`](https://github.com/rust-embedded/heapless),
  and [`cortex-m-rt`](https://github.com/rust-embedded/cortex-m) cover core embedded support,
  fixed-capacity strings, and exception handling. Panic recovery is implemented in the firmware.
- [`anyhow`](https://github.com/dtolnay/anyhow) is used by the build script.

## License

MIT
