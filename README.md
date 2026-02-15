# vumeter-usb

Bare-metal Rust firmware for a **STM32F411 "Blackpill"** board that acts as a USB-controlled 4-channel PWM VU meter.

The device enumerates as a USB CDC serial device. A host application sends simple 4-byte commands over the virtual serial port to set PWM duty cycles on four independent output channels (PA0–PA3), driving analog VU meter movements or LEDs.

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
| Serial       | Derived from MCU unique ID |

## Serial Protocol

Commands are 4 bytes:

```
Byte 0: 0xFD  (start marker 1)
Byte 1: 0x02  (start marker 2)
Byte 2: 0x30 + channel  (channel 0-3)
Byte 3: duty value  (0-255)
```

The duty value is scaled to the PWM timer's resolution. A value of 0 sets the minimum duty (not fully off); 255 sets maximum duty.

### Example

Set channel 2 to half of max value:

```
0xFD 0x02 0x32 0x80
```

## Startup Behavior

On power-up, after a 2-second delay the firmware runs a startup animation that ramps all four PWM channels from minimum to maximum and back down. A hardware watchdog (IWDG) is started with a 5-second timeout and fed every second.

## Building

Requires the `thumbv7em-none-eabihf` Rust target:

```bash
rustup target add thumbv7em-none-eabihf
```

Build:

```bash
./build build
```

The release profile is optimized for size (`opt-level = 'z'`, LTO enabled).

## Flashing

Flash the resulting binary to the Blackpill using your preferred method (ST-Link, DFU, etc.)
or use the utility script:

```bash
# Example using probe-rs
./build flash
# or
./build dfu-flash
```

## Dependencies

- [RTIC v2](https://rtic.rs/) -- Real-Time Interrupt-driven Concurrency framework
- [stm32f4xx-hal](https://github.com/stm32-rs/stm32f4xx-hal) -- Hardware abstraction layer
- [usb-device](https://github.com/rust-embedded-community/usb-device) + [usbd-serial](https://github.com/rust-embedded-community/usbd-serial) -- USB CDC serial stack
- [stm32-usbd](https://github.com/stm32-rs/stm32-usbd) -- USB peripheral driver

## License

MIT
