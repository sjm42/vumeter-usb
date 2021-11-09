// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
// #![deny(warnings)]

use panic_semihosting as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
mod app {
    use cortex_m::asm;
    use rtic::rtic_monotonic::Milliseconds;
    use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    use systick_monotonic::Systick;
    use usb_device::prelude::*;

    #[monotonic(binds=SysTick, default=true)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum Cmd {
        None = 0x00,
        Set1 = 0x31,
        Set2,
        Set3,
        Set4,
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum CmdState {
        Start1 = 0xFE,
        Start2 = 0x01,
        Cmd,
        Data,
        Pending,
    }

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        led_on: bool,
        led: ErasedPin<Output<PushPull>>,
        state: CmdState,
        cmd: Cmd,
        data: u8,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(72.mhz())
            .hclk(72.mhz())
            .pclk1(36.mhz())
            .pclk2(72.mhz())
            .adcclk(12.mhz())
            .freeze(&mut flash.acr);
        assert!(clocks.usbclk_valid());

        // Initialize the monotonic
        // #[cfg(feature = "nope")]
        let mono = MyMono::new(cx.core.SYST, clocks.sysclk().0);
        // mono.enable_timer();

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpioc = cx.device.GPIOC.split();

        // On blue pill stm32f103 user led is on PC13, active low
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh).erase();
        led.set_high();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        asm::delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: cx.device.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Siuro Hacklab")
        .product("Serial VU meter")
        .serial_number("4242")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        (
            Shared {
                usb_dev,
                serial,
                led,
                led_on: false,
                state: CmdState::Start1,
                cmd: Cmd::None,
                data: 0,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            // could be busyloop as well
            asm::wfi();
        }
    }

    #[task(priority=2, shared=[led_on, led])]
    fn led_blink(cx: led_blink::Context) {
        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if *led_on == false {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(Milliseconds(50u32)).ok();
            }
        });
    }

    #[task(priority=2, shared=[led_on, led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=5, binds = USB_HP_CAN_TX, shared = [usb_dev, serial, state, cmd, data])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;
        let mut data = cx.shared.data;
        (&mut usb_dev, &mut serial, &mut state, &mut cmd, &mut data).lock(
            |usb_dev, serial, state, cmd, data| {
                usb_poll(usb_dev, serial, state, cmd, data);
            },
        );
    }

    #[task(priority=5, binds = USB_LP_CAN_RX0, shared = [usb_dev, serial, state, cmd, data])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;
        let mut data = cx.shared.data;

        (&mut usb_dev, &mut serial, &mut state, &mut cmd, &mut data).lock(
            |usb_dev, serial, state, cmd, data| {
                usb_poll(usb_dev, serial, state, cmd, data);
            },
        );
    }

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        serial: &mut usbd_serial::SerialPort<'static, B>,
        state: &mut CmdState,
        cmd: &mut Cmd,
        data: &mut u8,
    ) {
        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 32];
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                for c in buf[0..count].iter_mut() {
                    match *state {
                        CmdState::Start1 => {
                            if *c == CmdState::Start1 as u8 {
                                *state = CmdState::Start2;
                            }
                        }
                        CmdState::Start2 => {
                            if *c == CmdState::Start2 as u8 {
                                *state = CmdState::Cmd;
                            } else {
                                *state = CmdState::Start1;
                            }
                        }
                        CmdState::Cmd => {
                            led_blink::spawn().ok();
                            *state = CmdState::Data;
                            // We cannot use TryInto trait here since we are no_std, sigh
                            match *c {
                                0x31 => *cmd = Cmd::Set1,
                                0x32 => *cmd = Cmd::Set2,
                                0x33 => *cmd = Cmd::Set3,
                                0x34 => *cmd = Cmd::Set4,
                                _ => {
                                    // Unknown Cmd
                                    *state = CmdState::Start1;
                                    *cmd = Cmd::None;
                                }
                            }
                        }
                        CmdState::Data => {
                            *state = CmdState::Pending;
                            *data = *c;
                            do_cmd::spawn().ok();
                        }
                        CmdState::Pending => {
                            // Well as long as we havethe command pending,
                            // we have to throw away incoming serial data
                        }
                    }
                }
            }
            _ => {}
        }
    }

    #[task(priority=3, shared=[cmd, data, state])]
    fn do_cmd(cx: do_cmd::Context) {
        let mut cmd = cx.shared.cmd;
        let mut data = cx.shared.data;
        let mut state = cx.shared.state;

        (&mut cmd, &mut data, &mut state).lock(|cmd, data, state| {
            if *state == CmdState::Pending {
                // act on the command here

                *state = CmdState::Start1;
                *cmd = Cmd::None;
                *data = 0;
            }
        });

        // led_blink::spawn().ok();
    }
}
// EOF
