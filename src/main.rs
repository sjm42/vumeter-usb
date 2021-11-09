// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
// #![deny(warnings)]

use panic_semihosting as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
mod app {
    use cortex_m::asm;
    // use rtic::rtic_monotonic::Milliseconds;
    use stm32f1xx_hal::prelude::*;
    use systick_monotonic::*;
    use usb_device::prelude::*;

    use stm32f1xx_hal::gpio::{ErasedPin, Output, PushPull};
    use stm32f1xx_hal::timer::{Tim2NoRemap, Timer};
    use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    use stm32f1xx_hal::{pac::TIM2, pwm::*};

    #[monotonic(binds=SysTick, default=true)]
    type MyMono = Systick<1000>; // 1000 Hz / 1 ms granularity

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
        Start1 = 0xFD,
        Start2 = 0x02,
        Cmd,
        Data,
        Pending,
    }

    #[derive(Copy, Clone, Debug, PartialEq)]
    #[repr(u8)]
    pub enum HelloState {
        Idle,
        GoUp,
        GoDown,
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
        hello_state: HelloState,
    }

    #[local]
    struct Local {
        i: u8,
        pwm_max: u16,
        pwm1: PwmChannel<TIM2, C1>,
        pwm2: PwmChannel<TIM2, C2>,
        pwm3: PwmChannel<TIM2, C3>,
        pwm4: PwmChannel<TIM2, C4>,
    }

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
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().0);

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpioc = cx.device.GPIOC.split();
        let mut afio = cx.device.AFIO.constrain();

        // Initialize PWM output pins
        let pins = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
        );

        // Set up the timer TIM2 as a PWM output. If selected pins may correspond to different remap options,
        // then you must specify the remap generic parameter. Otherwise, if there is no such ambiguity,
        // the remap generic parameter can be omitted without complains from the compiler.

        // c1 & co type is: PwmChannel<TIM2, C1> etc.
        let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = Timer::tim2(cx.device.TIM2, &clocks)
            .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 1.khz())
            .split();

        let pwm_max = pwm1.get_max_duty();
        pwm1.enable();
        pwm2.enable();
        pwm3.enable();
        pwm4.enable();

        pwm1.set_duty(0);
        pwm2.set_duty(0);
        pwm3.set_duty(0);
        pwm4.set_duty(0);

        // On blue pill stm32f103 user led is on PC13, active low
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh).erase();
        led.set_high();

        // *** Begin USB setup ***

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

        hello::spawn_after(1000.millis()).ok();

        (
            Shared {
                usb_dev,
                serial,
                led,
                led_on: false,
                state: CmdState::Start1,
                cmd: Cmd::None,
                data: 0,
                hello_state: HelloState::GoUp,
            },
            Local {
                pwm_max,
                pwm1,
                pwm2,
                pwm3,
                pwm4,
                i: 0,
            },
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

    #[task(priority=2, capacity=2, shared=[led_on, led])]
    fn led_blink(cx: led_blink::Context) {
        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if !(*led_on) {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(10.millis()).ok();
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

        let mut buf = [0u8; 4];
        if let Ok(count) = serial.read(&mut buf) {
            if count < 1 {
                return;
            }
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
                        set_pwm::spawn().ok();
                    }
                    CmdState::Pending => {
                        // Well as long as we have the command pending,
                        // we have to throw away incoming serial data
                    }
                }
            }
        }
    }

    #[task(priority=3, shared=[cmd, data, state], local=[pwm_max, pwm1, pwm2, pwm3, pwm4])]
    fn set_pwm(cx: set_pwm::Context) {
        let mut cmd = cx.shared.cmd;
        let mut data = cx.shared.data;
        let mut state = cx.shared.state;

        let pwm_max = cx.local.pwm_max;
        let pwm1 = cx.local.pwm1;
        let pwm2 = cx.local.pwm2;
        let pwm3 = cx.local.pwm3;
        let pwm4 = cx.local.pwm4;

        (&mut cmd, &mut data, &mut state).lock(|cmd, data, state| {
            if *state == CmdState::Pending {
                // act on the command here
                match *cmd {
                    Cmd::Set1 => {
                        pwm1.set_duty(((*data as u32 * *pwm_max as u32) / 256u32) as u16);
                    }
                    Cmd::Set2 => {
                        pwm2.set_duty(((*data as u32 * *pwm_max as u32) / 256u32) as u16);
                    }
                    Cmd::Set3 => {
                        pwm3.set_duty(((*data as u32 * *pwm_max as u32) / 256u32) as u16);
                    }
                    Cmd::Set4 => {
                        pwm4.set_duty(((*data as u32 * *pwm_max as u32) / 256u32) as u16);
                    }
                    _ => {}
                }
                *cmd = Cmd::None;
                *state = CmdState::Start1;
                *data = 0;
            }
        });
        led_blink::spawn().ok();
    }

    #[task(priority=1, shared=[cmd, data, state, hello_state], local=[i])]
    fn hello(cx: hello::Context) {
        let mut cmd = cx.shared.cmd;
        let mut data = cx.shared.data;
        let mut state = cx.shared.state;
        let mut hello_state = cx.shared.hello_state;
        let i = cx.local.i;

        (&mut hello_state).lock(|hello_state| {
            if let HelloState::Idle = hello_state {
                return;
            }
            match hello_state {
                HelloState::GoUp => {
                    if *i < 255 {
                        *i += 1;
                    } else {
                        *hello_state = HelloState::GoDown;
                    }
                }
                HelloState::GoDown => {
                    if *i > 0 {
                        *i -= 1;
                    } else {
                        *hello_state = HelloState::Idle;
                    }
                }
                _ => (),
            }

            // Note: all do_cmd calls will block (preempt, execute immediately)
            //  because the task has higher priority.
            // This is by design and exactly what we want here.

            (&mut state, &mut cmd, &mut data).lock(|state, cmd, data| {
                *state = CmdState::Pending;
                *cmd = Cmd::Set1;
                *data = *i;
            });
            set_pwm::spawn().ok();

            (&mut state, &mut cmd, &mut data).lock(|state, cmd, data| {
                *state = CmdState::Pending;
                *cmd = Cmd::Set2;
                *data = *i;
            });
            set_pwm::spawn().ok();

            (&mut state, &mut cmd, &mut data).lock(|state, cmd, data| {
                *state = CmdState::Pending;
                *cmd = Cmd::Set3;
                *data = *i;
            });
            set_pwm::spawn().ok();

            (&mut state, &mut cmd, &mut data).lock(|state, cmd, data| {
                *state = CmdState::Pending;
                *cmd = Cmd::Set4;
                *data = *i;
            });
            set_pwm::spawn().ok();

            if *hello_state != HelloState::Idle {
                hello::spawn_after(5.millis()).ok();
            }
        });
    }
}
// EOF
