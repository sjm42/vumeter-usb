// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
// #![deny(warnings)]

use panic_halt as _;

#[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
mod app {
    use cortex_m::asm;
    use stm32f1xx_hal as hal;
    // use systick_monotonic::{fugit::*, *};
    use systick_monotonic::*;
    use usb_device::prelude::*;

    use hal::timer::{Tim2NoRemap, Timer};
    use hal::usb::{Peripheral, UsbBus, UsbBusType};
    use hal::{gpio::*, pac, prelude::*};
    use hal::{timer::*, watchdog::IndependentWatchdog};

    #[monotonic(binds=SysTick, default=true)]
    type MyMono = Systick<10_000>; // 10 kHz / 100 µs granularity

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
        busy: ErasedPin<Output<PushPull>>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        hello_state: HelloState,
        i: u8,
        pwm_max: u16,
        pwm1: PwmChannel<pac::TIM2, C1>,
        pwm2: PwmChannel<pac::TIM2, C2>,
        pwm3: PwmChannel<pac::TIM2, C3>,
        pwm4: PwmChannel<pac::TIM2, C4>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = cx.device.FLASH.constrain();
        let rcc = cx.device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .hclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .adcclk(12.MHz())
            .freeze(&mut flash.acr);
        assert!(clocks.usbclk_valid());

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().raw());

        let mut gpioa = cx.device.GPIOA.split();
        let mut gpiob = cx.device.GPIOB.split();
        let mut gpioc = cx.device.GPIOC.split();
        let mut afio = cx.device.AFIO.constrain();

        let mut busy = gpiob.pb15.into_push_pull_output(&mut gpiob.crh).erase();
        busy.set_high();

        // On blue pill stm32f103 user led is on PC13, active low
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh).erase();
        led.set_high();

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

        let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = Timer::new(cx.device.TIM2, &clocks)
            .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz())
            .split();
        // c1 & co type is: PwmChannel<TIM2, C1> etc.
        // let (mut pwm1, mut pwm2, mut pwm3, mut pwm4)

        //= Timer::new(cx.device.TIM2, &clocks)
        //   .pwm::<Tim2NoRemap, _, _, _>(pins, &mut afio.mapr, 1.kHz())
        //   .split();

        let pwm_max = pwm1.get_max_duty();
        pwm1.enable();
        pwm2.enable();
        pwm3.enable();
        pwm4.enable();

        pwm1.set_duty(1);
        pwm2.set_duty(1);
        pwm3.set_duty(1);
        pwm4.set_duty(1);

        // *** Begin USB setup ***

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        asm::delay(clocks.sysclk().raw() / 100);

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
        .product("VU meter")
        .serial_number("vu42")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        // Set "busy" pin up each 100ms, feed the watchdog
        periodic::spawn().ok();

        let duty_init = 28;
        set_pwm::spawn_after(0u64.millis(), Cmd::Set1, duty_init).ok();
        set_pwm::spawn_after(0u64.millis(), Cmd::Set2, duty_init).ok();
        set_pwm::spawn_after(0u64.millis(), Cmd::Set3, duty_init).ok();
        set_pwm::spawn_after(0u64.millis(), Cmd::Set4, duty_init).ok();

        // Signal hello with pwm
        hello::spawn_after(3000u64.millis()).ok();

        // Start the hardware watchdog
        let mut watchdog = IndependentWatchdog::new(cx.device.IWDG);
        watchdog.start(500u32.millis());

        (
            Shared {
                usb_dev,
                serial,
                led,
                led_on: false,
                state: CmdState::Start1,
                cmd: Cmd::None,
                busy,
            },
            Local {
                watchdog,
                hello_state: HelloState::GoUp,
                pwm_max,
                pwm1,
                pwm2,
                pwm3,
                pwm4,
                i: duty_init,
            },
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle(shared=[busy])]
    fn idle(cx: idle::Context) -> ! {
        let mut busy = cx.shared.busy;
        loop {
            busy.lock(|busy| busy.set_low());
            // Wait for interrupt...
            asm::wfi();
        }
    }

    // Create pulses on "busy" pin each 100 milliseconds even when nothing else is active
    // and feed the watchdog to avoid hardware reset.
    #[task(priority=1, shared=[busy], local=[watchdog])]
    fn periodic(cx: periodic::Context) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());
        cx.local.watchdog.feed();
        periodic::spawn_after(100u64.millis()).ok();
    }

    #[task(priority=2, capacity=2, shared=[busy, led_on, led])]
    fn led_blink(cx: led_blink::Context, ms: u64) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let mut led_on = cx.shared.led_on;
        let mut led = cx.shared.led;

        (&mut led, &mut led_on).lock(|led, led_on| {
            if !(*led_on) {
                led.set_low();
                *led_on = true;
                led_off::spawn_after(ms.millis()).ok();
            }
        });
    }

    #[task(priority=2, shared=[busy, led_on, led])]
    fn led_off(cx: led_off::Context) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let mut led = cx.shared.led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=5, binds=USB_HP_CAN_TX, shared=[busy, usb_dev, serial, state, cmd])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;

        (&mut usb_dev, &mut serial, &mut state, &mut cmd).lock(|usb_dev, serial, state, cmd| {
            usb_poll(usb_dev, serial, state, cmd);
        });
    }

    #[task(priority=5, binds=USB_LP_CAN_RX0, shared=[busy, usb_dev, serial, state, cmd])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;

        (&mut usb_dev, &mut serial, &mut state, &mut cmd).lock(|usb_dev, serial, state, cmd| {
            usb_poll(usb_dev, serial, state, cmd);
        });
    }

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        serial: &mut usbd_serial::SerialPort<'static, B>,
        state: &mut CmdState,
        cmd: &mut Cmd,
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
                        // higher priority will execute immediately
                        set_pwm::spawn(*cmd, *c).ok();
                        *state = CmdState::Start1;
                    }
                }
            }
        }
    }

    #[task(priority=7, capacity=8, shared=[busy], local=[pwm_max, pwm1, pwm2, pwm3, pwm4])]
    fn set_pwm(cx: set_pwm::Context, ch: Cmd, da: u8) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let mut d = ((da as u32 * *cx.local.pwm_max as u32) / 256u32) as u16;
        if d < 1 {
            // do not shut down pwm output completely
            d = 1;
        }

        match ch {
            Cmd::Set1 => cx.local.pwm1.set_duty(d),
            Cmd::Set2 => cx.local.pwm2.set_duty(d),
            Cmd::Set3 => cx.local.pwm3.set_duty(d),
            Cmd::Set4 => cx.local.pwm4.set_duty(d),
            _ => {}
        }
        led_blink::spawn(10).ok();
    }

    #[task(priority=1, shared=[busy], local=[hello_state, i, d: usize = 0])]
    fn hello(cx: hello::Context) {
        let mut busy = cx.shared.busy;
        busy.lock(|busy| busy.set_high());

        let hello_state = cx.local.hello_state;
        let i = cx.local.i;

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
                let d = cx.local.d;
                // d is delay...
                if *d < 100 {
                    *d += 1;
                } else if *i > 28 {
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

        set_pwm::spawn(Cmd::Set1, *i).ok();
        set_pwm::spawn(Cmd::Set2, *i).ok();
        set_pwm::spawn(Cmd::Set3, *i).ok();
        set_pwm::spawn(Cmd::Set4, *i).ok();

        if *hello_state != HelloState::Idle {
            hello::spawn_after(20u64.millis()).ok();
        } else {
            let pwm_idle = 105;
            set_pwm::spawn_after(5000u64.millis(), Cmd::Set1, pwm_idle).ok();
            set_pwm::spawn_after(5000u64.millis(), Cmd::Set2, pwm_idle).ok();
            set_pwm::spawn_after(5000u64.millis(), Cmd::Set3, pwm_idle).ok();
            set_pwm::spawn_after(5000u64.millis(), Cmd::Set4, pwm_idle).ok();
        }
    }
}
// EOF
