// main.rs

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
// #![allow(non_snake_case)]

// #![deny(warnings)]
#![allow(unused_mut)]

extern crate alloc;
extern crate no_std_compat as std;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
use panic_halt as _;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[allow(clippy::empty_loop)]
#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

// use either RTIC line depending on hardware

// #[rtic::app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [DMA1_CHANNEL1, DMA1_CHANNEL2, DMA1_CHANNEL3])]
#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM1, DMA2_STREAM2, DMA2_STREAM3])]
mod app {
    #[cfg(feature = "stm32f103")]
    use stm32f1xx_hal as hal;

    #[cfg(feature = "stm32f411")]
    use stm32f4xx_hal as hal;

    // use crate::hal;
    use cortex_m::asm;

    #[cfg(feature = "stm32f411")]
    use hal::otg_fs::{UsbBus, UsbBusType, USB};
    #[cfg(feature = "stm32f103")]
    use hal::usb::{Peripheral, UsbBus, UsbBusType};

    use hal::watchdog::IndependentWatchdog;
    use hal::{gpio::*, prelude::*};
    use hal::{pac::TIM2, timer::*};
    use std::prelude::v1::*;
    use systick_monotonic::*;
    use usb_device::prelude::*;

    const CMD_OFFSET: u8 = 0x30;
    const PWM_MAX: u8 = 192;

    #[monotonic(binds=SysTick, default=true)]
    type MyMono = Systick<10_000>; // 10 kHz / 100 µs granularity

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(u8)]
    pub enum CmdState {
        Start1 = 0xFD,
        Start2 = 0x02,
        Cmd,
        Data,
    }

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    #[repr(u8)]
    pub enum HelloState {
        Idle,
        GoUp,
        GoDown,
    }

    pub struct MyPwm {
        max: u16,
        ch1: PwmChannel<TIM2, C1>,
        ch2: PwmChannel<TIM2, C2>,
        ch3: PwmChannel<TIM2, C3>,
        ch4: PwmChannel<TIM2, C4>,
    }
    impl MyPwm {
        pub fn set_duty(&mut self, ch: u8, value: u8) {
            let d = (((value as u32 * self.max as u32) / 256u32) as u16).max(1);
            match ch {
                1 => self.ch1.set_duty(d),
                2 => self.ch2.set_duty(d),
                3 => self.ch3.set_duty(d),
                4 => self.ch4.set_duty(d),
                _ => {}
            }
        }
    }

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        led_on: bool,
        led: ErasedPin<Output<PushPull>>,
        state: CmdState,
        cmd: u8,
        pwm: MyPwm,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        hello_state: HelloState,
        i: u8,
    }

    #[cfg(feature = "stm32f411")]
    static mut EP_MEMORY: [u32; 1024] = [0; 1024];

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let dp = cx.device;

        #[cfg(feature = "stm32f103")]
        let mut flash = dp.FLASH.constrain();

        let rcc = dp.RCC.constrain();

        #[cfg(feature = "stm32f103")]
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(72.MHz())
            .hclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .adcclk(12.MHz())
            .freeze(&mut flash.acr);
        #[cfg(feature = "stm32f411")]
        let clocks = rcc
            .cfgr
            .use_hse(25.MHz())
            .sysclk(84.MHz())
            .require_pll48clk()
            .freeze();

        #[cfg(feature = "stm32f103")]
        assert!(clocks.usbclk_valid());

        // Initialize the monotonic
        let mono = Systick::new(cx.core.SYST, clocks.sysclk().raw());

        let mut gpioa = dp.GPIOA.split();
        let mut gpioc = dp.GPIOC.split();

        #[cfg(feature = "stm32f103")]
        let mut afio = dp.AFIO.constrain();

        // On blue pill stm32f103 user led is on PC13, active low
        // On Blackpill stm32f411 user led is on PC13, active low
        #[cfg(feature = "blue_pill")]
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh).erase();
        #[cfg(feature = "black_pill")]
        let mut led = gpioc.pc13.into_push_pull_output().erase();

        led.set_high();

        // Initialize PWM output pins
        #[cfg(feature = "blue_pill")]
        let pins = (
            gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl),
        );
        // Set up the timer TIM2 as a PWM output. If selected pins may correspond to different remap options,
        // then you must specify the remap generic parameter. Otherwise, if there is no such ambiguity,
        // the remap generic parameter can be omitted without complains from the compiler.
        #[cfg(feature = "blue_pill")]
        let (ch1, ch2, ch3, ch4) = Timer::new(dp.TIM2, &clocks)
            .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1.kHz())
            .split();
        // c1 & co type is: PwmChannel<TIM2, C1> etc.

        #[cfg(feature = "black_pill")]
        let pins = (
            gpioa.pa0.into_alternate(),
            gpioa.pa1.into_alternate(),
            gpioa.pa2.into_alternate(),
            gpioa.pa3.into_alternate(),
        );
        #[cfg(feature = "black_pill")]
        let (ch1, ch2, ch3, ch4) = Timer::new(dp.TIM2, &clocks).pwm_hz(pins, 1.kHz()).split();

        let pwm = MyPwm {
            max: ch1.get_max_duty(),
            ch1,
            ch2,
            ch3,
            ch4,
        };

        // *** Begin USB setup ***

        #[cfg(feature = "stm32f103")]
        {
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
                usb: dp.USB,
                pin_dm: usb_dm,
                pin_dp: usb_dp,
            };
            unsafe {
                USB_BUS.replace(UsbBus::new(usb));
            }
        }

        #[cfg(feature = "stm32f411")]
        {
            let usb = USB {
                usb_global: dp.OTG_FS_GLOBAL,
                usb_device: dp.OTG_FS_DEVICE,
                usb_pwrclk: dp.OTG_FS_PWRCLK,
                pin_dm: gpioa.pa11.into_alternate(),
                pin_dp: gpioa.pa12.into_alternate(),
                hclk: clocks.hclk(),
            };
            unsafe {
                USB_BUS.replace(UsbBus::new(usb, &mut EP_MEMORY));
            }
        }

        let serial = usbd_serial::SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap() },
            UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("Siuro Hacklab")
        .product("PWM controller")
        .serial_number("pwm42")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        // Set "busy" pin up each 100ms, feed the watchdog
        periodic::spawn().ok();

        let duty_init = 28;
        for ch in 1..=4u8 {
            set_pwm::spawn_after(100u64.millis(), CMD_OFFSET + ch, duty_init).ok();
        }

        // Signal hello with pwm
        hello::spawn_after(2000u64.millis()).ok();

        // Start the hardware watchdog
        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(500u32.millis());

        (
            Shared {
                usb_dev,
                serial,
                led,
                led_on: false,
                state: CmdState::Start1,
                cmd: 0,
                pwm,
            },
            Local {
                watchdog,
                hello_state: HelloState::GoUp,
                i: duty_init,
            },
            init::Monotonics(mono),
        )
    }

    // Background task, runs whenever no other tasks are running
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            // Wait for interrupt...
            asm::wfi();
        }
    }

    // Create pulses on "busy" pin each 100 milliseconds even when nothing else is active
    // and feed the watchdog to avoid hardware reset.
    #[task(priority=1, local=[watchdog])]
    fn periodic(cx: periodic::Context) {
        cx.local.watchdog.feed();
        periodic::spawn_after(100u64.millis()).ok();
    }

    #[task(priority=2, capacity=2, shared=[led_on, led])]
    fn led_blink(cx: led_blink::Context, ms: u64) {
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

    #[task(priority=2, shared=[led_on, led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    // on stm32f411 use this for USB

    #[task(priority=5, binds=OTG_FS, shared=[usb_dev, serial, state, cmd])]
    fn usb_fs(cx: usb_fs::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;
        let mut state = cx.shared.state;
        let mut cmd = cx.shared.cmd;

        (&mut usb_dev, &mut serial, &mut state, &mut cmd).lock(|usb_dev, serial, state, cmd| {
            usb_poll(usb_dev, serial, state, cmd);
        });
    }

    // on stm32f103 use these for USB
    /*
       #[task(priority=5, binds=USB_HP_CAN_TX, shared=[usb_dev, serial, state, cmd])]
       fn usb_tx(cx: usb_tx::Context) {
           let mut usb_dev = cx.shared.usb_dev;
           let mut serial = cx.shared.serial;
           let mut state = cx.shared.state;
           let mut cmd = cx.shared.cmd;

           (&mut usb_dev, &mut serial, &mut state, &mut cmd).lock(|usb_dev, serial, state, cmd| {
               usb_poll(usb_dev, serial, state, cmd);
           });
       }

       #[task(priority=5, binds=USB_LP_CAN_RX0, shared=[usb_dev, serial, state, cmd])]
       fn usb_rx0(cx: usb_rx0::Context) {
           let mut usb_dev = cx.shared.usb_dev;
           let mut serial = cx.shared.serial;
           let mut state = cx.shared.state;
           let mut cmd = cx.shared.cmd;

           (&mut usb_dev, &mut serial, &mut state, &mut cmd).lock(|usb_dev, serial, state, cmd| {
               usb_poll(usb_dev, serial, state, cmd);
           });
       }
    */

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        serial: &mut usbd_serial::SerialPort<'static, B>,
        state: &mut CmdState,
        cmd: &mut u8,
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
                        if *c > CMD_OFFSET && *c < CMD_OFFSET + PWM_MAX {
                            *cmd = *c;
                        } else {
                            // Unknown Cmd
                            *state = CmdState::Start1;
                            *cmd = 0x00;
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

    #[task(priority=7, capacity=8, shared=[pwm])]
    fn set_pwm(cx: set_pwm::Context, ch: u8, da: u8) {
        let mut pwm = cx.shared.pwm;

        // we only support 4 channels here
        if !(CMD_OFFSET + 1..=CMD_OFFSET + 4).contains(&ch) {
            return;
        }

        // i will be 1..4 inclusive
        let i = (ch - CMD_OFFSET) as u8;
        pwm.lock(|pwm| pwm.set_duty(i, da));

        led_blink::spawn(10).ok();
    }

    #[task(priority=1, local=[hello_state, i, d: usize = 0])]
    fn hello(cx: hello::Context) {
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

        for ch in 1..=4u8 {
            set_pwm::spawn(CMD_OFFSET + ch, *i).ok();
        }

        if *hello_state != HelloState::Idle {
            hello::spawn_after(20u64.millis()).ok();
        } else {
            let pwm_idle = 105;
            for ch in 1..=4u8 {
                set_pwm::spawn_after(5000u64.millis(), CMD_OFFSET + ch, pwm_idle).ok();
            }
        }
    }
}
// EOF
