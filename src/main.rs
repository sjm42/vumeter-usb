// main.rs

#![no_std]
#![no_main]

// #![allow(unused_mut)]
// #![deny(warnings)]


#[allow(unused_imports)]
use panic_halt;

use stm32f4xx_hal as hal;

use cortex_m::asm;
use hal::otg_fs::{UsbBus, USB};
use hal::watchdog::IndependentWatchdog;
use hal::{gpio::*, prelude::*};
use hal::{pac::TIM5, timer::*};
use rtic_monotonics::stm32::prelude::*;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_serial::SerialPort;


stm32_tim2_monotonic!(Mono, 1_000_000);

const HSE_MHZ: u32 = 25; // external crystal freq
const SYS_MHZ: u32 = 84; // cpu target freq

const CH_OFFSET: u8 = 0x30;
const PWM_NCHAN: u8 = 4;

const USB_VID: u16 = 0x16c0;
const USB_PID: u16 = 0x27dd;

// Make USB serial device globally available
static mut G_USB_SERIAL: Option<SerialPort<UsbBus<USB>>> = None;

// Make USB device globally available
static mut G_USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum CmdState {
    Start1 = 0xFD,
    Start2 = 0x02,
    Chan,
    Data,
}


pub struct MyPwm {
    max: u16,
    pwm: [ErasedChannel<TIM5>; 4],
}
impl MyPwm {
    pub fn set_duty(&mut self, ch: u8, value: u8) {
        if let 0..=3 = ch {
            let d = (((value as u32 * self.max as u32) / 256u32) as u16).max(1);
            self.pwm[ch as usize].set_duty(d);
        }
    }
}


#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM4, DMA2_STREAM5, DMA2_STREAM6, DMA2_STREAM7]
)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        led: ErasedPin<Output<PushPull>>,
    }

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        pwm: crate::MyPwm,
        state: CmdState,
        chan: u8,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<stm32f4xx_hal::otg_fs::UsbBusType>> = None;

        let dp = ctx.device;
        let rcc = dp.RCC.constrain();
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // On Blackpill stm32f411 user led is on PC13, active low
        let mut led = gpioc.pc13.into_push_pull_output().erase();
        led.set_low();

        let clocks = rcc
            .cfgr
            .use_hse(HSE_MHZ.MHz())
            .sysclk(SYS_MHZ.MHz())
            .require_pll48clk()
            .freeze();
        Mono::start(SYS_MHZ * 1_000_000);

        let (_pwm_mgr, (c0, c1, c2, c3)) = dp.TIM5.pwm_hz(1.kHz(), &clocks);
        let mut pwm_c =
            [c0.with(gpioa.pa0).erase(), c1.with(gpioa.pa1).erase(), c2.with(gpioa.pa2).erase(), c3.with(gpioa.pa3).erase()];

        for p in &mut pwm_c {
            p.set_duty(1);
            p.enable();
        }

        let pwm = MyPwm {
            max: pwm_c[0].get_max_duty(),
            pwm: pwm_c,
        };


        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );

        let usb_bus = unsafe {
            USB_BUS = Some(stm32f4xx_hal::otg_fs::UsbBusType::new(usb, &mut *core::ptr::addr_of_mut!( EP_MEMORY)));
            USB_BUS.as_ref().unwrap()
        };

        // let sernum = format!("", hal::signature::Uid::get());

        unsafe {
            G_USB_SERIAL = Some(SerialPort::new(usb_bus));
            G_USB_DEVICE = Some(
                UsbDeviceBuilder::new(usb_bus, UsbVidPid(USB_VID, USB_PID))
                    .device_class(usbd_serial::USB_CLASS_CDC)
                    .strings(&[StringDescriptors::default()
                        .manufacturer("Siuro Hacklab")
                        .product("PWM Controller")
                        .serial_number("4242")])
                    .unwrap()
                    .build(),
            );
        }


        // Signal hello with pwm
        // hello::spawn().ok();

        // feed the watchdog
        periodic::spawn().ok();

        // Start the hardware watchdog
        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(5000u32.millis());

        // testing
        led.set_high();

        (
            Shared {
                led,
            },
            Local {
                watchdog,
                pwm,
                state: CmdState::Start1,
                chan: 0,
            },
        )
    }


    // Background task, runs whenever no other tasks are running
    #[idle()]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::wfe();
        }
    }


    // Feed the watchdog to avoid hardware reset.
    #[task(priority=2, local=[watchdog])]
    async fn periodic(ctx: periodic::Context) {
        loop {
            ctx.local.watchdog.feed();
            Mono::delay(1000u64.millis()).await;
        }
    }


    #[task(priority=1, shared=[led])]
    async fn led_blink(ctx: led_blink::Context, ms: u64) {
        let led_blink::SharedResources {
            mut led,
            ..
        } = ctx.shared;

        (&mut led,).lock(|led| {
            led.set_low();
        });
        led_off::spawn(ms).ok();
    }

    #[task(priority=1, shared=[led])]
    async fn led_off(ctx: led_off::Context, ms: u64) {
        Mono::delay(ms.millis()).await;

        let led_off::SharedResources {
            mut led,
            ..
        } = ctx.shared;
        (&mut led,).lock(|led| {
            led.set_high();
        });
    }


    #[task(priority=5, binds=OTG_FS, local=[state, chan])]
    fn usb_fs(cx: usb_fs::Context) {
        static mut USB_SERIAL: Option<SerialPort<UsbBus<USB>>> = None;
        static mut USB_DEVICE: Option<UsbDevice<UsbBus<USB>>> = None;

        let usb_dev = unsafe {
            USB_DEVICE.get_or_insert_with(|| {
                // Move USB device here, leaving a None in its place
                G_USB_DEVICE.take().unwrap()
            })
        };

        let serial = unsafe {
            USB_SERIAL.get_or_insert_with(|| {
                // Move USB serial device here, leaving a None in its place
                G_USB_SERIAL.take().unwrap()
            })
        };

        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        let usb_fs::LocalResources {
            state,
            chan,
            ..
        } = cx.local;


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
                            *state = CmdState::Chan;
                        } else {
                            *state = CmdState::Start1;
                        }
                    }
                    CmdState::Chan => {
                        // We cannot use TryInto trait here since we are no_std, sigh
                        if *c >= CH_OFFSET && *c < CH_OFFSET + PWM_NCHAN {
                            *state = CmdState::Data;
                            *chan = *c - CH_OFFSET;
                        } else {
                            // Unknown Cmd
                            *state = CmdState::Start1;
                            *chan = 0x00;
                        }
                    }
                    CmdState::Data => {
                        // higher priority will execute immediately
                        set_pwm::spawn(*chan, *c).ok();
                        *state = CmdState::Start1;
                    }
                }
            }
        }
    }


    #[task(priority=7, local=[pwm])]
    async fn set_pwm(cx: set_pwm::Context, ch: u8, da: u8) {
        // we only support 4 channels here
        if !(0..PWM_NCHAN).contains(&ch) {
            return;
        }
        cx.local.pwm.set_duty(ch, da);
        led_blink::spawn(10).ok();
    }


    const HELLO_MIN: u8 = 1;
    const HELLO_MAX: u8 = 255;
    const HELLO_DELAY: u64 = 100;

    #[task(priority = 1)]
    async fn hello(_cx: hello::Context) {
        Mono::delay(2000u64.millis()).await;

        for d in HELLO_MIN..=HELLO_MAX {
            for ch in 0..PWM_NCHAN {
                set_pwm::spawn(ch, d).ok();
            }
            Mono::delay(10u64.millis()).await;
        }

        Mono::delay(HELLO_DELAY.millis()).await;

        for d in (HELLO_MIN..=HELLO_MAX).rev() {
            for ch in 0..PWM_NCHAN {
                set_pwm::spawn(ch, d).ok();
            }
            Mono::delay(10u64.millis()).await;
        }
    }
}
// EOF
