#![no_std]
#![no_main]

mod control;
mod usb;

use core::{fmt::Write, panic::PanicInfo};

use cortex_m::asm;
use hal::{
    gpio::{ErasedPin, Output, PushPull},
    otg_fs::{USB, UsbBusType},
    pac::TIM5,
    prelude::*,
    timer::ErasedChannel,
    watchdog::IndependentWatchdog,
};
use heapless::String;
use rtic_monotonics::stm32::prelude::*;
use stm32f4xx_hal as hal;
use usb_device::{class_prelude::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort;

use crate::control::{Controller, PWM_NCHAN};

stm32_tim2_monotonic!(Mono, 1_000_000);

const HSE_MHZ: u32 = 25;
const SYS_MHZ: u32 = 84;
const USB_VID: u16 = 0x16c0;
const USB_PID: u16 = 0x27dd;
const SER_NUM_LEN: usize = 32;

include!(concat!(env!("OUT_DIR"), "/usb_device_release.rs"));

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    fault_reset()
}

#[cortex_m_rt::exception(trampoline = false)]
unsafe fn HardFault() -> ! {
    fault_reset()
}

#[cortex_m_rt::exception]
unsafe fn DefaultHandler(_irq: i16) {
    fault_reset()
}

fn fault_reset() -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: fault recovery never returns and all maskable interrupts are disabled.
    // Only TIM5's compare registers are touched, restoring the normal minimum duty.
    // Writes before TIM5 is clocked are harmless; reset subsequently initializes it.
    let timer = unsafe { &*TIM5::ptr() };
    for ccr in timer.ccr_iter() {
        ccr.write(|w| w.ccr().set(1));
    }
    cortex_m::peripheral::SCB::sys_reset()
}

struct Pwm {
    max: u16,
    channels: [ErasedChannel<TIM5>; PWM_NCHAN],
    applied: [u8; PWM_NCHAN],
}

impl Pwm {
    fn apply(&mut self, duties: &[u8; PWM_NCHAN]) {
        for (index, &value) in duties.iter().enumerate() {
            if value != self.applied[index] {
                // Preserve the original scaling, including the nonzero minimum.
                let duty = ((u32::from(value) * u32::from(self.max) / 256) as u16).max(1);
                self.channels[index].set_duty(duty);
                self.applied[index] = value;
            }
        }
    }
}

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [DMA2_STREAM4])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        watchdog: IndependentWatchdog,
        pwm: Pwm,
        led: ErasedPin<Output<PushPull>>,
        usb_device: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,
        control: Controller,
    }

    #[init(local = [
        ep_memory: [u32; 1024] = [0; 1024],
        usb_bus: Option<UsbBusAllocator<UsbBusType>> = None,
        serial_number: String<SER_NUM_LEN> = String::new(),
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let dp = ctx.device;
        let reset = dp.RCC.csr().read();
        let fault_boot = reset.sftrstf().bit_is_set() || reset.wdgrstf().bit_is_set() || reset.wwdgrstf().bit_is_set();
        dp.RCC.csr().modify(|_, w| w.rmvf().set_bit());

        // Cover initialization failures as well as failures in the service task.
        let mut watchdog = IndependentWatchdog::new(dp.IWDG);
        watchdog.start(5000u32.millis());

        let uid = hal::signature::Uid::get();
        let serial_number = ctx.local.serial_number;
        // At most 23 bytes: 7 lot characters, wafer number, X/Y and separators.
        write!(
            serial_number,
            "{}-{}-{}-{}",
            uid.lot_num(),
            uid.waf_num(),
            uid.x(),
            uid.y()
        )
        .expect("USB serial number fits its buffer");

        let mut rcc = dp.RCC.freeze(
            hal::rcc::Config::hse(HSE_MHZ.MHz())
                .sysclk(SYS_MHZ.MHz())
                .require_pll48clk(),
        );
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);
        let mut led = gpioc.pc13.into_push_pull_output().erase();
        led.set_high();

        Mono::start(rcc.clocks.timclk1().raw());

        let (_manager, (c0, c1, c2, c3)) = dp.TIM5.pwm_hz(50.kHz(), &mut rcc);
        let mut channels = [
            c0.with(gpioa.pa0).erase(),
            c1.with(gpioa.pa1).erase(),
            c2.with(gpioa.pa2).erase(),
            c3.with(gpioa.pa3).erase(),
        ];
        for channel in &mut channels {
            channel.set_duty(1);
            channel.enable();
        }
        let pwm = Pwm {
            max: channels[0].get_max_duty(),
            channels,
            applied: [0; PWM_NCHAN],
        };

        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &rcc.clocks,
        );
        // RTIC gives init-local storage a unique 'static lifetime. The device and
        // serial class then belong exclusively to the service task.
        let bus = ctx.local.usb_bus.insert(UsbBusType::new(usb, ctx.local.ep_memory));
        let serial = SerialPort::new(bus);
        let usb_device = UsbDeviceBuilder::new(bus, UsbVidPid(USB_VID, USB_PID))
            .device_class(usbd_serial::USB_CLASS_CDC)
            .device_release(USB_DEVICE_RELEASE_BCD)
            .strings(&[StringDescriptors::default()
                .manufacturer("Siuro Hacklab")
                .product("PWM Controller")
                .serial_number(serial_number.as_str())])
            .expect("one USB descriptor language fits")
            .build();

        // Ensure Linux observes a disconnect even after a rapid fault reset or reflash.
        let mut delay = ctx.core.SYST.delay(&rcc.clocks);
        usb_device.bus().force_reset(&mut delay).expect("USB reconnect failed");

        service::spawn().expect("service starts once");
        (
            Shared {},
            Local {
                watchdog,
                pwm,
                led,
                usb_device,
                serial,
                control: Controller::new(Mono::now().ticks() / 1000, !fault_boot),
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::wfe();
        }
    }

    // USB is polled every millisecond with a fixed receive budget. Buffered bytes
    // left at the budget limit are serviced on the next tick, even without an IRQ.
    #[task(priority = 2, local = [watchdog, pwm, led, usb_device, serial, control])]
    async fn service(ctx: service::Context) {
        loop {
            let now_ms = Mono::now().ticks() / 1000;
            usb::service(ctx.local.usb_device, ctx.local.serial, ctx.local.control, now_ms)
                .expect("USB receive failed");
            ctx.local.control.tick(now_ms);
            ctx.local.pwm.apply(ctx.local.control.duties());
            if ctx.local.control.led_on(now_ms) {
                ctx.local.led.set_low();
            } else {
                ctx.local.led.set_high();
            }
            // Feeding here proves that USB servicing, timeouts and PWM all ran.
            // A quiet host is healthy and does not prevent feeding the watchdog.
            ctx.local.watchdog.feed();
            Mono::delay(1u64.millis()).await;
        }
    }
}
