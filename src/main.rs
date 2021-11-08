// main.rs

#![no_std]
#![no_main]
#![allow(non_snake_case)]
// #![deny(unsafe_code)]
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

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: usbd_serial::SerialPort<'static, UsbBusType>,
        led_on: bool,
        led: ErasedPin<Output<PushPull>>,
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

        seppo::spawn().ok();

        (
            Shared {
                usb_dev,
                serial,
                led,
                led_on: false,
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

    #[task(priority=3, shared=[led_on, led])]
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

    #[task(priority=3, shared=[led_on, led])]
    fn led_off(cx: led_off::Context) {
        let mut led = cx.shared.led;
        let mut led_on = cx.shared.led_on;
        (&mut led, &mut led_on).lock(|led, led_on| {
            led.set_high();
            *led_on = false;
        });
    }

    #[task(priority=5, binds = USB_HP_CAN_TX, shared = [usb_dev, serial])]
    fn usb_tx(cx: usb_tx::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, serial);
        });
    }

    #[task(priority=5, binds = USB_LP_CAN_RX0, shared = [usb_dev, serial])]
    fn usb_rx0(cx: usb_rx0::Context) {
        let mut usb_dev = cx.shared.usb_dev;
        let mut serial = cx.shared.serial;

        (&mut usb_dev, &mut serial).lock(|usb_dev, serial| {
            usb_poll(usb_dev, serial);
        });
    }

    #[task(priority=1, shared = [serial])]
    fn seppo(cx: seppo::Context) {
        let mut serial = cx.shared.serial;
        (&mut serial).lock(|serial| {
            serial.write("Seppo!\n\r".as_ref()).ok();
        });
        seppo::spawn_after(Milliseconds(5000u32)).ok();
    }

    fn usb_poll<B: usb_device::bus::UsbBus>(
        usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
        serial: &mut usbd_serial::SerialPort<'static, B>,
    ) {
        if !usb_dev.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 8];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }
                serial.write(&buf[0..count]).ok();
                led_blink::spawn().ok();
            }
            _ => {}
        }
    }
}
// EOF
