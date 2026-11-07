//! Bounded USB servicing, shared with the host-side USB regression tests.

use usb_device::{UsbError, class_prelude::UsbClass, prelude::*};
use usbd_serial::SerialPort;

use crate::control::Controller;

pub const RX_PACKET_SIZE: usize = 64;
pub const RX_READ_BUDGET: usize = 4;

pub fn service<B: usb_device::bus::UsbBus>(
    device: &mut UsbDevice<'_, B>,
    serial: &mut SerialPort<'_, B>,
    control: &mut Controller,
    now_ms: u64,
) -> Result<(), UsbError> {
    let mut buffer = [0; RX_PACKET_SIZE];
    for _ in 0..RX_READ_BUDGET {
        // poll() reports bus events, not bytes already in SerialPort's buffer.
        // Always attempt a read while configured, regardless of poll's result.
        device.poll(&mut [serial]);
        let connected = device.state() == UsbDeviceState::Configured;
        control.set_connected(connected);
        if !connected {
            // Drop any class-buffered bytes as well as the partial application frame.
            serial.reset();
            break;
        }
        match serial.read(&mut buffer) {
            Ok(0) | Err(UsbError::WouldBlock) => break,
            Ok(count) => control.receive(&buffer[..count], now_ms),
            Err(error) => {
                control.fail_safe();
                return Err(error);
            }
        }
    }
    Ok(())
}
