use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
};

use usb_device::{
    UsbDirection, UsbError,
    bus::{PollResult, UsbBus, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointType},
    prelude::*,
};
use usbd_serial::SerialPort;

use crate::{
    control::{COMM_TIMEOUT_MS, Controller, FRAME_TIMEOUT_MS},
    release::usb_device_release,
    usb,
};

fn command(channel: u8, duty: u8) -> [u8; 4] {
    [0xfd, 0x02, 0x30 + channel, duty]
}

#[test]
fn every_duty_and_channel_survives_every_frame_partition() {
    for channel in 0..4 {
        for duty in 0..=255 {
            for boundaries in 0..8 {
                let mut control = Controller::new(0, false);
                let bytes = command(channel, duty);
                let mut start = 0;
                for end in 1..=4 {
                    if end == 4 || boundaries & (1 << (end - 1)) != 0 {
                        control.receive(&bytes[start..end], end as u64);
                        start = end;
                    }
                }
                let mut expected = [0; 4];
                expected[channel as usize] = duty;
                assert_eq!(control.duties(), &expected);
                assert!(control.led_on(4), "a complete command was parsed");
            }
        }
    }
}

#[test]
fn overlapping_headers_and_invalid_channels_resynchronize() {
    let mut control = Controller::new(0, false);
    control.receive(&[0, 0xfd, 0xfd, 2, 0x30, 0x80], 0);
    control.receive(&[0xfd, 2, 0xfd, 2, 0x31, 0x40], 1);
    control.receive(&[0xfd, 2, 0x34, 0xff], 2);
    control.receive(&command(2, 0xfd), 3);
    assert_eq!(control.duties(), &[0x80, 0x40, 0xfd, 0]);
}

#[test]
fn timeout_discards_each_possible_partial_frame() {
    for length in 1..=3 {
        let mut control = Controller::new(0, false);
        control.receive(&command(0, 123)[..length], 0);
        // Expiry must work even if no periodic tick occurred between reads.
        control.receive(&command(1, 42), FRAME_TIMEOUT_MS);
        assert_eq!(control.duties(), &[0, 42, 0, 0]);
    }
}

#[test]
fn partial_frame_timeout_is_inter_byte_not_total_duration() {
    let mut control = Controller::new(0, false);
    for (index, byte) in command(3, 255).iter().enumerate() {
        control.receive(&[*byte], index as u64 * (FRAME_TIMEOUT_MS - 1));
    }
    assert_eq!(control.duties(), &[0, 0, 0, 255]);
}

#[test]
fn ten_second_timeout_only_tracks_complete_valid_commands() {
    let mut control = Controller::new(0, false);
    control.receive(&command(0, 200), 500);
    control.receive(&command(3, 100), 1000);
    control.tick(1000 + COMM_TIMEOUT_MS - 1);
    assert_eq!(control.duties(), &[200, 0, 0, 100]);
    control.receive(&[0xfd, 2, 0x34, 255, 0xfd, 2, 0x31], 1000 + COMM_TIMEOUT_MS - 1);
    control.tick(1000 + COMM_TIMEOUT_MS);
    assert_eq!(control.duties(), &[0; 4]);
    control.receive(&[255], 1000 + COMM_TIMEOUT_MS + 1);
    assert_eq!(control.duties(), &[0; 4], "failsafe clears the partial frame too");
    control.receive(&command(2, 77), 20_000);
    assert_eq!(control.duties(), &[0, 0, 77, 0]);
}

#[test]
fn slow_applications_can_refresh_before_ten_seconds() {
    let mut control = Controller::new(0, false);
    for now in [0, 9000, 18_000, 27_000] {
        control.receive(&command(0, 128), now);
        control.tick(now + 8999);
        assert_eq!(control.duties(), &[128, 0, 0, 0]);
    }
}

#[test]
fn disconnect_clears_outputs_and_partial_frame_without_restarting_animation() {
    let mut control = Controller::new(0, true);
    control.set_connected(true);
    control.receive(&command(0, 200), 0);
    control.receive(&[0xfd, 2, 0x30], 1);
    control.set_connected(false);
    assert_eq!(control.duties(), &[0; 4]);
    control.set_connected(true);
    control.receive(&command(1, 42), 2);
    control.tick(3000);
    assert_eq!(control.duties(), &[0, 42, 0, 0]);
}

#[test]
fn startup_ramps_all_channels_and_finishes_at_minimum() {
    let mut control = Controller::new(100, true);
    for (elapsed, expected) in [
        (0, 0),
        (1999, 0),
        (2000, 1),
        (2010, 2),
        (4540, 255),
        (4550, 255),
        (4649, 255),
        (4650, 255),
        (4660, 254),
        (7199, 1),
        (7200, 0),
        (100_000, 0),
    ] {
        control.tick(100 + elapsed);
        assert_eq!(control.duties(), &[expected; 4], "elapsed {elapsed}");
    }
}

#[test]
fn first_command_cancels_startup_before_or_during_the_sweep() {
    for arrival in [0, 1999, 2000, 3000, 4600, 6000, 7199] {
        let mut control = Controller::new(0, true);
        control.tick(arrival);
        control.receive(&command(1, 42), arrival);
        for now in [arrival, arrival + 1, arrival + 100, 8000] {
            control.tick(now);
            assert_eq!(control.duties(), &[0, 42, 0, 0]);
        }
    }
}

#[test]
fn recovery_boot_and_disconnect_during_startup_stay_at_minimum() {
    let mut recovery = Controller::new(0, false);
    for now in [0, 2000, 3000, 7200, 10_000] {
        recovery.tick(now);
        assert_eq!(recovery.duties(), &[0; 4]);
    }
    let mut control = Controller::new(0, true);
    control.set_connected(true);
    control.tick(3000);
    assert_ne!(control.duties(), &[0; 4]);
    control.set_connected(false);
    control.tick(4000);
    assert_eq!(control.duties(), &[0; 4]);
}

#[test]
fn led_pulse_expires_without_additional_usb_traffic() {
    let mut control = Controller::new(0, false);
    control.receive(&command(0, 1), 123);
    assert!(control.led_on(123));
    assert!(control.led_on(132));
    assert!(!control.led_on(133));
}

#[test]
fn release_encoding_accepts_only_representable_bcd_versions() {
    for (major, minor, patch, expected) in [(0, 0, 0, 0), (1, 1, 0, 0x0110), (10, 2, 3, 0x1023), (99, 9, 9, 0x9999)] {
        assert_eq!(usb_device_release(major, minor, patch), Ok(expected));
    }
    for (major, minor, patch) in [(100, 0, 0), (1, 10, 0), (1, 1, 10), (1, 1, 16), (u16::MAX, 0, 0)] {
        assert!(usb_device_release(major, minor, patch).is_err());
    }
}

#[derive(Default)]
struct BusState {
    packets: VecDeque<Vec<u8>>,
    setup: Option<[u8; 8]>,
    reset: bool,
    suspend: bool,
    error: Option<UsbError>,
    bulk_out: usize,
    packet_reads: usize,
    polls: usize,
}

struct MockBus {
    state: Arc<Mutex<BusState>>,
    next_endpoint: usize,
}

impl UsbBus for MockBus {
    fn alloc_ep(
        &mut self,
        direction: UsbDirection,
        address: Option<EndpointAddress>,
        kind: EndpointType,
        _: u16,
        _: u8,
    ) -> usb_device::Result<EndpointAddress> {
        let address = address.unwrap_or_else(|| {
            if kind == EndpointType::Control {
                EndpointAddress::from_parts(0, direction)
            } else {
                let index = self.next_endpoint;
                self.next_endpoint += 1;
                EndpointAddress::from_parts(index, direction)
            }
        });
        if direction == UsbDirection::Out && kind == EndpointType::Bulk {
            self.state.lock().unwrap().bulk_out = address.index();
        }
        Ok(address)
    }

    fn enable(&mut self) {}
    fn reset(&self) {
        let mut state = self.state.lock().unwrap();
        state.packets.clear();
        state.setup = None;
    }
    fn set_device_address(&self, _: u8) {}
    fn write(&self, _: EndpointAddress, bytes: &[u8]) -> usb_device::Result<usize> {
        Ok(bytes.len())
    }
    fn read(&self, address: EndpointAddress, buffer: &mut [u8]) -> usb_device::Result<usize> {
        let mut state = self.state.lock().unwrap();
        if address.index() == 0 {
            if let Some(setup) = state.setup.take() {
                buffer[..setup.len()].copy_from_slice(&setup);
                return Ok(setup.len());
            }
        } else if address.index() == state.bulk_out {
            if let Some(error) = state.error.take() {
                return Err(error);
            }
            if let Some(bytes) = state.packets.pop_front() {
                assert!(bytes.len() <= buffer.len());
                buffer[..bytes.len()].copy_from_slice(&bytes);
                state.packet_reads += 1;
                return Ok(bytes.len());
            }
        }
        Err(UsbError::WouldBlock)
    }
    fn set_stalled(&self, _: EndpointAddress, _: bool) {}
    fn is_stalled(&self, _: EndpointAddress) -> bool {
        false
    }
    fn suspend(&self) {}
    fn resume(&self) {}
    fn poll(&self) -> PollResult {
        let mut state = self.state.lock().unwrap();
        state.polls += 1;
        if state.reset {
            state.reset = false;
            PollResult::Reset
        } else if state.suspend {
            state.suspend = false;
            PollResult::Suspend
        } else if state.setup.is_some() {
            PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: 1,
            }
        } else if !state.packets.is_empty() {
            PollResult::Data {
                ep_out: 1 << state.bulk_out,
                ep_in_complete: 0,
                ep_setup: 0,
            }
        } else {
            PollResult::None
        }
    }
}

fn with_usb(
    test: impl FnOnce(&Arc<Mutex<BusState>>, &mut UsbDevice<'_, MockBus>, &mut SerialPort<'_, MockBus>, &mut Controller),
) {
    let state = Arc::new(Mutex::new(BusState::default()));
    let bus = UsbBusAllocator::new(MockBus {
        state: state.clone(),
        next_endpoint: 1,
    });
    let mut serial = SerialPort::new(&bus);
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x16c0, 0x27dd)).build();
    let mut control = Controller::new(0, false);
    configure(&state, &mut device, &mut serial, &mut control, 0);
    test(&state, &mut device, &mut serial, &mut control);
}

fn configure(
    state: &Arc<Mutex<BusState>>,
    device: &mut UsbDevice<'_, MockBus>,
    serial: &mut SerialPort<'_, MockBus>,
    control: &mut Controller,
    now: u64,
) {
    // Standard SET_CONFIGURATION(1), passed through the actual USB control parser.
    state.lock().unwrap().setup = Some([0, 9, 1, 0, 0, 0, 0, 0]);
    usb::service(device, serial, control, now).unwrap();
    assert_eq!(device.state(), UsbDeviceState::Configured);
}

#[test]
fn usb_processes_all_commands_in_one_packet() {
    with_usb(|state, device, serial, control| {
        let bytes: Vec<_> = (0..4).flat_map(|channel| command(channel, 50 + channel)).collect();
        state.lock().unwrap().packets.push_back(bytes);
        usb::service(device, serial, control, 1).unwrap();
        assert_eq!(control.duties(), &[50, 51, 52, 53]);
    });
}

#[test]
fn usb_commands_cross_packet_and_budget_boundaries() {
    with_usb(|state, device, serial, control| {
        let mut stream = vec![0]; // Offset the four-byte frames from packet boundaries.
        for value in 1..=100 {
            stream.extend_from_slice(&command((value - 1) % 4, value));
        }
        for packet in stream.chunks(usb::RX_PACKET_SIZE) {
            state.lock().unwrap().packets.push_back(packet.to_vec());
        }
        usb::service(device, serial, control, 1).unwrap();
        assert_eq!(control.duties(), &[61, 62, 63, 60]);
        usb::service(device, serial, control, 2).unwrap();
        assert_eq!(control.duties(), &[97, 98, 99, 100]);
    });
}

#[test]
fn usb_deconfiguration_returns_to_minimum() {
    with_usb(|state, device, serial, control| {
        control.receive(&command(0, 200), 0);
        state.lock().unwrap().setup = Some([0, 9, 0, 0, 0, 0, 0, 0]);
        usb::service(device, serial, control, 1).unwrap();
        assert_ne!(device.state(), UsbDeviceState::Configured);
        assert_eq!(control.duties(), &[0; 4]);
    });
}

#[test]
fn usb_drains_class_buffer_even_without_another_bus_event() {
    with_usb(|state, device, serial, control| {
        state
            .lock()
            .unwrap()
            .packets
            .push_back([command(0, 128), command(1, 42)].concat());
        assert!(device.poll(&mut [serial]));
        let mut first = [0; 4];
        assert_eq!(serial.read(&mut first).unwrap(), 4);
        control.receive(&first, 1);
        assert!(!device.poll(&mut [serial]));
        usb::service(device, serial, control, 2).unwrap();
        assert_eq!(control.duties(), &[128, 42, 0, 0]);
    });
}

#[test]
fn usb_budget_bounds_work_and_next_tick_continues_without_losing_updates() {
    with_usb(|state, device, serial, control| {
        for value in 1..=10 {
            let packet: Vec<_> = (0..16).flat_map(|_| command(0, value)).collect();
            state.lock().unwrap().packets.push_back(packet);
        }
        let before = state.lock().unwrap().polls;
        usb::service(device, serial, control, 1).unwrap();
        assert_eq!(control.duties()[0], usb::RX_READ_BUDGET as u8);
        let state_after = state.lock().unwrap();
        assert_eq!(state_after.packet_reads, usb::RX_READ_BUDGET);
        assert_eq!(state_after.polls - before, usb::RX_READ_BUDGET);
        drop(state_after);
        usb::service(device, serial, control, 2).unwrap();
        usb::service(device, serial, control, 3).unwrap();
        assert_eq!(control.duties()[0], 10);
        assert!(state.lock().unwrap().packets.is_empty());
    });
}

#[test]
fn usb_reset_clears_a_partial_command_and_outputs() {
    with_usb(|state, device, serial, control| {
        state
            .lock()
            .unwrap()
            .packets
            .push_back([command(0, 200).as_slice(), &[0xfd, 2, 0x30]].concat());
        usb::service(device, serial, control, 1).unwrap();
        state.lock().unwrap().reset = true;
        usb::service(device, serial, control, 2).unwrap();
        assert_eq!(control.duties(), &[0; 4]);
        configure(state, device, serial, control, 3);
        state.lock().unwrap().packets.push_back(command(1, 42).to_vec());
        usb::service(device, serial, control, 4).unwrap();
        assert_eq!(control.duties(), &[0, 42, 0, 0]);
    });
}

#[test]
fn usb_suspend_discards_class_buffered_commands() {
    with_usb(|state, device, serial, control| {
        state
            .lock()
            .unwrap()
            .packets
            .push_back([command(0, 128), command(1, 42)].concat());
        assert!(device.poll(&mut [serial]));
        let mut bytes = [0; 4];
        serial.read(&mut bytes).unwrap();
        control.receive(&bytes, 1);
        state.lock().unwrap().suspend = true;
        usb::service(device, serial, control, 2).unwrap();
        assert_eq!(control.duties(), &[0; 4]);
        configure(state, device, serial, control, 3);
        usb::service(device, serial, control, 4).unwrap();
        assert_eq!(control.duties(), &[0; 4]);
    });
}

#[test]
fn unexpected_usb_error_returns_failure_and_clears_outputs() {
    with_usb(|state, device, serial, control| {
        control.receive(&command(0, 200), 0);
        state.lock().unwrap().error = Some(UsbError::BufferOverflow);
        assert_eq!(usb::service(device, serial, control, 1), Err(UsbError::BufferOverflow));
        assert_eq!(control.duties(), &[0; 4]);
    });
}

#[test]
fn quiet_usb_services_timeout_without_reporting_failure() {
    with_usb(|_, device, serial, control| {
        control.receive(&command(0, 200), 0);
        usb::service(device, serial, control, COMM_TIMEOUT_MS - 1).unwrap();
        control.tick(COMM_TIMEOUT_MS - 1);
        assert_eq!(control.duties(), &[200, 0, 0, 0]);
        usb::service(device, serial, control, COMM_TIMEOUT_MS).unwrap();
        control.tick(COMM_TIMEOUT_MS);
        assert_eq!(control.duties(), &[0; 4]);
    });
}
