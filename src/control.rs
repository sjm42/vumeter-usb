//! Hardware-independent command parsing and output policy. Times are monotonic milliseconds.

pub const PWM_NCHAN: usize = 4;
pub const COMM_TIMEOUT_MS: u64 = 10_000;
pub const FRAME_TIMEOUT_MS: u64 = 100;

#[derive(Clone, Copy)]
enum State {
    Start,
    Header,
    Channel,
    Data(usize),
}

struct Parser {
    state: State,
    last_byte_ms: Option<u64>,
}

impl Parser {
    fn new() -> Self {
        Self {
            state: State::Start,
            last_byte_ms: None,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }

    fn expire(&mut self, now_ms: u64) {
        if self
            .last_byte_ms
            .is_some_and(|last| now_ms.saturating_sub(last) >= FRAME_TIMEOUT_MS)
        {
            self.reset();
        }
    }

    fn push(&mut self, byte: u8, now_ms: u64) -> Option<(usize, u8)> {
        self.expire(now_ms);
        let restart = if byte == 0xfd { State::Header } else { State::Start };
        self.state = match self.state {
            State::Start => restart,
            State::Header if byte == 0x02 => State::Channel,
            State::Header => restart,
            State::Channel if (0x30..0x30 + PWM_NCHAN as u8).contains(&byte) => State::Data((byte - 0x30) as usize),
            State::Channel => restart,
            State::Data(channel) => {
                self.reset();
                return Some((channel, byte));
            }
        };
        self.last_byte_ms = if matches!(self.state, State::Start) {
            None
        } else {
            Some(now_ms)
        };
        None
    }
}

pub struct Controller {
    parser: Parser,
    duties: [u8; PWM_NCHAN],
    started_ms: u64,
    startup: bool,
    connected: bool,
    last_command_ms: Option<u64>,
}

impl Controller {
    pub fn new(now_ms: u64, startup: bool) -> Self {
        Self {
            parser: Parser::new(),
            duties: [0; PWM_NCHAN],
            started_ms: now_ms,
            startup,
            connected: false,
            last_command_ms: None,
        }
    }

    pub fn set_connected(&mut self, connected: bool) {
        if !connected {
            self.parser.reset();
            if self.connected {
                self.fail_safe();
            }
        }
        self.connected = connected;
    }

    pub fn fail_safe(&mut self) {
        self.parser.reset();
        self.duties.fill(0);
        self.startup = false;
        self.last_command_ms = None;
    }

    pub fn receive(&mut self, bytes: &[u8], now_ms: u64) {
        for &byte in bytes {
            if let Some((channel, duty)) = self.parser.push(byte, now_ms) {
                if self.startup {
                    // Animation values must not linger on channels the host has not set.
                    self.duties.fill(0);
                    self.startup = false;
                }
                self.duties[channel] = duty;
                self.last_command_ms = Some(now_ms);
            }
        }
    }

    pub fn tick(&mut self, now_ms: u64) {
        self.parser.expire(now_ms);
        if self
            .last_command_ms
            .is_some_and(|last| now_ms.saturating_sub(last) >= COMM_TIMEOUT_MS)
        {
            self.fail_safe();
        }
        if self.startup {
            let elapsed = now_ms.saturating_sub(self.started_ms);
            let duty = match elapsed {
                0..2000 => 0,
                2000..4550 => 1 + ((elapsed - 2000) / 10) as u8,
                4550..4650 => 255,
                4650..7200 => 255 - ((elapsed - 4650) / 10) as u8,
                _ => {
                    self.startup = false;
                    0
                }
            };
            self.duties.fill(duty);
        }
    }

    pub fn duties(&self) -> &[u8; PWM_NCHAN] {
        &self.duties
    }

    pub fn led_on(&self, now_ms: u64) -> bool {
        self.last_command_ms
            .is_some_and(|last| now_ms.saturating_sub(last) < 10)
    }
}
