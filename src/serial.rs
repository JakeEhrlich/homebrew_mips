//! The serial port: a UART built from GALs (equations in `cpu.rs`, block
//! `uart_block`) as a fast bus device, its software-level model shared
//! with the reference simulator ([`Port`]), and a bit-level terminal for
//! the netlist ([`Terminal`]) that types characters onto SIN and reads
//! what SOUT says.
//!
//! Registers (bus slot 0, docs/uart.md):
//!
//! | offset | read | write |
//! |---|---|---|
//! | 0 | RXD: the received byte; reading clears VALID | TXD: start sending the byte (when not busy) |
//! | 4 | STATUS: bit 0 TXBUSY, bit 1 RXVALID | - |
//! | 8 | - | DIV: bit period = DIV + 1 clocks |
//!
//! 8N1, LSB first, idle high.  115200 baud at 34 ns is DIV = 255
//! (114.9 kbaud, 0.3 % off, within a UART's 2 % tolerance).

use crate::board::PinKind;
use crate::cy7c131::{Level, Time};
use crate::netlist::Chip;
use std::collections::VecDeque;

pub const REG_DATA: u32 = 0;
pub const REG_STATUS: u32 = 4;
pub const REG_DIV: u32 = 8;
pub const STATUS_TXBUSY: u32 = 1;
pub const STATUS_RXVALID: u32 = 2;

/// What software sees, without time: transmission is instant, and the
/// terminal's characters arrive one at a time as the previous one is
/// read, from the moment the program first looks at the status register.
#[derive(Clone, Debug, Default)]
pub struct Port {
    pub div: u8,
    /// Characters the terminal is going to type.
    line: VecDeque<u8>,
    rxd: u8,
    valid: bool,
    listening: bool,
    /// Everything transmitted, in order.
    pub tx: Vec<u8>,
}

impl Port {
    pub fn send(&mut self, bytes: &[u8]) {
        self.line.extend(bytes);
    }
    fn deliver(&mut self) {
        if self.listening && !self.valid {
            if let Some(b) = self.line.pop_front() {
                self.rxd = b;
                self.valid = true;
            }
        }
    }
    pub fn read(&mut self, offset: u32) -> u32 {
        match offset & 0xC {
            REG_DATA => {
                let v = self.rxd as u32;
                self.valid = false;
                self.deliver();
                v
            }
            REG_STATUS => {
                self.listening = true;
                self.deliver();
                if self.valid { STATUS_RXVALID } else { 0 }
            }
            _ => 0,
        }
    }
    pub fn write(&mut self, offset: u32, v: u32) {
        match offset & 0xC {
            REG_DATA => self.tx.push(v as u8),
            REG_DIV => self.div = v as u8,
            _ => {}
        }
    }
}

/// A terminal on the wire: types `send` onto SIN from `start`, one
/// character after another at `bit` per bit, and decodes SOUT into `rx`.
/// Two pins: 1 = SOUT (from the UART, input), 2 = SIN (to the UART).
pub struct Terminal {
    pub bit: Time,
    pub start: Time,
    send: VecDeque<u8>,
    /// Bits being typed: (level, from).
    typing: Vec<(Level, Time)>,
    typing_at: usize,
    sout: Level,
    /// Decoding: (next sample time, bits so far, count).
    decoding: Option<(Time, u8, u8)>,
    pub rx: Vec<u8>,
    pub warnings: Vec<String>,
}

impl Terminal {
    pub fn new(bit: Time, start: Time, send: &[u8]) -> Terminal {
        let mut t = Terminal { bit, start, send: send.iter().copied().collect(), typing: Vec::new(), typing_at: 0, sout: Level::X, decoding: None, rx: Vec::new(), warnings: Vec::new() };
        t.lay_out();
        t
    }
    /// Start (or restart) typing at `t`.
    pub fn set_start(&mut self, t: Time) {
        self.start = t;
        self.typing.clear();
        self.typing_at = 0;
        self.lay_out();
    }
    /// Every bit of every character, with its start time.
    fn lay_out(&mut self) {
        let mut t = self.start;
        for &b in &self.send {
            self.typing.push((Level::L, t));
            t += self.bit;
            for i in 0..8 {
                self.typing.push((Level::from_bit(b >> i & 1 == 1), t));
                t += self.bit;
            }
            self.typing.push((Level::H, t));
            t += self.bit;
        }
        self.typing.push((Level::H, t));
    }
    fn sin_at(&self, t: Time) -> Level {
        let mut lvl = Level::H;
        for &(l, from) in &self.typing {
            if from <= t {
                lvl = l;
            } else {
                break;
            }
        }
        lvl
    }
}

impl Chip for Terminal {
    fn pin_count(&self) -> usize {
        2
    }
    fn pin_name(&self, pin: usize) -> String {
        ["?", "SOUT", "SIN"][pin.min(2)].into()
    }
    fn pin_kind(&self, pin: usize) -> PinKind {
        if pin == 1 { PinKind::In } else { PinKind::Out }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let s = if ext[1] == Level::Z { Level::X } else { ext[1] };
        // Decode: a falling edge from idle starts a character; sample each
        // bit in its middle.
        if let Some((at, bits, count)) = self.decoding {
            if t >= at {
                let b = match s {
                    Level::H => 1,
                    Level::L => 0,
                    _ => {
                        self.warnings.push(format!("terminal: t={:.3}ns SOUT unknown while sampling", t as f64 / 1000.0));
                        0
                    }
                };
                if count < 8 {
                    self.decoding = Some((at + self.bit, bits | (b << count), count + 1));
                } else {
                    if b != 1 {
                        self.warnings.push(format!("terminal: t={:.3}ns framing error (stop bit low)", t as f64 / 1000.0));
                    }
                    self.rx.push(bits);
                    self.decoding = None;
                }
            }
        } else if self.sout == Level::H && s == Level::L {
            // Start bit: sample data bits from 1.5 bit times on.
            self.decoding = Some((t + self.bit + self.bit / 2, 0, 0));
        }
        // Remember the last definite level (a driver's transition window
        // shows as unknown between the two).
        if matches!(s, Level::H | Level::L) {
            self.sout = s;
        }
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        out[2] = self.sin_at(t);
        while self.typing_at < self.typing.len() && self.typing[self.typing_at].1 <= t {
            self.typing_at += 1;
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        let mut ev: Vec<Time> = self.typing.iter().map(|&(_, from)| from).filter(|&f| f > t).take(1).collect();
        if let Some((at, _, _)) = self.decoding {
            if at > t {
                ev.push(at);
            }
        }
        ev.into_iter().min()
    }
    fn warnings(&self) -> Vec<String> {
        self.warnings.clone()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn port_registers() {
        let mut p = Port::default();
        p.send(b"ab");
        // The first status read starts listening and delivers 'a'.
        assert_eq!(p.read(REG_STATUS) & STATUS_RXVALID, STATUS_RXVALID);
        assert_eq!(p.read(REG_DATA), b'a' as u32);
        assert_eq!(p.read(REG_STATUS) & STATUS_RXVALID, STATUS_RXVALID);
        assert_eq!(p.read(REG_DATA), b'b' as u32);
        assert_eq!(p.read(REG_STATUS), 0);
        p.write(REG_DATA, b'x' as u32);
        assert_eq!(p.tx, vec![b'x']);
    }
}
