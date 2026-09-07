//! 16550-class UART (TI TL16C550C): a functional register core shared by
//! the reference simulator and the chip model, and the chip model itself
//! with the datasheet's CPU-bus timing checked on every access.
//!
//! The serial side is abstract: characters received "from the terminal"
//! are pushed into the receiver FIFO by the test bench, transmitted
//! characters come out of `Core::tx` when the (modelled) shift register
//! has clocked them out.  Bit-level timing of SIN / SOUT is the
//! manufacturer's business; what is modelled is what software sees: the
//! registers, the FIFOs, the line-status bits and how long a character
//! takes at the programmed divisor and crystal.

use crate::cy7c131::{Level, Time, NS};
use crate::netlist::Chip;
use std::collections::VecDeque;

/// Register addresses (A2..A0).
pub const RBR_THR_DLL: u8 = 0;
pub const IER_DLM: u8 = 1;
pub const IIR_FCR: u8 = 2;
pub const LCR: u8 = 3;
pub const MCR: u8 = 4;
pub const LSR: u8 = 5;
pub const MSR: u8 = 6;
pub const SCR: u8 = 7;

/// LSR bits.
pub const LSR_DR: u8 = 0x01;
pub const LSR_OE: u8 = 0x02;
pub const LSR_THRE: u8 = 0x20;
pub const LSR_TEMT: u8 = 0x40;

const FIFO_DEPTH: usize = 16;

/// The registers and FIFOs, without time.  Transmission is modelled by
/// the owner: [`Core::tx_start`] takes the next character into the shift
/// register, [`Core::tx_done`] delivers it.  The reference simulator
/// calls both at once (instant transmission); the chip model spaces them
/// by the character time.
#[derive(Clone, Debug, Default)]
pub struct Core {
    pub ier: u8,
    pub lcr: u8,
    pub mcr: u8,
    pub scr: u8,
    pub dll: u8,
    pub dlm: u8,
    pub fifo: bool,
    rx: VecDeque<u8>,
    /// Characters the far end is sending, not yet received.  They start
    /// arriving once the program first reads the line status register
    /// ("the terminal types when the machine is listening"), at the
    /// character rate on the chip, instantly in the reference simulator.
    line: VecDeque<u8>,
    listening: bool,
    tx_fifo: VecDeque<u8>,
    /// Character in the transmitter shift register.
    shifting: Option<u8>,
    last_rbr: u8,
    /// Receiver overrun seen (LSR bit 1), cleared by reading LSR.
    overrun: bool,
    /// Every character that has left the transmitter, in order.
    pub tx: Vec<u8>,
    /// Software-visible problems (writing THR when full, ...).
    pub faults: Vec<String>,
}

impl Core {
    /// Master reset (MR high): registers to their reset values, FIFOs
    /// cleared.  SCR is not affected by MR on the real part; it is left
    /// alone here too.
    pub fn reset(&mut self) {
        self.ier = 0;
        self.lcr = 0;
        self.mcr = 0;
        self.dll = 0;
        self.dlm = 0;
        self.fifo = false;
        self.rx.clear();
        self.tx_fifo.clear();
        self.shifting = None;
        self.overrun = false;
        self.listening = false;
    }

    fn dlab(&self) -> bool {
        self.lcr & 0x80 != 0
    }

    pub fn divisor(&self) -> u16 {
        (self.dlm as u16) << 8 | self.dll as u16
    }

    /// Bits per character at the current LCR: start + data + parity + stop.
    pub fn bits_per_char(&self) -> u32 {
        let data = 5 + (self.lcr & 3) as u32;
        let parity = (self.lcr >> 3 & 1) as u32;
        let stop = if self.lcr & 4 != 0 { 2 } else { 1 };
        1 + data + parity + stop
    }

    /// Queue characters at the far end (see `line`).
    pub fn send(&mut self, bytes: &[u8]) {
        self.line.extend(bytes);
    }

    /// Whether the program has started polling the line status.
    pub fn listening(&self) -> bool {
        self.listening
    }

    /// Receive the next queued character if the program is listening and
    /// the receiver has room.  Returns whether one arrived.
    pub fn rx_deliver(&mut self) -> bool {
        let cap = if self.fifo { FIFO_DEPTH } else { 1 };
        if self.listening && self.rx.len() < cap {
            if let Some(b) = self.line.pop_front() {
                self.rx.push_back(b);
                return true;
            }
        }
        false
    }

    /// A character arriving from the line right now.
    pub fn push_rx(&mut self, byte: u8) {
        let cap = if self.fifo { FIFO_DEPTH } else { 1 };
        if self.rx.len() >= cap {
            self.overrun = true;
        } else {
            self.rx.push_back(byte);
        }
    }

    pub fn lsr(&self) -> u8 {
        let mut v = 0;
        if !self.rx.is_empty() {
            v |= LSR_DR;
        }
        if self.overrun {
            v |= LSR_OE;
        }
        if self.tx_fifo.is_empty() {
            v |= LSR_THRE;
            if self.shifting.is_none() {
                v |= LSR_TEMT;
            }
        }
        v
    }

    /// Modem status with the board's loop-back ties: RTS# -> CTS#,
    /// DTR# -> DSR# and DCD#, RI# high.
    pub fn msr(&self) -> u8 {
        let dtr = self.mcr & 1 != 0;
        let rts = self.mcr & 2 != 0;
        (rts as u8) << 4 | (dtr as u8) << 5 | (dtr as u8) << 7
    }

    /// CPU read of register `a` (A2..A0).
    pub fn read(&mut self, a: u8) -> u8 {
        match a & 7 {
            RBR_THR_DLL if self.dlab() => self.dll,
            RBR_THR_DLL => {
                if let Some(b) = self.rx.pop_front() {
                    self.last_rbr = b;
                }
                self.last_rbr
            }
            IER_DLM if self.dlab() => self.dlm,
            IER_DLM => self.ier,
            IIR_FCR => 0x01 | if self.fifo { 0xC0 } else { 0 },
            LCR => self.lcr,
            MCR => self.mcr,
            LSR => {
                self.listening = true;
                let v = self.lsr();
                self.overrun = false;
                v
            }
            MSR => self.msr(),
            _ => self.scr,
        }
    }

    /// CPU write of register `a`.
    pub fn write(&mut self, a: u8, v: u8) {
        match a & 7 {
            RBR_THR_DLL if self.dlab() => self.dll = v,
            RBR_THR_DLL => {
                let cap = if self.fifo { FIFO_DEPTH } else { 1 };
                if self.tx_fifo.len() >= cap {
                    self.faults.push(format!("THR written while full ({:#04x} lost)", v));
                } else {
                    self.tx_fifo.push_back(v);
                }
            }
            IER_DLM if self.dlab() => self.dlm = v,
            IER_DLM => self.ier = v & 0x0F,
            IIR_FCR => {
                let was = self.fifo;
                self.fifo = v & 1 != 0;
                if self.fifo != was || v & 2 != 0 {
                    self.rx.clear();
                }
                if self.fifo != was || v & 4 != 0 {
                    self.tx_fifo.clear();
                }
            }
            LCR => self.lcr = v,
            MCR => self.mcr = v & 0x1F,
            LSR | MSR => {}
            _ => self.scr = v,
        }
    }

    /// Move the next character into the shift register, if it is free.
    /// Returns whether a transmission started.
    pub fn tx_start(&mut self) -> bool {
        if self.shifting.is_none() {
            if let Some(b) = self.tx_fifo.pop_front() {
                self.shifting = Some(b);
                return true;
            }
        }
        false
    }

    /// The shift register has clocked its character out.
    pub fn tx_done(&mut self) {
        if let Some(b) = self.shifting.take() {
            self.tx.push(b);
        }
    }

    pub fn shifting(&self) -> bool {
        self.shifting.is_some()
    }

    /// Instant transmission and reception (reference simulator).
    pub fn drain(&mut self) {
        while self.tx_start() {
            self.tx_done();
        }
        while self.rx_deliver() {}
    }
}

// ---------------------------------------------------------------------------
// Chip model

/// CPU-bus timing, TL16C550C (SLLS177I, section 5.7) and TL16C550D
/// (SLLS597E): the two tables are identical, and apply over the whole
/// supply range.
#[derive(Clone, Copy, Debug)]
pub struct BusTiming {
    /// Chip select / address valid before strobe start (ADS# low): td4, td5, td7, td8.
    pub t_setup: Time,
    /// Strobe width: tw6 (write), tw7 (read).
    pub t_strobe: Time,
    /// Chip select hold after strobe end: th3, th6.
    pub t_cs_hold: Time,
    /// Address hold after write strobe end: th4.
    pub t_wa_hold: Time,
    /// Address hold after read strobe end: th7.
    pub t_ra_hold: Time,
    /// Data valid before write strobe end: tsu3.
    pub t_data_setup: Time,
    /// Data hold after write strobe end: th5.
    pub t_data_hold: Time,
    /// Read strobe start to data valid (max): td10.
    pub t_rd_data: Time,
    /// Read strobe end to data floating (max): td11.
    pub t_rd_float: Time,
    /// Strobe start to next strobe start: tcR / tcW.
    pub t_cycle: Time,
    /// FIFO mode: minimum between reads of the receiver FIFO / status (note 3).
    pub t_fifo_read_cycle: Time,
    /// Master reset pulse width: tw8.
    pub t_mr: Time,
}

impl BusTiming {
    pub const fn tl16c550c() -> BusTiming {
        BusTiming {
            t_setup: 7 * NS,
            t_strobe: 40 * NS,
            t_cs_hold: 10 * NS,
            t_wa_hold: 10 * NS,
            t_ra_hold: 20 * NS,
            t_data_setup: 15 * NS,
            t_data_hold: 5 * NS,
            t_rd_data: 45 * NS,
            t_rd_float: 20 * NS,
            t_cycle: 87 * NS,
            t_fifo_read_cycle: 425 * NS,
            t_mr: 1000 * NS,
        }
    }
}

/// Logical pins of the chip model.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum UartPin {
    D(u8),
    A(u8),
    Cs0,
    Cs1,
    Cs2N,
    AdsN,
    Rd1N,
    Rd2,
    Wr1N,
    Wr2,
    /// Master reset, active high.
    Mr,
    Xin,
    Xout,
    Sin,
    Sout,
    Vcc,
    Gnd,
    /// Anything else (modem, DMA, interrupt, BAUDOUT, RCLK).
    Other,
}

/// TL16C550C / TL16C550D in the 48-pin LQFP (PT / PFB), datasheets
/// SLLS177I table 4-1 and SLLS597E terminal functions (same pinout).
/// NC: 1, 6, 13, 21, 25, 36, 37, 48.
pub fn uart_pin(pin: usize) -> UartPin {
    match pin {
        43 => UartPin::D(0),
        44 => UartPin::D(1),
        45 => UartPin::D(2),
        46 => UartPin::D(3),
        47 => UartPin::D(4),
        2 => UartPin::D(5),
        3 => UartPin::D(6),
        4 => UartPin::D(7),
        28 => UartPin::A(0),
        27 => UartPin::A(1),
        26 => UartPin::A(2),
        9 => UartPin::Cs0,
        10 => UartPin::Cs1,
        11 => UartPin::Cs2N,
        24 => UartPin::AdsN,
        19 => UartPin::Rd1N,
        20 => UartPin::Rd2,
        16 => UartPin::Wr1N,
        17 => UartPin::Wr2,
        35 => UartPin::Mr,
        14 => UartPin::Xin,
        15 => UartPin::Xout,
        7 => UartPin::Sin,
        8 => UartPin::Sout,
        42 => UartPin::Vcc,
        18 => UartPin::Gnd,
        // 5 RCLK, 12 BAUDOUT, 22 DDIS, 23 TXRDY, 29 RXRDY, 30 INTRPT,
        // 31 OUT2, 32 RTS, 33 DTR, 34 OUT1, 38 CTS, 39 DSR, 40 DCD, 41 RI.
        _ => UartPin::Other,
    }
}

pub const UART_PINS: usize = 48;

pub fn uart_pin_of(p: UartPin) -> usize {
    (1..=UART_PINS).find(|&i| uart_pin(i) == p).unwrap_or_else(|| panic!("no pin {p:?}"))
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Strobe {
    None,
    Read,
    Write,
}

/// A strobe input as seen through the driver's transition window: the
/// GAL output is unknown for a few ns while it moves, so an edge is only
/// known to lie inside [early, late].  Setup and width checks use the
/// pessimistic end of every window.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum StrobeIn {
    Off,
    /// Was off, now unknown since `t`: may have started.
    Starting(Time),
    On,
    /// Was on, now unknown since `t`: may have ended.
    Ending(Time),
}

/// The chip: registers plus bus timing.
pub struct Uart16550 {
    pub core: Core,
    pub timing: BusTiming,
    /// Crystal on XIN / XOUT (Hz).
    pub xin_hz: f64,
    now: Time,
    sel: Level,
    addr: [Level; 3],
    data_in: [Level; 8],
    rd: StrobeIn,
    wr: StrobeIn,
    mr: Level,
    /// When chip select, address and data last became stable (for setup
    /// checks: valid only from the end of their transition windows).
    cs_stable: Time,
    addr_stable: Time,
    data_stable: Time,
    strobe: Strobe,
    /// Start window of the current strobe: [early, late].
    strobe_start: (Time, Time),
    /// End of the last strobe (latest possible) and what it was.
    last_strobe_end: Option<(Time, Strobe)>,
    /// Latest possible start of the last strobe.
    last_strobe_start: Option<Time>,
    last_read_start: Option<Time>,
    /// Read in progress: (value, unknown from, valid from).
    read_out: Option<(u8, Time, Time)>,
    /// After a read strobe: output unknown from .0 until .1, then Z.
    float: Option<(Time, Time)>,
    mr_rise: Option<Time>,
    /// When the character in the shift register is done.
    tx_done_at: Option<Time>,
    /// When the next queued character arrives (once listening).
    rx_next_at: Option<Time>,
    warnings: Vec<String>,
}

impl Uart16550 {
    pub fn new(timing: BusTiming, xin_hz: f64) -> Uart16550 {
        Uart16550 {
            core: Core::default(),
            timing,
            xin_hz,
            now: 0,
            sel: Level::X,
            addr: [Level::X; 3],
            data_in: [Level::X; 8],
            rd: StrobeIn::Off,
            wr: StrobeIn::Off,
            mr: Level::X,
            cs_stable: 0,
            addr_stable: 0,
            data_stable: 0,
            strobe: Strobe::None,
            strobe_start: (0, 0),
            last_strobe_end: None,
            last_strobe_start: None,
            last_read_start: None,
            read_out: None,
            float: None,
            mr_rise: None,
            tx_done_at: None,
            rx_next_at: None,
            warnings: Vec::new(),
        }
    }

    fn warn(&mut self, t: Time, what: impl std::fmt::Display) {
        self.warnings.push(format!("uart: t={:.3}ns {what}", t as f64 / NS as f64));
    }

    /// One character time at the current divisor and line control.
    pub fn char_time(&self) -> Option<Time> {
        let d = self.core.divisor();
        if d == 0 {
            return None;
        }
        let bit_s = 16.0 * d as f64 / self.xin_hz;
        Some((bit_s * self.core.bits_per_char() as f64 * 1e12).round() as Time)
    }

    fn addr_value(&self) -> Option<u8> {
        let mut a = 0;
        for (i, &l) in self.addr.iter().enumerate() {
            match l {
                Level::H => a |= 1 << i,
                Level::L => {}
                _ => return None,
            }
        }
        Some(a)
    }

    fn data_value(&self) -> Option<u8> {
        let mut v = 0;
        for (i, &l) in self.data_in.iter().enumerate() {
            match l {
                Level::H => v |= 1 << i,
                Level::L => {}
                _ => return None,
            }
        }
        Some(v)
    }

    fn advance(&mut self, t: Time) {
        self.now = t;
        // Transmitter.
        if let Some(done) = self.tx_done_at {
            if t >= done {
                self.core.tx_done();
                self.tx_done_at = None;
            }
        }
        if self.tx_done_at.is_none() && self.core.tx_start() {
            match self.char_time() {
                Some(ct) => self.tx_done_at = Some(t + ct),
                None => self.warn(t, "character queued with divisor 0 (baud generator off)"),
            }
        }
        // Receiver: one character per character time once listening.
        if self.core.listening() {
            match (self.rx_next_at, self.char_time()) {
                (None, Some(ct)) => self.rx_next_at = Some(t + ct),
                (Some(at), Some(ct)) if t >= at => {
                    if self.core.rx_deliver() {
                        self.rx_next_at = Some(t + ct);
                    }
                }
                _ => {}
            }
        }
    }

    /// A strobe has started somewhere in [early, late].
    fn strobe_begin(&mut self, early: Time, late: Time, kind: Strobe) {
        let tm = self.timing;
        let ns = |t: Time| t as f64 / NS as f64;
        if early.saturating_sub(self.cs_stable) < tm.t_setup {
            self.warn(early, format!("{kind:?}: chip select stable only {:.1} ns before the strobe (need {})", ns(early.saturating_sub(self.cs_stable)), tm.t_setup / NS));
        }
        if early.saturating_sub(self.addr_stable) < tm.t_setup {
            self.warn(early, format!("{kind:?}: address stable only {:.1} ns before the strobe (need {})", ns(early.saturating_sub(self.addr_stable)), tm.t_setup / NS));
        }
        if let Some(s) = self.last_strobe_start {
            if early.saturating_sub(s) < tm.t_cycle {
                self.warn(early, format!("{kind:?}: {:.1} ns since the previous strobe (cycle time {})", ns(early.saturating_sub(s)), tm.t_cycle / NS));
            }
        }
        if kind == Strobe::Read && self.core.fifo {
            if let Some(s) = self.last_read_start {
                if early.saturating_sub(s) < tm.t_fifo_read_cycle {
                    self.warn(early, format!("read: {:.1} ns since the previous read in FIFO mode (need {})", ns(early.saturating_sub(s)), tm.t_fifo_read_cycle / NS));
                }
            }
        }
        self.strobe = kind;
        self.strobe_start = (early, late);
        self.last_strobe_start = Some(late);
        if kind == Strobe::Read {
            self.last_read_start = Some(late);
            let v = match self.addr_value() {
                Some(a) => self.core.read(a),
                None => {
                    self.warn(early, "read with unknown address");
                    0
                }
            };
            self.read_out = Some((v, early, late + tm.t_rd_data));
            self.float = None;
        }
    }

    /// The current strobe has ended somewhere in [early, late].
    fn strobe_end(&mut self, early: Time, late: Time) {
        let tm = self.timing;
        let kind = self.strobe;
        let ns = |t: Time| t as f64 / NS as f64;
        let width = early.saturating_sub(self.strobe_start.1);
        if width < tm.t_strobe {
            self.warn(early, format!("{kind:?}: strobe at least {:.1} ns wide (need {})", ns(width), tm.t_strobe / NS));
        }
        if kind == Strobe::Write {
            if early.saturating_sub(self.data_stable) < tm.t_data_setup {
                self.warn(early, format!("write: data stable only {:.1} ns before the strobe end (need {})", ns(early.saturating_sub(self.data_stable)), tm.t_data_setup / NS));
            }
            match (self.addr_value(), self.data_value()) {
                (Some(a), Some(v)) => self.core.write(a, v),
                _ => self.warn(early, "write with unknown address or data"),
            }
        } else {
            self.read_out = None;
            self.float = Some((early, late + tm.t_rd_float));
        }
        self.strobe = Strobe::None;
        self.last_strobe_end = Some((late, kind));
    }

    /// Hold checks: `what` started changing at `t` after the last strobe.
    fn check_hold(&mut self, t: Time, what: &str) {
        let tm = self.timing;
        if let Some((end, kind)) = self.last_strobe_end {
            let need = match (what, kind) {
                ("chip select", _) => tm.t_cs_hold,
                ("address", Strobe::Read) => tm.t_ra_hold,
                ("address", _) => tm.t_wa_hold,
                (_, Strobe::Write) => tm.t_data_hold,
                _ => 0,
            };
            if t.saturating_sub(end) < need {
                self.warn(t, format!("{what} changed {:.1} ns after the {kind:?} strobe end (hold {})", t.saturating_sub(end) as f64 / NS as f64, need / NS));
            }
        }
    }

    /// Track one strobe input (already reduced to on / off / unknown).
    /// Returns a started window or an ended window.
    fn track(prev: StrobeIn, level: Level, t: Time) -> (StrobeIn, Option<(Time, Time)>, Option<(Time, Time)>) {
        use StrobeIn::*;
        match (prev, level) {
            (Off, Level::H) => (On, Some((t, t)), None),
            (Off, Level::X) => (Starting(t), None, None),
            (Starting(e), Level::H) => (On, Some((e, t)), None),
            (Starting(_), Level::L) => (Off, None, None),
            (On, Level::L) => (Off, None, Some((t, t))),
            (On, Level::X) => (Ending(t), None, None),
            (Ending(e), Level::L) => (Off, None, Some((e, t))),
            (Ending(_), Level::H) => (On, None, None),
            (s, _) => (s, None, None),
        }
    }
}

impl Chip for Uart16550 {
    fn pin_count(&self) -> usize {
        UART_PINS
    }
    fn pin_name(&self, pin: usize) -> String {
        match uart_pin(pin) {
            UartPin::D(i) => format!("D{i}"),
            UartPin::A(i) => format!("A{i}"),
            UartPin::Cs0 => "CS0".into(),
            UartPin::Cs1 => "CS1".into(),
            UartPin::Cs2N => "CS2#".into(),
            UartPin::AdsN => "ADS#".into(),
            UartPin::Rd1N => "RD1#".into(),
            UartPin::Rd2 => "RD2".into(),
            UartPin::Wr1N => "WR1#".into(),
            UartPin::Wr2 => "WR2".into(),
            UartPin::Mr => "MR".into(),
            UartPin::Xin => "XIN".into(),
            UartPin::Xout => "XOUT".into(),
            UartPin::Sin => "SIN".into(),
            UartPin::Sout => "SOUT".into(),
            UartPin::Vcc => "VCC".into(),
            UartPin::Gnd => "GND".into(),
            UartPin::Other => match pin {
                5 => "RCLK".into(),
                12 => "BAUDOUT".into(),
                22 => "DDIS".into(),
                23 => "TXRDY".into(),
                29 => "RXRDY".into(),
                30 => "INTRPT".into(),
                31 => "OUT2#".into(),
                32 => "RTS#".into(),
                33 => "DTR#".into(),
                34 => "OUT1#".into(),
                38 => "CTS#".into(),
                39 => "DSR#".into(),
                40 => "DCD#".into(),
                41 => "RI#".into(),
                _ => "NC".into(),
            },
        }
    }

    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        self.advance(t);
        let z = |l: Level| if l == Level::Z { Level::X } else { l };
        let get = |p: UartPin| z(ext[uart_pin_of(p)]);
        // Master reset.
        let mr = get(UartPin::Mr);
        if mr != self.mr {
            match mr {
                Level::H => self.mr_rise = Some(t),
                Level::L => {
                    if let Some(r) = self.mr_rise.take() {
                        if t - r < self.timing.t_mr {
                            self.warn(t, format!("MR pulse {} ns (need {})", (t - r) / NS, self.timing.t_mr / NS));
                        }
                    }
                }
                _ => {}
            }
            self.mr = mr;
            if mr == Level::H {
                self.core.reset();
                self.tx_done_at = None;
                self.rx_next_at = None;
                self.strobe = Strobe::None;
                self.read_out = None;
            }
        }
        // Chip select: CS0 and CS1 high, CS2# low.
        let sel = match (get(UartPin::Cs0), get(UartPin::Cs1), get(UartPin::Cs2N)) {
            (Level::H, Level::H, Level::L) => Level::H,
            (Level::L, _, _) | (_, Level::L, _) | (_, _, Level::H) => Level::L,
            _ => Level::X,
        };
        if sel != self.sel {
            if self.sel != Level::X {
                self.check_hold(t, "chip select");
            }
            self.sel = sel;
            self.cs_stable = t;
        }
        let addr = [get(UartPin::A(0)), get(UartPin::A(1)), get(UartPin::A(2))];
        if addr != self.addr {
            if !self.addr.contains(&Level::X) {
                self.check_hold(t, "address");
            }
            self.addr = addr;
            self.addr_stable = t;
        }
        let mut data = [Level::X; 8];
        for (i, d) in data.iter_mut().enumerate() {
            *d = ext[uart_pin_of(UartPin::D(i as u8))];
        }
        if data != self.data_in {
            if self.strobe == Strobe::None && !self.data_in.contains(&Level::X) && !self.data_in.contains(&Level::Z) {
                self.check_hold(t, "data");
            }
            self.data_in = data;
            self.data_stable = t;
        }
        if get(UartPin::AdsN) != Level::L {
            self.warn(t, "ADS# not low: the model assumes the address is not latched");
        }
        // Strobes: RD1# / WR1# active low with RD2 / WR2 tied low, and
        // only while selected.  Reduced to on (H) / off (L) / unknown.
        let strobe_level = |n1: Level, p2: Level| -> Level {
            match (n1, p2) {
                (_, Level::H) => Level::H,
                (Level::L, Level::L) => Level::H,
                (Level::H, Level::L) => Level::L,
                _ => Level::X,
            }
        };
        let with_sel = |l: Level| match (sel, l) {
            (Level::L, _) => Level::L,
            (Level::H, l) => l,
            (_, Level::L) => Level::L,
            _ => Level::X,
        };
        let rd = with_sel(strobe_level(get(UartPin::Rd1N), get(UartPin::Rd2)));
        let wr = with_sel(strobe_level(get(UartPin::Wr1N), get(UartPin::Wr2)));
        if self.mr == Level::H {
            self.rd = if rd == Level::H { StrobeIn::On } else { StrobeIn::Off };
            self.wr = if wr == Level::H { StrobeIn::On } else { StrobeIn::Off };
            return;
        }
        let (rd_s, rd_start, rd_end) = Self::track(self.rd, rd, t);
        let (wr_s, wr_start, wr_end) = Self::track(self.wr, wr, t);
        self.rd = rd_s;
        self.wr = wr_s;
        match self.strobe {
            Strobe::None => match (rd_start, wr_start) {
                (Some(_), Some(_)) => self.warn(t, "read and write strobes both active"),
                (Some((e, l)), None) => self.strobe_begin(e, l, Strobe::Read),
                (None, Some((e, l))) => self.strobe_begin(e, l, Strobe::Write),
                (None, None) => {}
            },
            Strobe::Read => {
                if let Some((e, l)) = rd_end {
                    self.strobe_end(e, l);
                }
            }
            Strobe::Write => {
                if let Some((e, l)) = wr_end {
                    self.strobe_end(e, l);
                }
            }
        }
    }

    fn drive(&mut self, t: Time, out: &mut [Level]) {
        self.advance(t);
        for i in 0..8 {
            let level = match (self.read_out, self.float) {
                (Some((v, _, valid)), _) if t >= valid => Level::from_bit(v >> i & 1 == 1),
                (Some(_), _) => Level::X,
                (None, Some((from, until))) if t >= from && t < until => Level::X,
                _ => Level::Z,
            };
            out[uart_pin_of(UartPin::D(i))] = level;
        }
        // SOUT idles high; the bit stream is not modelled.
        out[uart_pin_of(UartPin::Sout)] = Level::H;
    }

    fn next_event(&self, t: Time) -> Option<Time> {
        let mut ev: Vec<Time> = Vec::new();
        if let Some((_, x, valid)) = self.read_out {
            ev.push(x);
            ev.push(valid);
        }
        if let Some((from, until)) = self.float {
            ev.push(from);
            ev.push(until);
        }
        if let Some(d) = self.tx_done_at {
            ev.push(d);
        }
        if let Some(r) = self.rx_next_at {
            ev.push(r);
        }
        ev.into_iter().filter(|&e| e > t).min()
    }

    fn warnings(&self) -> Vec<String> {
        let mut w = self.warnings.clone();
        w.extend(self.core.faults.iter().map(|f| format!("uart: {f}")));
        w
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn core_registers() {
        let mut c = Core::default();
        c.write(LCR, 0x80);
        c.write(RBR_THR_DLL, 4);
        c.write(IER_DLM, 0);
        assert_eq!(c.divisor(), 4);
        c.write(LCR, 0x03);
        assert_eq!(c.read(LCR), 0x03);
        assert_eq!(c.bits_per_char(), 10);
        assert_eq!(c.read(LSR), LSR_THRE | LSR_TEMT);
        c.write(RBR_THR_DLL, b'A');
        assert_eq!(c.read(LSR), 0);
        assert!(c.tx_start());
        assert_eq!(c.read(LSR), LSR_THRE);
        c.tx_done();
        assert_eq!(c.read(LSR), LSR_THRE | LSR_TEMT);
        assert_eq!(c.tx, vec![b'A']);
        c.push_rx(b'x');
        assert_eq!(c.read(LSR) & LSR_DR, LSR_DR);
        assert_eq!(c.read(RBR_THR_DLL), b'x');
        assert_eq!(c.read(LSR) & LSR_DR, 0);
    }

    #[test]
    fn fifo_and_overrun() {
        let mut c = Core::default();
        c.push_rx(1);
        c.push_rx(2);
        assert_eq!(c.read(LSR) & LSR_OE, LSR_OE);
        assert_eq!(c.read(LSR) & LSR_OE, 0);
        c.write(IIR_FCR, 0x07);
        assert_eq!(c.read(IIR_FCR), 0xC1);
        for i in 0..16 {
            c.push_rx(i);
        }
        c.push_rx(99);
        assert_eq!(c.read(LSR) & LSR_OE, LSR_OE);
        for i in 0..16 {
            assert_eq!(c.read(RBR_THR_DLL), i);
        }
    }
}
