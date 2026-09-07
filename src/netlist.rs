//! A small netlist simulator: chip models wired together by pin.
//!
//! * A [`Chip`] is anything with numbered pins that accepts externally driven
//!   levels, reports the levels it drives, and can say when its outputs next
//!   change on their own.
//! * A net joins chip pins, optional testbench drivers, ties and pull
//!   resistors.  Multiple strong drivers resolve to their common level, or to
//!   `X` (with a warning) when they disagree.
//! * The simulation is event driven.  At each event time the nets are settled
//!   iteratively (chips may respond with zero delay, e.g. an SRAM output
//!   going `X` the instant its address changes), then time jumps to the next
//!   testbench stimulus or chip-internal event.
//! * Every net-level change is traced and can be written as a VCD file.
//!
//! Each chip sees, on a pin it drives itself, only what *other* drivers put on
//! the net; the chip models deal with their own output vs. the outside world.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::as7c164a::{self, As7c164a};
use crate::cy7c131::{self, Bus, Cy7c131, Inputs, PortInputs};
use crate::ds1100::Ds1100;
use crate::gal22v10::Gal22v10;
pub use crate::cy7c131::{Level, Time, NS};

/// A chip model with 1-based pin numbers.
pub trait Chip {
    /// Highest pin number.
    fn pin_count(&self) -> usize;
    fn pin_name(&self, pin: usize) -> String;
    /// Externally driven level on every pin (`ext[pin]`, index 0 unused).
    fn set_inputs(&mut self, t: Time, ext: &[Level]);
    /// Levels the chip drives at time `t` (`out[pin]`; `Z` for inputs).
    fn drive(&mut self, t: Time, out: &mut [Level]);
    /// Next time strictly after `t` at which an output changes by itself.
    fn next_event(&self, t: Time) -> Option<Time>;
    fn warnings(&self) -> Vec<String>;
    /// For downcasting to the concrete model (to peek memory contents etc.).
    fn as_any(&self) -> &dyn std::any::Any;
}

// ---------------------------------------------------------------------------
// Chip impl: DS1100 delay line (8-pin DIP): 1 IN, 7 TAP1, 2 TAP2, 6 TAP3,
// 3 TAP4, 5 TAP5, 4 GND, 8 VCC.

/// Pin of tap `k` (0-based).
pub fn ds1100_tap_pin(k: usize) -> usize {
    [7, 2, 6, 3, 5][k]
}
pub const DS1100_IN: usize = 1;

impl Chip for Ds1100 {
    fn pin_count(&self) -> usize {
        8
    }
    fn pin_name(&self, pin: usize) -> String {
        match pin {
            1 => "IN".into(),
            7 => "TAP1".into(),
            2 => "TAP2".into(),
            6 => "TAP3".into(),
            3 => "TAP4".into(),
            5 => "TAP5".into(),
            4 => "GND".into(),
            8 => "VCC".into(),
            _ => format!("?{pin}"),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        self.set_input(t, ext[DS1100_IN]);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        for k in 0..5 {
            out[ds1100_tap_pin(k)] = self.tap(k, t);
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        Ds1100::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        self.warnings().iter().map(|w| format!("DS1100 {:?} at {} ps: {}", w.kind, w.t, w.detail)).collect()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: ATF22V10C (24-pin DIP)

impl Chip for Gal22v10 {
    fn pin_count(&self) -> usize {
        24
    }
    fn pin_name(&self, pin: usize) -> String {
        match pin {
            1 => "CLK/IN1".into(),
            2..=11 => format!("IN{pin}"),
            12 => "GND".into(),
            13 => "IN13".into(),
            14..=23 => format!("IO{pin}"),
            24 => "VCC".into(),
            _ => format!("?{pin}"),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut a = [Level::Z; 25];
        a[..25].copy_from_slice(&ext[..25]);
        Gal22v10::set_inputs(self, t, a);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        self.advance(t);
        let d = Gal22v10::drive(self);
        out[..25].copy_from_slice(&d);
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        Gal22v10::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        Gal22v10::warnings(self).iter().map(|w| w.to_string()).collect()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: CY7C131 (52-pin PLCC)

/// PLCC pin functions, from the datasheet's "PLCC Top View".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SramPin {
    Ce(cy7c131::Port),
    Rw(cy7c131::Port),
    Busy(cy7c131::Port),
    Int(cy7c131::Port),
    Oe(cy7c131::Port),
    A(cy7c131::Port, u8),
    Io(cy7c131::Port, u8),
    Nc,
    Gnd,
    Vcc,
}

pub fn sram_pin(pin: usize) -> SramPin {
    use cy7c131::Port::{Left as L, Right as R};
    match pin {
        1 => SramPin::Ce(L),
        2 => SramPin::Rw(L),
        3 => SramPin::Busy(L),
        4 => SramPin::Int(L),
        5 | 25 | 35 | 47 => SramPin::Nc,
        6 => SramPin::Oe(L),
        7..=16 => SramPin::A(L, (pin - 7) as u8),
        17..=24 => SramPin::Io(L, (pin - 17) as u8),
        26 => SramPin::Gnd,
        27..=34 => SramPin::Io(R, (pin - 27) as u8),
        36..=45 => SramPin::A(R, (45 - pin) as u8),
        46 => SramPin::Oe(R),
        48 => SramPin::Int(R),
        49 => SramPin::Busy(R),
        50 => SramPin::Rw(R),
        51 => SramPin::Ce(R),
        52 => SramPin::Vcc,
        _ => SramPin::Nc,
    }
}

/// Pin number for a given function (inverse of [`sram_pin`]).
pub fn sram_pin_of(f: SramPin) -> usize {
    (1..=52).find(|&p| sram_pin(p) == f).expect("no such pin")
}

impl Chip for Cy7c131 {
    fn pin_count(&self) -> usize {
        52
    }
    fn pin_name(&self, pin: usize) -> String {
        let s = |p: cy7c131::Port| if p == cy7c131::Port::Left { "L" } else { "R" };
        match sram_pin(pin) {
            SramPin::Ce(p) => format!("CE{}", s(p)),
            SramPin::Rw(p) => format!("RW{}", s(p)),
            SramPin::Busy(p) => format!("BUSY{}", s(p)),
            SramPin::Int(p) => format!("INT{}", s(p)),
            SramPin::Oe(p) => format!("OE{}", s(p)),
            SramPin::A(p, i) => format!("A{i}{}", s(p)),
            SramPin::Io(p, i) => format!("IO{i}{}", s(p)),
            SramPin::Nc => "NC".into(),
            SramPin::Gnd => "GND".into(),
            SramPin::Vcc => "VCC".into(),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut inp = Inputs { l: PortInputs::default(), r: PortInputs::default() };
        for (pin, &v) in ext.iter().enumerate().take(53).skip(1) {
            match sram_pin(pin) {
                SramPin::Ce(p) => inp.port_mut(p).ce_n = v,
                SramPin::Rw(p) => inp.port_mut(p).rw_n = v,
                SramPin::Oe(p) => inp.port_mut(p).oe_n = v,
                SramPin::A(p, i) => inp.port_mut(p).addr[i as usize] = v,
                SramPin::Io(p, i) => inp.port_mut(p).data[i as usize] = v,
                _ => {}
            }
        }
        // The 1K part has ten address pins; the model carries fourteen.
        for p in [cy7c131::Port::Left, cy7c131::Port::Right] {
            for b in 10..cy7c131::ADDR_BITS {
                inp.port_mut(p).addr[b] = Level::L;
            }
        }
        Cy7c131::set_inputs(self, t, inp);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        let o = self.outputs(t);
        for (pin, slot) in out.iter_mut().enumerate().take(53).skip(1) {
            *slot = match sram_pin(pin) {
                SramPin::Busy(p) => o.port(p).busy_n,
                SramPin::Int(p) => o.port(p).int_n,
                SramPin::Io(p, i) => match o.port(p).data {
                    Bus::Z => Level::Z,
                    Bus::X => Level::X,
                    Bus::V(v) => Level::from_bit(v >> i & 1 == 1),
                },
                _ => Level::Z,
            };
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        Cy7c131::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        Cy7c131::warnings(self).iter().map(|w| w.to_string()).collect()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: 16K x 8 dual-port SRAM (IDT7006 / CY7C006 class), same model
// as the CY7C131 with fourteen address lines.  Pin numbers here are a
// logical map (see `dp16k_pin`); the physical PLCC-68 assignment is a PCB
// task (docs/memory-timing.md).

pub struct DualPort16k(pub Cy7c131);

/// Logical pins: 1-14 A_L0-13, 15-22 IO_L0-7, 23 CE_L, 24 RW_L, 25 OE_L,
/// 26 BUSY_L, 27 INT_L, 28 GND; 35-48 A_R0-13, 49-56 IO_R0-7, 57 CE_R,
/// 58 RW_R, 59 OE_R, 60 BUSY_R, 61 INT_R, 62 VCC; the rest NC.
pub fn dp16k_pin(pin: usize) -> SramPin {
    use cy7c131::Port::{Left as L, Right as R};
    match pin {
        1..=14 => SramPin::A(L, (pin - 1) as u8),
        15..=22 => SramPin::Io(L, (pin - 15) as u8),
        23 => SramPin::Ce(L),
        24 => SramPin::Rw(L),
        25 => SramPin::Oe(L),
        26 => SramPin::Busy(L),
        27 => SramPin::Int(L),
        28 => SramPin::Gnd,
        35..=48 => SramPin::A(R, (pin - 35) as u8),
        49..=56 => SramPin::Io(R, (pin - 49) as u8),
        57 => SramPin::Ce(R),
        58 => SramPin::Rw(R),
        59 => SramPin::Oe(R),
        60 => SramPin::Busy(R),
        61 => SramPin::Int(R),
        62 => SramPin::Vcc,
        _ => SramPin::Nc,
    }
}
pub fn dp16k_pin_of(f: SramPin) -> usize {
    (1..=68).find(|&p| dp16k_pin(p) == f).expect("no such pin")
}

impl Chip for DualPort16k {
    fn pin_count(&self) -> usize {
        68
    }
    fn pin_name(&self, pin: usize) -> String {
        let s = |p: cy7c131::Port| if p == cy7c131::Port::Left { "L" } else { "R" };
        match dp16k_pin(pin) {
            SramPin::Ce(p) => format!("CE{}", s(p)),
            SramPin::Rw(p) => format!("RW{}", s(p)),
            SramPin::Busy(p) => format!("BUSY{}", s(p)),
            SramPin::Int(p) => format!("INT{}", s(p)),
            SramPin::Oe(p) => format!("OE{}", s(p)),
            SramPin::A(p, i) => format!("A{i}{}", s(p)),
            SramPin::Io(p, i) => format!("IO{i}{}", s(p)),
            SramPin::Nc => "NC".into(),
            SramPin::Gnd => "GND".into(),
            SramPin::Vcc => "VCC".into(),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut inp = Inputs { l: PortInputs::default(), r: PortInputs::default() };
        for (pin, &v) in ext.iter().enumerate().take(69).skip(1) {
            match dp16k_pin(pin) {
                SramPin::Ce(p) => inp.port_mut(p).ce_n = v,
                SramPin::Rw(p) => inp.port_mut(p).rw_n = v,
                SramPin::Oe(p) => inp.port_mut(p).oe_n = v,
                SramPin::A(p, i) => inp.port_mut(p).addr[i as usize] = v,
                SramPin::Io(p, i) => inp.port_mut(p).data[i as usize] = v,
                _ => {}
            }
        }
        self.0.set_inputs(t, inp);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        let o = self.0.outputs(t);
        for (pin, slot) in out.iter_mut().enumerate().take(69).skip(1) {
            *slot = match dp16k_pin(pin) {
                SramPin::Io(p, i) => match o.port(p).data {
                    Bus::Z => Level::Z,
                    Bus::X => Level::X,
                    Bus::V(v) => Level::from_bit(v >> i & 1 == 1),
                },
                SramPin::Busy(p) => o.port(p).busy_n,
                SramPin::Int(p) => o.port(p).int_n,
                _ => Level::Z,
            };
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        self.0.next_event(t)
    }
    fn warnings(&self) -> Vec<String> {
        Cy7c131::warnings(&self.0).iter().map(|w| w.to_string()).collect()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: a single fast 2-input NAND gate (SOT-23-5: 1 A, 2 B, 3 GND,
// 4 Y, 5 VCC), e.g. 74LVC1G00 / NC7SZ00 at 5 V.  Conservative: after any
// input change that can change the output, the output is unknown from
// tPD(min) to tPD(max); an input change that provably leaves the output
// alone (the other input already holds it) does nothing.

pub struct FastGate {
    pub tpd_min: Time,
    pub tpd_max: Time,
    a: Level,
    b: Level,
    out: Level,
    /// Pending settle: (when unknown starts, when it settles, final value).
    pending: Option<(Time, Time, Level)>,
    now: Time,
}

impl FastGate {
    /// 74LVC1G00-class at 5 V: 1.0 to 4.5 ns (to be confirmed against the
    /// chosen part's datasheet).
    pub fn nand_5v() -> FastGate {
        FastGate::new(NS, 4500)
    }
    pub fn new(tpd_min: Time, tpd_max: Time) -> FastGate {
        FastGate { tpd_min, tpd_max, a: Level::X, b: Level::X, out: Level::X, pending: None, now: 0 }
    }
    fn nand(a: Level, b: Level) -> Level {
        match (a, b) {
            (Level::L, _) | (_, Level::L) => Level::H,
            (Level::H, Level::H) => Level::L,
            _ => Level::X,
        }
    }
    fn advance(&mut self, t: Time) {
        self.now = t;
        if let Some((x0, x1, v)) = self.pending {
            if t >= x1 {
                self.out = v;
                self.pending = None;
            } else if t >= x0 {
                self.out = Level::X;
            }
        }
    }
}

impl Chip for FastGate {
    fn pin_count(&self) -> usize {
        5
    }
    fn pin_name(&self, pin: usize) -> String {
        ["?", "A", "B", "GND", "Y", "VCC"][pin.min(5)].into()
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        self.advance(t);
        let z = |l: Level| if l == Level::Z { Level::X } else { l };
        let (a, b) = (z(ext[1]), z(ext[2]));
        if a == self.a && b == self.b {
            return;
        }
        self.a = a;
        self.b = b;
        let target = Self::nand(a, b);
        let settled = self.pending.map_or(self.out, |(_, _, v)| v);
        if target == settled && self.pending.is_none() && target != Level::X {
            return; // the other input holds the output
        }
        self.pending = Some((t + self.tpd_min, t + self.tpd_max, target));
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        self.advance(t);
        out[4] = self.out;
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        self.pending.and_then(|(x0, x1, _)| [x0, x1].into_iter().find(|&e| e > t))
    }
    fn warnings(&self) -> Vec<String> {
        Vec::new()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: 256K x 16 single-port SRAM with byte enables (CY7C1041G,
// 44-pin TSOP II / SOJ, datasheet figure 6), modelled as two byte-lane
// cores sharing address, OE# and WE#; a byte enable acts on its lane like
// the chip enable (tDBE <= tACE, tHZBE, tBW = tSCE in the timing grade).

pub struct Sram16 {
    pub lo: As7c164a,
    pub hi: As7c164a,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sram16Pin {
    A(u8),
    Io(u8),
    CeN,
    OeN,
    WeN,
    BheN,
    BleN,
    Vcc,
    Vss,
    Nc,
}

pub fn sram16_pin(pin: usize) -> Sram16Pin {
    match pin {
        1..=5 => Sram16Pin::A((pin - 1) as u8),
        6 => Sram16Pin::CeN,
        7..=10 => Sram16Pin::Io((pin - 7) as u8),
        11 => Sram16Pin::Vcc,
        12 => Sram16Pin::Vss,
        13..=16 => Sram16Pin::Io((pin - 13 + 4) as u8),
        17 => Sram16Pin::WeN,
        18..=22 => Sram16Pin::A((pin - 18 + 5) as u8),
        23..=27 => Sram16Pin::A((pin - 23 + 10) as u8),
        28 => Sram16Pin::Nc,
        29..=32 => Sram16Pin::Io((pin - 29 + 8) as u8),
        33 => Sram16Pin::Vcc,
        34 => Sram16Pin::Vss,
        35..=38 => Sram16Pin::Io((pin - 35 + 12) as u8),
        39 => Sram16Pin::BleN,
        40 => Sram16Pin::BheN,
        41 => Sram16Pin::OeN,
        42..=44 => Sram16Pin::A((pin - 42 + 15) as u8),
        _ => Sram16Pin::Nc,
    }
}
pub fn sram16_pin_of(f: Sram16Pin) -> usize {
    (1..=44).find(|&p| sram16_pin(p) == f).expect("no such pin")
}

impl Sram16 {
    pub fn new(t: as7c164a::Timing) -> Sram16 {
        Sram16 { lo: As7c164a::with_timing_and_size(t, 18), hi: As7c164a::with_timing_and_size(t, 18) }
    }
    pub fn preload(&mut self, addr: u32, word: u16) {
        self.lo.preload(addr, word as u8);
        self.hi.preload(addr, (word >> 8) as u8);
    }
    pub fn peek(&self, addr: u32) -> Option<u16> {
        Some(self.lo.peek(addr)? as u16 | (self.hi.peek(addr)? as u16) << 8)
    }
}

/// A lane is selected when CE# and its byte enable are both low.
fn or_n(a: Level, b: Level) -> Level {
    match (a, b) {
        (Level::H, _) | (_, Level::H) => Level::H,
        (Level::L, Level::L) => Level::L,
        _ => Level::X,
    }
}

impl Chip for Sram16 {
    fn pin_count(&self) -> usize {
        44
    }
    fn pin_name(&self, pin: usize) -> String {
        match sram16_pin(pin) {
            Sram16Pin::A(i) => format!("A{i}"),
            Sram16Pin::Io(i) => format!("IO{i}"),
            Sram16Pin::CeN => "CE#".into(),
            Sram16Pin::OeN => "OE#".into(),
            Sram16Pin::WeN => "WE#".into(),
            Sram16Pin::BheN => "BHE#".into(),
            Sram16Pin::BleN => "BLE#".into(),
            Sram16Pin::Vcc => "VCC".into(),
            Sram16Pin::Vss => "VSS".into(),
            Sram16Pin::Nc => "NC".into(),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut lo = as7c164a::Inputs::default();
        let mut hi = as7c164a::Inputs::default();
        let (mut ce, mut bhe, mut ble) = (Level::Z, Level::Z, Level::Z);
        for (pin, &v) in ext.iter().enumerate().take(45).skip(1) {
            match sram16_pin(pin) {
                Sram16Pin::A(i) => {
                    lo.addr[i as usize] = v;
                    hi.addr[i as usize] = v;
                }
                Sram16Pin::Io(i) if i < 8 => lo.data[i as usize] = v,
                Sram16Pin::Io(i) => hi.data[i as usize - 8] = v,
                Sram16Pin::CeN => ce = v,
                Sram16Pin::BheN => bhe = v,
                Sram16Pin::BleN => ble = v,
                Sram16Pin::OeN => {
                    lo.oe_n = v;
                    hi.oe_n = v;
                }
                Sram16Pin::WeN => {
                    lo.we_n = v;
                    hi.we_n = v;
                }
                _ => {}
            }
        }
        lo.ce_n = or_n(ce, ble);
        hi.ce_n = or_n(ce, bhe);
        lo.ce2 = Level::H;
        hi.ce2 = Level::H;
        self.lo.set_inputs(t, lo);
        self.hi.set_inputs(t, hi);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        let (ol, oh) = (self.lo.output(t), self.hi.output(t));
        for (pin, slot) in out.iter_mut().enumerate().take(45).skip(1) {
            *slot = match sram16_pin(pin) {
                Sram16Pin::Io(i) => {
                    let (o, b) = if i < 8 { (ol, i) } else { (oh, i - 8) };
                    match o {
                        Bus::Z => Level::Z,
                        Bus::X => Level::X,
                        Bus::V(v) => Level::from_bit(v >> b & 1 == 1),
                    }
                }
                _ => Level::Z,
            };
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        match (self.lo.next_event(t), self.hi.next_event(t)) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }
    fn warnings(&self) -> Vec<String> {
        let mut w: Vec<String> = self.lo.warnings().iter().map(|x| format!("lo: {x}")).collect();
        w.extend(self.hi.warnings().iter().map(|x| format!("hi: {x}")));
        w
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Chip impl: AS7C164A (28-pin DIP/SOJ)

/// Pin functions of the AS7C164A, from the datasheet pin configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Sram8kPin {
    A(u8),
    Dq(u8),
    CeN,
    Ce2,
    OeN,
    WeN,
    Nc,
    Vss,
    Vcc,
}

pub fn sram8k_pin(pin: usize) -> Sram8kPin {
    match pin {
        1 => Sram8kPin::Nc,
        2 => Sram8kPin::A(12),
        3..=10 => Sram8kPin::A((10 - pin) as u8),
        11..=13 => Sram8kPin::Dq((pin - 11) as u8),
        14 => Sram8kPin::Vss,
        15..=19 => Sram8kPin::Dq((pin - 12) as u8),
        20 => Sram8kPin::CeN,
        21 => Sram8kPin::A(10),
        22 => Sram8kPin::OeN,
        23 => Sram8kPin::A(11),
        24 => Sram8kPin::A(9),
        25 => Sram8kPin::A(8),
        26 => Sram8kPin::Ce2,
        27 => Sram8kPin::WeN,
        28 => Sram8kPin::Vcc,
        _ => Sram8kPin::Nc,
    }
}

pub fn sram8k_pin_of(f: Sram8kPin) -> usize {
    (1..=28).find(|&p| sram8k_pin(p) == f).expect("no such pin")
}

impl Chip for As7c164a {
    fn pin_count(&self) -> usize {
        28
    }
    fn pin_name(&self, pin: usize) -> String {
        match sram8k_pin(pin) {
            Sram8kPin::A(i) => format!("A{i}"),
            Sram8kPin::Dq(i) => format!("DQ{i}"),
            Sram8kPin::CeN => "CE#".into(),
            Sram8kPin::Ce2 => "CE2".into(),
            Sram8kPin::OeN => "OE#".into(),
            Sram8kPin::WeN => "WE#".into(),
            Sram8kPin::Nc => "NC".into(),
            Sram8kPin::Vss => "VSS".into(),
            Sram8kPin::Vcc => "VCC".into(),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut inp = as7c164a::Inputs::default();
        for (pin, &v) in ext.iter().enumerate().take(29).skip(1) {
            match sram8k_pin(pin) {
                Sram8kPin::A(i) => inp.addr[i as usize] = v,
                Sram8kPin::Dq(i) => inp.data[i as usize] = v,
                Sram8kPin::CeN => inp.ce_n = v,
                Sram8kPin::Ce2 => inp.ce2 = v,
                Sram8kPin::OeN => inp.oe_n = v,
                Sram8kPin::WeN => inp.we_n = v,
                _ => {}
            }
        }
        // The 8K part has thirteen address pins; the model carries eighteen.
        for b in 13..as7c164a::ADDR_BITS {
            inp.addr[b] = Level::L;
        }
        As7c164a::set_inputs(self, t, inp);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        let o = self.output(t);
        for (pin, slot) in out.iter_mut().enumerate().take(29).skip(1) {
            *slot = match sram8k_pin(pin) {
                Sram8kPin::Dq(i) => match o {
                    Bus::Z => Level::Z,
                    Bus::X => Level::X,
                    Bus::V(v) => Level::from_bit(v >> i & 1 == 1),
                },
                _ => Level::Z,
            };
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        As7c164a::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        As7c164a::warnings(self).iter().map(|w| w.to_string()).collect()
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Netlist

pub type NetId = usize;
pub type ChipId = usize;

#[derive(Clone, Debug)]
struct Net {
    name: String,
    /// (chip, pin) connections.
    pins: Vec<(ChipId, usize)>,
    /// Constant driver (VCC / GND).
    tie: Level,
    /// Weak pull (resistor): L, H or Z for none.
    pull: Level,
}

/// Builder.
#[derive(Default)]
pub struct Netlist {
    chips: Vec<(String, Box<dyn Chip>)>,
    nets: Vec<Net>,
    by_name: BTreeMap<String, NetId>,
}

impl Netlist {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_chip(&mut self, name: &str, chip: impl Chip + 'static) -> ChipId {
        self.chips.push((name.to_string(), Box::new(chip)));
        self.chips.len() - 1
    }
    /// Get or create a net by name.
    pub fn net(&mut self, name: &str) -> NetId {
        if let Some(&id) = self.by_name.get(name) {
            return id;
        }
        let id = self.nets.len();
        self.nets.push(Net { name: name.to_string(), pins: vec![], tie: Level::Z, pull: Level::Z });
        self.by_name.insert(name.to_string(), id);
        id
    }
    /// Create `width` nets named `prefix0`, `prefix1`, ...
    pub fn bus(&mut self, prefix: &str, width: usize) -> Vec<NetId> {
        (0..width).map(|i| self.net(&format!("{prefix}{i}"))).collect()
    }
    pub fn connect(&mut self, net: NetId, chip: ChipId, pin: usize) {
        assert!(pin >= 1 && pin <= self.chips[chip].1.pin_count(), "pin {pin} out of range");
        self.nets[net].pins.push((chip, pin));
    }
    /// Constant driver on a net (VCC or GND).
    pub fn tie(&mut self, net: NetId, level: Level) {
        self.nets[net].tie = level;
    }
    /// Pull resistor on a net.
    pub fn pull(&mut self, net: NetId, level: Level) {
        self.nets[net].pull = level;
    }
    pub fn build(self) -> Sim {
        let n = self.nets.len();
        let mut pin_net: Vec<Vec<Option<NetId>>> =
            self.chips.iter().map(|(_, c)| vec![None; c.pin_count() + 1]).collect();
        for (id, net) in self.nets.iter().enumerate() {
            for &(c, p) in &net.pins {
                assert!(pin_net[c][p].is_none(), "chip {} pin {} on two nets", self.chips[c].0, p);
                pin_net[c][p] = Some(id);
            }
        }
        let drives = self.chips.iter().map(|(_, c)| vec![Level::Z; c.pin_count() + 1]).collect();
        let fed = self.chips.iter().map(|(_, c)| vec![Level::Z; c.pin_count() + 1]).collect();
        let mut sim = Sim {
            chips: self.chips,
            nets: self.nets,
            by_name: self.by_name,
            pin_net,
            drives,
            fed,
            ext: vec![Level::Z; n],
            value: vec![Level::Z; n],
            stim: BTreeMap::new(),
            seq: 0,
            now: 0,
            trace: Vec::new(),
            conflicts: Vec::new(),
        };
        sim.settle();
        sim
    }
}

/// A built, running netlist.
pub struct Sim {
    chips: Vec<(String, Box<dyn Chip>)>,
    nets: Vec<Net>,
    by_name: BTreeMap<String, NetId>,
    pin_net: Vec<Vec<Option<NetId>>>,
    /// Last levels each chip drove.
    drives: Vec<Vec<Level>>,
    /// Last external levels fed to each chip.
    fed: Vec<Vec<Level>>,
    /// Testbench driver per net.
    ext: Vec<Level>,
    /// Resolved net values.
    value: Vec<Level>,
    stim: BTreeMap<(Time, u64), (NetId, Level)>,
    seq: u64,
    now: Time,
    trace: Vec<(Time, NetId, Level)>,
    conflicts: Vec<(Time, NetId)>,
}

fn resolve(drivers: impl Iterator<Item = Level>, pull: Level) -> Level {
    let mut v = Level::Z;
    for d in drivers {
        v = match (v, d) {
            (a, Level::Z) => a,
            (Level::Z, b) => b,
            (a, b) if a == b => a,
            _ => Level::X,
        };
    }
    if v == Level::Z { pull } else { v }
}

impl Sim {
    pub fn now(&self) -> Time {
        self.now
    }
    pub fn net_id(&self, name: &str) -> NetId {
        *self.by_name.get(name).unwrap_or_else(|| panic!("no net {name}"))
    }
    pub fn net_name(&self, id: NetId) -> &str {
        &self.nets[id].name
    }
    /// A chip model, for downcasting.
    pub fn chip(&self, id: ChipId) -> &dyn std::any::Any {
        self.chips[id].1.as_any()
    }
    pub fn chip_id(&self, name: &str) -> ChipId {
        self.chips.iter().position(|(n, _)| n == name).unwrap_or_else(|| panic!("no chip {name}"))
    }
    /// What chip `id` currently drives on `pin`.
    pub fn chip_drive(&self, id: ChipId, pin: usize) -> Level {
        self.drives[id][pin]
    }
    /// Level last fed to `chip` on `pin`.
    pub fn fed_level(&self, chip: ChipId, pin: usize) -> Level {
        self.fed[chip][pin]
    }
    /// Pins of `chip` connected to `net`.
    pub fn pins_on(&self, net: NetId, chip: ChipId) -> Vec<usize> {
        self.nets[net].pins.iter().filter(|&&(c, _)| c == chip).map(|&(_, p)| p).collect()
    }
    /// Resolved level on a net right now.
    pub fn value(&self, net: NetId) -> Level {
        self.value[net]
    }
    /// Read a bus (bit i = nets[i]); `None` if any bit is not L/H.
    pub fn read_bus(&self, nets: &[NetId]) -> Option<u32> {
        let mut v = 0u32;
        for (i, &n) in nets.iter().enumerate() {
            v |= (self.value[n].bit()? as u32) << i;
        }
        Some(v)
    }

    /// Testbench: drive a net to `level` at time `t` (>= now).  `Z` releases.
    pub fn schedule(&mut self, t: Time, net: NetId, level: Level) {
        assert!(t >= self.now, "schedule() in the past");
        self.seq += 1;
        self.stim.insert((t, self.seq), (net, level));
    }
    pub fn schedule_bus(&mut self, t: Time, nets: &[NetId], value: u32) {
        for (i, &n) in nets.iter().enumerate() {
            self.schedule(t, n, Level::from_bit(value >> i & 1 == 1));
        }
    }
    pub fn schedule_bus_z(&mut self, t: Time, nets: &[NetId]) {
        for &n in nets {
            self.schedule(t, n, Level::Z);
        }
    }

    /// Time of the next thing that will happen after `now`.
    pub fn next_event(&self) -> Option<Time> {
        let s = self.stim.keys().next().map(|k| k.0);
        let c = self.chips.iter().filter_map(|(_, c)| c.next_event(self.now)).min();
        match (s, c) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// Advance to `t`, processing everything on the way.
    pub fn run_until(&mut self, t: Time) {
        assert!(t >= self.now);
        loop {
            let next = self.next_event();
            match next {
                Some(nt) if nt <= t => {
                    self.now = nt;
                    while let Some((&(st, seq), &(net, lvl))) = self.stim.iter().next() {
                        if st > nt {
                            break;
                        }
                        self.stim.remove(&(st, seq));
                        self.ext[net] = lvl;
                    }
                    self.settle();
                }
                _ => {
                    self.now = t;
                    self.settle();
                    return;
                }
            }
        }
    }

    /// Recompute nets and chip inputs at `now` until nothing changes.
    fn settle(&mut self) {
        let t = self.now;
        for iter in 0..1000 {
            // Feed every chip what the rest of the world drives on its pins.
            let mut any_input_change = false;
            for c in 0..self.chips.len() {
                let mut ext = vec![Level::Z; self.chips[c].1.pin_count() + 1];
                for (p, slot) in ext.iter_mut().enumerate().skip(1) {
                    if let Some(n) = self.pin_net[c][p] {
                        *slot = self.resolve_excluding(n, c, p);
                    }
                }
                if ext != self.fed[c] || iter == 0 {
                    self.fed[c] = ext.clone();
                    self.chips[c].1.set_inputs(t, &ext);
                    any_input_change = true;
                }
            }
            // Collect drives.
            let mut any_drive_change = false;
            for c in 0..self.chips.len() {
                let mut out = vec![Level::Z; self.chips[c].1.pin_count() + 1];
                self.chips[c].1.drive(t, &mut out);
                if out != self.drives[c] {
                    self.drives[c] = out;
                    any_drive_change = true;
                }
            }
            self.recompute_values();
            if !any_drive_change && !any_input_change {
                break;
            }
            if iter == 999 {
                panic!("netlist did not settle at t={t}");
            }
        }
    }

    fn resolve_excluding(&self, net: NetId, chip: ChipId, pin: usize) -> Level {
        let n = &self.nets[net];
        let others = n.pins.iter().filter(|&&(c, p)| !(c == chip && p == pin)).map(|&(c, p)| self.drives[c][p]);
        resolve(others.chain([n.tie, self.ext[net]]), n.pull)
    }

    fn recompute_values(&mut self) {
        let t = self.now;
        for id in 0..self.nets.len() {
            let n = &self.nets[id];
            let strong: Vec<Level> =
                n.pins.iter().map(|&(c, p)| self.drives[c][p]).chain([n.tie, self.ext[id]]).collect();
            let v = resolve(strong.iter().copied(), n.pull);
            // Contention: two different definite levels.
            let has_l = strong.contains(&Level::L);
            let has_h = strong.contains(&Level::H);
            if has_l && has_h && self.conflicts.last() != Some(&(t, id)) {
                self.conflicts.push((t, id));
            }
            if v != self.value[id] {
                self.value[id] = v;
                self.trace.push((t, id, v));
            }
        }
    }

    /// All warnings: chip warnings prefixed by chip name, plus net conflicts.
    pub fn warnings(&self) -> Vec<String> {
        let mut w: Vec<String> = self
            .chips
            .iter()
            .flat_map(|(name, c)| c.warnings().into_iter().map(move |s| format!("{name}: {s}")))
            .collect();
        for &(t, n) in &self.conflicts {
            w.push(format!("t={:.3}ns net {}: drive conflict", t as f64 / NS as f64, self.nets[n].name));
        }
        w
    }

    /// Every net change in `[t0, t1]`: (time, net name, level).
    pub fn changes_between(&self, t0: Time, t1: Time) -> Vec<(Time, String, Level)> {
        self.trace.iter().filter(|e| e.0 >= t0 && e.0 <= t1).map(|e| (e.0, self.nets[e.1].name.clone(), e.2)).collect()
    }

    /// Value history of one net: (time, level) at each change.
    pub fn history(&self, net: NetId) -> Vec<(Time, Level)> {
        self.trace.iter().filter(|e| e.1 == net).map(|e| (e.0, e.2)).collect()
    }

    /// Whole trace as a VCD document (timescale 1ps).
    pub fn vcd(&self) -> String {
        let mut s = String::new();
        writeln!(s, "$timescale 1ps $end").unwrap();
        writeln!(s, "$scope module netlist $end").unwrap();
        let code = |id: NetId| format!("n{id}");
        for (id, n) in self.nets.iter().enumerate() {
            writeln!(s, "$var wire 1 {} {} $end", code(id), n.name.replace(' ', "_")).unwrap();
        }
        writeln!(s, "$upscope $end").unwrap();
        writeln!(s, "$enddefinitions $end").unwrap();
        let ch = |l: Level| match l {
            Level::L => '0',
            Level::H => '1',
            Level::Z => 'z',
            Level::X => 'x',
        };
        let mut last: Option<Time> = None;
        writeln!(s, "#0").unwrap();
        writeln!(s, "$dumpvars").unwrap();
        for id in 0..self.nets.len() {
            let initial = self.trace.iter().find(|e| e.1 == id && e.0 == 0).map(|e| e.2).unwrap_or(Level::Z);
            writeln!(s, "{}{}", ch(initial), code(id)).unwrap();
        }
        writeln!(s, "$end").unwrap();
        for &(t, id, v) in &self.trace {
            if t == 0 {
                continue;
            }
            if last != Some(t) {
                writeln!(s, "#{t}").unwrap();
                last = Some(t);
            }
            writeln!(s, "{}{}", ch(v), code(id)).unwrap();
        }
        s
    }
}
