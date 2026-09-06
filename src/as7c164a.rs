//! Behavioural + timing model of the Alliance AS7C164A-15 (8K x 8 fast
//! asynchronous SRAM, 28-pin skinny DIP / SOJ).
//!
//! Source: Alliance Memory datasheet Rev 4.0 (May 2017), saved in `docs/`.
//!
//! Same philosophy as [`crate::cy7c131`]: an output is a definite level only
//! where the datasheet guarantees it, everything else is `X`; a write whose
//! constraints are not provably met leaves the cell unknown and records a
//! [`Warning`].  Inputs are timestamped snapshots, outputs are queried for
//! any time at or after the last snapshot.
//!
//! # Pins (28-pin DIP/SOJ)
//!
//! ```text
//!  1 NC    2 A12   3 A7    4 A6    5 A5    6 A4    7 A3    8 A2    9 A1   10 A0
//! 11 DQ0  12 DQ1  13 DQ2  14 VSS  15 DQ3  16 DQ4  17 DQ5  18 DQ6  19 DQ7
//! 20 CE#  21 A10  22 OE#  23 A11  24 A9   25 A8   26 CE2  27 WE#  28 VCC
//! ```
//! The chip is selected when CE# is low *and* CE2 is high.  A write is the
//! overlap of selected and WE# low; whichever of the three ends it defines
//! the write end (datasheet note 2).

use std::fmt;

pub use crate::cy7c131::{Level, Time, NS};
use crate::cy7c131::{Bus, Timeline};

/// Externally applied levels.  `data` is what *external* drivers put on the
/// DQ pins (`Z` when nobody drives them).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Inputs {
    pub addr: [Level; 13],
    pub ce_n: Level,
    pub ce2: Level,
    pub oe_n: Level,
    pub we_n: Level,
    pub data: [Level; 8],
}

impl Default for Inputs {
    fn default() -> Self {
        Inputs { addr: [Level::Z; 13], ce_n: Level::Z, ce2: Level::Z, oe_n: Level::Z, we_n: Level::Z, data: [Level::Z; 8] }
    }
}

impl Inputs {
    /// Deselected (CE# high, CE2 high), OE#/WE# high, address 0, data undriven.
    pub fn idle() -> Self {
        Inputs {
            addr: addr_levels(0),
            ce_n: Level::H,
            ce2: Level::H,
            oe_n: Level::H,
            we_n: Level::H,
            data: [Level::Z; 8],
        }
    }
    pub fn with_addr(mut self, a: u16) -> Self {
        self.addr = addr_levels(a);
        self
    }
    pub fn with_data(mut self, d: u8) -> Self {
        self.data = std::array::from_fn(|i| Level::from_bit(d >> i & 1 == 1));
        self
    }
    pub fn with_ce(mut self, l: Level) -> Self {
        self.ce_n = l;
        self
    }
    pub fn with_ce2(mut self, l: Level) -> Self {
        self.ce2 = l;
        self
    }
    pub fn with_oe(mut self, l: Level) -> Self {
        self.oe_n = l;
        self
    }
    pub fn with_we(mut self, l: Level) -> Self {
        self.we_n = l;
        self
    }
}

pub fn addr_levels(a: u16) -> [Level; 13] {
    std::array::from_fn(|i| Level::from_bit(a >> i & 1 == 1))
}
fn levels_value<const N: usize>(l: &[Level; N]) -> Option<u16> {
    let mut v = 0u16;
    for (i, b) in l.iter().enumerate() {
        v |= (b.bit()? as u16) << i;
    }
    Some(v)
}

/// AC parameters (picoseconds).  Names follow the datasheet.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    pub taa: Time,  // address access (max)
    pub tace: Time, // chip enable access (max)
    pub toe: Time,  // output enable access (max)
    pub tclz: Time, // chip enable to low-Z (min)
    pub tolz: Time, // output enable to low-Z (min)
    pub tchz: Time, // chip disable to high-Z (max)
    pub tohz: Time, // output disable to high-Z (max)
    pub toh: Time,  // output hold from address change (min)
    pub taw: Time,  // address valid to end of write (min)
    pub tcw: Time,  // chip enable to end of write (min)
    pub tas: Time,  // address setup to write start (min)
    pub twp: Time,  // write pulse width (min)
    pub twr: Time,  // write recovery: address hold from end of write (min)
    pub tdw: Time,  // data to write overlap (min)
    pub tdh: Time,  // data hold from end of write (min)
    pub tow: Time,  // output active from end of write (min)
    pub twhz: Time, // write to output high-Z (max)
}

impl Timing {
    /// AS7C164A-15, datasheet Rev 4.0 page 5.
    pub const fn grade_15() -> Timing {
        Timing {
            taa: 15 * NS,
            tace: 15 * NS,
            toe: 7 * NS,
            tclz: 4 * NS,
            tolz: 0,
            tchz: 7 * NS,
            tohz: 7 * NS,
            toh: 3 * NS,
            taw: 12 * NS,
            tcw: 12 * NS,
            tas: 0,
            twp: 10 * NS,
            twr: 0,
            tdw: 8 * NS,
            tdh: 0,
            tow: 4 * NS,
            twhz: 8 * NS,
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Warning {
    pub time: Time,
    pub kind: WarningKind,
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum WarningKind {
    /// Write (overlap of selected and WE# low) shorter than tWP.
    WritePulseTooShort { width: Time },
    /// Selected for less than tCW before write end.
    ChipEnableSetup { have: Time },
    /// Address stable for less than tAW before write end.
    AddrSetup { have: Time },
    /// Data stable for less than tDW before write end.
    DataSetup { have: Time },
    /// Address changed while a write was active (datasheet note 1).
    AddrChangedDuringWrite,
    /// Data pins not at a definite level at write end.
    DataNotDriven,
    /// The chip's own outputs may still have been driving the DQ pins in the
    /// data setup window (datasheet note 3).
    BusConflict,
    /// Address contains X/Z at write end: whole array corrupted.
    AddrUnknown,
    /// Write ended because a control pin went X/Z.
    ControlUnknown,
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "t={:.3}ns {:?}", self.time as f64 / NS as f64, self.kind)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DataOut {
    Z,
    X,
    Read(u16),
}

/// Three-valued "selected" (CE# low and CE2 high).
fn selected(i: &Inputs) -> Option<bool> {
    Some(!i.ce_n.bit()? && i.ce2.bit()?)
}
fn write_conditions(i: &Inputs) -> Option<bool> {
    Some(selected(i)? && !i.we_n.bit()?)
}
fn read_driving(i: &Inputs) -> Option<bool> {
    Some(selected(i)? && !i.oe_n.bit()? && i.we_n.bit()?)
}

/// The chip.
#[derive(Clone, Debug)]
pub struct As7c164a {
    t: Timing,
    now: Time,
    mem: Vec<Option<u8>>,
    inp: Inputs,
    addr_since: Time,
    /// When the chip last became selected.
    sel_since: Time,
    data_since: Time,
    write_start: Option<Time>,
    write_end: Option<Time>,
    data_out: Timeline<DataOut>,
    valid_not_before: Time,
    warnings: Vec<Warning>,
}

impl As7c164a {
    pub fn new() -> Self {
        Self::with_timing(Timing::grade_15())
    }
    pub fn with_timing(t: Timing) -> Self {
        As7c164a {
            t,
            now: 0,
            mem: vec![None; 8192],
            inp: Inputs::default(),
            addr_since: 0,
            sel_since: 0,
            data_since: 0,
            write_start: None,
            write_end: None,
            data_out: Timeline::new(DataOut::Z),
            valid_not_before: 0,
            warnings: Vec::new(),
        }
    }
    pub fn preload(&mut self, addr: u16, data: u8) {
        self.mem[addr as usize & 0x1FFF] = Some(data);
    }
    pub fn preload_slice(&mut self, addr: u16, data: &[u8]) {
        for (i, &d) in data.iter().enumerate() {
            self.preload(addr + i as u16, d);
        }
    }
    pub fn peek(&self, addr: u16) -> Option<u8> {
        self.mem[addr as usize & 0x1FFF]
    }
    pub fn timing(&self) -> &Timing {
        &self.t
    }
    pub fn now(&self) -> Time {
        self.now
    }
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }
    pub fn take_warnings(&mut self) -> Vec<Warning> {
        std::mem::take(&mut self.warnings)
    }
    fn warn(&mut self, kind: WarningKind) {
        self.warnings.push(Warning { time: self.now, kind });
    }

    /// Level the chip drives on DQ at time `t` (>= last `set_inputs`).
    pub fn output(&self, t: Time) -> Bus {
        assert!(t >= self.now, "output() queried in the past (t={t} < now={})", self.now);
        match self.data_out.at(t) {
            DataOut::Z => Bus::Z,
            DataOut::X => Bus::X,
            DataOut::Read(a) => match self.mem[a as usize] {
                Some(v) => Bus::V(v),
                None => Bus::X,
            },
        }
    }

    pub fn next_event(&self, t: Time) -> Option<Time> {
        self.data_out.next_change_after(t)
    }

    pub fn set_inputs(&mut self, t: Time, n: Inputs) {
        assert!(t >= self.now, "set_inputs() time went backwards ({t} < {})", self.now);
        self.now = t;
        let o = self.inp;
        self.inp = n;
        let addr_changed = o.addr != n.addr;
        if addr_changed {
            self.addr_since = t;
        }
        if o.data != n.data {
            self.data_since = t;
        }
        if selected(&o) != Some(true) && selected(&n) == Some(true) {
            self.sel_since = t;
        }

        // Write end (either control leaving its active level, or going X).
        let now_writing = write_conditions(&n);
        if self.write_start.is_some() && now_writing != Some(true) {
            self.finish_write(o, now_writing == Some(false));
            self.write_start = None;
        }
        if addr_changed {
            if self.write_start.is_some() && now_writing == Some(true) {
                self.warn(WarningKind::AddrChangedDuringWrite);
                match levels_value(&o.addr) {
                    Some(a) => self.mem[a as usize] = None,
                    None => self.corrupt_all(),
                }
            } else if let Some(we) = self.write_end
                && t < we + self.t.twr
            {
                // tWR = 0 for this part, so this never fires, but keep the
                // check so other grades can be modelled.
                if let Some(a) = levels_value(&o.addr) {
                    self.mem[a as usize] = None;
                }
            }
        }
        // Write start.
        if now_writing == Some(true) && self.write_start.is_none() {
            self.write_start = Some(t);
            self.write_end = None;
        }
        self.update_data_out(o, addr_changed);
    }

    fn corrupt_all(&mut self) {
        for c in &mut self.mem {
            *c = None;
        }
    }

    fn finish_write(&mut self, o: Inputs, end_known: bool) {
        let te = self.now;
        let tm = self.t;
        let start = self.write_start.unwrap();
        self.write_end = Some(te);
        let out_z = self.data_out.is_const_over(te.saturating_sub(tm.tdw), te, DataOut::Z);
        let Some(addr) = levels_value(&o.addr) else {
            self.warn(WarningKind::AddrUnknown);
            self.corrupt_all();
            return;
        };
        let cell = addr as usize;
        let mut ok = end_known;
        if !end_known {
            self.warn(WarningKind::ControlUnknown);
        }
        let width = te - start;
        if width < tm.twp {
            self.warn(WarningKind::WritePulseTooShort { width });
            ok = false;
        }
        if te - self.sel_since < tm.tcw {
            self.warn(WarningKind::ChipEnableSetup { have: te - self.sel_since });
            ok = false;
        }
        if te - self.addr_since < tm.taw {
            self.warn(WarningKind::AddrSetup { have: te - self.addr_since });
            ok = false;
        }
        let Some(data) = levels_value(&o.data) else {
            self.warn(WarningKind::DataNotDriven);
            self.mem[cell] = None;
            return;
        };
        if te - self.data_since < tm.tdw {
            self.warn(WarningKind::DataSetup { have: te - self.data_since });
            ok = false;
        }
        if !out_z {
            self.warn(WarningKind::BusConflict);
            ok = false;
        }
        self.mem[cell] = if ok { Some(data as u8) } else { None };
    }

    fn update_data_out(&mut self, o: Inputs, addr_changed: bool) {
        let t = self.now;
        let tm = self.t;
        let n = self.inp;
        let cur = self.data_out.at(t);
        match read_driving(&n) {
            None => {
                if selected(&n) != Some(false) {
                    self.data_out.set_from(t, DataOut::X);
                    self.valid_not_before = 0;
                }
            }
            Some(false) => {
                if cur != DataOut::Z {
                    // Which control turned it off decides the high-Z time.
                    let hz = if o.we_n != n.we_n && n.we_n == Level::L { tm.twhz } else { tm.tchz.max(tm.tohz) };
                    self.data_out.set_from(t, DataOut::X);
                    self.data_out.set_from(t + hz, DataOut::Z);
                    self.valid_not_before = 0;
                }
            }
            Some(true) => {
                let mut x_start = t;
                let mut valid = self.valid_not_before;
                if read_driving(&o) != Some(true) {
                    // Coming out of high-Z (or unknown).  Every enabling
                    // path that just switched must propagate before the
                    // outputs can turn on, so the earliest low-Z is the
                    // slowest of their minimums.
                    let mut lz = 0;
                    if selected(&o) != selected(&n) {
                        lz = lz.max(tm.tclz);
                        valid = valid.max(t + tm.tace);
                    }
                    if o.oe_n != n.oe_n {
                        lz = lz.max(tm.tolz);
                        valid = valid.max(t + tm.toe);
                    }
                    if o.we_n != n.we_n {
                        lz = lz.max(tm.tow);
                        // Datasheet gives no WE#-high-to-valid figure; an
                        // address access time is the safe assumption.
                        valid = valid.max(t + tm.taa);
                    }
                    x_start = t + lz;
                    if cur != DataOut::Z {
                        x_start = t;
                    }
                    if addr_changed {
                        valid = valid.max(t + tm.taa);
                    }
                } else if addr_changed {
                    // Previous data holds for tOH, then unknown until tAA.
                    x_start = t + tm.toh;
                    valid = valid.max(t + tm.taa);
                } else if cur != DataOut::X && valid <= t {
                    return;
                }
                match levels_value(&n.addr) {
                    None => {
                        self.data_out.set_from(x_start, DataOut::X);
                        self.valid_not_before = valid;
                    }
                    Some(a) => {
                        if cur == DataOut::Read(a) && self.data_out.at(x_start) == cur && valid <= t {
                            return;
                        }
                        self.data_out.set_from(x_start, DataOut::X);
                        self.data_out.set_from(valid.max(x_start), DataOut::Read(a));
                        self.valid_not_before = valid;
                    }
                }
            }
        }
        // Keep only what the bus-conflict lookback needs.
        self.data_out.forget_before(t.saturating_sub(tm.tdw));
    }
}

impl Default for As7c164a {
    fn default() -> Self {
        Self::new()
    }
}
