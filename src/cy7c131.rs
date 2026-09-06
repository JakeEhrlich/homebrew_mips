//! Behavioural + timing model of the Cypress CY7C131 (1K x 8 dual-port SRAM,
//! *master* variant: BUSY is an open-drain output, INT flags present).
//!
//! Source: Cypress datasheet 38-00027-L (May 1989, rev. March 27 1997) for the
//! CY7C130/131/140/141, plus the IDT7130/7140 datasheet (DSC-2689/9) which
//! Cypress declares "pin-compatible and functionally equivalent" and which
//! carries the truth tables the Cypress sheet omits.
//!
//! # Modelling philosophy
//!
//! The model is *conservative*: an output is reported as a definite level only
//! where the datasheet guarantees it.  Every window in which the real part may
//! be transitioning is reported as `X`.  Every write whose setup/hold/pulse
//! constraints are not provably met leaves the addressed cell `X` (unknown) and
//! records a [`Warning`].  A design that simulates cleanly against this model
//! therefore depends on nothing the datasheet does not promise, which is what
//! makes a later logic-analyser comparison meaningful: the analyser can only
//! ever *narrow* an `X`, never contradict a definite level.
//!
//! # Time
//!
//! Time is `u64` picoseconds.  Inputs are applied as a sequence of
//! monotonically non-decreasing timestamped snapshots via
//! [`Cy7c131::set_inputs`]; outputs are queried with [`Cy7c131::outputs`] for
//! any `t` at or after the last snapshot (future values are the prediction
//! assuming no further input changes).
//!
//! # Pins (52-pin PLCC, J69, from the datasheet's "PLCC Top View")
//!
//! ```text
//!  1 CEL   2 R/WL   3 BUSYL  4 INTL   5 NC    6 OEL   7 A0L
//!  8 A1L   9 A2L   10 A3L   11 A4L   12 A5L  13 A6L  14 A7L  15 A8L  16 A9L
//! 17 I/O0L 18 I/O1L 19 I/O2L 20 I/O3L 21 I/O4L 22 I/O5L 23 I/O6L 24 I/O7L
//! 25 NC   26 GND
//! 27 I/O0R 28 I/O1R 29 I/O2R 30 I/O3R 31 I/O4R 32 I/O5R 33 I/O6R 34 I/O7R
//! 35 NC   36 A9R   37 A8R   38 A7R   39 A6R   40 A5R   41 A4R   42 A3R
//! 43 A2R  44 A1R   45 A0R   46 OER   47 NC    48 INTR  49 BUSYR 50 R/WR
//! 51 CER  52 VCC
//! ```
//! CE, R/W (low = write), OE, BUSY and INT are all active low.  BUSY and INT
//! are open drain (pull-up required); the model reports them as `L` or `Z`.

use std::fmt;

/// Picoseconds.
pub type Time = u64;
/// One nanosecond in [`Time`] units.
pub const NS: Time = 1_000;

/// A digital level on a pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Level {
    L,
    H,
    /// Not driven.
    #[default]
    Z,
    /// Unknown / possibly transitioning.
    X,
}

impl Level {
    pub fn from_bit(b: bool) -> Level {
        if b { Level::H } else { Level::L }
    }
    pub fn bit(self) -> Option<bool> {
        match self {
            Level::L => Some(false),
            Level::H => Some(true),
            _ => None,
        }
    }
}

/// 8-bit data bus as driven *by the chip* on one port.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Bus {
    Z,
    X,
    V(u8),
}

impl Bus {
    pub fn levels(self) -> [Level; 8] {
        match self {
            Bus::Z => [Level::Z; 8],
            Bus::X => [Level::X; 8],
            Bus::V(v) => std::array::from_fn(|i| Level::from_bit(v >> i & 1 == 1)),
        }
    }
}

/// Which port.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Port {
    Left,
    Right,
}

impl Port {
    fn other(self) -> Port {
        match self {
            Port::Left => Port::Right,
            Port::Right => Port::Left,
        }
    }
    /// Mailbox whose *write* by this port raises the other port's INT.
    fn mailbox_set(self) -> u16 {
        match self {
            Port::Left => 0x3FF,
            Port::Right => 0x3FE,
        }
    }
    /// Mailbox whose *read* by this port clears this port's INT.
    fn mailbox_clear(self) -> u16 {
        self.other().mailbox_set()
    }
}

/// Externally applied levels on one port's input/bidirectional pins.
/// `data` is what *external* drivers put on the I/O pins (`Z` when nobody
/// drives them).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PortInputs {
    pub addr: [Level; 10],
    pub ce_n: Level,
    pub rw_n: Level,
    pub oe_n: Level,
    pub data: [Level; 8],
}

impl Default for PortInputs {
    /// Everything undriven (`Z`).
    fn default() -> Self {
        PortInputs { addr: [Level::Z; 10], ce_n: Level::Z, rw_n: Level::Z, oe_n: Level::Z, data: [Level::Z; 8] }
    }
}

impl PortInputs {
    /// Idle: deselected (CE high), R/W high, OE high, address 0, data undriven.
    pub fn idle() -> Self {
        PortInputs { addr: addr_levels(0), ce_n: Level::H, rw_n: Level::H, oe_n: Level::H, data: [Level::Z; 8] }
    }
    pub fn with_addr(mut self, a: u16) -> Self {
        self.addr = addr_levels(a);
        self
    }
    pub fn with_data(mut self, d: u8) -> Self {
        self.data = data_levels(d);
        self
    }
    pub fn with_data_z(mut self) -> Self {
        self.data = [Level::Z; 8];
        self
    }
    pub fn with_ce(mut self, l: Level) -> Self {
        self.ce_n = l;
        self
    }
    pub fn with_rw(mut self, l: Level) -> Self {
        self.rw_n = l;
        self
    }
    pub fn with_oe(mut self, l: Level) -> Self {
        self.oe_n = l;
        self
    }
}

pub fn addr_levels(a: u16) -> [Level; 10] {
    std::array::from_fn(|i| Level::from_bit(a >> i & 1 == 1))
}
pub fn data_levels(d: u8) -> [Level; 8] {
    std::array::from_fn(|i| Level::from_bit(d >> i & 1 == 1))
}
fn levels_value<const N: usize>(l: &[Level; N]) -> Option<u16> {
    let mut v = 0u16;
    for (i, b) in l.iter().enumerate() {
        v |= (b.bit()? as u16) << i;
    }
    Some(v)
}

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Inputs {
    pub l: PortInputs,
    pub r: PortInputs,
}

impl Inputs {
    pub fn idle() -> Self {
        Inputs { l: PortInputs::idle(), r: PortInputs::idle() }
    }
    pub fn port(&self, p: Port) -> &PortInputs {
        match p {
            Port::Left => &self.l,
            Port::Right => &self.r,
        }
    }
    pub fn port_mut(&mut self, p: Port) -> &mut PortInputs {
        match p {
            Port::Left => &mut self.l,
            Port::Right => &mut self.r,
        }
    }
}

/// Levels driven *by the chip* on one port.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct PortOutputs {
    pub data: Bus,
    /// Open drain: `L`, `Z` (released, pulled high externally) or `X`.
    pub busy_n: Level,
    /// Open drain: `L`, `Z` or `X`.
    pub int_n: Level,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Outputs {
    pub l: PortOutputs,
    pub r: PortOutputs,
}

impl Outputs {
    pub fn port(&self, p: Port) -> &PortOutputs {
        match p {
            Port::Left => &self.l,
            Port::Right => &self.r,
        }
    }
}

/// AC parameters (all in picoseconds).  Names follow the datasheet.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    // Read cycle
    pub taa: Time,   // address to data valid (max)
    pub toha: Time,  // data hold from address change (min)
    pub tace: Time,  // CE low to data valid (max)
    pub tdoe: Time,  // OE low to data valid (max)
    pub tlzoe: Time, // OE low to low-Z (min)
    pub thzoe: Time, // OE high to high-Z (max)
    pub tlzce: Time, // CE low to low-Z (min)
    pub thzce: Time, // CE high to high-Z (max)
    // Write cycle
    pub tsce: Time,  // CE low to write end (min)
    pub taw: Time,   // address setup to write end (min)
    pub tha: Time,   // address hold from write end (min)
    pub tpwe: Time,  // R/W pulse width (min)
    pub tsd: Time,   // data setup to write end (min)
    pub thd: Time,   // data hold from write end (min)
    pub thzwe: Time, // R/W low to high-Z (max)
    pub tlzwe: Time, // R/W high to low-Z (min)
    // Busy
    pub tbla: Time, // BUSY low from address match (max)
    pub tbha: Time, // BUSY high from address mismatch (max)
    pub tblc: Time, // BUSY low from CE low (max)
    pub tbhc: Time, // BUSY high from CE high (max)
    pub tps: Time,  // port setup for priority (min)
    pub twh: Time,  // R/W high after BUSY high (min)
    pub tbdd: Time, // BUSY high to valid data (max)
    // Interrupt
    pub tins: Time, // address/CE/R/W to INT set (max)  (tINS = tEINS = tWINS)
    pub tinr: Time, // address/CE/OE to INT reset (max) (tINR = tEINR = tOINR)
}

impl Timing {
    /// CY7C131-15 (commercial), datasheet 38-00027-L page 4-5.
    pub const fn grade_15() -> Timing {
        Timing {
            taa: 15 * NS,
            toha: 0,
            tace: 15 * NS,
            tdoe: 10 * NS,
            tlzoe: 3 * NS,
            thzoe: 10 * NS,
            tlzce: 3 * NS,
            thzce: 10 * NS,
            tsce: 12 * NS,
            taw: 12 * NS,
            tha: 2 * NS,
            tpwe: 12 * NS,
            tsd: 10 * NS,
            thd: 0,
            thzwe: 10 * NS,
            tlzwe: 0,
            tbla: 15 * NS,
            tbha: 15 * NS,
            tblc: 15 * NS,
            tbhc: 15 * NS,
            tps: 5 * NS,
            twh: 13 * NS,
            tbdd: 15 * NS,
            tins: 15 * NS,
            tinr: 15 * NS,
        }
    }
}

/// Something the model could not guarantee.  Whenever a warning is produced the
/// affected state has already been forced to `X`, so a clean simulation is one
/// with an empty warning list.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Warning {
    pub time: Time,
    pub port: Port,
    pub kind: WarningKind,
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum WarningKind {
    /// Write pulse (overlap of CE low and R/W low) shorter than tPWE.
    WritePulseTooShort { width: Time },
    /// Write ended less than tWH after BUSY was released.
    WriteAfterBusyTooShort { width: Time },
    /// CE low less than tSCE before write end.
    CeSetup { have: Time },
    /// Address stable less than tAW before write end.
    AddrSetup { have: Time },
    /// Data stable less than tSD before write end.
    DataSetup { have: Time },
    /// Address changed less than tHA after write end.
    AddrHold { have: Time },
    /// Address changed while a write was active.
    AddrChangedDuringWrite,
    /// Data pins not at a definite level at write end.
    DataNotDriven,
    /// The chip's own outputs were (possibly) still driving the I/O pins during
    /// the data setup window (datasheet note 22).
    BusConflict,
    /// Address contains X/Z while port enabled; a write here corrupts the
    /// whole array.
    AddrUnknown,
    /// A control was X while no write was active and then went inactive: a
    /// pulse of unknown width may have written the cell.
    GlitchWrite,
    /// Write ended while the internal write-inhibit was in an undefined state
    /// (BUSY transitioning).
    WriteDuringBusyTransition,
    /// Both ports hit the same location within tPS: arbitration outcome is
    /// undefined.  (Unknowable arbitration because a CE or address is X is
    /// not reported here; it only shows up if a write ends during it.)
    ArbitrationAmbiguous,
    /// Informational: this write was inhibited because BUSY was asserted.
    WriteInhibited,
    /// INT set and clear conditions were true at the same time.
    IntSetClearRace,
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "t={:.3}ns {:?}: {:?}", self.time as f64 / NS as f64, self.port, self.kind)
    }
}

// ---------------------------------------------------------------------------
// Timeline: a piecewise-constant signal, editable only in the future.

#[derive(Clone, Debug)]
pub(crate) struct Timeline<T: Copy + PartialEq> {
    segs: Vec<(Time, T)>, // sorted; segs[0].0 == 0
}

impl<T: Copy + PartialEq> Timeline<T> {
    pub(crate) fn new(init: T) -> Self {
        Timeline { segs: vec![(0, init)] }
    }
    pub(crate) fn at(&self, t: Time) -> T {
        let i = self.segs.partition_point(|s| s.0 <= t);
        self.segs[i - 1].1
    }
    /// From `t` onward the value is `v` (discarding anything scheduled at or
    /// after `t`).
    pub(crate) fn set_from(&mut self, t: Time, v: T) {
        while self.segs.len() > 1 && self.segs.last().unwrap().0 >= t {
            self.segs.pop();
        }
        if self.segs.last().unwrap().1 != v {
            self.segs.push((t, v));
        }
    }
    pub(crate) fn next_change_after(&self, t: Time) -> Option<Time> {
        self.segs.iter().map(|s| s.0).find(|&s| s > t)
    }
    /// True iff the value equals `v` throughout the closed interval [a, b].
    pub(crate) fn is_const_over(&self, a: Time, b: Time, v: T) -> bool {
        self.at(a) == v && self.segs.iter().all(|s| s.0 <= a || s.0 > b || s.1 == v)
    }
    /// Drop history older than `t` (keeps the value at `t`).
    pub(crate) fn forget_before(&mut self, t: Time) {
        let i = self.segs.partition_point(|s| s.0 <= t);
        if i > 1 {
            let cur = self.segs[i - 1].1;
            self.segs.drain(..i - 1);
            self.segs[0] = (0, cur);
        }
    }
}

// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum DataOut {
    Z,
    X,
    /// Driving the contents of this address (resolved against memory at query
    /// time; see module docs on why that is sound).
    Read(u16),
}

/// Arbitration state of the chip.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Arb {
    None,
    /// This port lost arbitration; its BUSY is being driven low.
    Loser(Port),
    /// tPS violated (or address unknown): exactly one port is busy, unknown which.
    Ambiguous,
}

#[derive(Clone, Debug)]
struct PortState {
    inp: PortInputs,
    /// Time the current (enable, address) arbitration key became stable.
    key_since: Time,
    addr_since: Time,
    /// Time CE last went low.
    ce_low_since: Time,
    data_since: Time,
    /// Time CE and R/W both went low (write active), if they are.
    write_start: Option<Time>,
    /// A control went X while a write was active: it may have ended since.
    write_x: Option<Time>,
    /// A control went X while no write was active: one may have started.
    start_x: Option<Time>,
    /// End of the most recent write, for the tHA check.
    write_end: Option<Time>,
    /// Latest time by which BUSY is guaranteed released after the most recent
    /// release; `None` if this port was never busy since its last enable.
    busy_release_by: Option<Time>,
    /// Time the current busy assertion began (for the tBLA transition window).
    busy_since: Option<Time>,
    /// The active write is a mailbox write that has raised the other INT.
    int_set_pending: bool,
    data_out: Timeline<DataOut>,
    busy: Timeline<Level>,
    int: Timeline<Level>,
    valid_not_before: Time,
}

impl PortState {
    fn new() -> Self {
        PortState {
            inp: PortInputs::default(),
            key_since: 0,
            addr_since: 0,
            ce_low_since: 0,
            data_since: 0,
            write_start: None,
            write_x: None,
            start_x: None,
            write_end: None,
            busy_release_by: None,
            busy_since: None,
            int_set_pending: false,
            data_out: Timeline::new(DataOut::Z),
            busy: Timeline::new(Level::Z),
            int: Timeline::new(Level::Z),
            valid_not_before: 0,
        }
    }
    fn addr(&self) -> Option<u16> {
        levels_value(&self.inp.addr)
    }
    fn key(&self) -> Option<(bool, u16)> {
        arb_key(&self.inp)
    }
    fn read_driving(&self) -> Option<bool> {
        read_driving(&self.inp)
    }
}

/// Arbitration key: `Some((enabled, addr))`; `None` if unknowable.
fn arb_key(i: &PortInputs) -> Option<(bool, u16)> {
    match i.ce_n {
        Level::L => Some((true, levels_value(&i.addr)?)),
        Level::H => Some((false, 0)),
        _ => None,
    }
}
/// CE and R/W both low (`None` if either is X/Z).
fn write_conditions(i: &PortInputs) -> Option<bool> {
    all_active(&[(i.ce_n, Level::L), (i.rw_n, Level::L)])
}
/// CE low, OE low, R/W high: the port drives its I/O pins.
fn read_driving(i: &PortInputs) -> Option<bool> {
    all_active(&[(i.ce_n, Level::L), (i.oe_n, Level::L), (i.rw_n, Level::H)])
}
/// Three-valued AND of "active" conditions: `Some(false)` as soon as any
/// input is known inactive, `Some(true)` if all are known active, else `None`.
pub(crate) fn all_active(conds: &[(Level, Level)]) -> Option<bool> {
    let mut all = true;
    for &(l, active) in conds {
        match l {
            x if x == active => {}
            Level::L | Level::H => return Some(false),
            _ => all = false,
        }
    }
    if all { Some(true) } else { None }
}

/// The chip.
#[derive(Clone, Debug)]
pub struct Cy7c131 {
    t: Timing,
    now: Time,
    mem: Vec<Option<u8>>,
    ports: [PortState; 2],
    arb: Arb,
    arb_since: Time,
    warnings: Vec<Warning>,
}

impl Cy7c131 {
    /// A CY7C131-15 with all memory unknown and all inputs undriven at t=0.
    pub fn new() -> Self {
        Self::with_timing(Timing::grade_15())
    }
    pub fn with_timing(t: Timing) -> Self {
        Cy7c131 {
            t,
            now: 0,
            mem: vec![None; 1024],
            ports: [PortState::new(), PortState::new()],
            arb: Arb::None,
            arb_since: 0,
            warnings: Vec::new(),
        }
    }

    /// Pre-load a memory cell (models a known power-up image).
    pub fn preload(&mut self, addr: u16, data: u8) {
        self.mem[addr as usize & 0x3FF] = Some(data);
    }
    /// Pre-load consecutive bytes starting at `addr`.
    pub fn preload_slice(&mut self, addr: u16, data: &[u8]) {
        for (i, &d) in data.iter().enumerate() {
            self.preload(addr + i as u16, d);
        }
    }
    /// Current contents of a cell (`None` = unknown).
    pub fn peek(&self, addr: u16) -> Option<u8> {
        self.mem[addr as usize & 0x3FF]
    }
    pub fn timing(&self) -> &Timing {
        &self.t
    }
    pub fn now(&self) -> Time {
        self.now
    }
    /// Warnings accumulated so far (see [`Warning`]).
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }
    pub fn take_warnings(&mut self) -> Vec<Warning> {
        std::mem::take(&mut self.warnings)
    }

    fn ps(&self, p: Port) -> &PortState {
        &self.ports[p as usize]
    }
    fn ps_mut(&mut self, p: Port) -> &mut PortState {
        &mut self.ports[p as usize]
    }
    fn warn(&mut self, port: Port, kind: WarningKind) {
        self.warnings.push(Warning { time: self.now, port, kind });
    }

    /// Chip outputs at time `t` (must be >= the last `set_inputs` time).
    pub fn outputs(&self, t: Time) -> Outputs {
        assert!(t >= self.now, "outputs() queried in the past (t={t} < now={})", self.now);
        let port = |p: &PortState| PortOutputs {
            data: match p.data_out.at(t) {
                DataOut::Z => Bus::Z,
                DataOut::X => Bus::X,
                DataOut::Read(a) => match self.mem[a as usize] {
                    Some(v) => Bus::V(v),
                    None => Bus::X,
                },
            },
            busy_n: p.busy.at(t),
            int_n: p.int.at(t),
        };
        Outputs { l: port(&self.ports[0]), r: port(&self.ports[1]) }
    }

    /// Earliest time strictly after `t` at which some output is scheduled to
    /// change, assuming no further input changes.
    pub fn next_event(&self, t: Time) -> Option<Time> {
        self.ports
            .iter()
            .flat_map(|p| [p.data_out.next_change_after(t), p.busy.next_change_after(t), p.int.next_change_after(t)])
            .flatten()
            .min()
    }

    /// Apply a new input snapshot at time `t` (>= previous `t`).
    pub fn set_inputs(&mut self, t: Time, inputs: Inputs) {
        assert!(t >= self.now, "set_inputs() time went backwards ({t} < {})", self.now);
        self.now = t;
        let old = [self.ports[0].inp, self.ports[1].inp];
        let new = [inputs.l, inputs.r];

        // 1. Record which inputs changed and when.
        let mut addr_changed = [false; 2];
        for (i, p) in [Port::Left, Port::Right].into_iter().enumerate() {
            let (o, n) = (old[i], new[i]);
            let ps = self.ps_mut(p);
            ps.inp = n;
            if o.addr != n.addr {
                ps.addr_since = t;
                addr_changed[i] = true;
            }
            if o.ce_n != n.ce_n && n.ce_n == Level::L {
                ps.ce_low_since = t;
            }
            if o.data != n.data {
                ps.data_since = t;
            }
        }

        // 2. Write state machine per port, using the arbitration state as it
        //    was before this event (busy changes take effect later).
        for (i, p) in [Port::Left, Port::Right].into_iter().enumerate() {
            let (o, n) = (old[i], new[i]);
            let wc = write_conditions(&n);
            let ps = self.ps(p);
            match (ps.write_start, ps.write_x, ps.start_x, wc) {
                (Some(_), None, _, Some(true)) => {}
                (Some(_), None, _, None) => self.ps_mut(p).write_x = Some(t),
                (Some(_), None, _, Some(false)) => {
                    self.finish_write(p, o, t, t);
                    self.end_write(p);
                }
                (Some(_), Some(tx), _, Some(false)) => {
                    self.finish_write(p, o, tx, t);
                    self.end_write(p);
                }
                (Some(_), Some(tx), _, Some(true)) => {
                    self.finish_write(p, o, tx, t);
                    self.end_write(p);
                    self.begin_write(p);
                }
                (Some(_), Some(_), _, None) => {}
                (None, _, None, Some(true)) => self.begin_write(p),
                (None, _, None, None) => self.ps_mut(p).start_x = Some(t),
                (None, _, None, Some(false)) => {}
                (None, _, Some(sx), Some(true)) => {
                    if self.ps(p).addr_since > sx {
                        self.warn(p, WarningKind::AddrChangedDuringWrite);
                        match levels_value(&o.addr) {
                            Some(a) => self.mem[a as usize] = None,
                            None => self.corrupt_all(),
                        }
                    }
                    self.ps_mut(p).start_x = None;
                    self.begin_write(p);
                }
                (None, _, Some(sx), Some(false)) => {
                    // Unknown since power-up resolving to inactive is not a glitch.
                    if sx > 0 {
                        self.warn(p, WarningKind::GlitchWrite);
                        match levels_value(&o.addr) {
                            Some(a) => self.mem[a as usize] = None,
                            None => self.corrupt_all(),
                        }
                    }
                    self.ps_mut(p).start_x = None;
                }
                (None, _, Some(_), None) => {}
            }
            if addr_changed[i] {
                if self.ps(p).write_start.is_some_and(|ws| ws < t) && wc != Some(false) {
                    // Address moved under a (possibly) active write.
                    self.warn(p, WarningKind::AddrChangedDuringWrite);
                    if let Some(a) = levels_value(&o.addr) {
                        self.mem[a as usize] = None;
                    } else {
                        self.corrupt_all();
                    }
                } else if let Some(we) = self.ps(p).write_end
                    && t < we + self.t.tha
                {
                    self.warn(p, WarningKind::AddrHold { have: t - we });
                    if let Some(a) = levels_value(&o.addr) {
                        self.mem[a as usize] = None;
                    }
                    if let Some(a) = levels_value(&n.addr) {
                        self.mem[a as usize] = None;
                    }
                }
            }
        }

        // 3. Arbitration.
        self.update_arbitration(old);

        // 5. Data outputs.
        for (i, p) in [Port::Left, Port::Right].into_iter().enumerate() {
            self.update_data_out(p, old[i], addr_changed[i]);
        }

        // 6. Interrupt flags.
        for (i, p) in [Port::Left, Port::Right].into_iter().enumerate() {
            self.update_int(p, old[i]);
        }

        // Keep the timelines from growing without bound, but retain the tSD
        // lookback window that the bus-conflict check needs.
        let keep_from = t.saturating_sub(self.t.tsd);
        for ps in &mut self.ports {
            ps.data_out.forget_before(keep_from);
            ps.busy.forget_before(keep_from);
            ps.int.forget_before(keep_from);
        }
    }

    fn corrupt_all(&mut self) {
        for c in &mut self.mem {
            *c = None;
        }
    }

    /// Port `p`'s BUSY is (or may be) internally asserted at time `t`.
    fn busy_state(&self, p: Port) -> Arb {
        match self.arb {
            Arb::Loser(q) if q == p => Arb::Loser(p),
            Arb::Loser(_) => Arb::None,
            other => other,
        }
    }

    /// Called when port `p`'s write conditions stop being true at `self.now`.
    /// `o` are the port's inputs just before the event.  `end_known` is false
    /// if the write ended because a control pin went X/Z.
    fn begin_write(&mut self, p: Port) {
        let t = self.now;
        let ps = self.ps_mut(p);
        ps.write_start = Some(t);
        ps.write_end = None;
    }
    fn end_write(&mut self, p: Port) {
        let ps = self.ps_mut(p);
        ps.write_start = None;
        ps.write_x = None;
        ps.int_set_pending = false;
    }

    /// Port `p`'s write ended at some instant in `[tx, tr]` (`tx == tr` when
    /// exact).  `o` are the port's inputs just before this event.
    fn finish_write(&mut self, p: Port, o: PortInputs, tx: Time, tr: Time) {
        let te = tx;
        let tm = self.t;
        let ps = self.ps(p);
        let start = ps.write_start.unwrap();
        let addr = levels_value(&o.addr);
        let data = levels_value(&o.data).map(|d| d as u8);
        let int_pending = ps.int_set_pending;
        let ce_since = ps.ce_low_since;
        let addr_since = ps.addr_since;
        let data_since = ps.data_since;
        let busy_release_by = ps.busy_release_by;
        let busy_since = ps.busy_since;
        let out_z = ps.data_out.is_const_over(te.saturating_sub(tm.tsd), tr, DataOut::Z);
        self.ps_mut(p).write_end = Some(tr);

        // Where did it go?
        let Some(addr) = addr else {
            self.warn(p, WarningKind::AddrUnknown);
            self.corrupt_all();
            return;
        };
        let cell = addr as usize;

        let mut ok = true;
        if addr_since > te {
            self.warn(p, WarningKind::AddrChangedDuringWrite);
            ok = false;
        }

        // Busy / write inhibit.
        let mut eff_start = start;
        let mut min_width = tm.tpwe;
        let mut after_busy = false;
        match self.busy_state(p) {
            Arb::Loser(_) => {
                let bs = busy_since.unwrap_or(0);
                if te < bs + tm.tbla {
                    // Internal inhibit may or may not have engaged yet.
                    self.warn(p, WarningKind::WriteDuringBusyTransition);
                    self.mem[cell] = None;
                } else {
                    self.warn(p, WarningKind::WriteInhibited);
                }
                self.finish_int_set(p, int_pending, false);
                return;
            }
            Arb::Ambiguous => {
                self.warn(p, WarningKind::ArbitrationAmbiguous);
                self.mem[cell] = None;
                self.finish_int_set(p, int_pending, false);
                return;
            }
            Arb::None => {
                if let Some(rel) = busy_release_by
                    && rel > start {
                        // Busy was released during this pulse.
                        if te < rel {
                            self.warn(p, WarningKind::WriteDuringBusyTransition);
                            self.mem[cell] = None;
                            self.finish_int_set(p, int_pending, false);
                            return;
                        }
                        eff_start = rel;
                        min_width = tm.twh;
                        after_busy = true;
                    }
            }
        }

        // Timing checks.
        let width = te.saturating_sub(eff_start);
        if width < min_width {
            if after_busy {
                self.warn(p, WarningKind::WriteAfterBusyTooShort { width });
            } else {
                self.warn(p, WarningKind::WritePulseTooShort { width });
            }
            ok = false;
        }
        if te.saturating_sub(ce_since) < tm.tsce {
            self.warn(p, WarningKind::CeSetup { have: te.saturating_sub(ce_since) });
            ok = false;
        }
        if te.saturating_sub(addr_since) < tm.taw {
            self.warn(p, WarningKind::AddrSetup { have: te.saturating_sub(addr_since) });
            ok = false;
        }
        let Some(data) = data else {
            self.warn(p, WarningKind::DataNotDriven);
            self.mem[cell] = None;
            self.finish_int_set(p, int_pending, false);
            return;
        };
        if data_since > te || te - data_since < tm.tsd {
            self.warn(p, WarningKind::DataSetup { have: te.saturating_sub(data_since) });
            ok = false;
        }
        if !out_z {
            self.warn(p, WarningKind::BusConflict);
            ok = false;
        }
        self.mem[cell] = if ok { Some(data) } else { None };
        self.finish_int_set(p, int_pending, ok);
    }

    /// If this write was the one that set the other port's INT and it turned
    /// out not to be a valid write, the flag state is unknown.
    fn finish_int_set(&mut self, p: Port, pending: bool, ok: bool) {
        if pending && !ok {
            let t = self.now;
            self.ps_mut(p.other()).int.set_from(t, Level::X);
        }
    }

    fn update_arbitration(&mut self, old: [PortInputs; 2]) {
        let t = self.now;
        let tm = self.t;
        // Refresh key timestamps.
        for (i, p) in [Port::Left, Port::Right].into_iter().enumerate() {
            if self.ps(p).key() != arb_key(&old[i]) {
                self.ps_mut(p).key_since = t;
            }
        }
        let (l, r) = (&self.ports[0], &self.ports[1]);
        let contention = if l.inp.ce_n == Level::H || r.inp.ce_n == Level::H {
            Some(false)
        } else if let (Some(a), Some(b)) = (l.addr(), r.addr())
            && a != b
        {
            Some(false)
        } else {
            match (l.key(), r.key()) {
                (Some((true, a)), Some((true, b))) => Some(a == b),
                _ => None, // CE or address not known on an enabled port
            }
        };
        let mut tps_violation = false;
        let new_arb = match contention {
            Some(false) => Arb::None,
            None => Arb::Ambiguous,
            Some(true) => {
                if l.key_since + tm.tps <= r.key_since {
                    Arb::Loser(Port::Right)
                } else if r.key_since + tm.tps <= l.key_since {
                    Arb::Loser(Port::Left)
                } else {
                    tps_violation = true;
                    Arb::Ambiguous
                }
            }
        };
        if new_arb == self.arb {
            return;
        }
        // Release ports that were busy under the old state.
        let released: Vec<Port> = match self.arb {
            Arb::None => vec![],
            Arb::Loser(p) => vec![p],
            Arb::Ambiguous => vec![Port::Left, Port::Right],
        };
        for p in released {
            let rel = t + tm.tbhc.max(tm.tbha);
            let ps = self.ps_mut(p);
            ps.busy.set_from(t, Level::X);
            ps.busy.set_from(rel, Level::Z);
            ps.busy_release_by = Some(rel);
            ps.busy_since = None;
        }
        match new_arb {
            Arb::None => {}
            Arb::Loser(p) => {
                let ps = self.ps_mut(p);
                ps.busy.set_from(t, Level::X);
                ps.busy.set_from(t + tm.tbla.max(tm.tblc), Level::L);
                ps.busy_since = Some(t);
            }
            Arb::Ambiguous => {
                if tps_violation {
                    self.warn(Port::Left, WarningKind::ArbitrationAmbiguous);
                }
                for p in [Port::Left, Port::Right] {
                    let ps = self.ps_mut(p);
                    ps.busy.set_from(t, Level::X);
                    ps.busy_since = Some(t);
                }
            }
        }
        self.arb = new_arb;
        self.arb_since = t;
    }

    fn update_data_out(&mut self, p: Port, o: PortInputs, addr_changed: bool) {
        let t = self.now;
        let tm = self.t;
        let busy = self.busy_state(p) != Arb::None;
        let ps = self.ps(p);
        let n = ps.inp;
        let driving = ps.read_driving();
        let was_driving = read_driving(&o);
        let cur = ps.data_out.at(t);
        let release_by = ps.busy_release_by;
        let mut vnb = ps.valid_not_before;

        match driving {
            None => {
                // Some control pin is X/Z on a port that might be enabled:
                // the output is simply unknown until the controls are.
                if n.ce_n != Level::H {
                    self.ps_mut(p).data_out.set_from(t, DataOut::X);
                    self.ps_mut(p).valid_not_before = 0;
                }
            }
            Some(false) => {
                // Already turning off keeps its schedule.
                let turning_off = cur == DataOut::X && self.ps(p).data_out.next_change_after(t).is_some();
                if cur != DataOut::Z && !turning_off {
                    let hz = tm.thzce.max(tm.thzoe).max(tm.thzwe);
                    let ps = self.ps_mut(p);
                    ps.data_out.set_from(t, DataOut::X);
                    ps.data_out.set_from(t + hz, DataOut::Z);
                    ps.valid_not_before = 0;
                }
            }
            Some(true) => {
                let mut x_start = t;
                let mut valid = vnb;
                if was_driving != Some(true) {
                    // Coming out of high-Z.  Every enabling path that just
                    // switched must propagate before the outputs can turn
                    // on, so the earliest low-Z is the slowest of their
                    // minimums.
                    let mut lz = 0;
                    if o.ce_n != n.ce_n {
                        lz = lz.max(tm.tlzce);
                        valid = valid.max(t + tm.tace);
                    }
                    if o.oe_n != n.oe_n {
                        lz = lz.max(tm.tlzoe);
                        valid = valid.max(t + tm.tdoe);
                    }
                    if o.rw_n != n.rw_n {
                        lz = lz.max(tm.tlzwe);
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
                    x_start = t + tm.toha;
                    valid = valid.max(t + tm.taa);
                } else if cur != DataOut::X && vnb <= t && !busy && release_by.is_none_or(|r| r + tm.tbdd <= t) {
                    // Nothing relevant changed; keep driving.
                    return;
                }
                if let Some(r) = release_by {
                    valid = valid.max(r + tm.tbdd);
                }
                let ps = self.ps_mut(p);
                if busy {
                    ps.data_out.set_from(x_start, DataOut::X);
                    ps.valid_not_before = valid;
                    return;
                }
                match levels_value(&n.addr) {
                    None => {
                        // Reading an unknown address just yields unknown data.
                        let ps = self.ps_mut(p);
                        ps.data_out.set_from(x_start, DataOut::X);
                        ps.valid_not_before = valid;
                    }
                    Some(a) => {
                        vnb = valid;
                        // If the output already shows the right address and
                        // nothing new invalidated it, don't glitch it to X.
                        if cur == DataOut::Read(a) && ps.data_out.at(x_start) == cur && valid <= t {
                            return;
                        }
                        ps.data_out.set_from(x_start, DataOut::X);
                        ps.data_out.set_from(valid.max(x_start), DataOut::Read(a));
                        ps.valid_not_before = vnb;
                    }
                }
            }
        }
    }

    fn update_int(&mut self, p: Port, o: PortInputs) {
        let t = self.now;
        let tm = self.t;
        let busy = self.busy_state(p);
        let ps = self.ps(p);
        let n = ps.inp;
        let addr = levels_value(&n.addr);
        let old_addr = levels_value(&o.addr);
        let set_cond = |i: &PortInputs, a: Option<u16>| {
            i.ce_n == Level::L && i.rw_n == Level::L && a == Some(p.mailbox_set())
        };
        let clr_cond = |i: &PortInputs, a: Option<u16>| {
            i.ce_n == Level::L && i.oe_n == Level::L && a == Some(p.mailbox_clear())
        };
        let set_now = set_cond(&n, addr);
        let set_was = set_cond(&o, old_addr);
        let clr_now = clr_cond(&n, addr);
        let clr_was = clr_cond(&o, old_addr);

        // Unknown-ness: an enabled port with X controls/address touching a
        // mailbox could be doing anything.  Only worry if a mailbox is
        // plausibly addressed.
        let maybe_mailbox = |a: &[Level; 10]| {
            a.iter().enumerate().all(|(i, l)| match l.bit() {
                Some(b) => i == 0 || b, // 3FE/3FF: bits 1..9 set, bit 0 either
                None => true,
            })
        };
        if n.ce_n != Level::H && (n.ce_n.bit().is_none() || n.rw_n.bit().is_none() || n.oe_n.bit().is_none() || addr.is_none())
            && maybe_mailbox(&n.addr)
        {
            let other = p.other();
            self.ps_mut(other).int.set_from(t, Level::X);
            self.ps_mut(p).int.set_from(t, Level::X);
            return;
        }

        if set_now && !set_was {
            match busy {
                Arb::Loser(_) => {} // "If BUSY = L, no change."
                Arb::Ambiguous => {
                    self.ps_mut(p.other()).int.set_from(t, Level::X);
                }
                Arb::None => {
                    let other = p.other();
                    let ps = self.ps_mut(other);
                    ps.int.set_from(t, Level::X);
                    ps.int.set_from(t + tm.tins, Level::L);
                    self.ps_mut(p).int_set_pending = true;
                }
            }
        }
        if clr_now && !clr_was {
            match busy {
                Arb::Loser(_) => {}
                Arb::Ambiguous => {
                    self.ps_mut(p).int.set_from(t, Level::X);
                }
                Arb::None => {
                    let ps = self.ps_mut(p);
                    ps.int.set_from(t, Level::X);
                    ps.int.set_from(t + tm.tinr, Level::Z);
                }
            }
        }
        // Set and clear of the same flag at once: this port clearing its own
        // INT while the other port sets it.
        let other = p.other();
        if clr_now && self.ps(other).int_set_pending && self.ps(other).write_start == Some(t) {
            self.warn(p, WarningKind::IntSetClearRace);
            self.ps_mut(p).int.set_from(t, Level::X);
        }
    }
}

impl Default for Cy7c131 {
    fn default() -> Self {
        Self::new()
    }
}
