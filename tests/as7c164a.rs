//! Datasheet-waveform tests for the AS7C164A-15 model.

use mips32::as7c164a::*;
use mips32::cy7c131::Bus;

const L: Level = Level::L;
const H: Level = Level::H;

fn ns(n: u64) -> Time {
    n * NS
}

struct Bench {
    chip: As7c164a,
    inp: Inputs,
}

impl Bench {
    fn new() -> Self {
        let mut chip = As7c164a::new();
        for a in 0..8192u16 {
            chip.preload(a, (a as u8) ^ 0xA5);
        }
        let inp = Inputs::idle();
        chip.set_inputs(0, inp);
        Bench { chip, inp }
    }
    fn at(&mut self, t: u64, f: impl FnOnce(&mut Inputs)) {
        f(&mut self.inp);
        self.chip.set_inputs(ns(t), self.inp);
    }
    fn out(&self, t: u64) -> Bus {
        self.chip.output(ns(t))
    }
    fn no_warnings(&self) {
        assert!(self.chip.warnings().is_empty(), "unexpected warnings: {:#?}", self.chip.warnings());
    }
    fn kinds(&self) -> Vec<WarningKind> {
        self.chip.warnings().iter().map(|w| w.kind.clone()).collect()
    }
}

fn preloaded(a: u16) -> u8 {
    (a as u8) ^ 0xA5
}

// Read cycle, address controlled: device continuously selected.
#[test]
fn read_cycle_address_controlled() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_oe(L).with_addr(0x123));
    assert_eq!(b.out(3), Bus::Z);
    assert_eq!(b.out(4), Bus::X); // tCLZ 4
    assert_eq!(b.out(15), Bus::V(preloaded(0x123))); // tACE 15
    // Address change: previous data holds tOH = 3, new data at tAA = 15.
    b.at(100, |i| i.addr = addr_levels(0x1FFF));
    assert_eq!(b.out(102), Bus::V(preloaded(0x123)));
    assert_eq!(b.out(103), Bus::X);
    assert_eq!(b.out(114), Bus::X);
    assert_eq!(b.out(115), Bus::V(preloaded(0x1FFF)));
    b.no_warnings();
}

// Read cycle, CE#/CE2/OE# controlled.
#[test]
fn read_cycle_enable_controlled() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_oe(L).with_addr(0x010));
    assert_eq!(b.out(50), Bus::Z);
    b.at(100, |i| i.ce_n = L);
    assert_eq!(b.out(103), Bus::Z);
    assert_eq!(b.out(104), Bus::X); // tCLZ 4
    assert_eq!(b.out(115), Bus::V(preloaded(0x010)));
    // OE# high -> high-Z within tOHZ 7.
    b.at(200, |i| i.oe_n = H);
    assert_eq!(b.out(200), Bus::X);
    assert_eq!(b.out(207), Bus::Z);
    // OE# low: tOLZ 0, tOE 7.
    b.at(300, |i| i.oe_n = L);
    assert_eq!(b.out(300), Bus::X);
    assert_eq!(b.out(306), Bus::X);
    assert_eq!(b.out(307), Bus::V(preloaded(0x010)));
    // CE2 low deselects: tCHZ 7.
    b.at(400, |i| i.ce2 = L);
    assert_eq!(b.out(400), Bus::X);
    assert_eq!(b.out(407), Bus::Z);
    // CE2 back high: chip-enable access.
    b.at(500, |i| i.ce2 = H);
    assert_eq!(b.out(504), Bus::X);
    assert_eq!(b.out(515), Bus::V(preloaded(0x010)));
    b.no_warnings();
}

// Write cycle, WE# controlled, OE# high.
#[test]
fn write_cycle_we_controlled() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(0x0200).with_data(0xC3));
    b.at(30, |i| i.we_n = L);
    b.at(42, |i| i.we_n = H); // tWP 12 >= 10, tAW/tCW 42 >= 12, tDW 42 >= 8
    assert_eq!(b.chip.peek(0x0200), Some(0xC3));
    b.no_warnings();
    // Read it back.
    b.at(50, |i| {
        i.data = [Level::Z; 8];
        i.oe_n = L;
    });
    assert_eq!(b.out(57), Bus::V(0xC3));
    b.no_warnings();
}

// Write cycle, CE# controlled (WE# low first).
#[test]
fn write_cycle_ce_controlled() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_we(L).with_addr(0x0300).with_data(0x44));
    b.at(30, |i| i.ce_n = L);
    b.at(42, |i| i.ce_n = H); // tCW 12
    assert_eq!(b.chip.peek(0x0300), Some(0x44));
    b.no_warnings();
    // Same with CE2 as the strobe.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_ce2(L).with_we(L).with_addr(0x0301).with_data(0x45));
    b.at(30, |i| i.ce2 = H);
    b.at(42, |i| i.ce2 = L);
    assert_eq!(b.chip.peek(0x0301), Some(0x45));
    b.no_warnings();
    // 11ns is not enough.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_we(L).with_addr(0x0300).with_data(0x44));
    b.at(30, |i| i.ce_n = L);
    b.at(41, |i| i.ce_n = H);
    assert_eq!(b.chip.peek(0x0300), None);
    assert!(b.kinds().contains(&WarningKind::ChipEnableSetup { have: ns(11) }));
}

// Write with OE# low: WE# low first tri-states the outputs (tWHZ 8), so the
// pulse must cover tWHZ + tDW = 16 (datasheet note 3).
#[test]
fn write_with_oe_low() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_oe(L).with_addr(0x100));
    assert_eq!(b.out(15), Bus::V(preloaded(0x100)));
    b.at(50, |i| i.we_n = L);
    assert_eq!(b.out(50), Bus::X);
    assert_eq!(b.out(58), Bus::Z);
    b.at(58, |i| i.data = Inputs::idle().with_data(0x77).data);
    b.at(66, |i| i.we_n = H); // 16ns pulse, data 8 before end
    assert_eq!(b.chip.peek(0x100), Some(0x77));
    // Outputs come back no sooner than tOW = 4 after the write end.
    assert_eq!(b.out(69), Bus::Z);
    assert_eq!(b.out(70), Bus::X);
    b.at(70, |i| i.data = [Level::Z; 8]);
    assert_eq!(b.out(81), Bus::V(0x77));
    b.no_warnings();

    // Too short: chip may still drive while data is set up.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_oe(L).with_addr(0x100));
    b.at(50, |i| {
        i.we_n = L;
        i.data = Inputs::idle().with_data(0x77).data;
    });
    b.at(62, |i| i.we_n = H); // 12 >= tWP but < tWHZ + tDW
    assert_eq!(b.chip.peek(0x100), None);
    assert!(b.kinds().contains(&WarningKind::BusConflict));
}

#[test]
fn write_violations() {
    // Pulse too short.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(1).with_data(1));
    b.at(30, |i| i.we_n = L);
    b.at(39, |i| i.we_n = H);
    assert_eq!(b.chip.peek(1), None);
    assert_eq!(b.kinds(), vec![WarningKind::WritePulseTooShort { width: ns(9) }]);

    // Data setup.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(1));
    b.at(30, |i| i.we_n = L);
    b.at(33, |i| i.data = Inputs::idle().with_data(2).data);
    b.at(40, |i| i.we_n = H);
    assert_eq!(b.chip.peek(1), None);
    assert_eq!(b.kinds(), vec![WarningKind::DataSetup { have: ns(7) }]);

    // Address setup.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(1).with_data(3));
    b.at(30, |i| {
        i.addr = addr_levels(2);
        i.we_n = L;
    });
    b.at(41, |i| i.we_n = H);
    assert_eq!(b.chip.peek(2), None);
    assert!(b.kinds().contains(&WarningKind::AddrSetup { have: ns(11) }));

    // Address change under an active write (note 1).
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(1).with_data(4));
    b.at(30, |i| i.we_n = L);
    b.at(40, |i| i.addr = addr_levels(2));
    b.at(55, |i| i.we_n = H);
    assert_eq!(b.chip.peek(1), None);
    assert_eq!(b.chip.peek(2), Some(4));
    assert_eq!(b.kinds(), vec![WarningKind::AddrChangedDuringWrite]);

    // Undriven data.
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_addr(1));
    b.at(30, |i| i.we_n = L);
    b.at(45, |i| i.we_n = H);
    assert_eq!(b.chip.peek(1), None);
    assert_eq!(b.kinds(), vec![WarningKind::DataNotDriven]);
}

/// Note 5: CE# falling with or after WE# falling never lets the outputs turn
/// on.
#[test]
fn ce_after_we_keeps_outputs_off() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_oe(L).with_we(L).with_addr(5).with_data(9));
    b.at(20, |i| i.ce_n = L);
    for t in 20..40 {
        assert_eq!(b.out(t), Bus::Z, "t={t}");
    }
    b.at(40, |i| i.ce_n = H);
    assert_eq!(b.chip.peek(5), Some(9));
    b.no_warnings();
}

#[test]
fn unknown_controls_are_x_not_violations() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_oe(L).with_addr(7));
    assert_eq!(b.out(15), Bus::V(preloaded(7)));
    b.at(20, |i| i.oe_n = Level::X);
    assert_eq!(b.out(20), Bus::X);
    b.at(30, |i| i.oe_n = L);
    assert_eq!(b.out(36), Bus::X);
    assert_eq!(b.out(37), Bus::V(preloaded(7)));
    b.no_warnings();
    // But a write ending through an X control is a violation.
    b.at(50, |i| {
        i.oe_n = H;
        i.data = Inputs::idle().with_data(1).data;
    });
    b.at(60, |i| i.we_n = L);
    b.at(75, |i| i.we_n = Level::X);
    assert_eq!(b.chip.peek(7), None);
    assert!(b.kinds().contains(&WarningKind::ControlUnknown));
}

#[test]
fn next_event() {
    let mut b = Bench::new();
    b.at(0, |i| *i = Inputs::idle().with_ce(L).with_oe(L).with_addr(1));
    assert_eq!(b.chip.next_event(0), Some(ns(4)));
    assert_eq!(b.chip.next_event(ns(4)), Some(ns(15)));
    assert_eq!(b.chip.next_event(ns(15)), None);
}
