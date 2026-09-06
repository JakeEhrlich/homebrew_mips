//! Datasheet-waveform tests for the CY7C131-15 model.  Each test reproduces one
//! of the switching waveforms / truth tables from the Cypress (38-00027-L) or
//! IDT7130 (DSC-2689/9) datasheets.

use mips32::cy7c131::*;

const L: Level = Level::L;
const H: Level = Level::H;

fn ns(n: u64) -> Time {
    n * NS
}

/// Drive both ports; helper keeps the test bodies readable.
struct Bench {
    chip: Cy7c131,
    inp: Inputs,
}

impl Bench {
    fn new() -> Self {
        let mut chip = Cy7c131::new();
        for a in 0..1024u16 {
            chip.preload(a, (a as u8) ^ 0x5A);
        }
        let inp = Inputs::idle();
        chip.set_inputs(0, inp);
        Bench { chip, inp }
    }
    fn at(&mut self, t: u64, f: impl FnOnce(&mut Inputs)) {
        f(&mut self.inp);
        self.chip.set_inputs(ns(t), self.inp);
    }
    fn l(&self, t: u64) -> PortOutputs {
        self.chip.outputs(ns(t)).l
    }
    fn r(&self, t: u64) -> PortOutputs {
        self.chip.outputs(ns(t)).r
    }
    fn no_warnings(&self) {
        assert!(self.chip.warnings().is_empty(), "unexpected warnings: {:#?}", self.chip.warnings());
    }
    fn warning_kinds(&self) -> Vec<WarningKind> {
        self.chip.warnings().iter().map(|w| w.kind.clone()).collect()
    }
}

fn preloaded(a: u16) -> u8 {
    (a as u8) ^ 0x5A
}

// ---------------------------------------------------------------------------
// Read Cycle No. 1: address access, device continuously selected.

#[test]
fn read_cycle_1_address_access() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x123));
    // Out of high-Z: X after tLZCE, valid after tACE.
    assert_eq!(b.l(0).data, Bus::Z);
    assert_eq!(b.l(2).data, Bus::Z);
    assert_eq!(b.l(3).data, Bus::X);
    assert_eq!(b.l(14).data, Bus::X);
    assert_eq!(b.l(15).data, Bus::V(preloaded(0x123)));
    assert_eq!(b.l(100).data, Bus::V(preloaded(0x123)));

    // Address change at t=100: tOHA = 0 so X immediately, valid at +tAA.
    b.at(100, |i| i.l.addr = addr_levels(0x3AB));
    assert_eq!(b.l(100).data, Bus::X);
    assert_eq!(b.l(114).data, Bus::X);
    assert_eq!(b.l(115).data, Bus::V(preloaded(0x3AB)));
    // BUSY/INT untouched.
    assert_eq!(b.l(115).busy_n, Level::Z);
    assert_eq!(b.l(115).int_n, Level::Z);
    assert_eq!(b.r(115).data, Bus::Z);
    b.no_warnings();
}

/// Address toggling faster than tRC never produces valid data.
#[test]
fn read_faster_than_trc_never_valid() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(1));
    b.at(10, |i| i.l.addr = addr_levels(2));
    b.at(20, |i| i.l.addr = addr_levels(3));
    for t in 20..35 {
        assert_ne!(b.l(t).data, Bus::V(preloaded(1)), "t={t}");
        assert_ne!(b.l(t).data, Bus::V(preloaded(2)), "t={t}");
    }
    assert_eq!(b.l(35).data, Bus::V(preloaded(3)));
}

// ---------------------------------------------------------------------------
// Read Cycle No. 2: CE / OE access, with power-down.

#[test]
fn read_cycle_2_ce_oe_access() {
    let mut b = Bench::new();
    // Address valid first, then CE low with OE already low.
    b.at(0, |i| i.l = PortInputs::idle().with_oe(L).with_addr(0x010));
    assert_eq!(b.l(50).data, Bus::Z); // CE high: high-Z regardless of OE
    b.at(100, |i| i.l.ce_n = L);
    assert_eq!(b.l(102).data, Bus::Z);
    assert_eq!(b.l(103).data, Bus::X); // tLZCE min 3
    assert_eq!(b.l(115).data, Bus::V(preloaded(0x010))); // tACE 15

    // OE high -> high-Z within tHZOE.
    b.at(200, |i| i.l.oe_n = H);
    assert_eq!(b.l(200).data, Bus::X);
    assert_eq!(b.l(209).data, Bus::X);
    assert_eq!(b.l(210).data, Bus::Z);

    // OE low again: tLZOE 3 min, tDOE 10 max.
    b.at(300, |i| i.l.oe_n = L);
    assert_eq!(b.l(302).data, Bus::Z);
    assert_eq!(b.l(303).data, Bus::X);
    assert_eq!(b.l(309).data, Bus::X);
    assert_eq!(b.l(310).data, Bus::V(preloaded(0x010)));

    // CE high -> high-Z within tHZCE.
    b.at(400, |i| i.l.ce_n = H);
    assert_eq!(b.l(400).data, Bus::X);
    assert_eq!(b.l(410).data, Bus::Z);
    b.no_warnings();
}

/// CE and OE asserted together with an address change: valid = max of all.
#[test]
fn read_combined_ce_oe_addr() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(5)); // OE high
    assert_eq!(b.l(20).data, Bus::Z);
    b.at(20, |i| i.l.oe_n = L); // tDOE 10 -> valid at 30
    assert_eq!(b.l(29).data, Bus::X);
    assert_eq!(b.l(30).data, Bus::V(preloaded(5)));
    // Address change 2ns after OE: valid at 22+15 = 37, not 30.
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(5));
    b.at(20, |i| i.l.oe_n = L);
    b.at(22, |i| i.l.addr = addr_levels(6));
    assert_eq!(b.l(36).data, Bus::X);
    assert_eq!(b.l(37).data, Bus::V(preloaded(6)));
}

// ---------------------------------------------------------------------------
// Write Cycle No. 1: OE three-states the I/Os, R/W-controlled write.

#[test]
fn write_cycle_1_oe_controlled() {
    let mut b = Bench::new();
    // Port idle-selected: CE low, OE high, R/W high.
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(H).with_addr(0x200));
    b.at(20, |i| i.l.data = data_levels(0xC3)); // data driven externally
    b.at(30, |i| i.l.rw_n = L); // write starts
    b.at(45, |i| i.l.rw_n = H); // tPWE = 15 >= 12, tSD = 25 >= 10
    assert_eq!(b.chip.peek(0x200), Some(0xC3));
    b.at(50, |i| i.l.addr = addr_levels(0x201)); // tHA = 5 >= 2
    b.at(60, |i| i.l.data = [Level::Z; 8]);
    b.no_warnings();

    // Read it back on the other port.
    b.at(100, |i| i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x200));
    assert_eq!(b.r(115).data, Bus::V(0xC3));
    b.no_warnings();
}

// ---------------------------------------------------------------------------
// Write Cycle No. 2: R/W three-states the I/Os (OE low throughout).

#[test]
fn write_cycle_2_rw_controlled() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x100));
    assert_eq!(b.l(15).data, Bus::V(preloaded(0x100)));
    b.at(50, |i| i.l.rw_n = L); // outputs go X now, Z by +tHZWE
    assert_eq!(b.l(50).data, Bus::X);
    assert_eq!(b.l(60).data, Bus::Z);
    b.at(60, |i| i.l.data = data_levels(0x77)); // drive only after the chip released the bus
    b.at(75, |i| i.l.rw_n = H); // pulse 25 >= tHZWE + tSD = 20; tSD = 15
    assert_eq!(b.chip.peek(0x100), Some(0x77));
    // tLZWE = 0: outputs may come back at once, valid after an access time.
    assert_eq!(b.l(75).data, Bus::X);
    assert_eq!(b.l(90).data, Bus::V(0x77));
    b.no_warnings();
}

/// Datasheet note 22: with OE low, a write pulse must cover tHZWE + tSD or the
/// chip may still be driving the bus while the external data is set up.
#[test]
fn write_with_oe_low_short_pulse_is_bus_conflict() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x100));
    b.at(50, |i| {
        i.l.rw_n = L;
        i.l.data = data_levels(0x77);
    });
    b.at(63, |i| i.l.rw_n = H); // 13ns: >= tPWE but < tHZWE + tSD
    assert_eq!(b.chip.peek(0x100), None);
    assert!(b.warning_kinds().contains(&WarningKind::BusConflict), "{:?}", b.warning_kinds());
}

/// Same conflict, but with an unrelated event (on the other port) landing
/// after the chip has released the bus: the model must still remember that it
/// was driving during the setup window.
#[test]
fn bus_conflict_survives_unrelated_events() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x100));
    b.at(50, |i| {
        i.l.rw_n = L;
        i.l.data = data_levels(0x77);
    });
    b.at(61, |i| i.r.addr = addr_levels(0x001));
    b.at(62, |i| i.r.addr = addr_levels(0x002));
    b.at(63, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x100), None);
    assert!(b.warning_kinds().contains(&WarningKind::BusConflict), "{:?}", b.warning_kinds());
}

// ---------------------------------------------------------------------------
// Write timing violations leave the cell unknown and are reported.

#[test]
fn write_pulse_too_short() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300).with_data(0x11));
    b.at(30, |i| i.l.rw_n = L);
    b.at(41, |i| i.l.rw_n = H); // 11 < tPWE 12
    assert_eq!(b.chip.peek(0x300), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::WritePulseTooShort { width: ns(11) }]);
}

#[test]
fn write_data_setup_violation() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300));
    b.at(30, |i| i.l.rw_n = L);
    b.at(36, |i| i.l.data = data_levels(0x22));
    b.at(45, |i| i.l.rw_n = H); // data only 9 < tSD 10
    assert_eq!(b.chip.peek(0x300), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::DataSetup { have: ns(9) }]);
}

#[test]
fn write_address_setup_violation() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300).with_data(0x33));
    b.at(30, |i| i.l.addr = addr_levels(0x301));
    b.at(30, |i| i.l.rw_n = L);
    b.at(41, |i| i.l.rw_n = H); // 11 < tAW 12 (and < tPWE)
    assert_eq!(b.chip.peek(0x301), None);
    assert!(b.warning_kinds().contains(&WarningKind::AddrSetup { have: ns(11) }));
}

#[test]
fn write_ce_setup_violation_ce_controlled() {
    // R/W low first, then CE pulses for only 11ns: CE-controlled write, tSCE not met.
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(H).with_rw(L).with_addr(0x300).with_data(0x44));
    b.at(30, |i| i.l.ce_n = L);
    b.at(41, |i| i.l.ce_n = H);
    assert_eq!(b.chip.peek(0x300), None);
    assert!(b.warning_kinds().contains(&WarningKind::CeSetup { have: ns(11) }), "{:?}", b.warning_kinds());
    // And a proper CE-controlled write works.
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(H).with_rw(L).with_addr(0x300).with_data(0x44));
    b.at(30, |i| i.l.ce_n = L);
    b.at(42, |i| i.l.ce_n = H);
    assert_eq!(b.chip.peek(0x300), Some(0x44));
    b.no_warnings();
}

#[test]
fn write_address_hold_violation() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300).with_data(0x55));
    b.at(30, |i| i.l.rw_n = L);
    b.at(45, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x300), Some(0x55));
    b.at(46, |i| i.l.addr = addr_levels(0x301)); // 1 < tHA 2
    assert_eq!(b.chip.peek(0x300), None);
    assert_eq!(b.chip.peek(0x301), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::AddrHold { have: ns(1) }]);
}

#[test]
fn address_change_during_write_corrupts_old_cell() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300).with_data(0x66));
    b.at(30, |i| i.l.rw_n = L);
    b.at(40, |i| i.l.addr = addr_levels(0x301));
    b.at(55, |i| i.l.rw_n = H); // 15 after the address change: fine for 0x301
    assert_eq!(b.chip.peek(0x300), None);
    assert_eq!(b.chip.peek(0x301), Some(0x66));
    assert_eq!(b.warning_kinds(), vec![WarningKind::AddrChangedDuringWrite]);
}

#[test]
fn write_with_undriven_data_is_unknown() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300));
    b.at(30, |i| i.l.rw_n = L);
    b.at(45, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x300), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::DataNotDriven]);
}

#[test]
fn write_with_unknown_address_corrupts_everything() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x300).with_data(0x66));
    b.at(30, |i| {
        i.l.addr[4] = Level::X;
        i.l.rw_n = L;
    });
    b.at(45, |i| i.l.rw_n = H);
    assert!((0..1024).all(|a| b.chip.peek(a).is_none()));
    assert!(b.warning_kinds().contains(&WarningKind::AddrUnknown));
}

/// R/W from a registered GAL output: X for a few ns after each edge, but
/// with CE high that is not a write.  And a CE strobe whose end is an X
/// window still gives a provable write.
#[test]
fn write_through_x_windows() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(H).with_rw(Level::X).with_addr(0x300).with_data(0x66));
    b.at(4, |i| i.l.rw_n = L);
    b.at(14, |i| i.l.ce_n = L);
    b.at(30, |i| i.l.ce_n = Level::X);
    b.at(34, |i| i.l.ce_n = H);
    assert_eq!(b.chip.peek(0x300), Some(0x66));
    b.no_warnings();
    b.at(36, |i| i.l.rw_n = Level::X); // next cycle's register update
    b.at(40, |i| i.l.rw_n = H);
    b.no_warnings();
}

// ---------------------------------------------------------------------------
// Truth Table I: independent ports, no contention.

#[test]
fn both_ports_independent_when_addresses_differ() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x001);
        i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x002);
    });
    assert_eq!(b.l(15).data, Bus::V(preloaded(1)));
    assert_eq!(b.r(15).data, Bus::V(preloaded(2)));
    assert_eq!(b.l(15).busy_n, Level::Z);
    assert_eq!(b.r(15).busy_n, Level::Z);
    // Right writes 0x002 while left keeps reading 0x001.
    b.at(20, |i| i.r.oe_n = H);
    b.at(40, |i| {
        i.r.data = data_levels(0xAB);
        i.r.rw_n = L;
    });
    b.at(55, |i| i.r.rw_n = H);
    assert_eq!(b.chip.peek(2), Some(0xAB));
    assert_eq!(b.l(55).data, Bus::V(preloaded(1)));
    b.no_warnings();
}

// ---------------------------------------------------------------------------
// Busy Timing Diagram No. 1: CE arbitration.

#[test]
fn busy_ce_arbitration_left_first() {
    let mut b = Bench::new();
    // Both addresses match, both deselected.
    b.at(0, |i| {
        i.l = PortInputs::idle().with_addr(0x123).with_oe(L);
        i.r = PortInputs::idle().with_addr(0x123).with_oe(L);
    });
    b.at(100, |i| i.l.ce_n = L);
    b.at(105, |i| i.r.ce_n = L); // exactly tPS later: right loses
    assert_eq!(b.l(200).busy_n, Level::Z);
    assert_eq!(b.r(105).busy_n, Level::X);
    assert_eq!(b.r(119).busy_n, Level::X);
    assert_eq!(b.r(120).busy_n, Level::L); // tBLC 15
    // Left reads fine, right's data is not valid while busy.
    assert_eq!(b.l(200).data, Bus::V(preloaded(0x123)));
    assert_eq!(b.r(200).data, Bus::X);

    // Left leaves: BUSY high within tBHC, right's data valid tBDD after that.
    b.at(300, |i| i.l.ce_n = H);
    assert_eq!(b.r(300).busy_n, Level::X);
    assert_eq!(b.r(315).busy_n, Level::Z);
    assert_eq!(b.r(329).data, Bus::X);
    assert_eq!(b.r(330).data, Bus::V(preloaded(0x123)));
    b.no_warnings();
}

#[test]
fn busy_ce_arbitration_right_first() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_addr(0x123);
        i.r = PortInputs::idle().with_addr(0x123);
    });
    b.at(100, |i| i.r.ce_n = L);
    b.at(110, |i| i.l.ce_n = L);
    assert_eq!(b.r(200).busy_n, Level::Z);
    assert_eq!(b.l(125).busy_n, Level::L);
    b.no_warnings();
}

// Busy Timing Diagram No. 2: address arbitration.

#[test]
fn busy_address_arbitration() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x050);
        i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x051);
    });
    // Right moves onto left's address: right loses.
    b.at(100, |i| i.r.addr = addr_levels(0x050));
    assert_eq!(b.r(114).busy_n, Level::X);
    assert_eq!(b.r(115).busy_n, Level::L);
    assert_eq!(b.l(115).busy_n, Level::Z);
    assert_eq!(b.l(115).data, Bus::V(preloaded(0x050)));
    // Right moves away: mismatch releases BUSY within tBHA.
    b.at(200, |i| i.r.addr = addr_levels(0x052));
    assert_eq!(b.r(200).busy_n, Level::X);
    assert_eq!(b.r(215).busy_n, Level::Z);
    // Its read of 0x052 is valid only once BUSY release + tBDD has elapsed.
    assert_eq!(b.r(229).data, Bus::X);
    assert_eq!(b.r(230).data, Bus::V(preloaded(0x052)));
    b.no_warnings();

    // Left moving onto right's address: left loses.
    b.at(300, |i| i.l.addr = addr_levels(0x052));
    assert_eq!(b.l(315).busy_n, Level::L);
    assert_eq!(b.r(315).busy_n, Level::Z);
}

/// The winner leaving and coming back makes it the loser.
#[test]
fn busy_reassigned_when_winner_retoggles() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_ce(L).with_addr(0x050);
    });
    b.at(50, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x050));
    assert_eq!(b.r(65).busy_n, Level::L);
    b.at(100, |i| i.l.ce_n = H);
    assert_eq!(b.r(115).busy_n, Level::Z);
    b.at(150, |i| i.l.ce_n = L);
    assert_eq!(b.l(165).busy_n, Level::L);
    assert_eq!(b.r(165).busy_n, Level::Z);
}

/// tPS violated: exactly one port is busy but we cannot know which.
#[test]
fn busy_arbitration_ambiguous_within_tps() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_addr(0x123).with_data(0x11);
        i.r = PortInputs::idle().with_addr(0x123).with_data(0x22);
    });
    b.at(100, |i| i.l.ce_n = L);
    b.at(104, |i| i.r.ce_n = L); // 4 < tPS 5
    assert_eq!(b.l(200).busy_n, Level::X);
    assert_eq!(b.r(200).busy_n, Level::X);
    assert!(b.warning_kinds().contains(&WarningKind::ArbitrationAmbiguous));
    // A write from either side during the ambiguity is unknowable.
    b.at(200, |i| i.l.rw_n = L);
    b.at(220, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x123), None);
    // Contention ends: both BUSYs guaranteed released after tBHC.
    b.at(300, |i| i.r.ce_n = H);
    assert_eq!(b.l(314).busy_n, Level::X);
    assert_eq!(b.l(315).busy_n, Level::Z);
    assert_eq!(b.r(315).busy_n, Level::Z);
}

// ---------------------------------------------------------------------------
// Truth Table III note 3 + Read Cycle No. 3: write inhibit and tWH.

#[test]
fn write_inhibited_while_busy_then_completes_after_release() {
    let mut b = Bench::new();
    // Left reads 0x040 first; right wants to write 0x040.
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x040));
    b.at(50, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x040).with_data(0x99));
    assert_eq!(b.r(65).busy_n, Level::L);
    // Right pulls R/W low and holds it: the write is inhibited.
    b.at(100, |i| i.r.rw_n = L);
    b.at(150, |i| i.r.rw_n = H);
    assert_eq!(b.chip.peek(0x040), Some(preloaded(0x040)));
    assert_eq!(b.l(150).data, Bus::V(preloaded(0x040)));
    assert_eq!(b.warning_kinds(), vec![WarningKind::WriteInhibited]);
    b.chip.take_warnings();

    // Now right holds R/W low across the release: the write happens once
    // BUSY is high, provided R/W stays low for tWH after that.
    b.at(200, |i| i.r.rw_n = L);
    b.at(250, |i| i.l.ce_n = H); // release; BUSY high by 265
    b.at(265 + 13, |i| i.r.rw_n = H); // tWH exactly met
    assert_eq!(b.chip.peek(0x040), Some(0x99));
    b.no_warnings();
}

#[test]
fn write_ending_too_soon_after_busy_release_is_unknown() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x040));
    b.at(50, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x040).with_data(0x99));
    b.at(200, |i| i.r.rw_n = L);
    b.at(250, |i| i.l.ce_n = H); // BUSY high somewhere in [250, 265]
    b.at(270, |i| i.r.rw_n = H); // only 5 after guaranteed release: < tWH
    assert_eq!(b.chip.peek(0x040), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::WriteAfterBusyTooShort { width: ns(5) }]);

    // Ending inside the release window itself.
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x040));
    b.at(50, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x040).with_data(0x99));
    b.at(200, |i| i.r.rw_n = L);
    b.at(250, |i| i.l.ce_n = H);
    b.at(260, |i| i.r.rw_n = H);
    assert_eq!(b.chip.peek(0x040), None);
    assert_eq!(b.warning_kinds(), vec![WarningKind::WriteDuringBusyTransition]);
}

/// A write that ends while BUSY is still falling (within tBLA of the
/// contention) is undefined.
#[test]
fn write_ending_during_busy_assertion_is_unknown() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x040));
    b.at(50, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x040).with_data(0x99).with_rw(L));
    b.at(64, |i| i.r.rw_n = H);
    assert_eq!(b.chip.peek(0x040), None);
    assert!(b.warning_kinds().contains(&WarningKind::WriteDuringBusyTransition));
}

/// The winner's write is unaffected by the loser's presence, and the loser,
/// once released, reads the new value.
#[test]
fn winner_writes_loser_reads_after_release() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x040).with_data(0x5C));
    b.at(20, |i| i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x040));
    b.at(100, |i| i.l.rw_n = L);
    b.at(115, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x040), Some(0x5C));
    assert_eq!(b.r(150).busy_n, Level::L);
    assert_eq!(b.r(150).data, Bus::X);
    b.at(200, |i| i.l.ce_n = H);
    assert_eq!(b.r(230).data, Bus::V(0x5C));
    b.no_warnings();
}

// ---------------------------------------------------------------------------
// Interrupt timing diagrams / Truth Table II.

#[test]
fn left_sets_intr_right_clears_it() {
    let mut b = Bench::new();
    assert_eq!(b.r(0).int_n, Level::Z);
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x3FF).with_data(0x42));
    b.at(50, |i| i.l.rw_n = L); // write to 3FF begins
    assert_eq!(b.r(50).int_n, Level::X);
    assert_eq!(b.r(64).int_n, Level::X);
    assert_eq!(b.r(65).int_n, Level::L); // tWINS 15
    b.at(70, |i| i.l.rw_n = H);
    assert_eq!(b.chip.peek(0x3FF), Some(0x42));
    assert_eq!(b.r(200).int_n, Level::L);
    assert_eq!(b.l(200).int_n, Level::Z); // INTL untouched

    // While the left port is still parked on 3FF, the right port's read of
    // 3FF loses arbitration and (Truth Table II note 3) does NOT clear INTR.
    b.at(100, |i| i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x3FF));
    assert_eq!(b.r(115).busy_n, Level::L);
    assert_eq!(b.r(150).int_n, Level::L);
    b.at(150, |i| i.r.ce_n = H);
    b.at(180, |i| i.l.ce_n = H); // left leaves the mailbox

    // Right reads 3FF with OE low: INTR clears within tOINR.
    b.at(200, |i| i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x3FF));
    assert_eq!(b.r(200).int_n, Level::X);
    assert_eq!(b.r(214).int_n, Level::X);
    assert_eq!(b.r(215).int_n, Level::Z);
    assert_eq!(b.r(215).data, Bus::V(0x42)); // and the mailbox contents are readable
    b.no_warnings();
}

#[test]
fn right_sets_intl_left_clears_it() {
    let mut b = Bench::new();
    b.at(0, |i| i.r = PortInputs::idle().with_ce(L).with_addr(0x3FE).with_data(0x01));
    b.at(50, |i| i.r.rw_n = L);
    assert_eq!(b.l(65).int_n, Level::L);
    assert_eq!(b.r(65).int_n, Level::Z);
    b.at(70, |i| i.r.rw_n = H);
    b.at(80, |i| i.r.ce_n = H); // right leaves 3FE so the left port isn't BUSY
    // Left *writing* 3FE (OE high) does not clear it: clear needs CE=OE=L.
    b.at(100, |i| i.l = PortInputs::idle().with_ce(L).with_oe(H).with_addr(0x3FE).with_data(0x02));
    b.at(110, |i| i.l.rw_n = L);
    b.at(125, |i| i.l.rw_n = H);
    assert_eq!(b.l(200).int_n, Level::L);
    // Left reading 3FF (the other mailbox) doesn't clear INTL either.
    b.at(200, |i| i.l.addr = addr_levels(0x3FF));
    b.at(200, |i| i.l.oe_n = L);
    assert_eq!(b.l(300).int_n, Level::L);
    // Left reading 3FE clears -- once the right port has left that address
    // (otherwise the left port is BUSY and the flag is unchanged).
    b.at(250, |i| i.r.ce_n = H);
    b.at(300, |i| i.l.addr = addr_levels(0x3FE));
    assert_eq!(b.l(315).int_n, Level::Z);
    b.no_warnings();
}

/// Writing the mailbox from the busy (losing) port leaves INT unchanged.
#[test]
fn mailbox_write_while_busy_does_not_set_int() {
    let mut b = Bench::new();
    b.at(0, |i| i.r = PortInputs::idle().with_ce(L).with_oe(L).with_addr(0x3FF));
    b.at(50, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x3FF).with_data(0x42));
    assert_eq!(b.l(65).busy_n, Level::L);
    b.at(100, |i| i.l.rw_n = L);
    b.at(150, |i| i.l.rw_n = H);
    assert_eq!(b.r(200).int_n, Level::Z);
    assert_eq!(b.warning_kinds(), vec![WarningKind::WriteInhibited]);
}

/// A mailbox write with a broken pulse leaves the flag unknown.
#[test]
fn bad_mailbox_write_leaves_int_unknown() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_addr(0x3FF).with_data(0x42));
    b.at(50, |i| i.l.rw_n = L);
    b.at(55, |i| i.l.rw_n = H); // 5 < tPWE
    assert_eq!(b.r(100).int_n, Level::X);
}

// ---------------------------------------------------------------------------
// Unknown controls.

#[test]
fn unknown_control_on_enabled_port_gives_x_output() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(7));
    assert_eq!(b.l(15).data, Bus::V(preloaded(7)));
    b.at(20, |i| i.l.oe_n = Level::X);
    assert_eq!(b.l(20).data, Bus::X);
    b.no_warnings(); // not a violation, just unknown
    // Known again: an OE-controlled access, valid tDOE later.
    b.at(30, |i| i.l.oe_n = L);
    assert_eq!(b.l(39).data, Bus::X);
    assert_eq!(b.l(40).data, Bus::V(preloaded(7)));
}

/// An X on one port's CE cannot cause contention when the addresses are
/// known and different, or when the other port is known deselected.
#[test]
fn unknown_ce_with_different_addresses_is_not_contention() {
    let mut b = Bench::new();
    b.at(0, |i| {
        i.l = PortInputs::idle().with_ce(Level::X).with_oe(L).with_addr(1);
        i.r = PortInputs::idle().with_ce(L).with_addr(2).with_data(0x5A);
    });
    b.at(20, |i| i.r.rw_n = L);
    b.at(40, |i| i.r.rw_n = H);
    assert_eq!(b.chip.peek(2), Some(0x5A));
    assert_eq!(b.r(40).busy_n, Level::Z);
    b.no_warnings();
}

#[test]
fn undriven_inputs_at_power_up_are_not_a_read() {
    let chip = Cy7c131::new();
    let o = chip.outputs(0);
    assert_eq!(o.l.data, Bus::Z); // CE=Z on a never-driven port: treated as disabled until proven otherwise
    assert_eq!(o.l.busy_n, Level::Z);
}

// ---------------------------------------------------------------------------
// next_event

#[test]
fn next_event_reports_scheduled_transitions() {
    let mut b = Bench::new();
    b.at(0, |i| i.l = PortInputs::idle().with_ce(L).with_oe(L).with_addr(1));
    assert_eq!(b.chip.next_event(0), Some(ns(3)));
    assert_eq!(b.chip.next_event(ns(3)), Some(ns(15)));
    assert_eq!(b.chip.next_event(ns(15)), None);
}
