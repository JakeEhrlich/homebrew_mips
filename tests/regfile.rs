//! Register-file tests: eight CY7C131 models driven with a two-phase cycle.

use mips32::cy7c131::{Level, NS, Time};
use mips32::regfile::*;

/// Where within a cycle the surrounding logic moves each signal (ns after the
/// clock edge).  Addresses, write data and R/W_W come from pipeline registers
/// and become valid at `t_valid`; CE_W and CE_R are phase signals.
#[derive(Clone, Copy, Debug)]
struct Schedule {
    period: u64,
    t_valid: u64,
    w_start: u64,
    w_end: u64,
    r_start: u64,
    /// Setup time of the register that captures the read data at the next edge.
    setup: u64,
    /// If false, CE_R is simply left low forever (the broken design).
    toggle_read_ce: bool,
}

/// Comfortable 20 MHz schedule.
const RELAXED: Schedule =
    Schedule { period: 50, t_valid: 6, w_start: 6, w_end: 20, r_start: 23, setup: 5, toggle_read_ce: true };

/// Everything at its datasheet minimum: 25 MHz.
const TIGHT: Schedule =
    Schedule { period: 40, t_valid: 6, w_start: 6, w_end: 18, r_start: 20, setup: 5, toggle_read_ce: true };

#[derive(Clone, Copy, Debug)]
struct Cycle {
    rs: u8,
    rt: u8,
    rd: u8,
    wdata: u32,
    we: bool,
}

type Event = (u64, fn(&mut RegFileInputs));

struct Sampled {
    rs: Option<u32>,
    rt: Option<u32>,
}

/// Drive `cycles` through the register file, returning what the next-stage
/// register would capture at the end of each cycle, plus whether BUSY was ever
/// seen.
fn run(rf: &mut RegFile, s: Schedule, cycles: &[Cycle]) -> (Vec<Sampled>, bool) {
    let ns = |n: u64| -> Time { n * NS };
    let mut inp = RegFileInputs::idle();
    if !s.toggle_read_ce {
        inp.ce_r_n = Level::L;
    }
    rf.set_inputs(0, inp);
    let mut out = Vec::new();
    let mut busy_seen = false;
    for (n, c) in cycles.iter().enumerate() {
        let base = n as u64 * s.period;
        // Pipeline registers update; read ports disabled for the write phase.
        inp.rs = reg_levels(c.rs);
        inp.rt = reg_levels(c.rt);
        inp.rd = reg_levels(c.rd);
        inp.wdata = word_levels(c.wdata);
        // Write-enable logic: never write r0.
        inp.rw_w_n = if c.we && c.rd != 0 { Level::L } else { Level::H };
        if s.toggle_read_ce {
            inp.ce_r_n = Level::H;
        }
        // Events within the cycle, applied in time order (phases may overlap
        // in the deliberately broken schedules).
        let mut events: Vec<Event> = vec![
            (s.w_start, |i| i.ce_w_n = Level::L),
            (s.w_end, |i| i.ce_w_n = Level::H),
        ];
        if s.toggle_read_ce {
            events.push((s.r_start, |i| i.ce_r_n = Level::L));
        }
        events.sort_by_key(|e| e.0);
        rf.set_inputs(ns(base + s.t_valid), inp);
        for &(t, f) in &events {
            f(&mut inp);
            rf.set_inputs(ns(base + t), inp);
        }
        let last = events.iter().map(|e| e.0).max().unwrap().max(s.t_valid);
        for t in (base + last)..(base + s.period) {
            let o = rf.outputs(ns(t));
            busy_seen |= o.read_busy || o.write_busy;
        }
        let o = rf.outputs(ns(base + s.period - s.setup));
        out.push(Sampled { rs: o.rs_word(), rt: o.rt_word() });
    }
    (out, busy_seen)
}

/// Software reference with write-first semantics.
fn reference(cycles: &[Cycle]) -> Vec<(u32, u32)> {
    let mut regs = [0u32; 32];
    cycles
        .iter()
        .map(|c| {
            if c.we && c.rd != 0 {
                regs[c.rd as usize] = c.wdata;
            }
            (regs[c.rs as usize], regs[c.rt as usize])
        })
        .collect()
}

fn no_warnings(rf: &RegFile) {
    let w = rf.warnings();
    assert!(w.is_empty(), "chip warnings: {:#?}", w);
}

#[test]
fn write_then_read_next_cycle() {
    let mut rf = RegFile::new();
    let cycles = [
        Cycle { rs: 0, rt: 0, rd: 5, wdata: 0xDEADBEEF, we: true },
        Cycle { rs: 5, rt: 0, rd: 0, wdata: 0, we: false },
    ];
    let (out, busy) = run(&mut rf, RELAXED, &cycles);
    assert_eq!(out[1].rs, Some(0xDEADBEEF));
    assert_eq!(out[1].rt, Some(0));
    assert_eq!(rf.peek(5), Some(0xDEADBEEF));
    assert!(!busy);
    no_warnings(&rf);
}

/// The ID/WB overlap: reading the register being written in the same cycle
/// returns the new value, on both read ports, with no arbitration.
#[test]
fn same_cycle_read_after_write_sees_new_value() {
    let mut rf = RegFile::new();
    rf.preload(7, 0x11111111);
    let cycles = [Cycle { rs: 7, rt: 7, rd: 7, wdata: 0x22222222, we: true }];
    let (out, busy) = run(&mut rf, RELAXED, &cycles);
    assert_eq!(out[0].rs, Some(0x22222222));
    assert_eq!(out[0].rt, Some(0x22222222));
    assert!(!busy);
    no_warnings(&rf);
}

#[test]
fn register_zero_is_never_written() {
    let mut rf = RegFile::new();
    let cycles = [
        Cycle { rs: 0, rt: 0, rd: 0, wdata: 0xFFFFFFFF, we: true },
        Cycle { rs: 0, rt: 0, rd: 0, wdata: 0, we: false },
    ];
    let (out, _) = run(&mut rf, RELAXED, &cycles);
    assert_eq!(out[0].rs, Some(0));
    assert_eq!(out[1].rs, Some(0));
    assert_eq!(rf.peek(0), Some(0));
    no_warnings(&rf);
}

fn random_program(seed: u64, n: usize) -> Vec<Cycle> {
    // Small xorshift so the test needs no dependencies.
    let mut x = seed | 1;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    (0..n)
        .map(|_| {
            let r = next();
            Cycle {
                rs: (r & 31) as u8,
                rt: (r >> 5 & 31) as u8,
                rd: (r >> 10 & 31) as u8,
                wdata: (next() & 0xFFFF_FFFF) as u32,
                we: r >> 15 & 3 != 0, // 75% of cycles write
            }
        })
        .collect()
}

fn check_program(s: Schedule) {
    let cycles = random_program(0x9E3779B97F4A7C15, 2000);
    let mut rf = RegFile::new();
    let (out, busy) = run(&mut rf, s, &cycles);
    let want = reference(&cycles);
    for (i, (got, want)) in out.iter().zip(&want).enumerate() {
        assert_eq!((got.rs, got.rt), (Some(want.0), Some(want.1)), "cycle {i}: {:?}", cycles[i]);
    }
    assert!(!busy, "BUSY asserted at some point");
    no_warnings(&rf);
}

#[test]
fn random_program_relaxed_schedule() {
    check_program(RELAXED);
}

#[test]
fn random_program_tight_schedule() {
    check_program(TIGHT);
}

/// One nanosecond faster than TIGHT anywhere and the model says no.
#[test]
fn tighter_than_datasheet_fails() {
    // Write pulse 11ns.
    let s = Schedule { w_end: 17, ..TIGHT };
    let cycles = [Cycle { rs: 0, rt: 0, rd: 3, wdata: 1, we: true }];
    let mut rf = RegFile::new();
    run(&mut rf, s, &cycles);
    assert_eq!(rf.peek(3), None);
    assert!(!rf.warnings().is_empty());

    // Read enabled 1ns too late for the capture setup.
    let s = Schedule { r_start: 21, ..TIGHT };
    let cycles = [Cycle { rs: 3, rt: 3, rd: 3, wdata: 1, we: true }];
    let mut rf = RegFile::new();
    let (out, _) = run(&mut rf, s, &cycles);
    assert_eq!(out[0].rs, None);
    no_warnings(&rf); // nothing illegal happened, the data just isn't there yet
}

/// Enabling the read ports before the write port has been released: the read
/// port loses arbitration whenever rs == rd, and its data is not there in time.
#[test]
fn overlapping_phases_make_reads_busy() {
    let s = Schedule { r_start: 18, ..RELAXED }; // 2ns before w_end = 20
    let cycles = [
        Cycle { rs: 9, rt: 4, rd: 9, wdata: 0xABCD0123, we: true },
        Cycle { rs: 9, rt: 9, rd: 1, wdata: 0, we: false },
    ];
    let mut rf = RegFile::new();
    let (out, busy) = run(&mut rf, s, &cycles);
    assert!(busy);
    assert_eq!(out[0].rs, None); // rs lost arbitration this cycle
    assert_eq!(out[0].rt, Some(0)); // rt was on another register: fine
    assert_eq!(rf.peek(9), Some(0xABCD0123)); // the write itself won and landed
    assert_eq!(out[1].rs, Some(0xABCD0123));
}

/// Leaving the read ports permanently enabled (no phases at all): now the read
/// port is always first, so writes to a register being read are *inhibited*.
#[test]
fn always_enabled_read_ports_inhibit_writes() {
    // Write pulse long enough (> tBLA) that the inhibit is definite rather than
    // "undefined because BUSY was still falling".
    let s = Schedule { toggle_read_ce: false, w_end: 25, ..RELAXED };
    let cycles = [
        Cycle { rs: 9, rt: 4, rd: 0, wdata: 0, we: false }, // rs parks on r9 first
        Cycle { rs: 9, rt: 4, rd: 9, wdata: 0xABCD0123, we: true },
    ];
    let mut rf = RegFile::new();
    let (_, busy) = run(&mut rf, s, &cycles);
    assert!(busy);
    // Bank A (rs = 9) inhibited the write; bank B (rt = 4) accepted it: the
    // two banks now disagree, which peek() reports as unknown.
    assert_eq!(rf.peek(9), None);
    assert_eq!(rf.chips[0].peek(9), Some(0));
    assert_eq!(rf.chips[4].peek(9), Some(0x23));
    assert!(rf.warnings().iter().any(|(_, w)| w.kind == mips32::cy7c131::WarningKind::WriteInhibited));
}

// ---------------------------------------------------------------------------
// Full overlap: reads and the write run in the same phase.  A read of the
// register being written this cycle is a forwarded value, so the SRAM read
// is a don't-care; the only requirement is that the write is never inhibited.

/// Variant A: decode *steers the read port off* (CE_R high for that bank)
/// whenever the source is the register being written, so the chip never sees
/// an address match.  The steer decision needs one GAL delay after the
/// addresses, so CE_R lands at `r_start`.
///
/// Variant B: the read ports are always enabled and the write port's address
/// and CE land half a cycle *before* the read addresses (MEM/WB register on
/// the inverted clock), so on a match the write port is always the winner and
/// the read port just gets BUSY'd, which nobody looks at.
#[derive(Clone, Copy, Debug)]
enum Overlap {
    Steer { r_start: u64 },
    Priority,
}

fn run_overlap(rf: &mut RegFile, period: u64, mode: Overlap, cycles: &[Cycle]) -> (Vec<Sampled>, bool) {
    let ns = |n: u64| -> Time { n * NS };
    let t_valid = 6; // pipeline register clock-to-out
    let setup = 5;
    let mut inp = RegFileInputs::idle();
    if let Overlap::Priority = mode {
        inp.ce_r_n = Level::L;
    }
    rf.set_inputs(0, inp);
    let mut out = Vec::new();
    let mut busy_seen = false;
    // Cycle n occupies [base, base + period); cycle 0 is left idle so the
    // Priority variant has a "previous half cycle" to put its write in.
    for (k, c) in cycles.iter().enumerate() {
        let base = (k as u64 + 1) * period;
        let we = c.we && c.rd != 0;
        match mode {
            Overlap::Steer { r_start } => {
                // Addresses, data, CE_W land at t_valid with the read ports
                // disabled; the (per-bank) steer decision re-enables them at
                // r_start, except for a bank whose source is this cycle's rd.
                inp.rs = reg_levels(c.rs);
                inp.rt = reg_levels(c.rt);
                inp.rd = reg_levels(c.rd);
                inp.wdata = word_levels(c.wdata);
                inp.rw_w_n = if we { Level::L } else { Level::H };
                inp.ce_w_n = if we { Level::L } else { Level::H }; // CE_W gated by write enable
                inp.ce_r_n = Level::H;
                rf.set_inputs(ns(base + t_valid), inp);
                let ce_r = |steer: bool| if steer { Level::H } else { Level::L };
                let ce_a = ce_r(we && c.rs == c.rd);
                let ce_b = ce_r(we && c.rt == c.rd);
                for (t, ce_w) in [(base + r_start, inp.ce_w_n), (base + t_valid + 14, Level::H)] {
                    inp.ce_w_n = ce_w;
                    for lane in 0..4 {
                        set_bank(rf, 0, lane, ns(t), RegFileInputs { ce_r_n: ce_a, ..inp });
                        set_bank(rf, 1, lane, ns(t), RegFileInputs { ce_r_n: ce_b, ..inp });
                    }
                }
            }
            Overlap::Priority => {
                // Read ports strobed off from (base - 4) to (base + t_valid);
                // the write side lands at (base - 2), inside that window, so it
                // is stable 8ns before the read keys and always wins.
                inp.ce_r_n = Level::H;
                rf.set_inputs(ns(base - 4), inp);
                inp.rd = reg_levels(c.rd);
                inp.wdata = word_levels(c.wdata);
                inp.rw_w_n = if we { Level::L } else { Level::H };
                inp.ce_w_n = if we { Level::L } else { Level::H }; // CE_W gated by write enable
                rf.set_inputs(ns(base - 2), inp);
                inp.rs = reg_levels(c.rs);
                inp.rt = reg_levels(c.rt);
                inp.ce_r_n = Level::L;
                rf.set_inputs(ns(base + t_valid), inp);
                inp.ce_w_n = Level::H;
                rf.set_inputs(ns(base + 10), inp); // 12ns CE pulse = tSCE
            }
        }
        let last = match mode {
            Overlap::Steer { r_start } => (base + r_start).max(base + t_valid + 14),
            Overlap::Priority => base + 10,
        };
        for t in last..(base + period) {
            let o = rf.outputs(ns(t));
            busy_seen |= o.write_busy; // read-port BUSY is expected in Priority mode and ignored
            if let Overlap::Steer { .. } = mode {
                busy_seen |= o.read_busy;
            }
        }
        let o = rf.outputs(ns(base + period - setup));
        // Forwarded operands come from the WB register, not the SRAM.
        let fwd = |r: u8, sram: Option<u32>| if we && r == c.rd { Some(c.wdata) } else { sram };
        out.push(Sampled { rs: fwd(c.rs, o.rs_word()), rt: fwd(c.rt, o.rt_word()) });
    }
    (out, busy_seen)
}

fn set_bank(rf: &mut RegFile, bank: usize, lane: usize, t: Time, i: RegFileInputs) {
    use mips32::cy7c131::{Inputs, PortInputs};
    let raddr = if bank == 0 { i.rs } else { i.rt };
    let pad = |r: [Level; 5]| -> [Level; mips32::cy7c131::ADDR_BITS] { std::array::from_fn(|k| if k < 5 { r[k] } else { Level::L }) };
    let read = PortInputs { addr: pad(raddr), ce_n: i.ce_r_n, rw_n: Level::H, oe_n: Level::L, data: [Level::Z; 8] };
    let write = PortInputs {
        addr: pad(i.rd),
        ce_n: i.ce_w_n,
        rw_n: i.rw_w_n,
        oe_n: Level::H,
        data: std::array::from_fn(|b| i.wdata[8 * lane + b]),
    };
    rf.chips[bank * 4 + lane].set_inputs(t, Inputs { l: read, r: write });
}

fn check_overlap(period: u64, mode: Overlap) {
    let cycles = random_program(0x1234_5678_9ABC_DEF1, 2000);
    let mut rf = RegFile::new();
    let (out, busy) = run_overlap(&mut rf, period, mode, &cycles);
    let want = reference(&cycles);
    for (i, (got, want)) in out.iter().zip(&want).enumerate() {
        assert_eq!((got.rs, got.rt), (Some(want.0), Some(want.1)), "cycle {i}: {:?}", cycles[i]);
    }
    assert!(!busy, "unexpected BUSY");
    no_warnings(&rf);
}

/// Steering: addresses at 6, compare GAL 7.5 -> CE_R at 14, data at 29,
/// capture at 34.
#[test]
fn overlap_steer_34ns() {
    check_overlap(34, Overlap::Steer { r_start: 14 });
}

#[test]
fn overlap_steer_33ns_misses_capture() {
    let cycles = [Cycle { rs: 3, rt: 3, rd: 4, wdata: 1, we: true }];
    let mut rf = RegFile::new();
    let (out, _) = run_overlap(&mut rf, 33, Overlap::Steer { r_start: 14 }, &cycles);
    assert_eq!(out[0].rs, None);
}

/// Priority: read addresses at 6, data at 21, capture at 26.  The write side
/// landed 8ns before the read keys (inside the read-CE strobe), so every
/// address match is lost by the read port and the write always lands.
#[test]
fn overlap_priority_26ns() {
    check_overlap(26, Overlap::Priority);
}

/// Same thing with the write side *not* early: both ports land together, the
/// arbitration is ambiguous, and the model refuses to say the write happened.
#[test]
fn overlap_without_priority_is_ambiguous() {
    let mut rf = RegFile::new();
    let mut inp = RegFileInputs::idle();
    inp.ce_r_n = Level::L;
    rf.set_inputs(0, inp);
    inp.rs = reg_levels(3);
    inp.rd = reg_levels(3);
    inp.wdata = word_levels(0x77);
    inp.rw_w_n = Level::L;
    inp.ce_w_n = Level::L;
    rf.set_inputs(6 * NS, inp);
    inp.ce_w_n = Level::H;
    rf.set_inputs(30 * NS, inp);
    assert_eq!(rf.peek(3), None);
    assert!(rf.warnings().iter().any(|(_, w)| w.kind == mips32::cy7c131::WarningKind::ArbitrationAmbiguous));
}
