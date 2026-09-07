//! The memory write timing (docs/memory-timing.md).  Register file: write
//! port enabled by the clock, address / enable from tap-clocked copies.
//! Data memory: single-port, write pulse from a delay-line tap gated by the
//! store flag through one fast gate, in the store's own MEM cycle.  These
//! tests sweep the clock period, the delay-line grade, the data SRAM speed
//! grade and the gate tap.
use mips32::as7c164a::Timing;
use mips32::asm::assemble;
use mips32::cpu::{Build, Cpu, DELAY_LINE_TOTAL};
use mips32::ds1100::Grade;
use mips32::iss::Cpu as Iss;

const PROG: &str = "
        li    $t0, 0x1234
        li    $t1, 16
        sw    $t0, 0($t1)
        sw    $t1, 4($t1)
        lw    $t2, 0($t1)
        nop
        addu  $t3, $t2, $t2
        lw    $t4, 4($t1)
        lw    $t5, 0($t1)
        addu  $t6, $t4, $t5
        sw    $t6, 8($t1)
        lw    $t7, 8($t1)
        nop
        sw    $t7, 12($t1)
        sw    $t7, 16($t1)
        lw    $s1, 16($t1)
        sw    $t2, 20($t1)
        lw    $s2, 20($t1)      # load right behind a store, same address
        sw    $t3, 24($t1)      # store right behind a load (not its register:
                                # a load's delay slot must not read it)
        lw    $s3, 4($t1)       # load right behind a store, other address
        sw    $t0, 28($t1)      # store right behind a load
        sw    $s2, 32($t1)      # store behind a store
        lw    $s4, 28($t1)
        lw    $s5, 32($t1)      # load behind a load
        li    $s0, 99
        lw    $s0, 0($t1)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ";

/// Warnings after reset and whether registers / memory match the reference.
fn run(period_ns: f64, grade: Grade) -> (Vec<String>, Vec<String>, bool) {
    run_with(period_ns, Build { grade, ..Build::default() })
}

fn run_with(period_ns: f64, opt: Build) -> (Vec<String>, Vec<String>, bool) {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 10_000).unwrap();
    let mut cpu = Cpu::build(&p.words, period_ns, opt);
    let mut ok = cpu.run_until_pc(stop, 400);
    for r in 1..32 {
        ok &= cpu.reg(r) == Some(iss.regs[r as usize]);
    }
    for a in (0..64).step_by(4) {
        ok &= cpu.dmem_word(a) == Some(iss.load_word(a));
    }
    ok &= cpu.reg(0) == Some(0);
    (cpu.warnings(), cpu.reset_warnings(), ok)
}

fn kinds(w: &[String]) -> Vec<String> {
    let mut v: Vec<String> = w.iter().map(|s| s.split(" at ").next().unwrap_or(s).split("ns ").last().unwrap_or(s).to_string()).collect();
    v.sort();
    v.dedup();
    v
}

/// Which (part, tap, period) combinations are clean; prints a table.
#[test]
fn part_tap_period_sweep() {
    let parts = [("CY7C1041G-10", Timing::cy7c1041g_10()), ("IS61C256AH-12", Timing::is61c256ah_12()), ("AS7C164A-15", Timing::grade_15())];
    let taps: [(u32, usize); 4] = [(DELAY_LINE_TOTAL, 0), (DELAY_LINE_TOTAL, 1), (40, 0), (50, 0)];
    for (name, dmem) in parts {
        for tap in taps {
            let mut first = None;
            let mut line = String::new();
            for period in [33.0, 34.0, 35.0, 36.0, 37.0, 38.0, 39.0, 40.0] {
                let opt = Build { grade: Grade::Commercial, dmem, gate_tap: tap, ..Build::default() };
                let (w, rw, ok) = run_with(period, opt);
                let clean = w.is_empty() && rw.is_empty() && ok;
                if clean && first.is_none() {
                    first = Some(period);
                }
                line += &format!(" {period}:{}", if clean { "ok" } else { "--" });
            }
            eprintln!("{name:14} tap {tap:?}: first clean {first:?} |{line}");
        }
    }
}

/// The documented operating points must be clean at room and commercial
/// (0..70 C) delay-line grades; industrial is reported only.
#[test]
fn documented_operating_points() {
    for (name, dmem, period) in [("CY7C1041G-10", Timing::cy7c1041g_10(), 34.0), ("AS7C164A-15", Timing::grade_15(), 38.0)] {
        let (w, rw, ok) = run_with(period, Build { grade: Grade::Industrial, dmem, ..Build::default() });
        eprintln!("{name} industrial grade at {period} ns: {}", if w.is_empty() && rw.is_empty() && ok { "clean" } else { "not clean" });
    }
    for grade in [Grade::Room, Grade::Commercial] {
        for (name, dmem, period) in [("CY7C1041G-10", Timing::cy7c1041g_10(), 34.0), ("AS7C164A-15", Timing::grade_15(), 38.0)] {
            let opt = Build { grade, dmem, ..Build::default() };
            let (w, rw, ok) = run_with(period, opt);
            assert!(rw.is_empty(), "{name} {grade:?} {period}: reset warnings {rw:?}");
            assert!(w.is_empty(), "{name} {grade:?} {period}: {} warnings, e.g. {:?}", w.len(), kinds(&w).iter().take(6).collect::<Vec<_>>());
            assert!(ok, "{name} {grade:?} {period}: wrong results");
        }
    }
}

/// Sweep for the record: `--nocapture` prints clean / not per period.
#[test]
fn period_sweep() {
    for grade in [Grade::Room, Grade::Commercial, Grade::Industrial] {
        for period in [34.0, 36.0, 38.0, 40.0, 50.0] {
            let (w, rw, ok) = run(period, grade);
            eprintln!("{grade:?} {period} ns: {} warnings {}, {} reset warnings, results {}", w.len(), if w.is_empty() { "CLEAN" } else { "" }, rw.len(), if ok { "ok" } else { "WRONG" });
            if !w.is_empty() {
                eprintln!("    {:?}", kinds(&w).iter().take(5).collect::<Vec<_>>());
            }
        }
    }
}
