//! The memory write timing: both SRAMs are dual-port; their write ports are
//! enabled by the clock itself and take their address / enable from
//! tap-clocked copies.  These tests sweep the clock period and the
//! delay-line grade; see docs/memory-timing.md.
use mips32::asm::assemble;
use mips32::cpu::Cpu;
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
        sw    $s2, 24($t1)
        lw    $s3, 4($t1)       # load right behind a store, other address
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
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 10_000).unwrap();
    let mut cpu = Cpu::with_grade(&p.words, period_ns, grade);
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

#[test]
fn clean_at_34_every_grade() {
    for grade in [Grade::Room, Grade::Commercial, Grade::Industrial] {
        let (w, rw, ok) = run(34.0, grade);
        assert!(rw.is_empty(), "{grade:?}: reset warnings {rw:?}");
        assert!(w.is_empty(), "{grade:?}: {} warnings, e.g. {:?}", w.len(), kinds(&w).iter().take(6).collect::<Vec<_>>());
        assert!(ok, "{grade:?}: wrong results");
    }
}

/// Sweep for the record: `--nocapture` prints clean / not per period.
#[test]
fn period_sweep() {
    for grade in [Grade::Room, Grade::Commercial, Grade::Industrial] {
        for period in [32.0, 33.0, 34.0, 36.0, 40.0, 50.0] {
            let (w, rw, ok) = run(period, grade);
            eprintln!("{grade:?} {period} ns: {} warnings {}, {} reset warnings, results {}", w.len(), if w.is_empty() { "CLEAN" } else { "" }, rw.len(), if ok { "ok" } else { "WRONG" });
            if !w.is_empty() {
                eprintln!("    {:?}", kinds(&w).iter().take(5).collect::<Vec<_>>());
            }
        }
    }
}
