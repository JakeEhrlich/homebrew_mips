//! Memory strobes from a real delay line instead of ideal stimulus.
use mips32::asm::assemble;
use mips32::cpu::{Cpu, StrobeTaps, Strobes};
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

/// Run the load/store program with the given strobes; return the warnings
/// (empty = clean) and whether the results match the reference.
fn run(period_ns: f64, strobes: Strobes) -> (Vec<String>, bool) {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 10_000).unwrap();
    let mut cpu = Cpu::with_strobes(&p.words, period_ns, strobes);
    let stopped = cpu.run_until_pc(stop, 400);
    let mut ok = stopped;
    for r in 1..32 {
        ok &= cpu.reg(r) == Some(iss.regs[r as usize]);
    }
    for a in (0..64).step_by(4) {
        ok &= cpu.dmem_word(a) == Some(iss.load_word(a));
    }
    (cpu.warnings(), ok)
}

#[test]
fn ideal_strobes_are_clean_at_34() {
    let (w, ok) = run(34.0, Strobes::Ideal);
    assert!(w.is_empty(), "{}", w.join("\n"));
    assert!(ok);
}

/// Sweep: where does the naive tap + GAL generator become clean?  (It never
/// does: a strobe end shaped by a GAL from a tap edge either lands after
/// the next clock edge or leaves too short a write pulse.)  Run with
/// `--ignored --nocapture` to see the numbers.
#[test]
#[ignore]
fn delay_line_strobes_sweep() {
    for grade in [Grade::Room, Grade::Commercial] {
        let mut first_clean = None;
        for period in [34.0, 36.0, 38.0, 40.0, 42.0, 44.0, 46.0, 48.0, 50.0, 55.0, 60.0] {
            let (w, ok) = run(period, Strobes::DelayLine { total: 25, grade, taps: StrobeTaps::nearest(25) });
            let mut kinds: Vec<String> = w.iter().map(|s| s.split(" at ").next().unwrap_or(s).to_string()).collect();
            kinds.sort();
            kinds.dedup();
            eprintln!("{grade:?} {period} ns: {} warnings, results {}: {:?}", w.len(), if ok { "ok" } else { "WRONG" }, kinds);
            if w.is_empty() && ok && first_clean.is_none() {
                first_clean = Some(period);
            }
        }
        eprintln!("{grade:?}: first clean period {:?}", first_clean);
    }
}

/// Which chips complain, for a strobe assignment.
fn chips_warning(w: &[String]) -> Vec<String> {
    let mut v: Vec<String> = w.iter().map(|s| s.split(':').next().unwrap_or("").trim_end_matches(|c: char| c.is_ascii_digit()).to_string()).collect();
    v.sort();
    v.dedup();
    v
}

/// Search tap assignments per strobe.  Run with `--ignored --nocapture`;
/// env STROBE_TOTAL (delay-line part), STROBE_PERIOD, STROBE_GRADE (room|comm).
#[test]
#[ignore]
fn delay_line_strobe_search() {
    let total: u32 = std::env::var("STROBE_TOTAL").ok().and_then(|s| s.parse().ok()).unwrap_or(35);
    let period: f64 = std::env::var("STROBE_PERIOD").ok().and_then(|s| s.parse().ok()).unwrap_or(34.0);
    let grade = if std::env::var("STROBE_GRADE").as_deref() == Ok("comm") { Grade::Commercial } else { Grade::Room };
    let step = total as f64 / 5.0;
    let ns = |k: u8| if k == 0 { 0.0 } else { k as f64 * step };
    let base = StrobeTaps::nearest(total);
    eprintln!("part DS1100-{total} taps {:?} grade {grade:?} period {period}", (1..=5).map(|k| ns(k)).collect::<Vec<_>>());
    // Each strobe on its own: the other two stay at their nearest choice
    // and we look only at the chips that strobe feeds.
    let cands: Vec<(u8, u8)> = (0..=5u8).flat_map(|f| (0..=5u8).map(move |r| (f, r))).collect();
    for which in ["ce", "sd", "rf"] {
        let mut clean = Vec::new();
        for &c in &cands {
            let (f, r) = c;
            // Window [f, half + r] must be a real, positive window.
            if ns(f) >= period / 2.0 + ns(r) {
                continue;
            }
            let mut taps = base;
            match which {
                "ce" => taps.ce = c,
                "sd" => taps.sd = c,
                _ => taps.rf = c,
            }
            let (w, ok) = run(period, Strobes::DelayLine { total, grade, taps });
            let chips = chips_warning(&w);
            let relevant: Vec<&String> = chips.iter().filter(|n| if which == "rf" { n.starts_with("rf") } else { n.starts_with("dmem") || n.starts_with("msd") }).collect();
            eprintln!("  {which} {:?} -> window [{}, {}]: {} warnings, chips {:?}, results {}", c, ns(f), period / 2.0 + ns(r), w.len(), chips, if ok { "ok" } else { "wrong" });
            if relevant.is_empty() {
                clean.push(c);
            }
        }
        eprintln!("{which}: clean choices {:?}", clean);
    }
}
