//! Physical fuzz (docs/fuzz.md): the soak programs and the real programs
//! run with a propagation delay on every chip pin, a clock with a
//! random duty cycle and jitter, and random power-up contents in every
//! memory and register, all from a seed.  The reference simulator gets
//! the same contents.
//!
//! `FUZZ_PROGRAMS`, `FUZZ_SEED`, `FUZZ_DELAY_NS` (the maximum pin delay)
//! `FUZZ_DUTY` (the clock duty range, "0.49,0.51": a divide-by-two
//! flop's), `FUZZ_JITTER_NS` and `FUZZ_GRADE` (the delay lines' tolerance
//! grade, "room" = binned) in the environment scale it.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu, fuzz_image};
use mips32::ds1100::Grade;
use mips32::iss::Cpu as Iss;
use mips32::soak::program;

fn duty() -> (f64, f64) {
    let s = std::env::var("FUZZ_DUTY").unwrap_or_else(|_| "0.49,0.51".into());
    let mut it = s.split(',').map(|x| x.trim().parse::<f64>().unwrap());
    (it.next().unwrap(), it.next().unwrap())
}

fn fuzz_build(seed: u64, delay_ns: f64) -> Build {
    let jitter: f64 = std::env::var("FUZZ_JITTER_NS").ok().and_then(|v| v.parse().ok()).unwrap_or(0.3);
    // The delay-line tolerance grade: "room" (binned, +-2 ns) or
    // "commercial" (+-3 ns, the datasheet over 0..70 C).
    let grade = match std::env::var("FUZZ_GRADE").as_deref() {
        Ok("commercial") => Grade::Commercial,
        Ok("industrial") => Grade::Industrial,
        _ => Grade::Room,
    };
    Build { boot: Boot::Preload, fuzz_seed: seed, pin_delay_ns: delay_ns, clock_duty: duty(), clock_jitter_ns: jitter, grade, ..Build::default() }
}

/// Run `src` under the fuzz for `seed`; the failure message names what
/// disagreed.
fn run_fuzzed(src: &str, seed: u64, delay_ns: f64, what: &str) -> Cpu {
    let p = assemble(src, 0).unwrap_or_else(|e| panic!("{what}: assembler {e:?}"));
    let stop = p.labels["stop"];
    let image = fuzz_image(seed);
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.dmem = image.dmem.clone();
    iss.regs = image.regs;
    let retired = iss.run_until(stop, 200_000).unwrap_or_else(|e| panic!("{what}: reference fault {e:?}"));
    assert_eq!(iss.pc, stop, "{what}: reference did not reach stop");
    let mut cpu = Cpu::build(&p.words, 34.0, fuzz_build(seed, delay_ns));
    let budget = retired * 3 + 300;
    assert!(cpu.run_until_pc(stop, budget), "{what}: netlist did not reach stop in {budget} cycles; pc tail {:?}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(30)..]);
    let rw = cpu.reset_warnings();
    assert!(rw.is_empty(), "{what}: during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = cpu.warnings();
    assert!(w.is_empty(), "{what}: {} warnings, e.g. {:?}", w.len(), w.iter().take(10).collect::<Vec<_>>());
    for r in 0..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "{what}: register {r}");
    }
    for a in (0..1024).step_by(4) {
        assert_eq!(cpu.dmem_word(a), Some(iss.load_word(a)), "{what}: dmem[{a:#x}]");
    }
    eprintln!("{what}: {retired} retired, {} cycles", cpu.cycles);
    cpu
}

#[test]
fn random_programs_with_delays_jitter_and_random_contents() {
    let programs: u64 = std::env::var("FUZZ_PROGRAMS").ok().and_then(|v| v.parse().ok()).unwrap_or(4);
    let seed0: u64 = std::env::var("FUZZ_SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(1);
    let delay: f64 = std::env::var("FUZZ_DELAY_NS").ok().and_then(|v| v.parse().ok()).unwrap_or(0.3);
    for i in 0..programs {
        let seed = seed0 + i;
        let src = program(seed, 160);
        run_fuzzed(&src, seed, delay, &format!("seed {seed} at {delay} ns"));
    }
}
