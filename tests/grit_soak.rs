//! grit soak: random programs on the netlist against the reference
//! interpreter (`GRIT_SOAK_PROGRAMS`, `GRIT_SOAK_SEED`), and the same
//! under the physical fuzz: a delay on every chip pin, CLK2X duty and
//! jitter, random power-up SRAM (`GRIT_FUZZ_PROGRAMS`, `GRIT_FUZZ_DELAY_NS`,
//! `GRIT_FUZZ_JITTER_NS`, `GRIT_FUZZ_DUTY`).
use mips32::grit::soak::program;
use mips32::grit::{assemble, fuzz_ram, Build, Grit, Iss};

fn env<T: std::str::FromStr>(name: &str, default: T) -> T {
    std::env::var(name).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

fn run_one(seed: u64, opt: Build) {
    let src = program(seed, 40);
    let p = assemble(&src).unwrap_or_else(|e| panic!("seed {seed}: {e}\n{src}"));
    let mut iss = Iss::new(&p.words);
    if opt.fuzz_seed != 0 {
        iss.ram = fuzz_ram(opt.fuzz_seed);
    }
    iss.run(200_000).unwrap_or_else(|e| panic!("seed {seed}: reference: {e}\n{src}"));
    let mut g = Grit::build(&p.words, &opt);
    let budget = iss.retired * 8 + 100;
    if !g.run_until_halt(budget) {
        let w = g.warnings();
        panic!("seed {seed}: netlist did not halt in {budget} clocks; ir {:?} step {:?}; {} warnings, first {:?}", g.ir(), g.step_count(), w.len(), w.iter().take(8).collect::<Vec<_>>());
    }
    let rw = g.reset_warnings();
    assert!(rw.is_empty(), "seed {seed}: during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = g.warnings();
    assert!(w.is_empty(), "seed {seed}: {} warnings, e.g. {:?}\n{src}", w.len(), w.iter().take(8).collect::<Vec<_>>());
    assert_eq!(g.a(), Some(iss.a), "seed {seed}: A\n{src}");
    assert_eq!(g.b(), Some(iss.b), "seed {seed}: B\n{src}");
    for r in 0..32 {
        assert_eq!(g.reg(r), Some(iss.reg(r)), "seed {seed}: r{r}\n{src}");
    }
    for i in 0..512 {
        assert_eq!(g.ram_word(0x8100 + 2 * i), Some(iss.ram[0x80 + i as usize]), "seed {seed}: data word {i}");
    }
    eprintln!("seed {seed}: {} instructions, {} clocks", iss.retired, g.clocks);
}

#[test]
fn random_programs() {
    let n: u64 = env("GRIT_SOAK_PROGRAMS", 20);
    let seed0: u64 = env("GRIT_SOAK_SEED", 1);
    for s in seed0..seed0 + n {
        run_one(s, Build::default());
    }
}

#[test]
fn random_programs_under_fuzz() {
    let n: u64 = env("GRIT_FUZZ_PROGRAMS", 6);
    let seed0: u64 = env("GRIT_FUZZ_SEED", 1);
    let delay: f64 = env("GRIT_FUZZ_DELAY_NS", 1.0);
    let jitter: f64 = env("GRIT_FUZZ_JITTER_NS", 1.0);
    let duty: String = env("GRIT_FUZZ_DUTY", "0.45,0.55".to_string());
    let mut it = duty.split(',').map(|x| x.trim().parse::<f64>().unwrap());
    let duty = (it.next().unwrap(), it.next().unwrap());
    for s in seed0..seed0 + n {
        let opt = Build { fuzz_seed: s, pin_delay_ns: delay, clock_duty: duty, clock_jitter_ns: jitter, reset_phase_ns: (s as f64 * 37.0) % 217.0, ..Build::default() };
        run_one(s, opt);
    }
}
