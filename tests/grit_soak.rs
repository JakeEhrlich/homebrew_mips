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
        assert_eq!(g.ram_word(0x8100 + i), Some(iss.ram[0x100 + i as usize]), "seed {seed}: data word {i}");
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
    // Clock skew on its own knob (`GRIT_FUZZ_SKEW_NS`); unset: the clock
    // pins get the same draw as every other pin.
    let skew: Option<f64> = std::env::var("GRIT_FUZZ_SKEW_NS").ok().and_then(|v| v.parse().ok());
    for s in seed0..seed0 + n {
        let opt = Build { fuzz_seed: s, pin_delay_ns: delay, clock_skew_ns: skew, clock_duty: duty, clock_jitter_ns: jitter, reset_phase_ns: (s as f64 * 37.0) % 217.0, ..Build::default() };
        run_one(s, opt);
    }
}

/// The one timing rule: clock skew between two GALs against the 2 ns
/// minimum clock-to-output.  PC-high's clock alone is delayed; a JMP
/// loads the PC from the flash the PC itself addresses, so PC-low's
/// address change reaches PC-high's data pins 2 ns plus wires after the
/// edge.  Under 2 ns of skew the program runs clean; over it, the model
/// reports the hold violation.  `GRIT_SKEW_NS` overrides the sweep.
#[test]
fn clock_skew_limit() {
    let src = program(3, 30);
    let p = assemble(&src).unwrap();
    let mut iss = Iss::new(&p.words);
    iss.run(200_000).unwrap();
    let sweep: Vec<f64> = match std::env::var("GRIT_SKEW_NS") {
        Ok(v) => vec![v.parse().unwrap()],
        Err(_) => vec![1.0, 1.8, 2.2, 3.0],
    };
    for skew in sweep {
        let mut g = Grit::build(&p.words, &Build::default());
        g.sim.set_pin_delays(|chip, pin| if chip == "pc1" && pin == 1 { Grit::ns(skew) } else { 0 });
        let halted = g.run_until_halt(iss.retired * 8 + 100);
        let w = g.warnings();
        let clean = halted && w.is_empty() && (0..32).all(|r| g.reg(r) == Some(iss.reg(r)));
        eprintln!("skew {skew} ns on pc1: {}{}", if clean { "clean" } else { "violation" }, w.first().map(|s| format!(" ({s})")).unwrap_or_default());
        if skew < 2.0 {
            assert!(clean, "skew {skew}: {:?}", w.iter().take(3).collect::<Vec<_>>());
        } else if std::env::var("GRIT_SKEW_NS").is_err() {
            assert!(!clean, "skew {skew}: expected a hold violation");
        }
    }
}
