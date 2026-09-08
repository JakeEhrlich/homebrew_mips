//! Soak test: random programs over the whole instruction subset, dense
//! with hazards, run on the netlist against the reference simulator.
//!
//! Each program: random register values; straight-line ALU, shift, load
//! and store instructions at random; forward branches of every kind with
//! delay slots; bounded loops; subroutines called with JAL and JALR and
//! returned from with JR; link branches.  What it avoids is what MIPS I
//! leaves undefined: a branch in a delay slot, a store of the loaded
//! register in a load's delay slot (a label in between does not end the
//! slot), an unaligned narrow access, and an access outside the data
//! region.
//!
//! `SOAK_PROGRAMS` and `SOAK_SEED` in the environment change the count
//! (default 6) and the seed.
use mips32::asm::assemble;
use mips32::cpu::Cpu;
use mips32::iss::Cpu as Iss;
use mips32::soak::program;

fn run_one(seed: u64, len: usize) {
    let src = program(seed, len);
    let p = match assemble(&src, 0) {
        Ok(p) => p,
        Err(e) => panic!("seed {seed}: assembler: {e:?}\n{src}"),
    };
    assert!(p.words.len() < 8192, "seed {seed}: program too long ({} words)", p.words.len());
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    let retired = iss.run_until(stop, 200_000).unwrap_or_else(|e| panic!("seed {seed}: reference fault {e:?}\n{src}"));
    assert_eq!(iss.pc, stop, "seed {seed}: reference did not reach stop\n{src}");
    let mut cpu = Cpu::new(&p.words, 34.0);
    let budget = retired * 3 + 300;
    assert!(cpu.run_until_pc(stop, budget), "seed {seed}: netlist did not reach stop in {budget} cycles; pc tail {:?}\n{src}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(30)..]);
    let rw = cpu.reset_warnings();
    assert!(rw.is_empty(), "seed {seed}: during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = cpu.warnings();
    assert!(w.is_empty(), "seed {seed}: {} warnings, e.g. {:?}\n{src}", w.len(), w.iter().take(10).collect::<Vec<_>>());
    for r in 0..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "seed {seed}: register {r}\n{src}");
    }
    for a in (0..1024).step_by(4) {
        assert_eq!(cpu.dmem_word(a), Some(iss.load_word(a)), "seed {seed}: dmem[{a:#x}]\n{src}");
    }
    eprintln!("seed {seed}: {} instructions, {} retired, {} cycles", p.words.len(), retired, cpu.cycles);
}

#[test]
fn random_programs_against_reference() {
    let programs: u64 = std::env::var("SOAK_PROGRAMS").ok().and_then(|v| v.parse().ok()).unwrap_or(6);
    let seed0: u64 = std::env::var("SOAK_SEED").ok().and_then(|v| v.parse().ok()).unwrap_or(1);
    for i in 0..programs {
        run_one(seed0 + i, 160);
    }
}
