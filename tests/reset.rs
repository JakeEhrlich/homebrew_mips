//! Power-on reset: the supervisor releases at an arbitrary point of the
//! clock cycle; the two-stage synchroniser must turn that into a clean,
//! edge-aligned release for every phase, and the machine must come up
//! with r0 = 0 and run correctly afterwards.
use mips32::asm::assemble;
use mips32::cpu::{Build, Cpu};
use mips32::iss::Cpu as Iss;
use mips32::netlist::Level;

const PROG: &str = "
        li    $t0, 5
        li    $t1, 7
        addu  $t2, $t0, $t1
        subu  $t3, $t0, $t1
        move  $t4, $zero
        sw    $t2, 0($zero)
        lw    $t5, 0($zero)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ";

#[test]
fn release_phase_sweep() {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 1000).unwrap();
    let period = 34.0;
    let mut phase = 0.0;
    while phase < period {
        let opt = Build { reset_phase_ns: phase, ..Build::default() };
        let mut cpu = Cpu::build(&p.words, period, opt);
        // The synchronised release lands 2 to 5.5 ns after a rising edge.
        let edge_off = (cpu.reset_release % ((period * 1000.0) as u64)) as f64 / 1000.0;
        assert!((2.0..=5.5).contains(&edge_off), "phase {phase}: RESET released {edge_off} ns after an edge");
        let rw = cpu.reset_warnings();
        assert!(rw.is_empty(), "phase {phase}: reset warnings {:?}", rw.iter().take(4).collect::<Vec<_>>());
        assert!(cpu.run_until_pc(stop, 200), "phase {phase}: did not reach stop");
        let w = cpu.warnings();
        assert!(w.is_empty(), "phase {phase}: {} warnings, e.g. {:?}", w.len(), w.iter().take(4).collect::<Vec<_>>());
        for r in 0..32 {
            assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "phase {phase}: register {r}");
        }
        assert_eq!(cpu.sim.value(cpu.sim.net_id("RST_n")), Level::H);
        phase += 1.0;
    }
}
