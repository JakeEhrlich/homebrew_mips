//! The bus (docs/bus.md) apart from any device: an access to an empty
//! slot costs one cycle like any other and the machine carries on; a
//! store to an empty slot is ignored; a narrow load right behind an I/O
//! access and an instruction forwarding across it are intact.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu};
use mips32::iss::Cpu as Iss;

const PROG: &str = "
        lui   $s0, 0x8000
        lui   $s1, 0x8080          # slot 1: nothing there
        li    $t0, 0x12345678
        sw    $t0, 0($zero)
        sw    $t0, 4($zero)
        li    $t1, 7
        sw    $t1, 0($s1)          # store to the empty slot: ignored
        addiu $t2, $t1, 1          # forwarding across the wait
        lw    $t3, 0($s1)          # load from the empty slot: garbage
        lb    $t4, 1($zero)        # a narrow load right behind it
        addiu $t5, $t3, 0          # uses the garbage (not compared)
        lh    $t6, 2($zero)
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
fn empty_slot_reads_garbage_and_carries_on() {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 10_000).unwrap();
    let mut cpu = Cpu::build(&p.words, 34.0, Build { boot: Boot::Preload, ..Build::default() });
    assert!(cpu.run_until_pc(stop, 400), "did not reach stop; pc {:?}", cpu.pc_trace);
    // The timed-out load captured a floating bus: those unknown captures
    // are the only warnings allowed.
    let w: Vec<String> = cpu.warnings().into_iter().filter(|w| !(w.starts_with("wb") && w.contains("CapturedX"))).collect();
    assert!(w.is_empty(), "{} warnings, e.g. {:?}", w.len(), w.iter().take(6).collect::<Vec<_>>());
    for r in 1..32 {
        if r == 11 || r == 13 {
            continue; // t3 (the garbage) and t5 (derived from it)
        }
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "register {r}");
    }
    // The floating bus keeps its last driven level for a while (the
    // GAL model's keepers), so the garbage is the previous store's data;
    // on the board it is whatever the capacitance held.  Not compared.
    assert_eq!(cpu.dmem_word(0), Some(0x12345678));
    // No stall anywhere: a dozen instructions, a dozen or so cycles.
    assert!(cpu.cycles < 40, "{} cycles", cpu.cycles);
}
