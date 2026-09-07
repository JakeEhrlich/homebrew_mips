//! The boot copier: while the CPU is in reset, the ROMs are copied into
//! instruction memory (code phase) and data memory (region by region); the
//! reset synchroniser releases the CPU only when the copy is done.  Small
//! regions keep the simulation short: 64 words of code, two 64-word data
//! regions.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu};
use mips32::iss::Cpu as Iss;

const K: u32 = 6;

/// Sums the data words 0..8 and the first word of region 1, stores the
/// results, and overwrites one data word.
const PROG: &str = "
        li    $t1, 0
        li    $t0, 0
        li    $t3, 8
    loop:
        lw    $t2, 0($t1)
        addiu $t3, $t3, -1
        addu  $t0, $t0, $t2
        bne   $t3, $zero, loop
        addiu $t1, $t1, 4
        lw    $t4, 256($zero)      # region 1, word 0
        nop
        sw    $t0, 512($zero)      # beyond the copied regions
        sw    $t4, 516($zero)
        li    $t5, 0x77
        sw    $t5, 8($zero)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ";

fn data_image() -> Vec<u32> {
    (0..128u32).map(|i| 0x1000_0000u32.wrapping_mul(i % 7).wrapping_add(i * 0x0101_0101)).collect()
}

fn run(phase: f64) -> (Cpu, Iss, Vec<u32>) {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let data = data_image();
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    for (i, &w) in data.iter().enumerate() {
        iss.store_word(4 * i as u32, w);
    }
    iss.run_until(stop, 10_000).unwrap();
    let opt = Build { boot: Boot::Copy { code_words_log2: K, data_regions: 2, data: data.clone() }, reset_phase_ns: phase, ..Build::default() };
    let mut cpu = Cpu::build(&p.words, 34.0, opt);
    assert!(cpu.run_until_pc(stop, 400), "did not reach stop; pc {:?}", cpu.pc_trace);
    (cpu, iss, data)
}

#[test]
fn copies_code_and_data_then_runs() {
    let (cpu, iss, data) = run(11.0);
    let rw = cpu.reset_warnings();
    assert!(rw.is_empty(), "during boot: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = cpu.warnings();
    assert!(w.is_empty(), "{} warnings, e.g. {:?}", w.len(), w.iter().take(6).collect::<Vec<_>>());
    // Instruction memory holds the program (and zeros to the region end).
    let p = assemble(PROG, 0).unwrap();
    for (i, &word) in p.words.iter().enumerate() {
        assert_eq!(cpu.imem_word(4 * i as u32), Some(word), "imem word {i}");
    }
    assert_eq!(cpu.imem_word(4 * (p.words.len() as u32)), Some(0));
    // Data memory holds both regions, except what the program changed.
    for (i, &word) in data.iter().enumerate() {
        let expect = if i == 2 { 0x77 } else { word };
        assert_eq!(cpu.dmem_word(4 * i as u32), Some(expect), "dmem word {i}");
    }
    for r in 1..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "register {r}");
    }
    assert_eq!(cpu.reg(0), Some(0));
    assert_eq!(cpu.dmem_word(512), Some(iss.load_word(512)));
    assert_eq!(cpu.dmem_word(516), Some(iss.load_word(516)));
}

/// The copier must come up from any supervisor release phase.
#[test]
fn boot_from_three_phases() {
    for phase in [0.0, 17.0, 29.0] {
        let (cpu, iss, _) = run(phase);
        let rw = cpu.reset_warnings();
        assert!(rw.is_empty(), "phase {phase}: {:?}", rw.iter().take(4).collect::<Vec<_>>());
        assert!(cpu.warnings().is_empty(), "phase {phase}");
        for r in 1..32 {
            assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "phase {phase}: register {r}");
        }
    }
}
