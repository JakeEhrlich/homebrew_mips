//! The UART on the data bus behind the bus wait: a program sets the line
//! up, echoes three characters (each incremented) as the terminal sends
//! them, and drains the transmitter, polling the line status register.  Checked against the
//! reference simulator (registers, memory) and the chip model's bus timing
//! (no warnings), and the transmitted characters against the expected
//! ones.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu};
use mips32::iss::Cpu as Iss;

const PROG: &str = "
        lui   $s0, 0x8000          # I/O base: bit 31 set, slot 0 (the UART)
        li    $t0, 0x83
        sw    $t0, 12($s0)         # LCR: DLAB, 8 bits
        li    $t0, 1
        sw    $t0, 0($s0)          # DLL = 1
        sw    $zero, 4($s0)        # DLM = 0
        li    $t0, 0x03
        sw    $t0, 12($s0)         # LCR: 8N1
        li    $t0, 0x07
        sw    $t0, 8($s0)          # FCR: FIFOs on and cleared
        li    $t0, 0x5A
        sw    $t0, 28($s0)         # scratch register
        li    $s1, 0
        li    $s2, 3
    rx:
        lw    $t0, 20($s0)         # LSR
        nop
        andi  $t0, $t0, 1          # DR
        beq   $t0, $zero, rx
        nop
        lw    $t1, 0($s0)          # RBR
        sll   $t2, $s1, 2
        sw    $t1, 256($t2)        # keep the character in memory
        addiu $t1, $t1, 1
    tx:
        lw    $t0, 20($s0)
        nop
        andi  $t0, $t0, 0x20       # THRE
        beq   $t0, $zero, tx
        nop
        sw    $t1, 0($s0)          # THR
        addiu $s1, $s1, 1
        bne   $s1, $s2, rx
        nop
    drain:
        lw    $t0, 20($s0)
        nop
        andi  $t0, $t0, 0x40       # TEMT
        beq   $t0, $zero, drain
        nop
        lw    $s3, 28($s0)         # scratch register back
        lw    $s4, 24($s0)         # modem status (loop-back ties, MCR = 0)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ";

const RX: &[u8] = b"abc";

fn run(period_ns: f64, xin_hz: f64) -> (Cpu, Iss) {
    let p = assemble(PROG, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.uart.send(RX);
    iss.run_until(stop, 100_000).unwrap();
    assert_eq!(iss.pc, stop, "reference did not reach stop");
    let opt = Build { boot: Boot::Preload, uart_xin_hz: xin_hz, uart_rx: RX.to_vec(), ..Build::default() };
    let mut cpu = Cpu::build(&p.words, period_ns, opt);
    assert!(cpu.run_until_pc(stop, 4000), "did not reach stop; pc {:?}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(40)..]);
    (cpu, iss)
}

fn check(cpu: &Cpu, iss: &Iss) {
    let rw = cpu.reset_warnings();
    assert!(rw.is_empty(), "during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = cpu.warnings();
    assert!(w.is_empty(), "{} warnings, e.g. {:?}", w.len(), w.iter().take(8).collect::<Vec<_>>());
    for r in 1..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "register {r}");
    }
    for i in 0..RX.len() {
        let a = 256 + 4 * i as u32;
        assert_eq!(cpu.dmem_word(a), Some(iss.load_word(a)), "dmem[{a:#x}]");
        assert_eq!(iss.load_word(a), RX[i] as u32);
    }
    assert_eq!(cpu.uart_tx(), b"bcd".to_vec());
    assert_eq!(iss.uart.tx, b"bcd".to_vec());
    assert_eq!(cpu.reg(19), Some(0x5A), "scratch register");
    assert_eq!(cpu.reg(20), Some(0), "modem status");
}

#[test]
fn echo_three_characters() {
    // A fast crystal keeps the character time at a handful of cycles.
    let (cpu, iss) = run(34.0, 1.0e9);
    check(&cpu, &iss);
    // Every UART read holds the pipeline for 14 clocks and every write
    // for 6: the program makes at least 6 writes and 3 * 3 reads.
    assert!(cpu.cycles > (6 * 6 + 9 * 14) as u64, "only {} cycles", cpu.cycles);
}

/// The same at the board crystal: the transmitter takes 1 / 921600 s
/// per bit at divisor 1, so the drain loop polls for real.
#[test]
fn echo_at_board_crystal() {
    let (cpu, iss) = run(34.0, mips32::cpu::UART_XIN_HZ);
    check(&cpu, &iss);
}
