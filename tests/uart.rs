//! The serial port (a UART built from GALs, bus slot 0): a program sets
//! the divisor, echoes three characters as the terminal types them (each
//! incremented) and waits for the transmitter to finish, polling the
//! status register.  Checked against the reference simulator (registers,
//! memory) and against what the terminal on the wire decoded.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu};
use mips32::iss::Cpu as Iss;

fn program(div: u8) -> String {
    format!(
        "
        lui   $s0, 0x8000          # I/O base: bit 31 set, slot 0 (the serial port)
        li    $t0, {div}
        sw    $t0, 8($s0)          # DIV
        li    $t0, 0x5A
        li    $s1, 0
        li    $s2, 3
    rx:
        lw    $t0, 4($s0)          # STATUS
        nop
        andi  $t0, $t0, 2          # RXVALID
        beq   $t0, $zero, rx
        nop
        lw    $t1, 0($s0)          # RXD
        sll   $t2, $s1, 2
        sw    $t1, 256($t2)        # keep the character in memory
        addiu $t1, $t1, 1
    tx:
        lw    $t0, 4($s0)
        nop
        andi  $t0, $t0, 1          # TXBUSY
        bne   $t0, $zero, tx
        nop
        sw    $t1, 0($s0)          # TXD
        addiu $s1, $s1, 1
        bne   $s1, $s2, rx
        nop
    drain:
        lw    $t0, 4($s0)
        nop
        andi  $t0, $t0, 1
        bne   $t0, $zero, drain
        nop
        lb    $s3, 4($s0)          # a byte load of the status: 0
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        "
    )
}

const RX: &[u8] = b"abc";

fn run(period_ns: f64, div: u8, max_cycles: u64) -> (Cpu, Iss) {
    let p = assemble(&program(div), 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.serial.send(RX);
    iss.run_until(stop, 100_000).unwrap();
    assert_eq!(iss.pc, stop, "reference did not reach stop");
    let opt = Build { boot: Boot::Preload, uart_div: div, uart_rx: RX.to_vec(), uart_rx_after: 12, ..Build::default() };
    let mut cpu = Cpu::build(&p.words, period_ns, opt);
    assert!(cpu.run_until_pc(stop, max_cycles), "did not reach stop; pc {:?}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(40)..]);
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
    assert_eq!(iss.serial.tx, b"bcd".to_vec());
    assert_eq!(cpu.reg(19), Some(0), "status after the drain");
}

/// Eight clocks per bit (the smallest divisor the receiver samples
/// mid-bit at): a character is 80 cycles.
#[test]
fn echo_three_characters() {
    let (cpu, iss) = run(34.0, 7, 4000);
    check(&cpu, &iss);
}

/// Sixteen clocks per bit.
#[test]
fn echo_at_a_slower_rate() {
    let (cpu, iss) = run(34.0, 15, 8000);
    check(&cpu, &iss);
}
