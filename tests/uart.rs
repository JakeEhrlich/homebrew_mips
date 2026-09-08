//! The UART behind the slow-device bridge (bus slot 0, docs/uart.md): a
//! program configures the 16550 through command words, echoes three
//! characters as the terminal sends them (each incremented), and drains
//! the transmitter, polling the bridge's busy bit and the chip's line
//! status.  Checked against the reference simulator (registers, memory),
//! the chip model's datasheet bus timing (no warnings), and the
//! transmitted characters.
use mips32::asm::assemble;
use mips32::cpu::{Boot, Build, Cpu};
use mips32::iss::Cpu as Iss;
use mips32::uart16550::Bridge;

/// `cmd` issues one bridge command and waits for it to finish; `rd`
/// then reads the result into $t1.
fn cmd(word: u32) -> String {
    format!(
        "
        li    $t0, {word:#x}
        sw    $t0, 0($s0)
    {l}:
        lw    $t0, 4($s0)          # STATUS
        nop
        andi  $t0, $t0, 1          # BUSY
        bne   $t0, $zero, {l}
        nop",
        l = format!("w{word:x}_{}", COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed))
    )
}
static COUNTER: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

fn program() -> String {
    let w = |a: u8, v: u8| cmd(Bridge::write_cmd(a, v));
    let r = |a: u8| cmd(Bridge::read_cmd(a));
    format!(
        "
        lui   $s0, 0x8000          # I/O base: bit 31 set, slot 0
        {lcr_dlab}
        {dll}
        {dlm}
        {lcr}
        {fcr}
        {mcr}
        li    $s1, 0
        li    $s2, 3
    rx:
        {rd_lsr}
        lw    $t1, 0($s0)          # RDATA = LSR
        nop
        andi  $t1, $t1, 1          # DR
        beq   $t1, $zero, rx
        nop
        {rd_rbr}
        lw    $t1, 0($s0)          # RDATA = the character
        sll   $t2, $s1, 2
        sw    $t1, 256($t2)        # keep it in memory
        addiu $t3, $t1, 1
    tx:
        {rd_lsr2}
        lw    $t1, 0($s0)
        nop
        andi  $t1, $t1, 0x20       # THRE
        beq   $t1, $zero, tx
        nop
        ori   $t0, $t3, 0          # write THR: address 0, data = t3
        sw    $t0, 0($s0)
    wt:
        lw    $t0, 4($s0)
        nop
        andi  $t0, $t0, 1
        bne   $t0, $zero, wt
        nop
        addiu $s1, $s1, 1
        bne   $s1, $s2, rx
        nop
    drain:
        {rd_lsr3}
        lw    $t1, 0($s0)
        nop
        andi  $t1, $t1, 0x40       # TEMT
        beq   $t1, $zero, drain
        nop
        {rd_scr}
        lw    $s3, 0($s0)          # the scratch register back
        {rd_msr}
        lb    $s4, 0($s0)          # modem status (byte load): CTS from RTS
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        lcr_dlab = w(3, 0x83),
        dll = w(0, 1),
        dlm = w(1, 0),
        lcr = w(3, 0x03),
        fcr = w(2, 0x07),
        mcr = w(4, 0x22),          // RTS on, autoflow
        rd_lsr = r(5),
        rd_rbr = r(0),
        rd_lsr2 = r(5),
        rd_lsr3 = r(5),
        rd_scr = { let s = w(7, 0x5A); s + &r(7) },
        rd_msr = r(6),
    )
}

const RX: &[u8] = b"abc";

fn run(period_ns: f64, xin_hz: f64) -> (Cpu, Iss) {
    let src = program();
    let p = assemble(&src, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.serial.core.send(RX);
    iss.run_until(stop, 100_000).unwrap();
    assert_eq!(iss.pc, stop, "reference did not reach stop");
    let opt = Build { boot: Boot::Preload, uart_xin_hz: xin_hz, uart_rx: RX.to_vec(), ..Build::default() };
    let mut cpu = Cpu::build(&p.words, period_ns, opt);
    assert!(cpu.run_until_pc(stop, 6000), "did not reach stop; pc {:?}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(40)..]);
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
    assert_eq!(iss.serial.core.tx, b"bcd".to_vec());
    assert_eq!(cpu.reg(19), Some(0x5A), "scratch register");
    assert_eq!(cpu.reg(20), Some(0x10), "modem status: CTS follows RTS");
}

#[test]
fn echo_three_characters() {
    // A fast crystal keeps the character time at a handful of cycles.
    let (cpu, iss) = run(34.0, 1.0e9);
    check(&cpu, &iss);
}

/// The same at the board crystal: the transmitter takes 1 / 921600 s
/// per bit at divisor 1, so the drain loop polls for real.
#[test]
fn echo_at_board_crystal() {
    let (cpu, iss) = run(34.0, mips32::cpu::UART_XIN_HZ);
    check(&cpu, &iss);
}
