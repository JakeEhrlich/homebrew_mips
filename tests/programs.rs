//! Real programs: the kind of code a compiler or a runtime would emit,
//! run on the netlist against the reference simulator.  Sorting with
//! nested loops and word accesses, byte-string handling, recursion with
//! a stack in memory, and a table-driven checksum.
use mips32::asm::assemble;
use mips32::cpu::Cpu;
use mips32::iss::Cpu as Iss;

fn run(src: &str, max_cycles: u64) -> (Cpu, Iss) {
    let p = assemble(src, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    let retired = iss.run_until(stop, 200_000).unwrap();
    assert_eq!(iss.pc, stop, "reference did not reach stop");
    let mut cpu = Cpu::new(&p.words, 34.0);
    assert!(cpu.run_until_pc(stop, max_cycles), "netlist did not reach stop; {retired} retired; pc tail {:?}", &cpu.pc_trace[cpu.pc_trace.len().saturating_sub(30)..]);
    let rw = cpu.reset_warnings();
    assert!(rw.is_empty(), "during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = cpu.warnings();
    assert!(w.is_empty(), "{} warnings, e.g. {:?}", w.len(), w.iter().take(10).collect::<Vec<_>>());
    for r in 0..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "register {r}");
    }
    for a in (0..1024).step_by(4) {
        assert_eq!(cpu.dmem_word(a), Some(iss.load_word(a)), "dmem[{a:#x}]");
    }
    eprintln!("{retired} instructions, {} cycles", cpu.cycles);
    (cpu, iss)
}

/// Bubble sort of 12 words written in descending order, then a check
/// that reads them back ascending.
#[test]
fn bubble_sort() {
    let (cpu, _) = run(
        "
        li    $s0, 0              # array
        li    $s1, 12             # n
        # fill: a[i] = 100 - 7 * i
        li    $t0, 0
        li    $t1, 100
    fill:
        sll   $t2, $t0, 2
        addu  $t2, $t2, $s0
        sw    $t1, 0($t2)
        addiu $t1, $t1, -7
        addiu $t0, $t0, 1
        bne   $t0, $s1, fill
        nop
        # outer: for i in 0..n-1
        li    $t0, 0
    outer:
        addiu $t9, $s1, -1
        subu  $t9, $t9, $t0       # n - 1 - i
        blez  $t9, done
        nop
        li    $t1, 0
    inner:
        sll   $t2, $t1, 2
        addu  $t2, $t2, $s0
        lw    $t3, 0($t2)
        lw    $t4, 4($t2)
        nop
        slt   $t5, $t4, $t3
        beq   $t5, $zero, noswap
        nop
        sw    $t4, 0($t2)
        sw    $t3, 4($t2)
    noswap:
        addiu $t1, $t1, 1
        bne   $t1, $t9, inner
        nop
        addiu $t0, $t0, 1
        j     outer
        nop
    done:
        # verify: v0 = number of adjacent pairs out of order (should be 0)
        li    $v0, 0
        li    $t0, 0
        addiu $t9, $s1, -1
    check:
        sll   $t2, $t0, 2
        lw    $t3, 0($t2)
        lw    $t4, 4($t2)
        nop
        slt   $t5, $t4, $t3
        addu  $v0, $v0, $t5
        addiu $t0, $t0, 1
        bne   $t0, $t9, check
        nop
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        20_000,
    );
    assert_eq!(cpu.reg(2), Some(0));
    assert_eq!(cpu.dmem_word(0), Some(100 - 77));
    assert_eq!(cpu.dmem_word(44), Some(100));
}

/// Byte strings: write a string with SB, strlen with LBU, strcpy to a
/// second buffer, then an upper-casing pass in place.
#[test]
fn byte_strings() {
    let (cpu, _) = run(
        "
        li    $s0, 0x100          # src
        li    $s1, 0x200          # dst
        # write \"hello, crag!\" byte by byte
        li    $t0, 104
        sb    $t0, 0($s0)
        li    $t0, 101
        sb    $t0, 1($s0)
        li    $t0, 108
        sb    $t0, 2($s0)
        sb    $t0, 3($s0)
        li    $t0, 111
        sb    $t0, 4($s0)
        li    $t0, 44
        sb    $t0, 5($s0)
        li    $t0, 32
        sb    $t0, 6($s0)
        li    $t0, 99
        sb    $t0, 7($s0)
        li    $t0, 114
        sb    $t0, 8($s0)
        li    $t0, 97
        sb    $t0, 9($s0)
        li    $t0, 103
        sb    $t0, 10($s0)
        li    $t0, 33
        sb    $t0, 11($s0)
        sb    $zero, 12($s0)
        # strlen
        li    $v0, 0
        addu  $t1, $s0, $zero
    len:
        lbu   $t2, 0($t1)
        addiu $t1, $t1, 1
        bne   $t2, $zero, len
        addiu $v0, $v0, 1
        addiu $v0, $v0, -1        # 12
        # strcpy
        addu  $t1, $s0, $zero
        addu  $t3, $s1, $zero
    cpy:
        lbu   $t2, 0($t1)
        addiu $t1, $t1, 1
        sb    $t2, 0($t3)
        bne   $t2, $zero, cpy
        addiu $t3, $t3, 1
        # upper-case dst in place
        addu  $t3, $s1, $zero
    up:
        lb    $t2, 0($t3)
        nop
        beq   $t2, $zero, upd
        addiu $t4, $t2, -97       # 'a'
        sltiu $t5, $t4, 26
        beq   $t5, $zero, skip
        nop
        addiu $t2, $t2, -32
        sb    $t2, 0($t3)
    skip:
        j     up
        addiu $t3, $t3, 1
    upd:
        lw    $v1, 0($s1)         # 'HELL' as a word
        lhu   $a0, 4($s1)         # 'O,'
        lh    $a1, 10($s1)        # 'G!' -> 0x2147
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        20_000,
    );
    assert_eq!(cpu.reg(2), Some(12));
    assert_eq!(cpu.reg(3), Some(u32::from_le_bytes(*b"HELL")));
    assert_eq!(cpu.reg(4), Some(u16::from_le_bytes(*b"O,") as u32));
    assert_eq!(cpu.reg(5), Some(u16::from_le_bytes(*b"G!") as u32));
}

/// Recursive Fibonacci with a stack in memory: fib(10) = 55, through
/// 177 calls, each saving $ra and an argument on the stack.
#[test]
fn recursive_fibonacci() {
    let (cpu, _) = run(
        "
        li    $sp, 0x3F0
        li    $a0, 10
        jal   fib
        nop
        addu  $s0, $v0, $zero
        j     stop
        nop
    fib:
        addiu $sp, $sp, -12
        sw    $ra, 0($sp)
        sw    $a0, 4($sp)
        slti  $t0, $a0, 2
        beq   $t0, $zero, rec
        nop
        addu  $v0, $a0, $zero     # fib(0) = 0, fib(1) = 1
        j     ret
        nop
    rec:
        addiu $a0, $a0, -1
        jal   fib
        nop
        sw    $v0, 8($sp)
        lw    $a0, 4($sp)
        nop
        addiu $a0, $a0, -2
        jal   fib
        nop
        lw    $t1, 8($sp)
        nop
        addu  $v0, $v0, $t1
    ret:
        lw    $ra, 0($sp)
        nop
        jr    $ra
        addiu $sp, $sp, 12
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        30_000,
    );
    assert_eq!(cpu.reg(16), Some(55));
}

/// A table-driven checksum: build a 256-entry table with a loop, then
/// hash a block of 64 bytes through it, mixing with shifts and XORs.
#[test]
fn table_checksum() {
    let (cpu, iss) = run(
        "
        li    $s0, 0x100          # table, 256 words
        li    $s1, 0x000          # data, 64 bytes
        # table[i] = (i * 0x9E37) ^ (i << 7)
        li    $t0, 0
        li    $t3, 0x9E37
    tbl:
        sll   $t1, $t0, 7
        # multiply by 0x9E37 with shifts and adds: i*(0x8000+0x1000+0x800+0x400+0x200+0x20+0x10+0x4+0x2+0x1)
        sll   $t2, $t0, 15
        sll   $t4, $t0, 12
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 11
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 10
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 9
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 5
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 4
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 2
        addu  $t2, $t2, $t4
        sll   $t4, $t0, 1
        addu  $t2, $t2, $t4
        addu  $t2, $t2, $t0
        xor   $t2, $t2, $t1
        sll   $t4, $t0, 2
        addu  $t4, $t4, $s0
        sw    $t2, 0($t4)
        addiu $t0, $t0, 1
        slti  $t5, $t0, 256
        bne   $t5, $zero, tbl
        nop
        # data[i] = (i * 37) & 0xFF, as bytes
        li    $t0, 0
        li    $t1, 0
    dat:
        addu  $t4, $s1, $t0
        sb    $t1, 0($t4)
        addiu $t1, $t1, 37
        addiu $t0, $t0, 1
        slti  $t5, $t0, 64
        bne   $t5, $zero, dat
        nop
        # h = 0x811C9DC5; for each byte b: h = (h ^ table[b]) rotl 5 + b
        lui   $v0, 0x811C
        ori   $v0, $v0, 0x9DC5
        li    $t0, 0
    hsh:
        addu  $t4, $s1, $t0
        lbu   $t1, 0($t4)
        nop
        sll   $t2, $t1, 2
        addu  $t2, $t2, $s0
        lw    $t3, 0($t2)
        nop
        xor   $v0, $v0, $t3
        sll   $t6, $v0, 5
        srl   $t7, $v0, 27
        or    $v0, $t6, $t7
        addu  $v0, $v0, $t1
        addiu $t0, $t0, 1
        slti  $t5, $t0, 64
        bne   $t5, $zero, hsh
        nop
        sw    $v0, 0x400($zero)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        60_000,
    );
    assert_eq!(cpu.reg(2), Some(iss.regs[2]));
    assert_eq!(cpu.dmem_word(0x400), Some(iss.regs[2]));
}
