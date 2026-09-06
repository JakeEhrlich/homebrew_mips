//! Encoder/decoder, assembler and reference simulator tests.

use mips32::asm::assemble;
use mips32::isa::{Instr, Op};
use mips32::iss::{Cpu, Fault};

/// Encodings checked against binutils output.
#[test]
fn known_encodings() {
    let cases: &[(&str, u32)] = &[
        ("addiu $sp, $sp, -16", 0x27BD_FFF0),
        ("lw $ra, 0($sp)", 0x8FBF_0000),
        ("sw $a0, 4($sp)", 0xAFA4_0004),
        ("jr $ra", 0x03E0_0008),
        ("addu $v0, $a0, $a1", 0x0085_1021),
        ("subu $t0, $t1, $t2", 0x012A_4023),
        ("lui $t0, 0x1234", 0x3C08_1234),
        ("ori $t0, $t0, 0x5678", 0x3508_5678),
        ("slt $v0, $a0, $a1", 0x0085_102A),
        ("sltiu $v0, $a0, 10", 0x2C82_000A),
        ("nor $t0, $t1, $t2", 0x012A_4027),
        ("nop", 0x0000_0000),
        ("j 0x400", 0x0800_0100),
        ("jal 0x400", 0x0C00_0100),
        ("beq $a0, $a1, 4", 0x1085_0004),
        ("bne $a0, $zero, -1", 0x1480_FFFF),
        ("andi $t0, $t1, 0xFFFF", 0x3128_FFFF),
    ];
    for &(src, want) in cases {
        let p = assemble(src, 0).unwrap();
        assert_eq!(p.words, vec![want], "{src}");
        if want != 0 {
            let i = Instr::decode(want).unwrap();
            assert_eq!(i.encode(), want, "{src} round trip ({i})");
        }
    }
}

#[test]
fn add_and_addi_decode_as_unsigned_forms() {
    assert_eq!(Instr::decode(0x0085_1020).unwrap().op(), Op::Addu); // add
    assert_eq!(Instr::decode(0x2082_0001).unwrap().op(), Op::Addiu); // addi
    assert!(Instr::decode(0x0000_0040).is_none()); // sll with shamt: phase 2
    assert!(Instr::decode(0x7000_0000).is_none());
}

#[test]
fn labels_and_pseudo_instructions() {
    let p = assemble(
        "
        li   $t0, 0x12345678   # two words
        li   $t1, -5
        li   $t2, 0xFFFF       # ori
    top:
        addiu $t1, $t1, 1
        bne  $t1, $zero, top
        nop
        b    done
        nop
        .word 0xDEADBEEF, top
    done:
        jr   $ra
        move $v0, $t0
        ",
        0x100,
    )
    .unwrap();
    assert_eq!(p.labels["top"], 0x110);
    assert_eq!(p.labels["done"], 0x12C);
    assert_eq!(p.words[0], 0x3C08_1234);
    assert_eq!(p.words[1], 0x3508_5678);
    assert_eq!(p.words[2], 0x2409_FFFB);
    assert_eq!(p.words[3], 0x340A_FFFF);
    assert_eq!(p.words[5], 0x1520_FFFE); // bne back to top: offset -2
    assert_eq!(p.words[7], 0x1000_0003); // b done: offset +3
    assert_eq!(p.words[9], 0xDEAD_BEEF);
    assert_eq!(p.words[10], 0x110);
    assert_eq!(p.words[12], 0x0100_1021); // move = addu $v0, $t0, $zero
}

fn run(src: &str, max: u64) -> Cpu {
    let p = assemble(src, 0).unwrap();
    let mut cpu = Cpu::new();
    cpu.load_program(0, &p.words);
    let stop = p.labels.get("stop").copied().unwrap_or(p.words.len() as u32 * 4);
    cpu.run_until(stop, max).unwrap();
    assert_eq!(cpu.pc, stop, "did not reach stop");
    cpu
}

#[test]
fn sum_loop() {
    let cpu = run(
        "
        li   $t0, 0        # sum
        li   $t1, 10       # i
    loop:
        addu $t0, $t0, $t1
        bne  $t1, $zero, loop
        addiu $t1, $t1, -1   # delay slot: executes before the branch takes
    stop:
        nop
        ",
        1000,
    );
    assert_eq!(cpu.regs[8], 55);
    assert_eq!(cpu.regs[9], 0xFFFF_FFFF); // the slot ran once more after i hit 0
}

#[test]
fn branch_delay_slot_executes_even_when_taken() {
    let cpu = run(
        "
        li  $t0, 1
        beq $zero, $zero, skip
        li  $t0, 2          # delay slot
        li  $t0, 3          # skipped
    skip:
        nop
    stop:
        nop
        ",
        100,
    );
    assert_eq!(cpu.regs[8], 2);
}

#[test]
fn load_delay_slot_sees_old_value() {
    let cpu = run(
        "
        li   $t0, 7
        sw   $t0, 0($zero)
        li   $t1, 99
        lw   $t1, 0($zero)
        move $t2, $t1        # load delay slot: old value
        move $t3, $t1        # new value
    stop:
        nop
        ",
        100,
    );
    assert_eq!(cpu.regs[10], 99);
    assert_eq!(cpu.regs[11], 7);
    assert_eq!(cpu.regs[9], 7);
}

/// A write in the load delay slot to the load's destination is overwritten
/// by the load (the load's WB comes later in the pipeline).
#[test]
fn load_delay_slot_write_to_same_register_loses() {
    let cpu = run(
        "
        li   $t0, 7
        sw   $t0, 0($zero)
        lw   $t1, 0($zero)
        li   $t1, 5
        nop
    stop:
        nop
        ",
        100,
    );
    assert_eq!(cpu.regs[9], 5);
}

#[test]
fn jal_jr_and_link_address() {
    let cpu = run(
        "
        jal  sub
        li   $a0, 5          # delay slot
        move $t0, $v0
        b    stop
        nop
    sub:
        addiu $v0, $a0, 1
        jr   $ra
        nop
    stop:
        nop
        ",
        100,
    );
    assert_eq!(cpu.regs[31], 8);
    assert_eq!(cpu.regs[8], 6);
}

#[test]
fn register_zero_stays_zero_and_illegal_faults() {
    let mut cpu = Cpu::new();
    cpu.load_program(0, &assemble("li $zero, 5\n.word 0x7000_0000", 0).unwrap().words);
    cpu.step().unwrap();
    assert_eq!(cpu.regs[0], 0);
    assert_eq!(cpu.step(), Err(Fault::Illegal { pc: 4, word: 0x7000_0000 }));
}

#[test]
fn slt_variants_and_logic() {
    let cpu = run(
        "
        li   $t0, -1
        li   $t1, 1
        slt  $t2, $t0, $t1     # -1 < 1 signed: 1
        sltu $t3, $t0, $t1     # 0xFFFFFFFF < 1 unsigned: 0
        slti $t4, $t0, 0       # 1
        sltiu $t5, $t1, -1     # 1 < 0xFFFFFFFF: 1
        nor  $t6, $t1, $zero   # ~1
        xori $t7, $t0, 0xFFFF  # 0xFFFF0000
    stop:
        nop
        ",
        100,
    );
    assert_eq!(cpu.regs[10], 1);
    assert_eq!(cpu.regs[11], 0);
    assert_eq!(cpu.regs[12], 1);
    assert_eq!(cpu.regs[13], 1);
    assert_eq!(cpu.regs[14], !1);
    assert_eq!(cpu.regs[15], 0xFFFF_0000);
}
