//! End to end: programs on the CPU netlist against the reference simulator.

use mips32::asm::assemble;
use mips32::cpu::{Cpu, gal_specs};
use mips32::iss::Cpu as Iss;

fn run_program(src: &str, period_ns: f64, max_cycles: u64) -> (Cpu, Iss) {
    let p = assemble(src, 0).unwrap();
    let stop = p.labels["stop"];
    let mut iss = Iss::new();
    iss.load_program(0, &p.words);
    iss.run_until(stop, 100_000).unwrap();
    assert_eq!(iss.pc, stop, "reference did not reach stop");
    let mut cpu = Cpu::new(&p.words, period_ns);
    let stopped = cpu.run_until_pc(stop, max_cycles);
    assert!(stopped, "netlist did not reach stop; PC trace: {:?}", cpu.pc_trace);
    (cpu, iss)
}

fn check(src: &str, period_ns: f64) -> Cpu {
    let (cpu, iss) = run_program(src, period_ns, 2000);
    let w = cpu.reset_warnings();
    assert!(w.is_empty(), "warnings during reset:\n{}", w.join("\n"));
    let w = cpu.warnings();
    assert!(w.is_empty(), "chip warnings:\n{}", w.join("\n"));
    // r0 powers up as garbage and is zeroed by the reset sequence.
    assert_eq!(cpu.reg(0), Some(0), "r0 after reset");
    for r in 1..32 {
        assert_eq!(cpu.reg(r), Some(iss.regs[r as usize]), "register {r}; PC trace {:?}", cpu.pc_trace);
    }
    for a in (0..64).step_by(4) {
        assert_eq!(cpu.dmem_word(a), Some(iss.load_word(a)), "dmem[{a:#x}]");
    }
    cpu
}

#[test]
fn chip_count() {
    let specs = gal_specs();
    let mut by_prefix: Vec<(String, usize)> = Vec::new();
    for s in &specs {
        let p: String = s.name.trim_end_matches(|c: char| c.is_ascii_digit()).to_string();
        match by_prefix.iter_mut().find(|(k, _)| *k == p) {
            Some(e) => e.1 += 1,
            None => by_prefix.push((p, 1)),
        }
    }
    eprintln!("GALs: {} total: {:?}", specs.len(), by_prefix);
    assert!(specs.len() < 150, "{}", specs.len());
}

#[test]
fn straight_line_alu() {
    check(
        "
        li   $t0, 5
        li   $t1, 7
        addu $t2, $t0, $t1
        subu $t3, $t0, $t1
        and  $t4, $t0, $t1
        or   $t5, $t0, $t1
        xor  $t6, $t0, $t1
        nor  $t7, $t0, $t1
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn immediates_and_compares() {
    check(
        "
        li    $t0, -1
        li    $t1, 1
        lui   $t2, 0x1234
        ori   $t2, $t2, 0x5678
        addiu $t3, $t2, -0x100
        andi  $t4, $t2, 0xFF0F
        xori  $t5, $t2, 0xFFFF
        slt   $t6, $t0, $t1      # 1
        sltu  $t7, $t0, $t1      # 0
        slti  $s0, $t0, 0        # 1
        sltiu $s1, $t1, -1       # 1
        slt   $s2, $t1, $t0      # 0
        sltu  $s3, $t1, $t0      # 1
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

/// Every forwarding distance for ALU results, into both operands.
#[test]
fn forwarding_chains() {
    check(
        "
        li    $t0, 1
        addu  $t1, $t0, $t0     # d1 into both
        addu  $t2, $t1, $t0     # d1 (t1), d2 (t0)
        addu  $t3, $t0, $t2     # d3 (t0), d1 (t2)
        addu  $t4, $t3, $t3     # d1
        nop
        addu  $t5, $t4, $t1     # d2 (t4), old (t1)
        nop
        nop
        addu  $t6, $t5, $t2     # d3 (t5)
        subu  $t7, $t6, $t5     # d1 with subtract (B inverted after forwarding)
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn loads_stores_and_load_delay() {
    check(
        "
        li    $t0, 0x1234
        li    $t1, 16
        sw    $t0, 0($t1)       # store forwarded data (d1)
        sw    $t1, 4($t1)
        lw    $t2, 0($t1)
        nop                     # load delay slot
        addu  $t3, $t2, $t2     # d2 from a load
        lw    $t4, 4($t1)
        lw    $t5, 0($t1)
        addu  $t6, $t4, $t5     # d3 (t4) and d2 (t5) from loads
        sw    $t6, 8($t1)       # store of a d1 ALU result
        lw    $t7, 8($t1)
        nop
        sw    $t7, 12($t1)      # store of a d2 load result
        li    $s0, 99
        lw    $s0, 0($t1)       # write in the delay slot loses to... no: the load wins later
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn branches_and_loop() {
    check(
        "
        li   $t0, 0        # sum
        li   $t1, 10       # i
    loop:
        addu $t0, $t0, $t1
        bne  $t1, $zero, loop
        addiu $t1, $t1, -1   # delay slot
        li   $t2, 1
        beq  $t0, $t0, taken
        li   $t3, 2          # delay slot
        li   $t3, 3          # killed
    taken:
        bne  $t0, $t0, nottaken
        li   $t4, 4          # delay slot
        li   $t5, 5          # falls through
    nottaken:
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn jumps_jal_jr() {
    check(
        "
        jal  sub
        li   $a0, 5          # delay slot
        move $t0, $v0        # 6
        li   $t1, 1
        jal  sub
        addiu $a0, $t0, 10   # delay slot: a0 = 16
        move $t2, $v0        # 17
        j    done
        li   $t3, 3          # delay slot
        li   $t3, 4          # skipped
    done:
        li   $t4, 7
        b    stop
        nop
        nop
    sub:
        addiu $v0, $a0, 1
        jr   $ra
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

/// JR through a freshly computed register (forwarded into the PC).
#[test]
fn jr_with_forwarded_target() {
    check(
        "
        li   $t0, target
        nop
        jr   $t0             # d2
        nop
        li   $t1, 0xBAD
    target:
        li   $t2, 1
        li   $t3, target2
        jr   $t3             # d1
        nop
        li   $t1, 0xBAD
    target2:
        li   $t4, 2
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn rs_conditioned_branches() {
    check(
        "
        li   $t0, -3
        li   $t1, 0
        li   $t2, 5
        li   $s0, 0
        bltz $t0, a          # taken (sign)
        nop
        li   $s0, 1          # killed
    a:  bgez $t0, b          # not taken
        nop
        li   $s1, 1          # runs
    b:  blez $t1, c          # taken (zero)
        nop
        li   $s1, 2          # killed
    c:  bgtz $t1, d          # not taken (zero)
        nop
        li   $s2, 1          # runs
    d:  bgtz $t2, e          # taken
        nop
        li   $s2, 2          # killed
    e:  bgez $t2, f          # taken (positive)
        addiu $t2, $t2, 1    # delay slot
        li   $s2, 3          # killed
    f:  bltz $t2, g          # not taken (forwarded t2 = 6)
        nop
        li   $s3, 6
    g:  nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

/// JAL and JALR link through the ALU's pass-B path; the link is visible to
/// the delay slot (distance 1) and to the callee (distance 2).
#[test]
fn jal_jalr_links() {
    check(
        "
        jal  sub
        move $t0, $ra        # delay slot reads the link at distance 1
        move $t1, $v0
        li   $t3, sub
        nop
        jalr $t3             # target forwarded at distance 2
        li   $a0, 20         # delay slot
        move $t2, $v0
        li   $t3, sub2
        jalr $s4, $t3        # link into s4, target at distance 1
        li   $a0, 30
        move $t4, $v0
        move $t5, $s4
        b    stop
        nop
        nop
    sub:
        addiu $v0, $a0, 1
        jr   $ra
        move $t6, $ra        # delay slot: ra unchanged here
        nop
    sub2:
        addiu $v0, $a0, 2
        jr   $s4
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn shifts_constant_and_variable() {
    check(
        "
        li   $t0, 0x80000001
        li   $t1, 4
        sll  $t2, $t0, 1
        srl  $t3, $t0, 1
        sra  $t4, $t0, 1
        sllv $t5, $t0, $t1
        srlv $t6, $t0, $t1
        srav $t7, $t0, $t1
        sra  $s0, $t0, 31
        sll  $s1, $t0, 0
        srl  $s2, $t0, 0
        sra  $s3, $t0, 0
        srl  $s4, $t0, 31
        sll  $s5, $t0, 31
        li   $t1, 7          # forwarded amount at distance 1
        srav $s6, $t0, $t1
        addiu $t0, $t0, 0x1234 # forwarded operand at distance 1
        sll  $s7, $t0, 12
        nop
        nop
        nop
    stop:
        nop
        nop
        nop
        nop
        ",
        34.0,
    );
}

#[test]
fn shifts_random_against_reference() {
    // 24 shifts with mixed amounts and both directions, results chained.
    let mut src = String::from("        li $t0, 0xDEADBEEF\n        li $t1, 0x7\n        li $t2, 0x13\n");
    let ops = ["sll", "srl", "sra", "sllv", "srlv", "srav"];
    for k in 0..24 {
        let op = ops[k % 6];
        let dst = 16 + (k % 8); // s0..s7
        if k % 6 < 3 {
            src += &format!("        {op} ${dst}, $t0, {}\n", (k * 7 + 3) % 32);
        } else {
            src += &format!("        {op} ${dst}, $t0, ${}\n", if k % 2 == 0 { 9 } else { 10 });
        }
        src += &format!("        xor $t0, $t0, ${dst}\n");
    }
    src += "        nop\n        nop\n        nop\n    stop:\n        nop\n        nop\n        nop\n        nop\n";
    check(&src, 34.0);
}
