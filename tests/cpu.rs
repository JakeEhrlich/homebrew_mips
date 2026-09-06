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
    let w = cpu.warnings();
    assert!(w.is_empty(), "chip warnings:\n{}", w.join("\n"));
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
