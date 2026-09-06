//! MIPS-I instruction subset: encoding and decoding.
//!
//! Phase 1 (see `docs/pipeline-plan.md`).  ADD/SUB/ADDI decode as their
//! unsigned forms since there are no overflow traps.

use std::fmt;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Op {
    /// `SLL r0, r0, 0`, the all-zero word.  The only shift in phase 1.
    Nop,
    // R-type (opcode 0, funct)
    Addu,
    Subu,
    And,
    Or,
    Xor,
    Nor,
    Slt,
    Sltu,
    Jr,
    // I-type
    Addiu,
    Andi,
    Ori,
    Xori,
    Slti,
    Sltiu,
    Lui,
    Lw,
    Sw,
    Beq,
    Bne,
    // J-type
    J,
    Jal,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Instr {
    R { op: Op, rd: u8, rs: u8, rt: u8 },
    I { op: Op, rt: u8, rs: u8, imm: u16 },
    J { op: Op, target: u32 },
}

impl Instr {
    pub fn op(&self) -> Op {
        match *self {
            Instr::R { op, .. } | Instr::I { op, .. } | Instr::J { op, .. } => op,
        }
    }
    /// `SLL r0, r0, 0`: all zeros.
    pub const NOP: u32 = 0;

    pub fn encode(&self) -> u32 {
        match *self {
            Instr::R { op: Op::Nop, .. } => 0,
            Instr::R { op, rd, rs, rt } => {
                let funct = match op {
                    Op::Addu => 0x21,
                    Op::Subu => 0x23,
                    Op::And => 0x24,
                    Op::Or => 0x25,
                    Op::Xor => 0x26,
                    Op::Nor => 0x27,
                    Op::Slt => 0x2A,
                    Op::Sltu => 0x2B,
                    Op::Jr => 0x08,
                    _ => panic!("{op:?} is not R-type"),
                };
                (rs as u32) << 21 | (rt as u32) << 16 | (rd as u32) << 11 | funct
            }
            Instr::I { op, rt, rs, imm } => {
                let opcode = match op {
                    Op::Beq => 0x04,
                    Op::Bne => 0x05,
                    Op::Addiu => 0x09,
                    Op::Slti => 0x0A,
                    Op::Sltiu => 0x0B,
                    Op::Andi => 0x0C,
                    Op::Ori => 0x0D,
                    Op::Xori => 0x0E,
                    Op::Lui => 0x0F,
                    Op::Lw => 0x23,
                    Op::Sw => 0x2B,
                    _ => panic!("{op:?} is not I-type"),
                };
                opcode << 26 | (rs as u32) << 21 | (rt as u32) << 16 | imm as u32
            }
            Instr::J { op, target } => {
                let opcode = match op {
                    Op::J => 0x02,
                    Op::Jal => 0x03,
                    _ => panic!("{op:?} is not J-type"),
                };
                opcode << 26 | (target & 0x03FF_FFFF)
            }
        }
    }

    /// `None` for anything outside the subset (the hardware's behaviour on
    /// such words is undefined).
    pub fn decode(w: u32) -> Option<Instr> {
        let opcode = w >> 26;
        let rs = (w >> 21 & 31) as u8;
        let rt = (w >> 16 & 31) as u8;
        let rd = (w >> 11 & 31) as u8;
        let shamt = w >> 6 & 31;
        let funct = w & 0x3F;
        let imm = w as u16;
        match opcode {
            0 if w == 0 => Some(Instr::R { op: Op::Nop, rd: 0, rs: 0, rt: 0 }),
            0 => {
                let op = match funct {
                    0x20 | 0x21 => Op::Addu, // ADD treated as ADDU
                    0x22 | 0x23 => Op::Subu,
                    0x24 => Op::And,
                    0x25 => Op::Or,
                    0x26 => Op::Xor,
                    0x27 => Op::Nor,
                    0x2A => Op::Slt,
                    0x2B => Op::Sltu,
                    0x08 => Op::Jr,
                    _ => return None,
                };
                if shamt != 0 {
                    return None;
                }
                Some(Instr::R { op, rd, rs, rt })
            }
            0x02 => Some(Instr::J { op: Op::J, target: w & 0x03FF_FFFF }),
            0x03 => Some(Instr::J { op: Op::Jal, target: w & 0x03FF_FFFF }),
            _ => {
                let op = match opcode {
                    0x04 => Op::Beq,
                    0x05 => Op::Bne,
                    0x08 | 0x09 => Op::Addiu, // ADDI treated as ADDIU
                    0x0A => Op::Slti,
                    0x0B => Op::Sltiu,
                    0x0C => Op::Andi,
                    0x0D => Op::Ori,
                    0x0E => Op::Xori,
                    0x0F => Op::Lui,
                    0x23 => Op::Lw,
                    0x2B => Op::Sw,
                    _ => return None,
                };
                Some(Instr::I { op, rt, rs, imm })
            }
        }
    }
}

pub const REG_NAMES: [&str; 32] = [
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "t7", "s0", "s1",
    "s2", "s3", "s4", "s5", "s6", "s7", "t8", "t9", "k0", "k1", "gp", "sp", "fp", "ra",
];

impl fmt::Display for Instr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let r = |n: u8| format!("${}", REG_NAMES[n as usize]);
        let name = format!("{:?}", self.op()).to_lowercase();
        match *self {
            Instr::R { op: Op::Nop, .. } => write!(f, "nop"),
            Instr::R { op: Op::Jr, rs, .. } => write!(f, "jr {}", r(rs)),
            Instr::R { rd, rs, rt, .. } => write!(f, "{name} {}, {}, {}", r(rd), r(rs), r(rt)),
            Instr::I { op: Op::Lui, rt, imm, .. } => write!(f, "lui {}, 0x{imm:x}", r(rt)),
            Instr::I { op: Op::Lw | Op::Sw, rt, rs, imm } => write!(f, "{name} {}, {}({})", r(rt), imm as i16, r(rs)),
            Instr::I { op: Op::Beq | Op::Bne, rt, rs, imm } => write!(f, "{name} {}, {}, {}", r(rs), r(rt), imm as i16),
            Instr::I { op: Op::Andi | Op::Ori | Op::Xori, rt, rs, imm } => {
                write!(f, "{name} {}, {}, 0x{imm:x}", r(rt), r(rs))
            }
            Instr::I { rt, rs, imm, .. } => write!(f, "{name} {}, {}, {}", r(rt), r(rs), imm as i16),
            Instr::J { target, .. } => write!(f, "{name} 0x{:x}", target << 2),
        }
    }
}
