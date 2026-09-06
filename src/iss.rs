//! Reference instruction-set simulator for the subset, with the pipeline's
//! architecturally visible timing: one branch delay slot, one load delay
//! slot (the instruction after a load sees the register's old value), no
//! interlocks, Harvard memory of 8K words each.
//!
//! This is the oracle the netlist is checked against, so it deliberately
//! mirrors what the hardware does rather than what a friendlier MIPS would.

use crate::isa::{Instr, Op};

pub const IMEM_WORDS: usize = 8192;
pub const DMEM_WORDS: usize = 8192;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Fault {
    /// Word at `pc` is not in the subset.
    Illegal { pc: u32, word: u32 },
}

#[derive(Clone, Debug)]
pub struct Cpu {
    pub pc: u32,
    pub regs: [u32; 32],
    pub imem: Vec<u32>,
    pub dmem: Vec<u32>,
    /// Register write from the previous instruction's load, applied after
    /// the current instruction has read its operands.
    load_delay: Option<(u8, u32)>,
    /// Branch/jump target to take after the current (delay slot) instruction.
    branch: Option<u32>,
    /// Instructions retired.
    pub count: u64,
}

impl Cpu {
    pub fn new() -> Self {
        Cpu {
            pc: 0,
            regs: [0; 32],
            imem: vec![0; IMEM_WORDS],
            dmem: vec![0; DMEM_WORDS],
            load_delay: None,
            branch: None,
            count: 0,
        }
    }

    /// Load a program (words) at instruction address `base`.
    pub fn load_program(&mut self, base: u32, words: &[u32]) {
        for (i, &w) in words.iter().enumerate() {
            self.imem[(base as usize / 4 + i) % IMEM_WORDS] = w;
        }
    }

    pub fn fetch(&self, pc: u32) -> u32 {
        self.imem[(pc as usize >> 2) % IMEM_WORDS]
    }
    pub fn load_word(&self, addr: u32) -> u32 {
        self.dmem[(addr as usize >> 2) % DMEM_WORDS]
    }
    pub fn store_word(&mut self, addr: u32, v: u32) {
        self.dmem[(addr as usize >> 2) % DMEM_WORDS] = v;
    }

    fn set_reg(&mut self, r: u8, v: u32) {
        if r != 0 {
            self.regs[r as usize] = v;
        }
    }

    /// Execute one instruction.
    pub fn step(&mut self) -> Result<(), Fault> {
        let pc = self.pc;
        let word = self.fetch(pc);
        let instr = Instr::decode(word).ok_or(Fault::Illegal { pc, word })?;
        let pending_branch = self.branch.take();
        let pending_load = self.load_delay.take();
        let next_pc = pc.wrapping_add(4);

        // Read operands (before the delayed load lands).
        let (rs, rt) = match instr {
            Instr::R { rs, rt, .. } | Instr::I { rs, rt, .. } => (self.regs[rs as usize], self.regs[rt as usize]),
            Instr::J { .. } => (0, 0),
        };
        // Now the previous load's value becomes visible.
        if let Some((r, v)) = pending_load {
            self.set_reg(r, v);
        }

        match instr {
            Instr::R { op, rd, .. } => {
                let v = match op {
                    Op::Nop => {
                        self.finish(pending_branch, next_pc);
                        return Ok(());
                    }
                    Op::Addu => rs.wrapping_add(rt),
                    Op::Subu => rs.wrapping_sub(rt),
                    Op::And => rs & rt,
                    Op::Or => rs | rt,
                    Op::Xor => rs ^ rt,
                    Op::Nor => !(rs | rt),
                    Op::Slt => ((rs as i32) < (rt as i32)) as u32,
                    Op::Sltu => (rs < rt) as u32,
                    Op::Jr => {
                        self.branch = Some(rs);
                        self.finish(pending_branch, next_pc);
                        return Ok(());
                    }
                    _ => unreachable!(),
                };
                self.set_reg(rd, v);
            }
            Instr::I { op, rt: rt_n, imm, .. } => {
                let sext = imm as i16 as i32 as u32;
                let zext = imm as u32;
                match op {
                    Op::Addiu => self.set_reg(rt_n, rs.wrapping_add(sext)),
                    Op::Andi => self.set_reg(rt_n, rs & zext),
                    Op::Ori => self.set_reg(rt_n, rs | zext),
                    Op::Xori => self.set_reg(rt_n, rs ^ zext),
                    Op::Slti => self.set_reg(rt_n, ((rs as i32) < (sext as i32)) as u32),
                    Op::Sltiu => self.set_reg(rt_n, (rs < sext) as u32),
                    Op::Lui => self.set_reg(rt_n, zext << 16),
                    Op::Lw => {
                        let v = self.load_word(rs.wrapping_add(sext));
                        self.load_delay = Some((rt_n, v));
                    }
                    Op::Sw => self.store_word(rs.wrapping_add(sext), rt),
                    Op::Beq | Op::Bne => {
                        let taken = (rs == rt) == (op == Op::Beq);
                        if taken {
                            self.branch = Some(next_pc.wrapping_add(sext << 2));
                        }
                    }
                    _ => unreachable!(),
                }
            }
            Instr::J { op, target } => {
                let dest = (next_pc & 0xF000_0000) | (target << 2);
                if op == Op::Jal {
                    self.set_reg(31, pc.wrapping_add(8));
                }
                self.branch = Some(dest);
            }
        }
        self.finish(pending_branch, next_pc);
        Ok(())
    }

    fn finish(&mut self, pending_branch: Option<u32>, next_pc: u32) {
        self.pc = pending_branch.unwrap_or(next_pc);
        self.count += 1;
    }

    /// Run until `pc` hits `stop` or `max` instructions have retired.
    pub fn run_until(&mut self, stop: u32, max: u64) -> Result<u64, Fault> {
        let start = self.count;
        while self.pc != stop && self.count - start < max {
            self.step()?;
        }
        Ok(self.count - start)
    }
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}
