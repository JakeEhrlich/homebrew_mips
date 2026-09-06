//! A small two-pass assembler for the subset, so test programs are readable.
//!
//! Syntax: one instruction or directive per line, `#` comments, `label:`
//! (alone or before an instruction), registers as `$5` or `$t0`, immediates
//! in decimal or `0x` hex, branch/jump targets as labels or numbers.
//! Directives: `.word v, v, ...`.  Pseudo-instructions: `nop`,
//! `move rd, rs`, `li rt, imm` (one or two words), `b label`.

use std::collections::HashMap;

use crate::isa::{Instr, Op, REG_NAMES};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AsmError {
    pub line: usize,
    pub msg: String,
}

impl std::fmt::Display for AsmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.msg)
    }
}

pub struct Program {
    pub words: Vec<u32>,
    pub labels: HashMap<String, u32>,
}

fn reg(s: &str, line: usize) -> Result<u8, AsmError> {
    let s = s.trim();
    let name = s.strip_prefix('$').ok_or(AsmError { line, msg: format!("bad register {s}") })?;
    if let Ok(n) = name.parse::<u8>()
        && n < 32
    {
        return Ok(n);
    }
    REG_NAMES
        .iter()
        .position(|&r| r == name)
        .map(|n| n as u8)
        .ok_or(AsmError { line, msg: format!("bad register {s}") })
}

fn num(s: &str, line: usize) -> Result<i64, AsmError> {
    let s = s.trim().replace('_', "");
    let s = s.as_str();
    let (neg, body) = match s.strip_prefix('-') {
        Some(b) => (true, b),
        None => (false, s),
    };
    let v = if let Some(h) = body.strip_prefix("0x") {
        i64::from_str_radix(h, 16)
    } else {
        body.parse::<i64>()
    }
    .map_err(|_| AsmError { line, msg: format!("bad number {s}") })?;
    Ok(if neg { -v } else { v })
}

/// Assemble `src` with its first word at address `base`.
pub fn assemble(src: &str, base: u32) -> Result<Program, AsmError> {
    struct Item<'a> {
        line: usize,
        mnemonic: &'a str,
        args: Vec<&'a str>,
        addr: u32,
    }
    let mut items = Vec::new();
    let mut labels = HashMap::new();
    let mut addr = base;
    // Pass 1: labels and sizes.
    for (i, raw) in src.lines().enumerate() {
        let line = i + 1;
        let mut text = raw.split('#').next().unwrap().trim();
        while let Some(colon) = text.find(':') {
            let (label, rest) = text.split_at(colon);
            let label = label.trim();
            if label.is_empty() || label.contains(' ') {
                return Err(AsmError { line, msg: "bad label".into() });
            }
            labels.insert(label.to_string(), addr);
            text = rest[1..].trim();
        }
        if text.is_empty() {
            continue;
        }
        let (mnemonic, rest) = text.split_once(char::is_whitespace).unwrap_or((text, ""));
        let args: Vec<&str> = rest.split(',').map(str::trim).filter(|s| !s.is_empty()).collect();
        let size = match mnemonic {
            ".word" => args.len() as u32,
            "li" => {
                // A label (resolved in pass 2) always fits in one word here.
                match num(args.get(1).ok_or(AsmError { line, msg: "li needs 2 args".into() })?, line) {
                    Ok(v) if !((-32768..=32767).contains(&v) || (0..=0xFFFF).contains(&v)) => 2,
                    _ => 1,
                }
            }
            _ => 1,
        };
        items.push(Item { line, mnemonic, args, addr });
        addr += 4 * size;
    }
    // Pass 2: encode.
    let mut words = Vec::new();
    for it in &items {
        let line = it.line;
        let a = |i: usize| -> Result<&str, AsmError> {
            it.args.get(i).copied().ok_or(AsmError { line, msg: format!("{} needs more operands", it.mnemonic) })
        };
        let imm16 = |s: &str| -> Result<u16, AsmError> {
            let v = if let Some(&l) = labels.get(s.trim()) { l as i64 } else { num(s, line)? };
            if !(-32768..=65535).contains(&v) {
                return Err(AsmError { line, msg: format!("immediate {v} out of range") });
            }
            Ok(v as u16)
        };
        let branch_off = |s: &str| -> Result<u16, AsmError> {
            if let Some(&target) = labels.get(s.trim()) {
                let off = (target as i64 - (it.addr as i64 + 4)) / 4;
                if !(-32768..=32767).contains(&off) {
                    return Err(AsmError { line, msg: "branch out of range".into() });
                }
                Ok(off as i16 as u16)
            } else {
                imm16(s)
            }
        };
        let jump_target = |s: &str| -> Result<u32, AsmError> {
            let v = if let Some(&l) = labels.get(s.trim()) { l as i64 } else { num(s, line)? };
            Ok(((v as u32) >> 2) & 0x03FF_FFFF)
        };
        let mem_operand = |s: &str| -> Result<(u16, u8), AsmError> {
            // imm(rs) or (rs)
            let open = s.find('(').ok_or(AsmError { line, msg: format!("bad memory operand {s}") })?;
            let close = s.rfind(')').ok_or(AsmError { line, msg: format!("bad memory operand {s}") })?;
            let off = s[..open].trim();
            let off = if off.is_empty() { 0 } else { imm16(off)? };
            Ok((off, reg(&s[open + 1..close], line)?))
        };
        let r3 = |op: Op| -> Result<u32, AsmError> {
            Ok(Instr::R { op, rd: reg(a(0)?, line)?, rs: reg(a(1)?, line)?, rt: reg(a(2)?, line)? }.encode())
        };
        let i3 = |op: Op| -> Result<u32, AsmError> {
            Ok(Instr::I { op, rt: reg(a(0)?, line)?, rs: reg(a(1)?, line)?, imm: imm16(a(2)?)? }.encode())
        };
        let mem = |op: Op| -> Result<u32, AsmError> {
            let (imm, rs) = mem_operand(a(1)?)?;
            Ok(Instr::I { op, rt: reg(a(0)?, line)?, rs, imm }.encode())
        };
        let br = |op: Op| -> Result<u32, AsmError> {
            Ok(Instr::I { op, rs: reg(a(0)?, line)?, rt: reg(a(1)?, line)?, imm: branch_off(a(2)?)? }.encode())
        };
        match it.mnemonic {
            ".word" => {
                for s in &it.args {
                    let v = if let Some(&l) = labels.get(*s) { l as i64 } else { num(s, line)? };
                    words.push(v as u32);
                }
            }
            "nop" => words.push(Instr::NOP),
            "move" => words.push(Instr::R { op: Op::Addu, rd: reg(a(0)?, line)?, rs: reg(a(1)?, line)?, rt: 0 }.encode()),
            "li" => {
                let rt = reg(a(0)?, line)?;
                let v = if let Some(&lv) = labels.get(a(1)?.trim()) { lv as i64 } else { num(a(1)?, line)? };
                if (-32768..=32767).contains(&v) {
                    words.push(Instr::I { op: Op::Addiu, rt, rs: 0, imm: v as u16 }.encode());
                } else if (0..=0xFFFF).contains(&v) {
                    words.push(Instr::I { op: Op::Ori, rt, rs: 0, imm: v as u16 }.encode());
                } else {
                    let v = v as u32;
                    words.push(Instr::I { op: Op::Lui, rt, rs: 0, imm: (v >> 16) as u16 }.encode());
                    words.push(Instr::I { op: Op::Ori, rt, rs: rt, imm: v as u16 }.encode());
                }
            }
            "b" => words.push(Instr::I { op: Op::Beq, rs: 0, rt: 0, imm: branch_off(a(0)?)? }.encode()),
            "addu" | "add" => words.push(r3(Op::Addu)?),
            "subu" | "sub" => words.push(r3(Op::Subu)?),
            "and" => words.push(r3(Op::And)?),
            "or" => words.push(r3(Op::Or)?),
            "xor" => words.push(r3(Op::Xor)?),
            "nor" => words.push(r3(Op::Nor)?),
            "slt" => words.push(r3(Op::Slt)?),
            "sltu" => words.push(r3(Op::Sltu)?),
            "jr" => words.push(Instr::R { op: Op::Jr, rd: 0, rs: reg(a(0)?, line)?, rt: 0 }.encode()),
            "jalr" => {
                // jalr rs  |  jalr rd, rs
                let (rd, rs) = if it.args.len() == 1 { (31, reg(a(0)?, line)?) } else { (reg(a(0)?, line)?, reg(a(1)?, line)?) };
                words.push(Instr::R { op: Op::Jalr, rd, rs, rt: 0 }.encode());
            }
            "sll" | "srl" | "sra" => {
                let op = match it.mnemonic { "sll" => Op::Sll, "srl" => Op::Srl, _ => Op::Sra };
                let shamt = num(a(2)?, line)?;
                if !(0..32).contains(&shamt) {
                    return Err(AsmError { line, msg: "shift amount out of range".into() });
                }
                words.push(Instr::Sh { op, rd: reg(a(0)?, line)?, rt: reg(a(1)?, line)?, shamt: shamt as u8 }.encode());
            }
            "sllv" | "srlv" | "srav" => {
                let op = match it.mnemonic { "sllv" => Op::Sllv, "srlv" => Op::Srlv, _ => Op::Srav };
                // rd, rt, rs
                words.push(Instr::R { op, rd: reg(a(0)?, line)?, rt: reg(a(1)?, line)?, rs: reg(a(2)?, line)? }.encode());
            }
            "blez" | "bgtz" | "bltz" | "bgez" => {
                let op = match it.mnemonic { "blez" => Op::Blez, "bgtz" => Op::Bgtz, "bltz" => Op::Bltz, _ => Op::Bgez };
                words.push(Instr::I { op, rs: reg(a(0)?, line)?, rt: 0, imm: branch_off(a(1)?)? }.encode());
            }
            "addiu" | "addi" => words.push(i3(Op::Addiu)?),
            "andi" => words.push(i3(Op::Andi)?),
            "ori" => words.push(i3(Op::Ori)?),
            "xori" => words.push(i3(Op::Xori)?),
            "slti" => words.push(i3(Op::Slti)?),
            "sltiu" => words.push(i3(Op::Sltiu)?),
            "lui" => words.push(Instr::I { op: Op::Lui, rt: reg(a(0)?, line)?, rs: 0, imm: imm16(a(1)?)? }.encode()),
            "lw" => words.push(mem(Op::Lw)?),
            "sw" => words.push(mem(Op::Sw)?),
            "beq" => words.push(br(Op::Beq)?),
            "bne" => words.push(br(Op::Bne)?),
            "j" => words.push(Instr::J { op: Op::J, target: jump_target(a(0)?)? }.encode()),
            "jal" => words.push(Instr::J { op: Op::Jal, target: jump_target(a(0)?)? }.encode()),
            m => return Err(AsmError { line, msg: format!("unknown mnemonic {m}") }),
        }
    }
    Ok(Program { words, labels })
}
