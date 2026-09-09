//! grit: the first board (docs/grit.md).  A microprogrammed 16-bit
//! machine: a small instruction set in the program flash, a microcode
//! ROM in a second flash pair addressed by {opcode, NE, step}, a
//! pipeline register that latches one microword per clock, two latches
//! A (also the address register) and B, a 16-bit ALU, a PC, an 8K x 16
//! SRAM holding the registers and data, and a TL16C550 on the bus.
//!
//! Every control line is a flop output of the pipeline register and
//! changes only at CLK's rising edge.  Hold times and bus turnarounds
//! are met by the order of the microwords (two rules, `microcode`).
//!
//! This module holds the microword definition and the microcode table,
//! the instruction set and a tiny assembler, a reference interpreter,
//! the GAL equations, the netlist and board file, and the runner.
use crate::as7c164a::{self, As7c164a};
use crate::board::{Board, ChipMeta, Column, Load, Model};
use crate::ds1100::Grade;
use crate::galpack::{self, Eq, GalSpec, Mode, SLit, lit, nlit};
use crate::gal22v10::{Gal22v10, olmc_pin};
use crate::netlist::{Level, NS, NetId, Netlist, Passive, ResetSupervisor, Rom, RomPin, Sim, Sram8kPin, Time, rom_pin_of, sram8k_pin_of};
use crate::uart16550::{BusTiming, Core, Uart16550, UartPin, uart_pin_of};
use std::collections::BTreeMap;

// ---------------------------------------------------------------------------
// The microword

pub const PCDRV: u16 = 1 << 0;
pub const MEMRD: u16 = 1 << 1;
pub const ALUOE: u16 = 1 << 2;
pub const WE: u16 = 1 << 3;
pub const ALD: u16 = 1 << 4;
pub const BLD: u16 = 1 << 5;
pub const PCLD: u16 = 1 << 6;
pub const PCINC: u16 = 1 << 7;
pub const IRLD: u16 = 1 << 8;
pub const F0: u16 = 1 << 9;
pub const F1: u16 = 1 << 10;
/// A drives Addr (and so the ALU sees A).  Never in the word next to a
/// PCDRV word: the address bus changes hands with an idle word between.
pub const ADRV: u16 = 1 << 11;
pub const F_ADD: u16 = 0;
pub const F_AND: u16 = F0;
pub const F_NOR: u16 = F1;
pub const F_PASSB: u16 = F1 | F0;
/// The fetch: the PC addresses the flash, the IR takes the opcode.
pub const FETCH: u16 = PCDRV | MEMRD | IRLD;

/// Microword bit names, bit 0 first.
pub const BIT_NAMES: [&str; 12] = ["PCDRV", "MEMRD", "ALUOE", "WE", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV"];

// ---------------------------------------------------------------------------
// The instruction set

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Op {
    Reset = 0,
    LdaImm = 1,
    LdbImm = 2,
    LdaA = 3,
    LdbA = 4,
    StbA = 5,
    AddA = 6,
    AddB = 7,
    AndA = 8,
    AndB = 9,
    NorA = 10,
    NorB = 11,
    MovAB = 12,
    Jmp = 13,
    Jeq = 14,
    Nop = 15,
    Halt = 16,
}

impl Op {
    pub const ALL: [Op; 17] = [Op::Reset, Op::LdaImm, Op::LdbImm, Op::LdaA, Op::LdbA, Op::StbA, Op::AddA, Op::AddB, Op::AndA, Op::AndB, Op::NorA, Op::NorB, Op::MovAB, Op::Jmp, Op::Jeq, Op::Nop, Op::Halt];
    pub fn from_code(c: u8) -> Option<Op> {
        Op::ALL.iter().copied().find(|o| *o as u8 == c)
    }
    /// The instruction word (the opcode in bits 15:11).
    pub fn word(self) -> u16 {
        (self as u16) << 11
    }
    pub fn has_imm(self) -> bool {
        matches!(self, Op::LdaImm | Op::LdbImm | Op::Jmp | Op::Jeq)
    }
    pub fn mnemonic(self) -> &'static str {
        match self {
            Op::Reset => "RESET",
            Op::LdaImm => "LDA",
            Op::LdbImm => "LDB",
            Op::LdaA => "LDA (A)",
            Op::LdbA => "LDB (A)",
            Op::StbA => "STB (A)",
            Op::AddA => "ADDA",
            Op::AddB => "ADDB",
            Op::AndA => "ANDA",
            Op::AndB => "ANDB",
            Op::NorA => "NORA",
            Op::NorB => "NORB",
            Op::MovAB => "MOVAB",
            Op::Jmp => "JMP",
            Op::Jeq => "JEQ",
            Op::Nop => "NOP",
            Op::Halt => "HALT",
        }
    }
}

/// Opcode of an instruction word.
pub fn opcode(word: u16) -> u8 {
    (word >> 11) as u8
}

// ---------------------------------------------------------------------------
// The microcode

/// The microword sequence for an opcode and a value of NE, one word per
/// clock, in the order the pipeline register executes them (step k's
/// word is at ROM address {op, ne, k}).  Two rules shape them: the
/// driver of D never changes between consecutive words (an idle word in
/// between lets the old driver float for a clock), and a MEMRD or WE
/// word is followed by a word with the same Addr driver, with ALUOE kept
/// after WE, so every hold time is a clock long.  The word after a
/// fetch is a nop: the ROM's word for the next step is already in
/// flight when the IR changes.
pub fn sequence(op: Op, ne: bool) -> Vec<u16> {
    // Every access at A starts with a word that only puts A on the bus:
    // the address (and, for the ALU, the operand) is valid a clock before
    // anything strobes or samples it.
    let alu = |f: u16, ld: u16| vec![ADRV, ADRV | ALUOE | f | ld, 0, PCINC, FETCH, 0];
    match op {
        Op::Reset => vec![FETCH, 0],
        Op::LdaImm => vec![PCINC, PCDRV | MEMRD | ALD, PCINC, FETCH, 0],
        Op::LdbImm => vec![PCINC, PCDRV | MEMRD | BLD, PCINC, FETCH, 0],
        Op::LdaA => vec![ADRV, ADRV | MEMRD | ALD, ADRV, PCINC, FETCH, 0],
        Op::LdbA => vec![ADRV, ADRV | MEMRD | BLD, ADRV, PCINC, FETCH, 0],
        Op::StbA => vec![ADRV, ADRV | ALUOE | F_PASSB | WE, ADRV | ALUOE | F_PASSB, 0, PCINC, FETCH, 0],
        Op::AddA => alu(F_ADD, ALD),
        Op::AddB => alu(F_ADD, BLD),
        Op::AndA => alu(F_AND, ALD),
        Op::AndB => alu(F_AND, BLD),
        Op::NorA => alu(F_NOR, ALD),
        Op::NorB => alu(F_NOR, BLD),
        Op::MovAB => alu(F_PASSB, ALD),
        Op::Jmp => vec![PCINC, PCDRV | MEMRD | PCLD, 0, FETCH, 0],
        // The condition: word 0 puts A on the bus so that NE is valid and
        // the NEL flop latches it at the word's end; the ROM read for step
        // 2, during word 1, is the one that sees the fresh NEL, so the two
        // variants differ at step 2 only.
        Op::Jeq if !ne => vec![ADRV, PCINC, PCDRV | MEMRD | PCLD, 0, FETCH, 0],
        Op::Jeq => vec![ADRV, PCINC, PCINC, 0, FETCH, 0],
        Op::Nop => vec![PCINC, FETCH, 0],
        Op::Halt => vec![],
    }
}

/// ROM address of a microword.
pub fn ucode_addr(op: u8, ne: bool, step: u8) -> usize {
    (op as usize) << 5 | (ne as usize) << 4 | (step as usize & 15)
}

/// The whole table: 1024 words, unused opcodes are HALT (all zero).
pub fn microcode() -> Vec<u16> {
    let mut rom = vec![0u16; 1024];
    for op in Op::ALL {
        for ne in [false, true] {
            let s = sequence(op, ne);
            assert!(s.len() <= 16, "{op:?}: {} words", s.len());
            check_sequence(op, &s);
            for (k, w) in s.iter().enumerate() {
                rom[ucode_addr(op as u8, ne, k as u8)] = *w;
            }
        }
    }
    rom
}

fn driver(w: u16) -> u16 {
    w & (MEMRD | ALUOE)
}

/// The ordering rules, plus at most one driver per word.
fn check_sequence(op: Op, s: &[u16]) {
    for (k, &w) in s.iter().enumerate() {
        assert!(driver(w).count_ones() <= 1, "{op:?} step {k}: two drivers on D");
        assert!(w & PCDRV == 0 || w & ADRV == 0, "{op:?} step {k}: two drivers on Addr");
        assert!(w & MEMRD == 0 || w & WE == 0, "{op:?} step {k}: read and write");
        assert!(w & (MEMRD | WE | ALUOE) == 0 || w & (PCDRV | ADRV) != 0, "{op:?} step {k}: a bus access with nobody on Addr");
        if w & ADRV != 0 && w & (MEMRD | WE | ALUOE) != 0 {
            assert!(k > 0 && s[k - 1] & ADRV != 0, "{op:?} step {k}: an access at A without A on the bus in the word before");
        }
        if k + 1 < s.len() {
            let n = s[k + 1];
            if driver(w) != 0 && driver(n) != 0 && driver(w) != driver(n) {
                panic!("{op:?} step {k}: driver of D changes without an idle word");
            }
            if (w & PCDRV != 0 && n & ADRV != 0) || (w & ADRV != 0 && n & PCDRV != 0) {
                panic!("{op:?} step {k}: driver of Addr changes without an idle word");
            }
            if w & (MEMRD | WE) != 0 && w & ADRV != 0 {
                assert!(n & ADRV != 0, "{op:?} step {k}: address not held after a memory access at A");
            }
            if w & WE != 0 {
                assert!(n & ALUOE != 0, "{op:?} step {k}: data not held after WE");
            }
        }
        if w & IRLD != 0 {
            assert!(k + 1 < s.len() && s[k + 1] == 0, "{op:?} step {k}: the word after a fetch must be a nop");
        }
    }
}

/// A microword as text: `PCDRV MEMRD ALD`.
pub fn word_text(w: u16) -> String {
    let mut parts: Vec<&str> = Vec::new();
    for (i, name) in BIT_NAMES.iter().enumerate() {
        if i == 9 || i == 10 {
            continue;
        }
        if w >> i & 1 == 1 {
            parts.push(name);
        }
    }
    if w & ALUOE != 0 {
        parts.push(match w & F_PASSB {
            F_ADD => "add",
            F_AND => "and",
            F_NOR => "nor",
            _ => "passB",
        });
    }
    if parts.is_empty() { "nop".into() } else { parts.join(" ") }
}

// ---------------------------------------------------------------------------
// The assembler

/// A program: words and labels.
pub struct Program {
    pub words: Vec<u16>,
    pub labels: BTreeMap<String, u16>,
}

/// Assemble.  One instruction per line; `label:`; `; comment`;
/// immediates are numbers (`0x..` or decimal), labels, or `&rN` for the
/// address of register N.  `.word v` places data.
///
/// ```text
///     LDA &r1        ; A = address of r1
///     LDB (A)        ; B = r1
///     LDA 0x8040
///     STB (A)
/// loop:
///     JMP loop
/// ```
pub fn assemble(src: &str) -> Result<Program, String> {
    struct Line<'a> {
        op: Option<Op>,
        imm: Option<&'a str>,
        data: Option<&'a str>,
    }
    let mut lines: Vec<Line> = Vec::new();
    let mut labels = BTreeMap::new();
    let mut pc: u16 = 0;
    for (ln, raw) in src.lines().enumerate() {
        let text = raw.split(';').next().unwrap().trim();
        if text.is_empty() {
            continue;
        }
        let mut text = text;
        if let Some(i) = text.find(':') {
            let name = text[..i].trim();
            if labels.insert(name.to_string(), pc).is_some() {
                return Err(format!("line {}: label {name} twice", ln + 1));
            }
            text = text[i + 1..].trim();
            if text.is_empty() {
                continue;
            }
        }
        let (mn, arg) = match text.split_once(char::is_whitespace) {
            Some((m, a)) => (m, Some(a.trim())),
            None => (text, None),
        };
        let mn = mn.to_ascii_uppercase();
        let line = match (mn.as_str(), arg) {
            (".WORD", Some(v)) => Line { op: None, imm: None, data: Some(v) },
            ("LDA", Some("(A)")) => Line { op: Some(Op::LdaA), imm: None, data: None },
            ("LDB", Some("(A)")) => Line { op: Some(Op::LdbA), imm: None, data: None },
            ("STB", Some("(A)")) => Line { op: Some(Op::StbA), imm: None, data: None },
            ("LDA", Some(v)) => Line { op: Some(Op::LdaImm), imm: Some(v), data: None },
            ("LDB", Some(v)) => Line { op: Some(Op::LdbImm), imm: Some(v), data: None },
            ("JMP", Some(v)) => Line { op: Some(Op::Jmp), imm: Some(v), data: None },
            ("JEQ", Some(v)) => Line { op: Some(Op::Jeq), imm: Some(v), data: None },
            ("ADDA", None) => Line { op: Some(Op::AddA), imm: None, data: None },
            ("ADDB", None) => Line { op: Some(Op::AddB), imm: None, data: None },
            ("ANDA", None) => Line { op: Some(Op::AndA), imm: None, data: None },
            ("ANDB", None) => Line { op: Some(Op::AndB), imm: None, data: None },
            ("NORA", None) => Line { op: Some(Op::NorA), imm: None, data: None },
            ("NORB", None) => Line { op: Some(Op::NorB), imm: None, data: None },
            ("MOVAB", None) => Line { op: Some(Op::MovAB), imm: None, data: None },
            ("NOP", None) => Line { op: Some(Op::Nop), imm: None, data: None },
            ("HALT", None) => Line { op: Some(Op::Halt), imm: None, data: None },
            _ => return Err(format!("line {}: cannot parse {text:?}", ln + 1)),
        };
        pc += 2 * (1 + line.imm.is_some() as u16);
        lines.push(line);
    }
    let value = |s: &str| -> Result<u16, String> {
        if let Some(r) = s.strip_prefix("&r") {
            let n: u16 = r.parse().map_err(|_| format!("bad register {s}"))?;
            return Ok(0x8000 + 2 * n);
        }
        if let Some(h) = s.strip_prefix("0x") {
            return u16::from_str_radix(h, 16).map_err(|_| format!("bad number {s}"));
        }
        if let Some(n) = s.strip_prefix('-') {
            let v: i32 = n.parse().map_err(|_| format!("bad number {s}"))?;
            return Ok((-v) as u16);
        }
        if let Ok(v) = s.parse::<u16>() {
            return Ok(v);
        }
        labels.get(s).copied().ok_or_else(|| format!("unknown label {s}"))
    };
    let mut words = Vec::new();
    for l in &lines {
        if let Some(d) = l.data {
            words.push(value(d)?);
            continue;
        }
        let op = l.op.unwrap();
        words.push(op.word());
        if let Some(i) = l.imm {
            words.push(value(i)?);
        }
    }
    Ok(Program { words, labels })
}

// ---------------------------------------------------------------------------
// The reference interpreter

pub const FLASH_WORDS: usize = 1 << 14;
pub const RAM_WORDS: usize = 1 << 13;

/// The programmer's model: A, B, PC, the memories, the UART's registers.
pub struct Iss {
    pub a: u16,
    pub b: u16,
    pub pc: u16,
    pub flash: Vec<u16>,
    pub ram: Vec<u16>,
    pub uart: Core,
    pub halted: bool,
    pub retired: u64,
}

impl Iss {
    pub fn new(program: &[u16]) -> Iss {
        let mut flash = vec![0u16; FLASH_WORDS];
        flash[..program.len()].copy_from_slice(program);
        Iss { a: 0, b: 0, pc: 0, flash, ram: vec![0; RAM_WORDS], uart: Core::default(), halted: false, retired: 0 }
    }
    pub fn read(&mut self, addr: u16) -> u16 {
        match addr >> 14 {
            0 | 1 => self.flash[(addr as usize >> 1) & (FLASH_WORDS - 1)],
            2 => self.ram[(addr as usize >> 1) & (RAM_WORDS - 1)],
            _ => {
                // The reference has no line time: whatever the far end
                // queued has arrived by the time the program looks.
                let v = self.uart.read((addr >> 1) as u8 & 7) as u16;
                while self.uart.rx_deliver() {}
                v
            }
        }
    }
    pub fn write(&mut self, addr: u16, v: u16) {
        match addr >> 14 {
            2 => self.ram[(addr as usize >> 1) & (RAM_WORDS - 1)] = v,
            3 => {
                self.uart.write((addr >> 1) as u8 & 7, v as u8);
                // The reference has no line time: what is written is sent.
                while self.uart.tx_start() {
                    self.uart.tx_done();
                }
            }
            _ => {}
        }
    }
    pub fn reg(&self, n: usize) -> u16 {
        self.ram[n]
    }
    /// Execute one instruction.
    pub fn step(&mut self) -> Result<(), String> {
        if self.halted {
            return Ok(());
        }
        let w = self.read(self.pc);
        let op = Op::from_code(opcode(w)).ok_or_else(|| format!("pc {:#06x}: bad opcode {:#06x}", self.pc, w))?;
        let imm = if op.has_imm() { self.read(self.pc.wrapping_add(2)) } else { 0 };
        let next = self.pc.wrapping_add(if op.has_imm() { 4 } else { 2 });
        let (a, b) = (self.a, self.b);
        self.pc = next;
        match op {
            Op::Reset | Op::Nop => {}
            Op::LdaImm => self.a = imm,
            Op::LdbImm => self.b = imm,
            Op::LdaA => self.a = self.read(a),
            Op::LdbA => self.b = self.read(a),
            Op::StbA => self.write(a, b),
            Op::AddA => self.a = a.wrapping_add(b),
            Op::AddB => self.b = a.wrapping_add(b),
            Op::AndA => self.a = a & b,
            Op::AndB => self.b = a & b,
            Op::NorA => self.a = !(a | b),
            Op::NorB => self.b = !(a | b),
            Op::MovAB => self.a = b,
            Op::Jmp => self.pc = imm,
            Op::Jeq => {
                if a == b {
                    self.pc = imm;
                }
            }
            Op::Halt => {
                self.pc = self.pc.wrapping_sub(2);
                self.halted = true;
            }
        }
        self.retired += 1;
        Ok(())
    }
    pub fn run(&mut self, max: u64) -> Result<(), String> {
        for _ in 0..max {
            if self.halted {
                return Ok(());
            }
            self.step()?;
        }
        Err(format!("not halted after {max} instructions, pc {:#06x}", self.pc))
    }
}

// ---------------------------------------------------------------------------
// The GAL equations

fn n(prefix: &str, i: usize) -> String {
    format!("{prefix}{i}")
}
fn l(s: &str) -> SLit {
    lit(s)
}
fn nl_(s: &str) -> SLit {
    nlit(s)
}
fn strs(v: &[String]) -> Vec<&str> {
    v.iter().map(String::as_str).collect()
}

/// Net of A latch bit `i`: bit 0 goes to the ALU only, the rest are the
/// address bus.
fn a_net(i: usize) -> String {
    if i == 0 { "A0".into() } else { n("ADDR", i) }
}

/// `Q := LD & D + !LD & Q`.
fn latch(out: &str, d: &str, ld: &str) -> Eq {
    Eq::sop(out, Mode::Reg, vec![vec![l(ld), l(d)], vec![nl_(ld), l(out)]])
}

/// A synchronous reset: every product term gated with !RESET, so the
/// register clears at the next edge while RESET is high and RESET is an
/// ordinary data input with an ordinary setup time.  (The 22V10's
/// asynchronous reset would race the clock edge that RESET itself comes
/// from.)
fn with_reset(mut e: Eq) -> Eq {
    for t in &mut e.terms {
        t.push(nl_("RESET"));
    }
    e
}

/// The A latch, two chips: `a0` bits 0..7, `a1` bits 8..15.  Bits 1..15
/// drive Addr while PCDRV is low.
fn a_specs() -> Vec<GalSpec> {
    (0..2)
        .map(|half| {
            let eqs = (8 * half..8 * half + 8)
                .map(|i| {
                    let e = latch(&a_net(i), &n("D", i), "ALD");
                    if i == 0 { e } else { e.with_oe(vec![l("ADRV")]) }
                })
                .collect();
            GalSpec { name: format!("a{half}"), clk: Some("CLK".into()), ar: None, eqs }
        })
        .collect()
}

/// The B latch: `b0` bits 0..7 plus the reset synchroniser, `b1` bits
/// 8..15.
fn b_specs() -> Vec<GalSpec> {
    let mut b0: Vec<Eq> = (0..8).map(|i| latch(&n("B", i), &n("D", i), "BLD")).collect();
    b0.push(Eq::sop("RS1", Mode::Reg, vec![vec![l("RST_n")]]).active_low().sync());
    // RESET is high at power-up (Q = 0 behind an active-low pin) and
    // follows RS1 a clock later.
    b0.push(Eq::sop("RESET", Mode::Reg, vec![vec![nl_("RS1")]]).active_low());
    let b1: Vec<Eq> = (8..16).map(|i| latch(&n("B", i), &n("D", i), "BLD")).collect();
    vec![
        GalSpec { name: "b0".into(), clk: Some("CLK".into()), ar: None, eqs: b0 },
        GalSpec { name: "b1".into(), clk: Some("CLK".into()), ar: None, eqs: b1 },
    ]
}

/// The pipeline register: `mir0` holds M0..M7, `mir1` M8..M15.  The
/// pins the memories want are active low.
fn mir_specs() -> Vec<GalSpec> {
    let names = ["PCDRV", "MEMRD_n", "ALUOE", "WE_n", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV", "AUX0", "AUX1", "AUX2", "AUX3"];
    let eq = |i: usize| {
        let e = with_reset(Eq::sop(names[i], Mode::Reg, vec![vec![l(&n("M", i))]]));
        if names[i].ends_with("_n") { e.active_low() } else { e }
    };
    vec![
        GalSpec { name: "mir0".into(), clk: Some("CLK".into()), ar: None, eqs: (0..8).map(eq).collect() },
        GalSpec { name: "mir1".into(), clk: Some("CLK".into()), ar: None, eqs: (8..16).map(eq).collect() },
    ]
}

/// The IR and the step counter, one chip.
fn seq_spec() -> GalSpec {
    let mut eqs: Vec<Eq> = (0..5).map(|i| with_reset(latch(&n("IR", i), &n("D", 11 + i), "IRLD"))).collect();
    // A 4-bit counter cleared by IRLD: bit k toggles when all lower bits
    // are 1.
    let steps: Vec<String> = (0..4).map(|i| n("STEP", i)).collect();
    for k in 0..4 {
        let mut ins: Vec<&str> = vec!["IRLD"];
        ins.extend(steps[..=k].iter().map(String::as_str));
        eqs.push(with_reset(Eq::table_pos(&steps[k], Mode::Reg, &ins, move |m| {
            let irld = m & 1 == 1;
            let s = m >> 1;
            let lower_all_one = (0..k).all(|j| s >> j & 1 == 1);
            let q = s >> k & 1 == 1;
            Some(!irld && (q ^ lower_all_one))
        })));
    }
    // NEL: the ALU's not-equal, latched at the end of every word in
    // which A is on the bus (NE is only meaningful then), held otherwise.
    // It is the microcode ROM's condition address bit.
    eqs.push(with_reset(Eq::sop("NEL", Mode::Reg, vec![vec![l("ADRV"), l("NE")], vec![nl_("ADRV"), l("NEL")]])));
    GalSpec { name: "seq0".into(), clk: Some("CLK".into()), ar: None, eqs }
}

/// The PC: `pc0` bits 1..8 and the carry into bit 9, `pc1` bits 9..15.
/// Counts a word on PCINC, loads D on PCLD (which wins), drives Addr
/// while PCDRV is high.
fn pc_specs() -> Vec<GalSpec> {
    // Bit k of a counter whose lower bits are `lower` (all must be 1 for
    // a toggle) and whose count enable is `inc`.
    let bit = |k: usize, lower: &[String], inc: &str| {
        let out = n("ADDR", k);
        let d = n("D", k);
        let mut ins: Vec<String> = vec!["PCLD".into(), d.clone(), inc.into(), out.clone()];
        ins.extend(lower.iter().cloned());
        let nl = lower.len();
        with_reset(Eq::table_pos(&out, Mode::Reg, &strs(&ins), move |m| {
            let pcld = m & 1 == 1;
            let dk = m >> 1 & 1 == 1;
            let inc = m >> 2 & 1 == 1;
            let q = m >> 3 & 1 == 1;
            let lower_all = (0..nl).all(|j| m >> (4 + j) & 1 == 1);
            Some(if pcld { dk } else { q ^ (inc && lower_all) })
        }))
        .with_oe(vec![l("PCDRV")])
    };
    let low: Vec<String> = (1..=8).map(|i| n("ADDR", i)).collect();
    let mut pc0: Vec<Eq> = (1..=8).map(|k| bit(k, &low[..k - 1], "PCINC")).collect();
    // CO: the count carries into bit 9.
    let mut co_ins: Vec<String> = vec!["PCINC".into()];
    co_ins.extend(low.iter().cloned());
    pc0.push(Eq::table_pos("CO", Mode::Comb, &strs(&co_ins), |m| Some(m == 0x1FF)));
    let high: Vec<String> = (9..=15).map(|i| n("ADDR", i)).collect();
    let pc1: Vec<Eq> = (9..=15).map(|k| bit(k, &high[..k - 9], "CO")).collect();
    vec![
        GalSpec { name: "pc0".into(), clk: Some("CLK".into()), ar: None, eqs: pc0 },
        GalSpec { name: "pc1".into(), clk: Some("CLK".into()), ar: None, eqs: pc1 },
    ]
}

/// The ALU: four 4-bit slices with ripple carry inside and between, the
/// function folded into every sum term, and a not-equal chain beside
/// the carry chain.  Slice 0 also divides CLK2X into CLK.
fn alu_specs() -> Vec<GalSpec> {
    (0..4)
        .map(|s| {
            let a: Vec<String> = (0..4).map(|i| a_net(4 * s + i)).collect();
            let b: Vec<String> = (0..4).map(|i| n("B", 4 * s + i)).collect();
            let cin: Option<String> = (s > 0).then(|| n("COUT", s - 1));
            let nein: Option<String> = (s > 0).then(|| n("NE", s - 1));
            let carries: Vec<String> = (1..4).map(|i| format!("ALU{s}_C{i}")).collect();
            let mut eqs = Vec::new();
            // Carry into bit i: cin for bit 0, else the slice's own.
            let carry_in = |i: usize| -> Option<String> { if i == 0 { cin.clone() } else { Some(carries[i - 1].clone()) } };
            for i in 0..4 {
                let mut ins: Vec<String> = vec![a[i].clone(), b[i].clone(), "F1".into(), "F0".into()];
                let has_c = carry_in(i).is_some();
                if let Some(c) = carry_in(i) {
                    ins.push(c);
                }
                // Sum bit onto the data bus.
                eqs.push(
                    Eq::table(&n("D", 4 * s + i), Mode::Comb, &strs(&ins), move |m| {
                        let (ai, bi, f1, f0) = (m & 1 == 1, m >> 1 & 1 == 1, m >> 2 & 1 == 1, m >> 3 & 1 == 1);
                        let c = has_c && m >> 4 & 1 == 1;
                        Some(match (f1, f0) {
                            (false, false) => ai ^ bi ^ c,
                            (false, true) => ai & bi,
                            (true, false) => !(ai | bi),
                            (true, true) => bi,
                        })
                    })
                    .with_oe(vec![l("ALUOE")]),
                );
                // Carry out of bit i (add only).
                let cout_name = if i < 3 { carries[i].clone() } else { n("COUT", s) };
                if i < 3 || s < 3 {
                    eqs.push(Eq::table(&cout_name, Mode::Comb, &strs(&ins), move |m| {
                        let (ai, bi, f1, f0) = (m & 1 == 1, m >> 1 & 1 == 1, m >> 2 & 1 == 1, m >> 3 & 1 == 1);
                        let c = has_c && m >> 4 & 1 == 1;
                        Some(!f1 && !f0 && ((ai && bi) || (ai && c) || (bi && c)))
                    }));
                }
            }
            // The not-equal chain.
            let mut ne_ins: Vec<String> = Vec::new();
            if let Some(x) = &nein {
                ne_ins.push(x.clone());
            }
            ne_ins.extend(a.iter().cloned());
            ne_ins.extend(b.iter().cloned());
            let has_nein = nein.is_some();
            let ne_out = if s == 3 { "NE".to_string() } else { n("NE", s) };
            eqs.push(Eq::table(&ne_out, Mode::Comb, &strs(&ne_ins), move |m| {
                let off = has_nein as u32;
                let prev = has_nein && m & 1 == 1;
                let av = m >> off & 0xF;
                let bv = m >> (off + 4) & 0xF;
                Some(prev || av != bv)
            }));
            let clk = if s == 0 {
                eqs.push(Eq::sop("CLK", Mode::Reg, vec![vec![nl_("CLK")]]));
                Some("CLK2X".to_string())
            } else {
                None
            };
            GalSpec { name: format!("alu{s}"), clk, ar: None, eqs }
        })
        .collect()
}

/// Every GAL, with a check that each fits its chip.
pub fn gal_specs() -> Vec<GalSpec> {
    let mut v = Vec::new();
    v.push(seq_spec());
    v.extend(mir_specs());
    v.extend(pc_specs());
    v.extend(a_specs());
    v.extend(b_specs());
    v.extend(alu_specs());
    for s in &v {
        if let Err(e) = s.fits() {
            panic!("{e}");
        }
    }
    v
}

// ---------------------------------------------------------------------------
// The netlist and board file

pub const CLK2X_HZ: f64 = 9_216_000.0;
/// One CLK2X period in ps.
pub const PERIOD2X: Time = 108_507;
/// One CLK period in ps.
pub const PERIOD: Time = 2 * PERIOD2X;

fn block_of(name: &str) -> (&'static str, &'static str) {
    let prefix: String = name.chars().take_while(|c| c.is_ascii_alphabetic()).collect();
    match prefix.as_str() {
        "seq" => ("IR + step counter", "Control"),
        "uc" => ("Microcode ROM", "Control"),
        "mir" => ("Pipeline register", "Control"),
        "pc" => ("PC", "Datapath"),
        "a" => ("A", "Datapath"),
        "b" => ("B", "Datapath"),
        "alu" => ("ALU", "Datapath"),
        "rom" => ("Program flash", "Memory"),
        "ram" => ("SRAM", "Memory"),
        "uart" => ("UART", "I/O"),
        "xcvr" | "c" | "j" => ("Serial port", "I/O"),
        "osc" => ("Clock", "I/O"),
        "rst" | "sw" => ("Reset", "I/O"),
        _ => ("?", "?"),
    }
}

fn meta(part: &str, package: &str, name: &str, role: Option<String>, model: Model) -> ChipMeta {
    let (block, stage) = block_of(name);
    ChipMeta { part: part.into(), package: package.into(), block: block.into(), stage: stage.into(), role, model }
}

pub fn layout() -> Vec<Column> {
    let col = |title: &str, width: u32, blocks: &[&str]| Column { title: title.into(), width, blocks: blocks.iter().map(|b| b.to_string()).collect(), reg: false };
    vec![
        col("Control", 300, &["IR + step counter", "Microcode ROM", "Pipeline register"]),
        col("Datapath", 330, &["PC", "A", "B", "ALU"]),
        col("Memory", 240, &["Program flash", "SRAM"]),
        col("I/O", 300, &["UART", "Serial port", "Clock", "Reset"]),
    ]
}

/// Every chip and every connection; no stimulus.
pub fn build_netlist() -> Netlist {
    let mut nl = Netlist::new();
    for spec in gal_specs() {
        let (id, pins) = galpack::instantiate(&mut nl, &spec);
        let model = Model::Gal { clk: spec.clk.clone(), ar: spec.ar.clone(), eqs: spec.eqs.clone(), pins };
        nl.set_meta(id, meta("ATF22V10C-7PX", "DIP-24", &spec.name, None, model));
    }
    let clk2x = nl.net("CLK2X");
    nl.set_net_role(clk2x, "clk");
    let reset = nl.net("RESET");
    nl.set_net_role(reset, "reset");
    let gnd = nl.net("GND");
    let vcc = nl.net("VCC");
    nl.tie(gnd, Level::L);
    nl.tie(vcc, Level::H);

    // The oscillator (an SMD 4-pin module: 1 EN, 2 GND, 3 OUT, 4 VCC).
    let osc = nl.add_chip("osc0", Passive::new(vec![(1, "EN".into()), (2, "GND".into()), (3, "OUT".into()), (4, "VCC".into())]));
    nl.set_meta(osc, meta("SG-8018CA 9.216MHz", "3225", "osc0", None, Model::Passive));
    nl.connect(vcc, osc, 1);
    nl.connect(gnd, osc, 2);
    nl.connect(clk2x, osc, 3);
    nl.connect(vcc, osc, 4);

    // Reset supervisor; MR# is the button (pulled up in the part).
    let sup = nl.add_chip("rst0", ResetSupervisor::new(0, 0));
    nl.set_meta(sup, meta("MAX811LEUS+T", "SOT-143", "rst0", Some("supervisor".into()), Model::Supervisor));
    let rst_n = nl.net("RST_n");
    nl.connect(rst_n, sup, 2);
    let mr_n = nl.net("MR_n");
    nl.connect(mr_n, sup, 3);
    nl.pull(mr_n, Level::H);
    nl.set_net_role(mr_n, "reset_button");
    nl.connect(gnd, sup, 1);
    nl.connect(vcc, sup, 4);

    // Program flash: two lanes, A0..A13 from Addr1..Addr14, bank jumpers
    // on A16..A18, CE# from Addr15, OE# from MEMRD_n.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("rom{lane}"), Rom::sst39sf040_70());
        nl.set_meta(c, meta("SST39SF040-70-4C-PHE", "DIP-32", &format!("rom{lane}"), Some(format!("rom:{lane}")), Model::Rom { tacc_ns: 70, toe_ns: 35, tdf_ns: 25 }));
        for j in 0..19u32 {
            let net = match j {
                0..=13 => nl.net(&n("ADDR", j as usize + 1)),
                16..=18 => nl.net(&n("PBANK", j as usize - 16)),
                _ => gnd,
            };
            nl.connect(net, c, rom_pin_of(RomPin::A(j as u8)));
        }
        for b in 0..8 {
            let net = nl.net(&n("D", 8 * lane + b));
            nl.connect(net, c, rom_pin_of(RomPin::Dq(b as u8)));
        }
        let a15 = nl.net("ADDR15");
        nl.connect(a15, c, rom_pin_of(RomPin::CeN));
        let memrd = nl.net("MEMRD_n");
        nl.connect(memrd, c, rom_pin_of(RomPin::OeN));
        nl.connect(vcc, c, rom_pin_of(RomPin::WeN));
        nl.connect(gnd, c, rom_pin_of(RomPin::Gnd));
        nl.connect(vcc, c, rom_pin_of(RomPin::Vcc));
    }
    // While neither the PC nor A drives Addr (the idle word at every
    // handoff) the selects are defined: Addr15 and Addr14 pulled low
    // select the flash, whose OE# is high then.
    for name in ["ADDR15", "ADDR14"] {
        let net = nl.net(name);
        nl.pull(net, Level::L);
    }
    for i in 0..3 {
        let net = nl.net(&n("PBANK", i));
        nl.tie(net, Level::L);
        let net = nl.net(&n("UBANK", i));
        nl.tie(net, Level::L);
    }
    // Microcode ROM: two lanes on private wires.  A0..A3 = STEP, A4 = NE,
    // A5..A9 = IR, bank jumpers on A10..A12, the rest grounded; always
    // selected and enabled.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("uc{lane}"), Rom::sst39sf040_70());
        nl.set_meta(c, meta("SST39SF040-70-4C-PHE", "DIP-32", &format!("uc{lane}"), Some(format!("ucode:{lane}")), Model::Rom { tacc_ns: 70, toe_ns: 35, tdf_ns: 25 }));
        for j in 0..19u32 {
            let net = match j {
                0..=3 => nl.net(&n("STEP", j as usize)),
                4 => nl.net("NEL"),
                5..=9 => nl.net(&n("IR", j as usize - 5)),
                10..=12 => nl.net(&n("UBANK", j as usize - 10)),
                _ => gnd,
            };
            nl.connect(net, c, rom_pin_of(RomPin::A(j as u8)));
        }
        for b in 0..8 {
            let net = nl.net(&n("M", 8 * lane + b));
            nl.connect(net, c, rom_pin_of(RomPin::Dq(b as u8)));
        }
        nl.connect(gnd, c, rom_pin_of(RomPin::CeN));
        nl.connect(gnd, c, rom_pin_of(RomPin::OeN));
        nl.connect(vcc, c, rom_pin_of(RomPin::WeN));
        nl.connect(gnd, c, rom_pin_of(RomPin::Gnd));
        nl.connect(vcc, c, rom_pin_of(RomPin::Vcc));
    }
    // SRAM: two lanes, A0..A12 from Addr1..Addr13, CE2 = Addr15,
    // CE1# = Addr14, OE# = MEMRD_n, WE# = WE_n.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("ram{lane}"), As7c164a::with_timing(as7c164a::Timing::grade_15()));
        nl.set_meta(c, meta("AS7C164A-15PCN", "DIP-28", &format!("ram{lane}"), Some(format!("ram:{lane}")), Model::Sram8k { timing: "AS7C164A-15".into() }));
        for a in 0..13 {
            let net = nl.net(&n("ADDR", a + 1));
            nl.connect(net, c, sram8k_pin_of(Sram8kPin::A(a as u8)));
        }
        for b in 0..8 {
            let net = nl.net(&n("D", 8 * lane + b));
            nl.connect(net, c, sram8k_pin_of(Sram8kPin::Dq(b as u8)));
        }
        let a15 = nl.net("ADDR15");
        let a14 = nl.net("ADDR14");
        nl.connect(a15, c, sram8k_pin_of(Sram8kPin::Ce2));
        nl.connect(a14, c, sram8k_pin_of(Sram8kPin::CeN));
        let memrd = nl.net("MEMRD_n");
        nl.connect(memrd, c, sram8k_pin_of(Sram8kPin::OeN));
        let we = nl.net("WE_n");
        nl.connect(we, c, sram8k_pin_of(Sram8kPin::WeN));
        nl.connect(gnd, c, sram8k_pin_of(Sram8kPin::Vss));
        nl.connect(vcc, c, sram8k_pin_of(Sram8kPin::Vcc));
    }
    // UART: D0..D7, A0..A2 from Addr1..Addr3, CS0 = Addr14, CS1 = Addr15,
    // CS2# ADS# RD2 WR2 low, RD1# = MEMRD_n, WR1# = WE_n, MR = RESET,
    // XIN = CLK2X, BAUDOUT# -> RCLK, DTR# -> DSR# + DCD#, RI# high,
    // RTS#/CTS# and SOUT/SIN through the transceiver.
    {
        let c = nl.add_chip("uart0", Uart16550::new(BusTiming::tl16c550c(), CLK2X_HZ));
        nl.set_meta(c, meta("TL16C550DPTR", "LQFP-48", "uart0", Some("uart".into()), Model::Uart { xin_hz: CLK2X_HZ }));
        for i in 0..8u8 {
            let net = nl.net(&n("D", i as usize));
            nl.connect(net, c, uart_pin_of(UartPin::D(i)));
        }
        for i in 0..3u8 {
            let net = nl.net(&n("ADDR", i as usize + 1));
            nl.connect(net, c, uart_pin_of(UartPin::A(i)));
        }
        for (net, p) in [("ADDR14", UartPin::Cs0), ("ADDR15", UartPin::Cs1), ("MEMRD_n", UartPin::Rd1N), ("WE_n", UartPin::Wr1N), ("RESET", UartPin::Mr), ("SIN", UartPin::Sin), ("SOUT", UartPin::Sout), ("CLK2X", UartPin::Xin)] {
            let net = nl.net(net);
            nl.connect(net, c, uart_pin_of(p));
        }
        for p in [UartPin::Cs2N, UartPin::AdsN, UartPin::Rd2, UartPin::Wr2] {
            nl.connect(gnd, c, uart_pin_of(p));
        }
        nl.connect(gnd, c, uart_pin_of(UartPin::Gnd));
        nl.connect(vcc, c, uart_pin_of(UartPin::Vcc));
        for (net, pins) in [("UDTR_n", vec![33, 39, 40]), ("UBAUD", vec![12, 5])] {
            let net = nl.net(net);
            for p in pins {
                nl.connect(net, c, p);
            }
        }
        nl.connect(vcc, c, 41);
        let rts = nl.net("URTS_n");
        let cts = nl.net("UCTS_n");
        nl.connect(rts, c, 32);
        nl.connect(cts, c, 38);
        let sout = nl.net("SOUT");
        let sin = nl.net("SIN");
        // RS-232 transceiver (SP3232, TSSOP-16): 1 C1+, 2 V+, 3 C1-,
        // 4 C2+, 5 C2-, 6 V-, 7 T2OUT, 8 R2IN, 9 R2OUT, 10 T2IN, 11 T1IN,
        // 12 R1OUT, 13 R1IN, 14 T1OUT, 15 GND, 16 VCC.
        let xc_names = ["C1+", "V+", "C1-", "C2+", "C2-", "V-", "T2OUT", "R2IN", "R2OUT", "T2IN", "T1IN", "R1OUT", "R1IN", "T1OUT", "GND", "VCC"];
        let xc = nl.add_chip("xcvr0", Passive::new(xc_names.iter().enumerate().map(|(i, s)| (i + 1, s.to_string())).collect()));
        nl.set_meta(xc, meta("SP3232EEY-L/TR", "TSSOP-16", "xcvr0", None, Model::Passive));
        nl.connect(sout, xc, 11);
        nl.connect(sin, xc, 12);
        nl.connect(rts, xc, 10);
        nl.connect(cts, xc, 9);
        let tx = nl.net("RS232_TX");
        let rx = nl.net("RS232_RX");
        let rts232 = nl.net("RS232_RTS");
        let cts232 = nl.net("RS232_CTS");
        nl.connect(tx, xc, 14);
        nl.connect(rx, xc, 13);
        nl.connect(rts232, xc, 7);
        nl.connect(cts232, xc, 8);
        nl.connect(gnd, xc, 15);
        nl.connect(vcc, xc, 16);
        for (name, a, b) in [("c1", 1, 3), ("c2", 4, 5)] {
            let cap = nl.add_chip(name, Passive::new(vec![(1, "1".into()), (2, "2".into())]));
            nl.set_meta(cap, meta("100nF 0603 X7R", "0603", name, None, Model::Passive));
            let na = nl.net(&format!("XC_{name}A"));
            let nb = nl.net(&format!("XC_{name}B"));
            nl.connect(na, xc, a);
            nl.connect(nb, xc, b);
            nl.connect(na, cap, 1);
            nl.connect(nb, cap, 2);
        }
        for (name, pin, rail_net) in [("c3", 2, "XC_VP"), ("c4", 6, "XC_VM")] {
            let cap = nl.add_chip(name, Passive::new(vec![(1, "1".into()), (2, "2".into())]));
            nl.set_meta(cap, meta("100nF 0603 X7R", "0603", name, None, Model::Passive));
            let net = nl.net(rail_net);
            nl.connect(net, xc, pin);
            nl.connect(net, cap, 1);
            nl.connect(gnd, cap, 2);
        }
        let j = nl.add_chip("j1", Passive::new(vec![(1, "GND".into()), (2, "TX".into()), (3, "RX".into()), (4, "RTS".into()), (5, "CTS".into())]));
        nl.set_meta(j, meta("Header 1x5 2.54mm", "PinHeader_1x05", "j1", None, Model::Passive));
        nl.connect(gnd, j, 1);
        nl.connect(tx, j, 2);
        nl.connect(rx, j, 3);
        nl.connect(rts232, j, 4);
        nl.connect(cts232, j, 5);
    }
    nl
}

/// The board file.
pub fn board() -> Board {
    let nl = build_netlist();
    nl.export(
        "grit",
        "Microprogrammed 16-bit machine: ATF22V10C logic, SST39SF040 program flash and microcode ROM, AS7C164A SRAM, TL16C550 UART, MAX811L reset, 9.216 MHz oscillator.",
        PERIOD as f64 / NS as f64,
        BTreeMap::new(),
        layout(),
    )
}

// ---------------------------------------------------------------------------
// The runner

/// Build options.
pub struct Build {
    /// When the supervisor releases RST_n: ns into a CLK period, after
    /// `reset_clocks` clocks.  Any value must work.
    pub reset_phase_ns: f64,
    pub reset_clocks: u64,
    /// The UART's clock as the model sees it; the board's is CLK2X.  Tests
    /// use a faster one so characters take tens of clocks.
    pub uart_xin_hz: f64,
    /// Characters the terminal sends, from when the program first polls.
    pub uart_rx: Vec<u8>,
    /// Power-up contents of the SRAM (None: zero).
    pub ram_image: Option<Vec<u16>>,
}

impl Default for Build {
    fn default() -> Build {
        Build { reset_phase_ns: 37.0, reset_clocks: 8, uart_xin_hz: CLK2X_HZ, uart_rx: Vec::new(), ram_image: None }
    }
}

pub struct Grit {
    pub sim: Sim,
    pub clocks: u64,
    /// When RESET (the synchronised one) fell (ps).
    pub reset_release: Time,
    clk2x: NetId,
    reset: NetId,
    ir: Vec<NetId>,
    step: Vec<NetId>,
    addr: Vec<NetId>,
    pcdrv: NetId,
    ram: Vec<usize>,
}

fn warning_time(w: &str) -> Option<Time> {
    let i = w.find("t=")?;
    let rest = &w[i + 2..];
    let end = rest.find("ns")?;
    rest[..end].parse::<f64>().ok().map(|ns| (ns * NS as f64).round() as Time)
}

impl Grit {
    pub fn ns(x: f64) -> Time {
        (x * NS as f64).round() as Time
    }

    pub fn build(program: &[u16], opt: &Build) -> Grit {
        let b = board();
        Grit::from_board(&b, program, opt)
    }

    /// Instantiate a board file, load the program, the microcode and the
    /// images, and run the power-on reset until RESET is released.
    pub fn from_board(board: &Board, program: &[u16], opt: &Build) -> Grit {
        let release = opt.reset_clocks as Time * PERIOD + Self::ns(opt.reset_phase_ns);
        let load = Load { grade: Grade::Room, gate_tpd: (500, 5500), reset_release: release, mr_timeout: 4 * PERIOD, uart_xin_hz: Some(opt.uart_xin_hz), dmem_timing: None };
        let mut nl = board.instantiate(&load);
        assert!(program.len() <= FLASH_WORDS, "program too long");
        for (id, role) in nl.chips_with_role("rom:") {
            let lane: usize = role["rom:".len()..].parse().unwrap();
            let chip = nl.chip_mut::<Rom>(id);
            for (i, w) in program.iter().enumerate() {
                chip.preload(i as u32, (w >> (8 * lane)) as u8);
            }
            for i in program.len()..FLASH_WORDS {
                chip.preload(i as u32, 0);
            }
        }
        let ucode = microcode();
        for (id, role) in nl.chips_with_role("ucode:") {
            let lane: usize = role["ucode:".len()..].parse().unwrap();
            let chip = nl.chip_mut::<Rom>(id);
            for (i, w) in ucode.iter().enumerate() {
                chip.preload(i as u32, (w >> (8 * lane)) as u8);
            }
        }
        let mut ram = Vec::new();
        for (id, role) in nl.chips_with_role("ram:") {
            let lane: usize = role["ram:".len()..].parse().unwrap();
            let chip = nl.chip_mut::<As7c164a>(id);
            for i in 0..RAM_WORDS {
                let v = opt.ram_image.as_ref().map_or(0, |im| im[i]);
                chip.preload(i as u32, (v >> (8 * lane)) as u8);
            }
            ram.push(id);
        }
        ram.sort();
        if let Some(id) = nl.chips_with_role("uart").first().map(|(id, _)| *id) {
            nl.chip_mut::<Uart16550>(id).core.send(&opt.uart_rx);
        }
        let sim = nl.build();
        let clk2x = sim.net_id("CLK2X");
        let reset = sim.net_id("RESET");
        let ir = (0..5).map(|i| sim.net_id(&n("IR", i))).collect();
        let step = (0..4).map(|i| sim.net_id(&n("STEP", i))).collect();
        let addr = (1..=15).map(|i| sim.net_id(&n("ADDR", i))).collect();
        let pcdrv = sim.net_id("PCDRV");
        let mut g = Grit { sim, clocks: 0, reset_release: 0, clk2x, reset, ir, step, addr, pcdrv, ram };
        g.wait_reset_release();
        g
    }

    /// One CLK2X period: low for the first half, high for the second, so
    /// that at power-up the clock settles low before its first rising
    /// edge.
    fn tick(&mut self) {
        let base = self.sim.now();
        self.sim.schedule(base, self.clk2x, Level::L);
        self.sim.schedule(base + PERIOD2X / 2, self.clk2x, Level::H);
        self.sim.run_until(base + PERIOD2X);
    }

    /// One CLK period (two ticks).
    pub fn clock(&mut self) {
        self.tick();
        self.tick();
        self.clocks += 1;
    }

    pub fn run(&mut self, clocks: u64) {
        for _ in 0..clocks {
            self.clock();
        }
    }

    fn wait_reset_release(&mut self) {
        for _ in 0..200 {
            self.clock();
            if self.sim.value(self.reset) == Level::L && self.clocks > 2 {
                break;
            }
        }
        assert_eq!(self.sim.value(self.reset), Level::L, "reset never released");
        self.reset_release = self.sim.history(self.reset).iter().rev().find(|(_, l)| *l == Level::L).map(|(t, _)| *t).expect("release time");
        self.clocks = 0;
    }

    /// Press the reset button for `hold_ns` from `phase_ns` into the
    /// current clock, then wait for the release.
    pub fn press_reset(&mut self, phase_ns: f64, hold_ns: f64) {
        let mr_n = self.sim.net_id("MR_n");
        let down = self.sim.now() + Self::ns(phase_ns);
        self.sim.schedule(down, mr_n, Level::L);
        self.sim.schedule(down + Self::ns(hold_ns), mr_n, Level::Z);
        for _ in 0..8 {
            self.clock();
            if self.sim.value(self.reset) == Level::H {
                break;
            }
        }
        assert_eq!(self.sim.value(self.reset), Level::H, "button press not seen");
        self.wait_reset_release();
    }

    /// The opcode in the IR and the step counter, as the nets read now.
    pub fn ir(&self) -> Option<u8> {
        self.sim.read_bus(&self.ir).map(|v| v as u8)
    }
    pub fn step_count(&self) -> Option<u8> {
        self.sim.read_bus(&self.step).map(|v| v as u8)
    }
    /// The address bus (the PC while PCDRV, else A).
    pub fn addr(&self) -> Option<u16> {
        self.sim.read_bus(&self.addr).map(|v| (v << 1) as u16)
    }
    pub fn pcdrv(&self) -> Level {
        self.sim.value(self.pcdrv)
    }

    /// A register of a GAL by net name: the flop behind the pin, whether
    /// or not the pin is enabled.
    pub fn gal_q(&self, chip: &str, net: &str) -> Level {
        let id = self.sim.chip_id(chip);
        let nid = self.sim.net_id(net);
        let pins = self.sim.pins_on(nid, id);
        let pin = *pins.iter().find(|&&p| (14..=23).contains(&p)).expect("net is not an output of that chip");
        let k = (0..10).find(|&k| olmc_pin(k) as usize == pin).unwrap();
        self.sim.chip(id).downcast_ref::<Gal22v10>().unwrap().q(k)
    }
    fn gal_bus(&self, bits: &[(&str, String)]) -> Option<u16> {
        let mut v = 0u16;
        for (i, (chip, net)) in bits.iter().enumerate() {
            v |= (self.gal_q(chip, net).bit()? as u16) << i;
        }
        Some(v)
    }
    /// The latches and the PC, from their flops.
    pub fn a(&self) -> Option<u16> {
        let bits: Vec<(&str, String)> = (0..16).map(|i| (if i < 8 { "a0" } else { "a1" }, a_net(i))).collect();
        self.gal_bus(&bits)
    }
    pub fn b(&self) -> Option<u16> {
        let bits: Vec<(&str, String)> = (0..16).map(|i| (if i < 8 { "b0" } else { "b1" }, n("B", i))).collect();
        self.gal_bus(&bits)
    }
    pub fn pc(&self) -> Option<u16> {
        let bits: Vec<(&str, String)> = (1..16).map(|i| (if i < 9 { "pc0" } else { "pc1" }, n("ADDR", i))).collect();
        self.gal_bus(&bits).map(|v| v << 1)
    }

    /// A word of the SRAM (byte address in 0x8000..0xBFFF, or a word index).
    pub fn ram_word(&self, addr: u16) -> Option<u16> {
        let i = ((addr & 0x3FFF) >> 1) as u32;
        let lo = self.sim.chip(self.ram[0]).downcast_ref::<As7c164a>().unwrap().peek(i)?;
        let hi = self.sim.chip(self.ram[1]).downcast_ref::<As7c164a>().unwrap().peek(i)?;
        Some(lo as u16 | (hi as u16) << 8)
    }
    pub fn reg(&self, r: usize) -> Option<u16> {
        self.ram_word(0x8000 + 2 * r as u16)
    }

    pub fn uart(&self) -> &Uart16550 {
        let id = self.sim.chip_id("uart0");
        self.sim.chip(id).downcast_ref::<Uart16550>().expect("uart model")
    }
    pub fn uart_tx(&self) -> Vec<u8> {
        self.uart().core.tx.clone()
    }

    /// Is the machine in HALT's microcode (IR = HALT, past the fetch)?
    pub fn halted(&self) -> bool {
        self.ir() == Some(Op::Halt as u8) && self.step_count().is_some_and(|s| s >= 2)
    }
    /// Run until halted or `max` clocks pass; returns whether it halted.
    pub fn run_until_halt(&mut self, max: u64) -> bool {
        for _ in 0..max {
            if self.halted() {
                return true;
            }
            self.clock();
        }
        self.halted()
    }

    /// Chip warnings from RESET release onwards.
    pub fn warnings(&self) -> Vec<String> {
        self.sim.warnings().into_iter().filter(|w| warning_time(w).is_none_or(|t| t >= self.reset_release)).collect()
    }
    /// Warnings while RESET was held, minus the expected unknown captures
    /// of registers without a reset.
    pub fn reset_warnings(&self) -> Vec<String> {
        self.sim.warnings().into_iter().filter(|w| warning_time(w).is_some_and(|t| t < self.reset_release)).filter(|w| !w.contains("CapturedX")).collect()
    }
}
