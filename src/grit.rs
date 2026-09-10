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
use crate::netlist::{ChipId, Level, NS, NetId, Netlist, Passive, ResetSupervisor, Rom, RomPin, Sim, Sram8kPin, Time, rom_pin_of, sram8k_pin_of};
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
/// The T latch: sixteen flops that load from D (TLD) and drive D (TDRV).
/// Memory data bound for a register that drives the address bus (A on
/// `LDA (A)`, the PC on a jump) lands here first, so that no address
/// register ever changes at the edge that ends a strobe: the one bus
/// rule then holds for memories and devices alike.  Invisible to the
/// instruction set; every instruction may clobber it.
pub const TLD: u16 = 1 << 12;
pub const TDRV: u16 = 1 << 13;
pub const F_ADD: u16 = 0;
pub const F_AND: u16 = F0;
pub const F_NOR: u16 = F1;
pub const F_PASSB: u16 = F1 | F0;
/// The fetch: the PC addresses the flash, the IR takes the opcode.
pub const FETCH: u16 = PCDRV | MEMRD | IRLD;

/// Microword bit names, bit 0 first.
pub const BIT_NAMES: [&str; 14] = ["PCDRV", "MEMRD", "ALUOE", "WE", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV", "TLD", "TDRV"];

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
    // anything strobes or samples it.  A word that loads a latch from
    // memory is followed by the same word without the load: address and
    // data outlive the capture by a clock, so no clock skew can turn the
    // capture into a hold violation.  The word after a fetch is that hold
    // word (`FETCH_HOLD`), so an instruction whose first word takes the
    // address bus from the PC starts with an idle word.
    const FETCH_HOLD: u16 = PCDRV | MEMRD;
    let alu = |f: u16, ld: u16| vec![0, ADRV, ADRV | ALUOE | f | ld, 0, PCINC, FETCH, FETCH_HOLD];
    match op {
        Op::Reset => vec![FETCH, FETCH_HOLD],
        Op::LdaImm => vec![PCINC, PCDRV | MEMRD | ALD, PCDRV | MEMRD, PCINC, FETCH, FETCH_HOLD],
        Op::LdbImm => vec![PCINC, PCDRV | MEMRD | BLD, PCDRV | MEMRD, PCINC, FETCH, FETCH_HOLD],
        // Loading A changes the address at the edge, so the strobe drops in
        // the hold word (a read at the new address could be a UART read
        // with side effects).  A's data hold then rests on the memory's
        // output-disable time against the clock skew between the pipeline
        // register and the A chips: under 2 ns of skew (docs/grit.md).
        // A load into an address register goes through T: the read lands
        // in T with the address held, and the register takes it from T in
        // a word with no strobe.
        Op::LdaA => vec![0, ADRV, ADRV | MEMRD | TLD, ADRV | MEMRD, 0, TDRV | ALD, PCINC, FETCH, FETCH_HOLD],
        Op::LdbA => vec![0, ADRV, ADRV | MEMRD | BLD, ADRV | MEMRD, PCINC, FETCH, FETCH_HOLD],
        Op::StbA => vec![0, ADRV, ADRV | ALUOE | F_PASSB | WE, ADRV | ALUOE | F_PASSB, 0, PCINC, FETCH, FETCH_HOLD],
        Op::AddA => alu(F_ADD, ALD),
        Op::AddB => alu(F_ADD, BLD),
        Op::AndA => alu(F_AND, ALD),
        Op::AndB => alu(F_AND, BLD),
        Op::NorA => alu(F_NOR, ALD),
        Op::NorB => alu(F_NOR, BLD),
        Op::MovAB => alu(F_PASSB, ALD),
        Op::Jmp => vec![PCINC, PCDRV | MEMRD | TLD, PCDRV | MEMRD, 0, TDRV | PCLD, 0, FETCH, FETCH_HOLD],
        // The condition: word 1 puts A on the bus so that NE is valid and
        // the NEL flop latches it at the word's end; the ROM read for step
        // 3, during word 2, is the one that sees the fresh NEL, so the two
        // variants differ from step 3 on.
        Op::Jeq if !ne => vec![0, ADRV, PCINC, PCDRV | MEMRD | TLD, PCDRV | MEMRD, 0, TDRV | PCLD, 0, FETCH, FETCH_HOLD],
        Op::Jeq => vec![0, ADRV, PCINC, PCINC, FETCH, FETCH_HOLD],
        Op::Nop => vec![PCINC, FETCH, FETCH_HOLD],
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
    w & (MEMRD | ALUOE | TDRV)
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
            // A load from memory into A, B or the IR, or a write: the next
            // word keeps the same drivers on Addr and D.  A load into the
            // PC changes the address at the edge: the next word keeps the
            // PC on Addr but drops the strobe.
            // The one bus rule: a register that drives the address bus is
            // never loaded at an edge that ends a strobe.
            let strobed = w & (MEMRD | WE) != 0;
            assert!(!(strobed && w & ALD != 0 && w & ADRV != 0), "{op:?} step {k}: A loaded while it drives the address under a strobe");
            assert!(!(strobed && w & PCLD != 0 && w & PCDRV != 0), "{op:?} step {k}: the PC loaded while it drives the address under a strobe");
            if (w & MEMRD != 0 && w & (BLD | IRLD | TLD) != 0) || w & WE != 0 {
                let drivers = PCDRV | ADRV | MEMRD | ALUOE | TDRV;
                assert_eq!(n & drivers, w & drivers, "{op:?} step {k}: address or data not held after a load or a write");
            }
        }
        if w & IRLD != 0 {
            assert!(k + 1 < s.len() && s[k + 1] == PCDRV | MEMRD, "{op:?} step {k}: the word after a fetch must be the fetch hold");
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
        pc += 1 + line.imm.is_some() as u16;
        lines.push(line);
    }
    let value = |s: &str| -> Result<u16, String> {
        if let Some(r) = s.strip_prefix("&r") {
            let n: u16 = r.parse().map_err(|_| format!("bad register {s}"))?;
            return Ok(0x8000 + n);
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

pub const FLASH_WORDS: usize = 1 << 15;
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
            0 | 1 => self.flash[addr as usize & (FLASH_WORDS - 1)],
            2 => self.ram[addr as usize & (RAM_WORDS - 1)],
            _ => {
                // The reference has no line time: whatever the far end
                // queued has arrived by the time the program looks.
                let v = self.uart.read(addr as u8 & 7) as u16;
                while self.uart.rx_deliver() {}
                v
            }
        }
    }
    pub fn write(&mut self, addr: u16, v: u16) {
        match addr >> 14 {
            2 => self.ram[addr as usize & (RAM_WORDS - 1)] = v,
            3 => {
                self.uart.write(addr as u8 & 7, v as u8);
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
        let imm = if op.has_imm() { self.read(self.pc.wrapping_add(1)) } else { 0 };
        let next = self.pc.wrapping_add(if op.has_imm() { 2 } else { 1 });
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
                self.pc = self.pc.wrapping_sub(1);
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

/// Net of A latch bit `i`: the address bus.  Addr is a 16-bit word
/// address: everything on the bus is a 16-bit word (the UART's registers
/// are the low byte of consecutive words).
fn a_net(i: usize) -> String {
    n("ADDR", i)
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

/// The A latch, two chips: `a0` bits 0..7, `a1` bits 8..15, driving Addr
/// on ADRV.
fn a_specs() -> Vec<GalSpec> {
    (0..2)
        .map(|half| {
            let eqs = (8 * half..8 * half + 8).map(|i| latch(&a_net(i), &n("D", i), "ALD").with_oe(vec![l("ADRV")])).collect();
            GalSpec { name: format!("a{half}"), clk: Some("CLK".into()), ar: None, eqs }
        })
        .collect()
}

/// The T latch, two chips: loads from D on TLD, drives D on TDRV.  Its
/// register nets are T0..T15, merged with D0..D15 in the netlist (the
/// pin is on the data bus); the equations load from the D pin and hold
/// from the register.
fn t_specs() -> Vec<GalSpec> {
    (0..2)
        .map(|half| {
            let eqs = (8 * half..8 * half + 8).map(|i| latch(&n("T", i), &n("D", i), "TLD").with_oe(vec![l("TDRV")])).collect();
            GalSpec { name: format!("t{half}"), clk: Some("CLK".into()), ar: None, eqs }
        })
        .collect()
}

/// The B latch: `b0` bits 0..7, `b1` bits 8..15.
fn b_specs() -> Vec<GalSpec> {
    (0..2)
        .map(|half| {
            let eqs = (8 * half..8 * half + 8).map(|i| latch(&n("B", i), &n("D", i), "BLD")).collect();
            GalSpec { name: format!("b{half}"), clk: Some("CLK".into()), ar: None, eqs }
        })
        .collect()
}

/// The pipeline register: `mir0` holds M0..M7, `mir1` M8..M15.  The
/// pins the memories want are active low.
fn mir_specs() -> Vec<GalSpec> {
    let names = ["PCDRV", "MEMRD_n", "ALUOE", "WE_n", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV", "TLD", "TDRV", "AUX0", "AUX1"];
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
    GalSpec { name: "seq0".into(), clk: Some("CLK".into()), ar: None, eqs }
}

/// The sequencer's second chip, the flags: the reset synchroniser and
/// the condition.  RS1 samples RST_n on CLK (a synchroniser stage);
/// RESET is the second stage, high at power-up (a zero register behind
/// an active-low pin).  NEL is the ALU's not-equal, latched at the end
/// of every word in which A is on the bus (NE3 is only meaningful then),
/// held otherwise: the microcode ROM's condition address bit.
fn flags_spec() -> GalSpec {
    let eqs = vec![
        Eq::sop("RS1", Mode::Reg, vec![vec![l("RST_n")]]).active_low().sync(),
        Eq::sop("RESET", Mode::Reg, vec![vec![nl_("RS1")]]).active_low(),
        with_reset(Eq::sop("NEL", Mode::Reg, vec![vec![l("ADRV"), l("NE3")], vec![nl_("ADRV"), l("NEL")]])),
    ];
    GalSpec { name: "seq1".into(), clk: Some("CLK".into()), ar: None, eqs }
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
    let low: Vec<String> = (0..8).map(|i| n("ADDR", i)).collect();
    let mut pc0: Vec<Eq> = (0..8).map(|k| bit(k, &low[..k], "PCINC")).collect();
    // CO: the count carries into bit 8.
    let mut co_ins: Vec<String> = vec!["PCINC".into()];
    co_ins.extend(low.iter().cloned());
    pc0.push(Eq::table_pos("CO", Mode::Comb, &strs(&co_ins), |m| Some(m == 0x1FF)));
    let high: Vec<String> = (8..16).map(|i| n("ADDR", i)).collect();
    let pc1: Vec<Eq> = (8..16).map(|k| bit(k, &high[..k - 8], "CO")).collect();
    vec![
        GalSpec { name: "pc0".into(), clk: Some("CLK".into()), ar: None, eqs: pc0 },
        GalSpec { name: "pc1".into(), clk: Some("CLK".into()), ar: None, eqs: pc1 },
    ]
}

/// The ALU: four 4-bit slices with ripple carry inside and between, the
/// function folded into every sum term, and a not-equal chain beside
/// the carry chain (NE0..NE3, cumulative; NE3 is "A differs from B").
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
            let ne_out = n("NE", s);
            eqs.push(Eq::table(&ne_out, Mode::Comb, &strs(&ne_ins), move |m| {
                let off = has_nein as u32;
                let prev = has_nein && m & 1 == 1;
                let av = m >> off & 0xF;
                let bv = m >> (off + 4) & 0xF;
                Some(prev || av != bv)
            }));
            GalSpec { name: format!("alu{s}"), clk: None, ar: None, eqs }
        })
        .collect()
}

/// Every GAL, with a check that each fits its chip.
pub fn gal_specs() -> Vec<GalSpec> {
    let mut v = Vec::new();
    v.push(seq_spec());
    v.push(flags_spec());
    v.extend(mir_specs());
    v.extend(pc_specs());
    v.extend(a_specs());
    v.extend(b_specs());
    v.extend(t_specs());
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

/// The oscillator: the machine clock, straight from the can.
pub const OSC_HZ: f64 = 4_000_000.0;
/// One CLK period in ps.
pub const PERIOD: Time = 250_000;
/// The UART's own crystal, the one crag uses (a stocked 3225 part):
/// divisor 8 is 115200 baud.  Its clock has nothing to do with the bus;
/// the 16550's bus side is strobe-driven.
pub const UART_XIN_HZ: f64 = 14_745_600.0;

fn block_of(name: &str) -> (&'static str, &'static str) {
    let prefix: String = name.chars().take_while(|c| c.is_ascii_alphabetic()).collect();
    match prefix.as_str() {
        "seq" => ("Sequencer", "Control"),
        "uc" => ("Microcode ROM", "Control"),
        "mir" => ("Pipeline register", "Control"),
        "pc" => ("PC", "Datapath"),
        "a" => ("A", "Datapath"),
        "b" => ("B", "Datapath"),
        "t" => ("T", "Datapath"),
        "alu" => ("ALU", "Datapath"),
        "rom" => ("Program flash", "Memory"),
        "ram" => ("SRAM", "Memory"),
        "uart" => ("UART", "I/O"),
        "xcvr" | "c" | "j" => ("Serial port", "I/O"),
        "x" | "xc" => ("UART", "I/O"),
        "osc" | "swm" | "swstep" | "rstep" | "cst" | "uinv" | "umux" | "jclk" => ("Clock", "Board"),
        "rst" | "swr" => ("Reset", "Board"),
        "jpwr" | "f" | "q" | "cb" | "cd" | "ledp" | "rlp" => ("Power", "Board"),
        "ubuf" | "led" | "rl" | "ubus" | "lb" | "rb" => ("Debug LEDs", "Board"),
        "ja" | "jd" | "jc" | "js" => ("Analyzer headers", "Board"),
        "swb" | "rp" | "rd" => ("Bank switches", "Board"),
        _ => ("?", "?"),
    }
}

fn meta(part: &str, package: &str, name: &str, role: Option<String>, model: Model) -> ChipMeta {
    let (block, stage) = block_of(name);
    ChipMeta { part: part.into(), package: package.into(), block: block.into(), stage: stage.into(), role, model, lcsc: None }
}

/// Metadata with the LCSC part number.
fn lcsc(m: ChipMeta, code: &str) -> ChipMeta {
    ChipMeta { lcsc: Some(code.into()), ..m }
}

/// A two-pin part (resistor, capacitor, LED, crystal).
fn two_pin(nl: &mut Netlist, name: &str, part: &str, package: &str, code: Option<&str>) -> ChipId {
    let c = nl.add_chip(name, Passive::new(vec![(1, "1".into()), (2, "2".into())]));
    let m = meta(part, package, name, None, Model::Passive);
    nl.set_meta(c, match code { Some(code) => lcsc(m, code), None => m });
    c
}

pub fn layout() -> Vec<Column> {
    let col = |title: &str, width: u32, blocks: &[&str]| Column { title: title.into(), width, blocks: blocks.iter().map(|b| b.to_string()).collect(), reg: false };
    vec![
        col("Control", 300, &["Sequencer", "Microcode ROM", "Pipeline register"]),
        col("Datapath", 330, &["PC", "A", "B", "T", "ALU"]),
        col("Memory", 240, &["Program flash", "SRAM"]),
        col("I/O", 300, &["UART", "Serial port"]),
        col("Board", 300, &["Clock", "Reset", "Power", "Debug LEDs", "Analyzer headers", "Bank switches"]),
    ]
}

/// Every chip and every connection; no stimulus.
pub fn build_netlist() -> Netlist {
    let mut nl = Netlist::new();
    for spec in gal_specs() {
        let (id, pins) = galpack::instantiate(&mut nl, &spec);
        let model = Model::Gal { clk: spec.clk.clone(), ar: spec.ar.clone(), eqs: spec.eqs.clone(), pins };
        nl.set_meta(id, meta("ATF22V10C-7PX in DS1009-24AT1NX-0A2 socket", "DIP-24 (300 mil)", &spec.name, None, model));
    }
    // T's register pins sit on the data bus.
    for i in 0..16 {
        let t = nl.net(&n("T", i));
        let d = nl.net(&n("D", i));
        nl.merge(d, t);
    }
    let clk = nl.net("CLK");
    nl.set_net_role(clk, "clk");
    let reset = nl.net("RESET");
    nl.set_net_role(reset, "reset");
    let gnd = nl.net("GND");
    let vcc = nl.net("VCC");
    nl.tie(gnd, Level::L);
    nl.tie(vcc, Level::H);

    // The oscillator (an SMD 4-pin module: 1 EN, 2 GND, 3 OUT, 4 VCC),
    // a 5 V CMOS part still to be picked from stock.  Its output OSC
    // reaches CLK through the run/step multiplexer (see `physical`).
    let osc = nl.add_chip("osc0", Passive::new(vec![(1, "EN".into()), (2, "GND".into()), (3, "OUT".into()), (4, "VCC".into())]));
    nl.set_meta(osc, meta("Oscillator 4.000 MHz 5 V CMOS", "7050", "osc0", None, Model::Passive));
    let osc_net = nl.net("OSC");
    nl.connect(vcc, osc, 1);
    nl.connect(gnd, osc, 2);
    nl.connect(osc_net, osc, 3);
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

    // Program flash: two lanes, A0..A14 from Addr0..Addr14, bank jumpers
    // on A15..A18, CE# from Addr15, OE# from MEMRD_n.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("rom{lane}"), Rom::sst39sf040_70());
        nl.set_meta(c, lcsc(meta("SST39SF040-70-4C-PHE in DS1009-32AT1WX-0A2 socket", "DIP-32 (600 mil)", &format!("rom{lane}"), Some(format!("rom:{lane}")), Model::Rom { tacc_ns: 70, toe_ns: 35, tdf_ns: 25 }), "C72122"));
        for j in 0..19u32 {
            let net = match j {
                0..=14 => nl.net(&n("ADDR", j as usize)),
                _ => nl.net(&n("PBANK", j as usize - 15)),
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
    for i in 0..4 {
        let net = nl.net(&n("PBANK", i));
        nl.tie(net, Level::L);
    }
    for i in 0..3 {
        let net = nl.net(&n("UBANK", i));
        nl.tie(net, Level::L);
    }
    // Microcode ROM: two lanes on private wires.  A0..A3 = STEP, A4 = NE,
    // A5..A9 = IR, bank jumpers on A10..A12, the rest grounded; always
    // selected and enabled.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("uc{lane}"), Rom::sst39sf040_70());
        nl.set_meta(c, lcsc(meta("SST39SF040-70-4C-PHE in DS1009-32AT1WX-0A2 socket", "DIP-32 (600 mil)", &format!("uc{lane}"), Some(format!("ucode:{lane}")), Model::Rom { tacc_ns: 70, toe_ns: 35, tdf_ns: 25 }), "C72122"));
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
    // SRAM: two lanes, A0..A12 from Addr0..Addr12, CE2 = Addr15,
    // CE1# = Addr14, OE# = MEMRD_n, WE# = WE_n.
    for lane in 0..2 {
        let c = nl.add_chip(&format!("ram{lane}"), As7c164a::with_timing(as7c164a::Timing::grade_15()));
        nl.set_meta(c, lcsc(meta("AS7C164A-15PCN in DS1009-28AT1WX-0A2 socket", "DIP-28 (600 mil)", &format!("ram{lane}"), Some(format!("ram:{lane}")), Model::Sram8k { timing: "AS7C164A-15".into() }), "C72121"));
        for a in 0..13 {
            let net = nl.net(&n("ADDR", a));
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
    // UART: D0..D7, A0..A2 from Addr0..Addr2 (register n at 0xC000 + n),
    // CS0 = Addr14, CS1 = Addr15, CS2# ADS# RD2 WR2 low, RD1# = MEMRD_n,
    // WR1# = WE_n, MR = RESET, its own 1.8432 MHz crystal on XIN/XOUT,
    // BAUDOUT# -> RCLK, DTR# -> DSR# + DCD#, RI# high, RTS#/CTS# and
    // SOUT/SIN through the transceiver.
    {
        let c = nl.add_chip("uart0", Uart16550::new(BusTiming::tl16c550c(), UART_XIN_HZ));
        nl.set_meta(c, meta("TL16C550DPTR", "LQFP-48", "uart0", Some("uart".into()), Model::Uart { xin_hz: UART_XIN_HZ }));
        for i in 0..8u8 {
            let net = nl.net(&n("D", i as usize));
            nl.connect(net, c, uart_pin_of(UartPin::D(i)));
        }
        for i in 0..3u8 {
            let net = nl.net(&n("ADDR", i as usize));
            nl.connect(net, c, uart_pin_of(UartPin::A(i)));
        }
        for (net, p) in [("ADDR14", UartPin::Cs0), ("ADDR15", UartPin::Cs1), ("MEMRD_n", UartPin::Rd1N), ("WE_n", UartPin::Wr1N), ("RESET", UartPin::Mr), ("SIN", UartPin::Sin), ("SOUT", UartPin::Sout), ("XIN", UartPin::Xin), ("XOUT", UartPin::Xout)] {
            let net = nl.net(net);
            nl.connect(net, c, uart_pin_of(p));
        }
        // Crystal and load capacitors.
        let xin = nl.net("XIN");
        let xout = nl.net("XOUT");
        let x = two_pin(&mut nl, "x1", "Crystal 14.7456 MHz 12 pF", "3225", Some("C2885591"));
        nl.connect(xin, x, 1);
        nl.connect(xout, x, 2);
        for (name, net) in [("xc1", xin), ("xc2", xout)] {
            let cap = two_pin(&mut nl, name, "18pF 0603 C0G", "0603", Some("C1653"));
            nl.connect(net, cap, 1);
            nl.connect(gnd, cap, 2);
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
        nl.set_meta(xc, lcsc(meta("SP3232EEY-L/TR", "TSSOP-16", "xcvr0", None, Model::Passive), "C13482"));
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
            let cap = two_pin(&mut nl, name, "100nF 0603 X7R", "0603", Some("C14663"));
            let na = nl.net(&format!("XC_{name}A"));
            let nb = nl.net(&format!("XC_{name}B"));
            nl.connect(na, xc, a);
            nl.connect(nb, xc, b);
            nl.connect(na, cap, 1);
            nl.connect(nb, cap, 2);
        }
        for (name, pin, rail_net) in [("c3", 2, "XC_VP"), ("c4", 6, "XC_VM")] {
            let cap = two_pin(&mut nl, name, "100nF 0603 X7R", "0603", Some("C14663"));
            let net = nl.net(rail_net);
            nl.connect(net, xc, pin);
            nl.connect(net, cap, 1);
            nl.connect(gnd, cap, 2);
        }
        // DB9 female, wired as a DCE: a straight cable to a PC or a
        // USB-serial DTE.  Pin 2 is data out of grit, 3 data in, 7 the
        // DTE's RTS (grit's CTS#), 8 grit's RTS# (the DTE's CTS), 5 GND;
        // 1, 4, 6, 9 open.
        let j = nl.add_chip("j1", Passive::new(vec![(1, "DCD".into()), (2, "RXD".into()), (3, "TXD".into()), (4, "DTR".into()), (5, "GND".into()), (6, "DSR".into()), (7, "RTS".into()), (8, "CTS".into()), (9, "RI".into())]));
        nl.set_meta(j, lcsc(meta("DB9 female, right angle, through hole", "DSUB-9", "j1", None, Model::Passive), "C9900026339"));
        nl.connect(tx, j, 2);
        nl.connect(rx, j, 3);
        nl.connect(gnd, j, 5);
        nl.connect(cts232, j, 7);
        nl.connect(rts232, j, 8);
    }
    physical(&mut nl);
    nl
}

/// The board around the machine: power entry, decoupling, the run/step
/// clock, the reset button, bank switches, debug LEDs behind buffers,
/// and logic-analyzer headers.  Nothing here is modelled: every part is
/// passive to the simulator, and CLK is driven by the test bench at the
/// multiplexer's output exactly as the oscillator would through it.
fn physical(nl: &mut Netlist) {
    let gnd = nl.net("GND");
    let vcc = nl.net("VCC");
    let clk = nl.net("CLK");

    // Power: 5 V barrel jack (1 centre +, 2 sleeve, 3 switch), a 1.1 A
    // polyfuse, a P-MOSFET against reverse polarity (drain on the input,
    // source on the rail, gate to ground: the body diode starts it and
    // the channel carries it), bulk capacitors, a power LED.
    let vin = nl.net("VIN");
    let vf = nl.net("VF");
    // The jack is the one already in the flatland library (5.5 / 2.0 mm;
    // a 2.1 mm plug fits).  Pin 1 centre, 3 sleeve, 2 the switch contact.
    let jack = nl.add_chip("jpwr0", Passive::new(vec![(1, "PIN".into()), (2, "SW".into()), (3, "SLEEVE".into())]));
    nl.set_meta(jack, lcsc(meta("CUI PJ-002AH-SMT barrel jack 5.5/2.0 mm", "SMT", "jpwr0", None, Model::Passive), "C22434687"));
    nl.connect(vin, jack, 1);
    nl.connect(gnd, jack, 3);
    let fuse = two_pin(nl, "f0", "Polyfuse 1.1 A SMD1206P110TF/16", "1206", Some("C523825"));
    nl.connect(vin, fuse, 1);
    nl.connect(vf, fuse, 2);
    let fet = nl.add_chip("q0", Passive::new(vec![(1, "G".into()), (2, "S".into()), (3, "D".into())]));
    nl.set_meta(fet, lcsc(meta("AO3401A P-MOSFET", "SOT-23", "q0", None, Model::Passive), "C15127"));
    nl.connect(gnd, fet, 1);
    nl.connect(vcc, fet, 2);
    nl.connect(vf, fet, 3);
    let cb = two_pin(nl, "cb0", "100uF 16V electrolytic RVT1C101M0605", "SMD 6.3x5.4", Some("C970684"));
    nl.connect(vcc, cb, 1);
    nl.connect(gnd, cb, 2);
    for i in 1..=4 {
        let c = two_pin(nl, &format!("cb{i}"), "10uF 0805 X5R 25V", "0805", Some("C15850"));
        nl.connect(vcc, c, 1);
        nl.connect(gnd, c, 2);
    }
    let r = two_pin(nl, "rlp0", "1k 0603", "0603", Some("C21190"));
    let led = two_pin(nl, "ledp0", "LED red 0603", "0603", Some("C2286"));
    let la = nl.net("LEDA_POWER");
    nl.connect(vcc, r, 1);
    nl.connect(la, r, 2);
    nl.connect(la, led, 1);
    nl.connect(gnd, led, 2);

    // One 100 nF at every chip's supply pins.
    let chips = ["seq0", "seq1", "mir0", "mir1", "pc0", "pc1", "a0", "a1", "b0", "b1", "t0", "t1", "alu0", "alu1", "alu2", "alu3", "rom0", "rom1", "uc0", "uc1", "ram0", "ram1", "uart0", "xcvr0", "osc0", "rst0", "umux0", "uinv0", "ubuf0", "ubuf1", "ubuf2", "ubuf3"];
    for name in chips {
        let c = two_pin(nl, &format!("cd_{name}"), "100nF 0603 X7R", "0603", Some("C14663"));
        nl.connect(vcc, c, 1);
        nl.connect(gnd, c, 2);
    }

    // The clock.  A slide switch picks run (OSC) or single step; the
    // step button is debounced by an RC into a Schmitt inverter, so one
    // press is one rising edge of CLK.  Flip the switch while holding
    // reset: the multiplexer is not glitch-free at the moment of
    // switching.
    let osc = nl.net("OSC");
    let mode = nl.net("STEPMODE");
    let step_n = nl.net("STEPBTN_n");
    let stepclk = nl.net("STEPCLK");
    let swm = nl.add_chip("swm0", Passive::new(vec![(1, "RUN".into()), (2, "COM".into()), (3, "STEP".into())]));
    nl.set_meta(swm, lcsc(meta("MSK12C02 slide switch SPDT", "SMD 8x2.8", "swm0", None, Model::Passive), "C431540"));
    nl.connect(gnd, swm, 1);
    nl.connect(mode, swm, 2);
    nl.connect(vcc, swm, 3);
    let sw = nl.add_chip("swstep0", Passive::new(vec![(1, "1".into()), (2, "2".into()), (3, "3".into()), (4, "4".into())]));
    nl.set_meta(sw, lcsc(meta("TS-1187A-B-A-B tactile switch", "SMD 5.1x5.1", "swstep0", None, Model::Passive), "C318884"));
    // Net on pad 1 and ground on the diagonal pad 3: right whichever
    // pair of pads the part shorts internally.
    nl.connect(step_n, sw, 1);
    nl.connect(gnd, sw, 3);
    let rp = two_pin(nl, "rstep0", "10k 0603", "0603", Some("C25804"));
    nl.connect(vcc, rp, 1);
    nl.connect(step_n, rp, 2);
    let cst = two_pin(nl, "cst0", "100nF 0603 X7R", "0603", Some("C14663"));
    nl.connect(step_n, cst, 1);
    nl.connect(gnd, cst, 2);
    let inv = nl.add_chip("uinv0", Passive::new(vec![(1, "NC".into()), (2, "A".into()), (3, "GND".into()), (4, "Y".into()), (5, "VCC".into())]));
    nl.set_meta(inv, lcsc(meta("SN74LVC1G14DBVR Schmitt inverter", "SOT-23-5", "uinv0", None, Model::Passive), "C7835"));
    nl.connect(step_n, inv, 2);
    nl.connect(gnd, inv, 3);
    nl.connect(stepclk, inv, 4);
    nl.connect(vcc, inv, 5);
    let mux = nl.add_chip("umux0", Passive::new(vec![(1, "I0".into()), (2, "I1".into()), (3, "S".into()), (4, "Y".into()), (5, "GND".into()), (6, "VCC".into())]));
    nl.set_meta(mux, lcsc(meta("74LVC1G157GW,125 2:1 multiplexer", "SC-88", "umux0", None, Model::Passive), "C135822"));
    nl.connect(osc, mux, 1);
    nl.connect(stepclk, mux, 2);
    nl.connect(mode, mux, 3);
    nl.connect(clk, mux, 4);
    nl.connect(gnd, mux, 5);
    nl.connect(vcc, mux, 6);
    let jclk = nl.add_chip("jclk0", Passive::new(vec![(1, "CLK".into()), (2, "OSC".into()), (3, "GND".into())]));
    nl.set_meta(jclk, meta("Header 1x3 2.54 mm", "PinHeader_1x03", "jclk0", None, Model::Passive));
    nl.connect(clk, jclk, 1);
    nl.connect(osc, jclk, 2);
    nl.connect(gnd, jclk, 3);

    // The reset button, on the supervisor's MR# (pulled up inside it).
    let mr_n = nl.net("MR_n");
    let sw = nl.add_chip("swr0", Passive::new(vec![(1, "1".into()), (2, "2".into()), (3, "3".into()), (4, "4".into())]));
    nl.set_meta(sw, lcsc(meta("TS-1187A-B-A-B tactile switch", "SMD 5.1x5.1", "swr0", None, Model::Passive), "C318884"));
    nl.connect(mr_n, sw, 1);
    nl.connect(gnd, sw, 3);

    // Bank switches: 4-position DIP switches with 10k pull-ups; a
    // position ON pulls its bank bit low.  The simulation ties the bank
    // nets low: every switch ON.  Pull-downs on Addr14 and Addr15 keep
    // the selects at "flash" while nothing drives the address bus.
    for (name, nets) in [("swb0", vec!["PBANK0", "PBANK1", "PBANK2", "PBANK3"]), ("swb1", vec!["UBANK0", "UBANK1", "UBANK2"])] {
        let sw = nl.add_chip(name, Passive::new((1..=8).map(|p| (p, format!("{p}"))).collect()));
        nl.set_meta(sw, lcsc(meta("EM-04-Q DIP switch, 4 positions", "SMD-8P 6x10", name, None, Model::Passive), "C501635"));
        for (k, net) in nets.iter().enumerate() {
            let n_ = nl.net(net);
            nl.connect(n_, sw, k + 1);
            nl.connect(gnd, sw, 8 - k);
            let r = two_pin(nl, &format!("rp_{}", net.to_ascii_lowercase()), "10k 0603", "0603", Some("C25804"));
            nl.connect(vcc, r, 1);
            nl.connect(n_, r, 2);
        }
    }
    for (name, net) in [("rd0", "ADDR14"), ("rd1", "ADDR15")] {
        let r = two_pin(nl, name, "10k 0603", "0603", Some("C25804"));
        let n_ = nl.net(net);
        nl.connect(n_, r, 1);
        nl.connect(gnd, r, 2);
    }

    // Debug LEDs behind 74HC541 buffers, one CMOS input per signal so
    // the logic sees no load.  An LED shows its pin's level: an
    // active-low net's LED hangs from VCC and lights when the net is
    // low, that is, when the strobe is active.
    let groups: [[&str; 8]; 4] = [
        ["CLK", "RESET", "PCDRV", "ADRV", "MEMRD_n", "WE_n", "ALUOE", "TDRV"],
        ["ALD", "BLD", "TLD", "PCLD", "PCINC", "IRLD", "NEL", "AUX0"],
        ["IR0", "IR1", "IR2", "IR3", "IR4", "STEP0", "STEP1", "STEP2"],
        ["STEP3", "F0", "F1", "AUX1", "RST_n", "", "", ""],
    ];
    for (b, group) in groups.iter().enumerate() {
        let name = format!("ubuf{b}");
        let mut names: Vec<(usize, String)> = vec![(1, "OE1_n".into()), (19, "OE2_n".into()), (10, "GND".into()), (20, "VCC".into())];
        for i in 0..8 {
            names.push((2 + i, format!("A{i}")));
            names.push((18 - i, format!("Y{i}")));
        }
        let buf = nl.add_chip(&name, Passive::new(names));
        nl.set_meta(buf, lcsc(meta("74HC541D,653 octal buffer", "SOIC-20", &name, None, Model::Passive), "C126008"));
        nl.connect(gnd, buf, 1);
        nl.connect(gnd, buf, 19);
        nl.connect(gnd, buf, 10);
        nl.connect(vcc, buf, 20);
        for (i, sig) in group.iter().enumerate() {
            if sig.is_empty() {
                nl.connect(gnd, buf, 2 + i);
                continue;
            }
            let src = nl.net(sig);
            nl.connect(src, buf, 2 + i);
            let out = nl.net(&format!("LEDB_{sig}"));
            nl.connect(out, buf, 18 - i);
            let mid = nl.net(&format!("LEDA_{sig}"));
            let r = two_pin(nl, &format!("rl_{}", sig.to_ascii_lowercase()), "1k 0603", "0603", Some("C21190"));
            let led = two_pin(nl, &format!("led_{}", sig.to_ascii_lowercase()), "LED red 0603", "0603", Some("C2286"));
            if sig.ends_with("_n") {
                nl.connect(vcc, led, 1);
                nl.connect(mid, led, 2);
                nl.connect(mid, r, 1);
                nl.connect(out, r, 2);
            } else {
                nl.connect(out, r, 1);
                nl.connect(mid, r, 2);
                nl.connect(mid, led, 1);
                nl.connect(gnd, led, 2);
            }
        }
    }

    // Bus LEDs: an LED and 4.7k straight on every data and address line,
    // lit when the bit is 1.  About 0.6 mA at a CMOS high, within the
    // rating of every driver on the bus (the UART, the weakest, sources
    // 1 mA); a modern red 0603 is plainly visible at that.
    for sig in (0..16).map(|i| format!("D{i}")).chain((0..16).map(|i| format!("ADDR{i}"))) {
        let src = nl.net(&sig);
        let mid = nl.net(&format!("LEDA_{sig}"));
        let r = two_pin(nl, &format!("rb_{}", sig.to_ascii_lowercase()), "4.7k 0603", "0603", Some("C23162"));
        let led = two_pin(nl, &format!("lb_{}", sig.to_ascii_lowercase()), "LED red 0603", "0603", Some("C2286"));
        nl.connect(src, r, 1);
        nl.connect(mid, r, 2);
        nl.connect(mid, led, 1);
        nl.connect(gnd, led, 2);
    }

    // Logic-analyzer headers, 2 x 10 pins each.
    let header = |nl: &mut Netlist, name: &str, pins: Vec<&str>| {
        let names: Vec<(usize, String)> = pins.iter().enumerate().map(|(i, s)| (i + 1, s.to_string())).collect();
        let j = nl.add_chip(name, Passive::new(names));
        nl.set_meta(j, meta("Header 2x10 2.54 mm", "PinHeader_2x10", name, None, Model::Passive));
        for (i, s) in pins.iter().enumerate() {
            let net = nl.net(s);
            nl.connect(net, j, i + 1);
        }
    };
    let mut addr: Vec<String> = (0..16).map(|i| n("ADDR", i)).collect();
    addr.extend(["CLK", "RESET", "GND", "GND"].map(String::from));
    header(nl, "ja0", addr.iter().map(String::as_str).collect());
    let mut data: Vec<String> = (0..16).map(|i| n("D", i)).collect();
    data.extend(["CLK", "MEMRD_n", "WE_n", "GND"].map(String::from));
    header(nl, "jd0", data.iter().map(String::as_str).collect());
    header(nl, "jc0", vec!["PCDRV", "MEMRD_n", "ALUOE", "WE_n", "ALD", "BLD", "PCLD", "PCINC", "IRLD", "F0", "F1", "ADRV", "TLD", "TDRV", "CLK", "RESET", "NEL", "AUX0", "AUX1", "GND"]);
    header(nl, "js0", vec!["IR0", "IR1", "IR2", "IR3", "IR4", "STEP0", "STEP1", "STEP2", "STEP3", "NEL", "CLK", "RESET", "RST_n", "STEPMODE", "OSC", "AUX0", "AUX1", "GND", "GND", "GND"]);
}

/// What each bus is, for the chip map.
pub fn bus_descriptions() -> BTreeMap<String, String> {
    let d: &[(&str, &str)] = &[
        ("ADDR", "The address bus: a 16-bit word address (everything on the bus is a 16-bit word), which is also the A latch's value. A drives it on ADRV, the PC on PCDRV, nobody in the word between. Addr15 and Addr14 pick the chip: 0x = flash (0x0000..0x7FFF), 10 = SRAM (0x8000..0x9FFF, r0..r31 first), 11 = UART (register n at 0xC000 + n)."),
        ("D", "The data bus. One driver per clock: the memory or device that Addr selects on MEMRD, the ALU on ALUOE, or T on TDRV. Read by A, B, T, the PC and the IR at the clock edge."),
        ("B", "The B latch: the ALU's second operand and the value a store writes. Point-to-point into the ALU."),
        ("M", "The microcode ROM's data: the next microword, latched into the pipeline register at every edge."),
        ("IR", "The instruction register: the opcode, bits 15:11 of the instruction word. Address bits A5..A9 of the microcode ROM."),
        ("STEP", "The step counter: cleared by the fetch, counting every other clock. Address bits A0..A3 of the microcode ROM."),
        ("NEL", "The condition, in the sequencer's flags chip: NE3 latched at the end of every word in which A is on the bus (ADRV), held otherwise. Address bit A4 of the microcode ROM."),
        ("NE", "The not-equal chain: NE0 out of ALU slice 0 (bits 0..3 differ), NE1 (bits 0..7), NE2 (0..11) and NE3 (all: A differs from B), each slice taking the previous one in. NE3 feeds the NEL flop in the sequencer's flags chip."),
        ("COUT", "Carry between ALU slices: COUT0 from slice 0 into slice 1, and so on."),
        ("ALU0_C1", "Carry into bit 1 of ALU slice 0: a macrocell fed back inside the chip; the pin goes nowhere."),
        ("ALU0_C2", "Carry into bit 2 of ALU slice 0 (internal)."),
        ("ALU0_C3", "Carry into bit 3 of ALU slice 0 (internal)."),
        ("ALU1_C1", "Carry into bit 5 of the sum, inside ALU slice 1 (internal)."),
        ("ALU1_C2", "Carry into bit 6, inside ALU slice 1 (internal)."),
        ("ALU1_C3", "Carry into bit 7, inside ALU slice 1 (internal)."),
        ("ALU2_C1", "Carry into bit 9, inside ALU slice 2 (internal)."),
        ("ALU2_C2", "Carry into bit 10, inside ALU slice 2 (internal)."),
        ("ALU2_C3", "Carry into bit 11, inside ALU slice 2 (internal)."),
        ("ALU3_C1", "Carry into bit 13, inside ALU slice 3 (internal)."),
        ("ALU3_C2", "Carry into bit 14, inside ALU slice 3 (internal)."),
        ("ALU3_C3", "Carry into bit 15, inside ALU slice 3 (internal)."),
        ("CO", "The PC's carry from bit 7 into bit 8."),
        ("PCDRV", "Microword bit 0: the PC drives Addr, and counts a word at the edge. Also the flash's OE# through MEMRD."),
        ("ADRV", "Microword bit 11: A drives Addr (and so the ALU sees A). Never in the word next to a PCDRV word."),
        ("MEMRD_n", "Microword bit 1, active low: the memory or device that Addr selects drives D. OE# of the flash and SRAM, RD# of the UART."),
        ("ALUOE", "Microword bit 2: the ALU drives D."),
        ("WE_n", "Microword bit 3, active low: the memory or device that Addr selects takes D. WE# of the SRAM, WR# of the UART, for the whole clock."),
        ("ALD", "Microword bit 4: A copies D at the ending edge."),
        ("BLD", "Microword bit 5: B copies D at the ending edge."),
        ("PCLD", "Microword bit 6: the PC copies D at the ending edge."),
        ("PCINC", "Microword bit 7: PC += 2 at the ending edge."),
        ("IRLD", "Microword bit 8: the IR copies D[15:11] and the step counter clears at the ending edge. The fetch."),
        ("F", "Microword bits 9 and 10, the ALU function: F1 F0 = 00 add, 01 and, 10 nor, 11 pass B."),
        ("TLD", "Microword bit 12: T copies D at the ending edge."),
        ("TDRV", "Microword bit 13: T drives D."),
        ("AUX", "Microword bits 14 and 15, spare: debug outputs."),
        ("CLK", "The machine clock: the run/step multiplexer's output, 4 MHz from the oscillator or one edge per press of the step button. Every GAL's clock; every flop moves on its rising edge."),
        ("OSC", "The 4 MHz oscillator's output, into the run/step multiplexer and the clock header."),
        ("STEPMODE", "The slide switch: low = run (CLK is OSC), high = single step (CLK is the step button). Change it while holding reset."),
        ("STEPBTN", "STEPBTN_n: the step button, pulled up, with 100 nF for debounce; low while pressed."),
        ("STEPCLK", "The debounced step button through a Schmitt inverter: high while pressed, so a press is a rising edge of CLK in step mode."),
        ("VIN", "The barrel jack's centre pin, 5 V in."),
        ("VF", "5 V after the polyfuse, at the reverse-polarity MOSFET's drain."),
        ("VCC", "The 5 V rail."),
        ("LEDA", "Between a debug LED and its resistor."),
        ("LEDB", "A 74HC541 buffer output driving a debug LED: the level of the signal named."),
        ("MR", "MR_n: the MAX811L's manual reset input, pulled up inside it; the reset button pulls it low."),
        ("XIN", "The UART's own crystal, 14.7456 MHz: divisor 8 is 115200 baud. Nothing to do with the bus clock."),
        ("XOUT", "The other side of the UART's crystal."),
        ("RST_n", "The MAX811L's reset output, active low: power-on and the button."),
        ("RS", "RS1: the first stage of the reset synchroniser in the sequencer's flags chip, RST_n sampled on CLK. RESET is the second stage."),
        ("RESET", "Reset synchronised to CLK, active high, from the sequencer's flags chip: a factor of every product term in the IR, step counter, pipeline register, PC and NEL; the UART's MR."),
        ("MR_n", "The reset button, to the MAX811L's manual-reset input (pulled up inside the part)."),
        ("PBANK", "Program flash bank: A15..A18 of the program flash, from the 4-position DIP switch (pulled up; ON = 0; every switch ON here): 16 banks of 32 K words."),
        ("UBANK", "Microcode bank: A10..A12 of the microcode ROM, from three positions of a DIP switch (pulled up; ON = 0; every switch ON here): 8 microcode images."),
        ("SIN", "Serial in, from the transceiver."),
        ("SOUT", "Serial out, to the transceiver."),
        ("URTS_n", "The UART's RTS#, to the transceiver's second pair."),
        ("UCTS_n", "The UART's CTS#, from the transceiver's second pair."),
        ("UDTR_n", "DTR# looped back to DSR# and DCD#: the modem status always reads ready."),
        ("UBAUD", "BAUDOUT# to RCLK: the baud generator clocks its own receiver."),
        ("RS232_TX", "RS-232 TX, to the header."),
        ("RS232_RX", "RS-232 RX, from the header."),
        ("RS232_RTS", "RS-232 RTS, to the header."),
        ("RS232_CTS", "RS-232 CTS, from the header."),
        ("XC_c1A", "SP3232 charge pump: C1+ to its capacitor."),
        ("XC_c1B", "SP3232 charge pump: C1- to its capacitor."),
        ("XC_c2A", "SP3232 charge pump: C2+ to its capacitor."),
        ("XC_c2B", "SP3232 charge pump: C2- to its capacitor."),
        ("XC_VP", "SP3232 V+ reservoir."),
        ("XC_VM", "SP3232 V- reservoir."),
        ("GND", "Ground."),
        ("VCC", "+5 V."),
    ];
    d.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
}

/// The board file.
pub fn board() -> Board {
    let nl = build_netlist();
    let mut b = nl.export(
        "grit",
        "Microprogrammed 16-bit machine: ATF22V10C logic, SST39SF040 program flash and microcode ROM, AS7C164A SRAM, TL16C550 UART on its own crystal, MAX811L reset, 4 MHz oscillator with a single-step switch, debug LEDs, analyzer headers, 5 V barrel jack.",
        PERIOD as f64 / NS as f64,
        BTreeMap::new(),
        layout(),
    );
    b.bus_descriptions = bus_descriptions();
    b
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
    /// Power-up contents of the SRAM (None: zero, or random from the fuzz
    /// seed).
    pub ram_image: Option<Vec<u16>>,
    /// Physical fuzz (docs/fuzz.md): 0 = off.  Otherwise the seed for a
    /// propagation delay on every chip pin drawn from 0..`pin_delay_ns`,
    /// CLK2X's duty cycle drawn per period from `clock_duty` and up to
    /// `clock_jitter_ns` on every edge, and random power-up SRAM
    /// contents (give the reference the same, `fuzz_ram`).
    pub fuzz_seed: u64,
    pub pin_delay_ns: f64,
    /// Delay on the clock pins drawn from 0..this instead of
    /// `pin_delay_ns`: the clock skew between chips, on its own knob.
    /// None: clock pins get the same draw as every other pin.
    pub clock_skew_ns: Option<f64>,
    pub clock_duty: (f64, f64),
    pub clock_jitter_ns: f64,
}

impl Default for Build {
    fn default() -> Build {
        Build { reset_phase_ns: 37.0, reset_clocks: 8, uart_xin_hz: UART_XIN_HZ, uart_rx: Vec::new(), ram_image: None, fuzz_seed: 0, pin_delay_ns: 0.0, clock_skew_ns: None, clock_duty: (0.5, 0.5), clock_jitter_ns: 0.0 }
    }
}

/// The random power-up SRAM image for a fuzz seed.
pub fn fuzz_ram(seed: u64) -> Vec<u16> {
    let mut r = crate::cpu::Lcg(seed ^ 0x9e3779b97f4a7c15);
    (0..RAM_WORDS).map(|_| r.next() as u16).collect()
}

pub struct Grit {
    pub sim: Sim,
    pub clocks: u64,
    /// Fuzz: (generator, duty range, jitter ps).
    clock_fuzz: Option<(crate::cpu::Lcg, (f64, f64), Time)>,
    /// When RESET (the synchronised one) fell (ps).
    pub reset_release: Time,
    clk: NetId,
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
        let image: Option<Vec<u16>> = opt.ram_image.clone().or_else(|| (opt.fuzz_seed != 0).then(|| fuzz_ram(opt.fuzz_seed)));
        let mut ram = Vec::new();
        for (id, role) in nl.chips_with_role("ram:") {
            let lane: usize = role["ram:".len()..].parse().unwrap();
            let chip = nl.chip_mut::<As7c164a>(id);
            for i in 0..RAM_WORDS {
                let v = image.as_ref().map_or(0, |im| im[i]);
                chip.preload(i as u32, (v >> (8 * lane)) as u8);
            }
            ram.push(id);
        }
        ram.sort();
        if let Some(id) = nl.chips_with_role("uart").first().map(|(id, _)| *id) {
            nl.chip_mut::<Uart16550>(id).core.send(&opt.uart_rx);
        }
        let mut sim = nl.build();
        if opt.fuzz_seed != 0 && (opt.pin_delay_ns > 0.0 || opt.clock_skew_ns.is_some()) {
            let mut r = crate::cpu::Lcg(opt.fuzz_seed ^ 0xde1a);
            let max = opt.pin_delay_ns;
            let skew = opt.clock_skew_ns.unwrap_or(max);
            let clock_pins: std::collections::HashSet<(String, usize)> = board.chips.iter().flat_map(|c| c.pins.iter().filter(|p| p.net == "CLK").map(move |p| (c.name.clone(), p.pin))).collect();
            sim.set_pin_delays(|chip, pin| {
                let m = if clock_pins.contains(&(chip.to_string(), pin)) { skew } else { max };
                Self::ns(r.unit() * m)
            });
        }
        let clock_fuzz = (opt.fuzz_seed != 0).then(|| (crate::cpu::Lcg(opt.fuzz_seed ^ 0xc10c), opt.clock_duty, Self::ns(opt.clock_jitter_ns)));
        let clk = sim.net_id("CLK");
        let reset = sim.net_id("RESET");
        let ir = (0..5).map(|i| sim.net_id(&n("IR", i))).collect();
        let step = (0..4).map(|i| sim.net_id(&n("STEP", i))).collect();
        let addr = (0..16).map(|i| sim.net_id(&n("ADDR", i))).collect();
        let pcdrv = sim.net_id("PCDRV");
        let mut g = Grit { sim, clocks: 0, clock_fuzz, reset_release: 0, clk, reset, ir, step, addr, pcdrv, ram };
        g.wait_reset_release();
        g
    }

    /// One CLK2X period: low for the first half, high for the second, so
    /// that at power-up the clock settles low before its first rising
    /// edge.
    /// Schedule one clock period's edges from now and return the period's
    /// end, without running (for fine-grained tracing).  The clock is low
    /// for the first part of the period and high for the second, so that
    /// at power-up it settles low before its first rising edge.
    pub fn schedule_tick(&mut self) -> Time {
        let base = self.sim.now();
        let (fall, rise) = match &mut self.clock_fuzz {
            None => (base, base + PERIOD / 2),
            Some((r, duty, jitter)) => {
                let d = duty.0 + r.unit() * (duty.1 - duty.0);
                let j1 = (r.unit() * *jitter as f64) as Time;
                let j2 = (r.unit() * *jitter as f64) as Time;
                (base + j1, base + (PERIOD as f64 * (1.0 - d)) as Time + j2)
            }
        };
        self.sim.schedule(fall, self.clk, Level::L);
        self.sim.schedule(rise, self.clk, Level::H);
        base + PERIOD
    }

    /// One CLK period.
    pub fn clock(&mut self) {
        let end = self.schedule_tick();
        self.sim.run_until(end);
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
        self.sim.read_bus(&self.addr).map(|v| v as u16)
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
        let bits: Vec<(&str, String)> = (0..16).map(|i| (if i < 8 { "pc0" } else { "pc1" }, n("ADDR", i))).collect();
        self.gal_bus(&bits)
    }

    /// A word of the SRAM (address in 0x8000..0x9FFF, or a word index).
    pub fn ram_word(&self, addr: u16) -> Option<u16> {
        let i = (addr & 0x1FFF) as u32;
        let lo = self.sim.chip(self.ram[0]).downcast_ref::<As7c164a>().unwrap().peek(i)?;
        let hi = self.sim.chip(self.ram[1]).downcast_ref::<As7c164a>().unwrap().peek(i)?;
        Some(lo as u16 | (hi as u16) << 8)
    }
    pub fn reg(&self, r: usize) -> Option<u16> {
        self.ram_word(0x8000 + r as u16)
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

// ---------------------------------------------------------------------------
// Random programs

/// Random programs for the soak and the fuzz: statements over the
/// registers r1..r7 (SRAM words), two pointer registers into a data area,
/// a table of constants in the flash, forward conditional branches and
/// bounded loops, ending in HALT.  Everything is defined behaviour; the
/// reference interpreter says what the result must be.
pub mod soak {
    use super::*;
    use crate::cpu::Lcg;

    fn imm(r: &mut Lcg) -> u16 {
        match r.next() % 4 {
            0 => (r.next() % 16) as u16,
            1 => (r.next() % 256) as u16,
            2 => 0u16.wrapping_sub((r.next() % 16) as u16),
            _ => r.next() as u16,
        }
    }
    fn reg(r: &mut Lcg) -> u32 {
        1 + r.next() % 7
    }
    fn alu_b(r: &mut Lcg) -> &'static str {
        ["ADDB", "ANDB", "NORB"][(r.next() % 3) as usize]
    }
    fn alu_a(r: &mut Lcg) -> &'static str {
        ["ADDA", "ANDA", "NORA"][(r.next() % 3) as usize]
    }

    /// The program text for `seed`, about `len` statements long.
    pub fn program(seed: u64, len: usize) -> String {
        let mut r = Lcg(seed.wrapping_mul(0x2545F491_4F6CDD1D) ^ 0x5eed);
        let mut out = String::new();
        let mut labels = 0;
        let mut table: Vec<u16> = (0..8).map(|_| r.next() as u16).collect();
        // Pointers: r8 and r9 hold addresses in the data area.
        let p8 = 0x8100 + (r.next() % 64) as u16;
        let p9 = 0x8200 + (r.next() % 64) as u16;
        out.push_str(&format!("    LDA &r8\n    LDB {p8:#06x}\n    STB (A)\n    LDA &r9\n    LDB {p9:#06x}\n    STB (A)\n"));
        for i in 1..8 {
            let v = imm(&mut r);
            out.push_str(&format!("    LDA &r{i}\n    LDB {v:#06x}\n    STB (A)\n"));
        }
        let mut pending_loops: Vec<(String, u32)> = Vec::new();
        let mut n = 0;
        while n < len {
            n += 1;
            match r.next() % 12 {
                0 => {
                    let (x, v) = (reg(&mut r), imm(&mut r));
                    out.push_str(&format!("    LDA &r{x}\n    LDB {v:#06x}\n    STB (A)\n"));
                }
                1 | 2 => {
                    let (x, y, z, op) = (reg(&mut r), reg(&mut r), reg(&mut r), alu_b(&mut r));
                    out.push_str(&format!("    LDA &r{z}\n    LDB (A)\n    LDA &r{y}\n    LDA (A)\n    {op}\n    LDA &r{x}\n    STB (A)\n"));
                }
                3 => {
                    let (x, y, v, op) = (reg(&mut r), reg(&mut r), imm(&mut r), alu_b(&mut r));
                    out.push_str(&format!("    LDA &r{y}\n    LDA (A)\n    LDB {v:#06x}\n    {op}\n    LDA &r{x}\n    STB (A)\n"));
                }
                4 => {
                    // Through A: rX = rY op rZ computed into A, moved to B.
                    let (x, y, z, op) = (reg(&mut r), reg(&mut r), reg(&mut r), alu_a(&mut r));
                    out.push_str(&format!("    LDA &r{z}\n    LDB (A)\n    LDA &r{y}\n    LDA (A)\n    {op}\n    LDB 0\n    ADDB\n    LDA &r{x}\n    STB (A)\n"));
                }
                5 => {
                    // Store rX through a pointer.
                    let (x, p) = (reg(&mut r), 8 + r.next() % 2);
                    out.push_str(&format!("    LDA &r{x}\n    LDB (A)\n    LDA &r{p}\n    LDA (A)\n    STB (A)\n"));
                }
                6 => {
                    // Load rX through a pointer.
                    let (x, p) = (reg(&mut r), 8 + r.next() % 2);
                    out.push_str(&format!("    LDA &r{p}\n    LDA (A)\n    LDB (A)\n    LDA &r{x}\n    STB (A)\n"));
                }
                7 => {
                    // A constant from the flash.
                    let (x, k) = (reg(&mut r), r.next() % 8);
                    out.push_str(&format!("    LDA table{k}\n    LDB (A)\n    LDA &r{x}\n    STB (A)\n"));
                }
                8 => {
                    // MOVAB and a store of A.
                    let (x, y) = (reg(&mut r), reg(&mut r));
                    out.push_str(&format!("    LDA &r{y}\n    LDB (A)\n    MOVAB\n    LDB 0\n    ADDB\n    LDA &r{x}\n    STB (A)\n"));
                }
                9 => {
                    // Forward conditional skip: compare rX with rY or an immediate.
                    let (x, l) = (reg(&mut r), labels);
                    labels += 1;
                    if r.next() % 2 == 0 {
                        let y = reg(&mut r);
                        out.push_str(&format!("    LDA &r{y}\n    LDB (A)\n"));
                    } else {
                        let v = imm(&mut r);
                        out.push_str(&format!("    LDB {v:#06x}\n"));
                    }
                    out.push_str(&format!("    LDA &r{x}\n    LDA (A)\n    JEQ skip{l}\n"));
                    let (z, v) = (reg(&mut r), imm(&mut r));
                    out.push_str(&format!("    LDA &r{z}\n    LDB {v:#06x}\n    STB (A)\n    NOP\nskip{l}:\n"));
                }
                10 if pending_loops.len() < 2 => {
                    // A bounded loop: r(10 + depth) counts down from 1..4.
                    let depth = pending_loops.len() as u32;
                    let c = 10 + depth;
                    let count = 1 + r.next() % 4;
                    let l = labels;
                    labels += 1;
                    out.push_str(&format!("    LDA &r{c}\n    LDB {count}\n    STB (A)\nloop{l}:\n"));
                    pending_loops.push((format!("loop{l}"), c));
                }
                _ => {
                    // Close the innermost loop, if any: counter -= 1, exit at zero.
                    if let Some((label, c)) = pending_loops.pop() {
                        out.push_str(&format!("    LDA &r{c}\n    LDA (A)\n    LDB -1\n    ADDB\n    LDA &r{c}\n    STB (A)\n    LDA 0\n    JEQ {label}_x\n    JMP {label}\n{label}_x:\n"));
                    } else {
                        out.push_str("    NOP\n");
                    }
                }
            }
        }
        while let Some((label, c)) = pending_loops.pop() {
            out.push_str(&format!("    LDA &r{c}\n    LDA (A)\n    LDB -1\n    ADDB\n    LDA &r{c}\n    STB (A)\n    LDA 0\n    JEQ {label}_x\n    JMP {label}\n{label}_x:\n"));
        }
        out.push_str("    HALT\n");
        for (k, v) in table.drain(..).enumerate() {
            out.push_str(&format!("table{k}:\n    .word {v:#06x}\n"));
        }
        out
    }
}
