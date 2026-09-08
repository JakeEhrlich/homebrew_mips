//! Random program generation for the soak and fuzz tests (`tests/soak.rs`,
//! `tests/fuzz.rs`): every instruction of the subset, dense with hazards,
//! avoiding only what MIPS I leaves undefined.
use std::fmt::Write;

pub struct Rng(pub u64);
impl Rng {
    fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 33) as u32
    }
    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }
    fn pick<'a, T>(&mut self, v: &'a [T]) -> &'a T {
        &v[self.below(v.len() as u32) as usize]
    }
    fn chance(&mut self, percent: u32) -> bool {
        self.below(100) < percent
    }
}

/// Registers the generator writes and reads freely.
const DATA_REGS: &[u8] = &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 20, 21, 22, 23];
/// Address bases: 0, 256, 512, 768 (never modified).
const BASES: &[u8] = &[16, 17, 18, 19];
const LOOP_REG: u8 = 24;
const JUMP_REG: u8 = 26;

struct Gen {
    rng: Rng,
    out: String,
    labels: usize,
    /// The previous instruction was a load into this register (its
    /// delay slot must not store it).
    last_load: Option<u8>,
    /// The last few destinations: sources are drawn from them half the
    /// time, so forwarding and the interlocks are exercised constantly.
    recent: Vec<u8>,
}

impl Gen {
    fn label(&mut self) -> String {
        self.labels += 1;
        format!("L{}", self.labels)
    }
    fn reg(&mut self) -> u8 {
        *self.rng.pick(DATA_REGS)
    }
    /// A source register, biased towards recent destinations.
    fn src(&mut self) -> u8 {
        if !self.recent.is_empty() && self.rng.chance(50) {
            *self.rng.pick(&self.recent.clone())
        } else {
            self.reg()
        }
    }
    fn dest(&mut self) -> u8 {
        let d = self.reg();
        self.recent.push(d);
        if self.recent.len() > 3 {
            self.recent.remove(0);
        }
        d
    }
    fn emit(&mut self, s: &str) {
        writeln!(self.out, "        {s}").unwrap();
        self.last_load = None;
    }
    fn alu(&mut self) {
        let (s, t) = (self.src(), self.src());
        let d = self.dest();
        let k = self.rng.below(19);
        let imm = self.rng.next() as i16 as i32;
        let uimm = self.rng.next() & 0xFFFF;
        let sh = self.rng.below(32);
        let ins = match k {
            0 => format!("addu ${d}, ${s}, ${t}"),
            1 => format!("subu ${d}, ${s}, ${t}"),
            2 => format!("and ${d}, ${s}, ${t}"),
            3 => format!("or ${d}, ${s}, ${t}"),
            4 => format!("xor ${d}, ${s}, ${t}"),
            5 => format!("nor ${d}, ${s}, ${t}"),
            6 => format!("slt ${d}, ${s}, ${t}"),
            7 => format!("sltu ${d}, ${s}, ${t}"),
            8 => format!("addiu ${d}, ${s}, {imm}"),
            9 => format!("andi ${d}, ${s}, {uimm}"),
            10 => format!("ori ${d}, ${s}, {uimm}"),
            11 => format!("xori ${d}, ${s}, {uimm}"),
            12 => format!("slti ${d}, ${s}, {imm}"),
            13 => format!("sltiu ${d}, ${s}, {imm}"),
            14 => format!("lui ${d}, {uimm}"),
            15 => format!("sll ${d}, ${t}, {sh}"),
            16 => format!("srl ${d}, ${t}, {sh}"),
            17 => format!("sra ${d}, ${t}, {sh}"),
            _ => {
                let v = *self.rng.pick(&["sllv", "srlv", "srav"]);
                format!("{v} ${d}, ${t}, ${s}")
            }
        };
        self.emit(&ins);
    }
    fn load(&mut self) {
        let d = self.dest();
        let b = *self.rng.pick(BASES);
        let (op, align) = *self.rng.pick(&[("lw", 4), ("lb", 1), ("lbu", 1), ("lh", 2), ("lhu", 2)]);
        let off = self.rng.below(256 / align) * align;
        self.emit(&format!("{op} ${d}, {off}(${b})"));
        self.last_load = Some(d);
    }
    fn store(&mut self) {
        let mut t = self.src();
        if self.last_load == Some(t) {
            t = if t == 1 { 2 } else { 1 };
        }
        let b = *self.rng.pick(BASES);
        let (op, align) = *self.rng.pick(&[("sw", 4), ("sb", 1), ("sh", 2)]);
        let off = self.rng.below(256 / align) * align;
        self.emit(&format!("{op} ${t}, {off}(${b})"));
    }
    /// One instruction that is safe anywhere (a delay slot included).
    fn filler(&mut self) {
        if self.rng.chance(70) {
            self.alu();
        } else if self.rng.chance(50) {
            self.load();
        } else {
            self.emit("nop");
        }
    }
    /// A straight-line instruction.
    fn straight(&mut self) {
        match self.rng.below(10) {
            0..=5 => self.alu(),
            6..=7 => self.load(),
            _ => self.store(),
        }
    }
    /// A forward branch over a few instructions, taken or not, with a
    /// delay slot.
    fn branch(&mut self, links_allowed: bool) {
        let (s, t) = (self.src(), self.src());
        let l = self.label();
        let mut kinds = vec![format!("beq ${s}, ${t}, {l}"), format!("bne ${s}, ${t}, {l}"), format!("blez ${s}, {l}"), format!("bgtz ${s}, {l}"), format!("bltz ${s}, {l}"), format!("bgez ${s}, {l}")];
        if links_allowed {
            kinds.push(format!("bltzal ${s}, {l}"));
            kinds.push(format!("bgezal ${s}, {l}"));
        }
        let ins = self.rng.pick(&kinds).clone();
        self.emit(&ins);
        self.filler(); // delay slot
        for _ in 0..self.rng.below(4) {
            self.straight();
        }
        writeln!(self.out, "    {l}:").unwrap();
    }
    /// A jump to a forward label through a register.
    fn jump_reg(&mut self) {
        let l = self.label();
        self.emit(&format!("li ${JUMP_REG}, {l}"));
        self.emit(&format!("jr ${JUMP_REG}"));
        self.filler();
        for _ in 0..self.rng.below(3) {
            self.straight();
        }
        writeln!(self.out, "    {l}:").unwrap();
    }
    /// A counted loop.
    fn bounded_loop(&mut self) {
        let n = 1 + self.rng.below(4);
        let l = self.label();
        self.emit(&format!("li ${LOOP_REG}, {n}"));
        writeln!(self.out, "    {l}:").unwrap();
        for _ in 0..2 + self.rng.below(5) {
            self.straight();
        }
        self.emit(&format!("addiu ${LOOP_REG}, ${LOOP_REG}, -1"));
        self.emit(&format!("bne ${LOOP_REG}, $zero, {l}"));
        self.filler();
    }
    /// A call to subroutine `k` (JAL, or JALR through a register).
    fn call(&mut self, k: usize) {
        if self.rng.chance(50) {
            self.emit(&format!("jal sub{k}"));
        } else {
            self.emit(&format!("li ${JUMP_REG}, sub{k}"));
            self.emit(&format!("jalr ${JUMP_REG}"));
        }
        self.filler();
    }
    fn block(&mut self, subs: usize) {
        match self.rng.below(12) {
            0..=6 => self.straight(),
            7..=8 => self.branch(true),
            9 => self.jump_reg(),
            10 => self.bounded_loop(),
            _ => {
                if subs > 0 {
                    let k = self.rng.below(subs as u32) as usize;
                    self.call(k);
                } else {
                    self.straight();
                }
            }
        }
    }
}

/// A random program of about `len` lines for `seed`.
pub fn program(seed: u64, len: usize) -> String {
    let mut g = Gen { rng: Rng(seed), out: String::new(), labels: 0, last_load: None, recent: Vec::new() };
    // Register set-up: random values, the address bases.
    for &r in DATA_REGS {
        let v = g.rng.next();
        g.emit(&format!("lui ${r}, {}", v >> 16));
        g.emit(&format!("ori ${r}, ${r}, {}", v & 0xFFFF));
    }
    for (i, &b) in BASES.iter().enumerate() {
        g.emit(&format!("li ${b}, {}", 256 * i));
    }
    let subs = 2;
    let mut count = 0;
    while count < len {
        g.block(subs);
        count = g.out.lines().count();
    }
    g.emit("j stop");
    g.emit("nop");
    // Subroutines: no link branches inside (they would clobber $ra).
    for k in 0..subs {
        writeln!(g.out, "    sub{k}:").unwrap();
        for _ in 0..4 + g.rng.below(8) {
            if g.rng.chance(20) {
                g.branch(false);
            } else {
                g.straight();
            }
        }
        g.emit("jr $ra");
        g.filler();
    }
    writeln!(g.out, "        nop\n        nop\n        nop\n    stop:\n        nop\n        nop\n        nop\n        nop").unwrap();
    g.out
}

