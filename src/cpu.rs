//! The phase-1 CPU as a netlist: equations for every GAL, the sixteen SRAMs,
//! and a harness that runs programs on it.
//!
//! See `docs/pipeline-plan.md` for the stage plan.  Decisions taken here that
//! refine it:
//!
//! * PC is 13 bits (`PC2..PC14`, word address in the 32 KB instruction
//!   space); upper address bits are hard zero.  The branch-target adder is
//!   therefore 13 bits too.
//! * Branches and JR resolve in EX: a separate 32-bit equality comparator
//!   on the forwarded operands, a 13-bit target adder in ID whose final
//!   select is folded into the ID/EX register, JR's target straight from the
//!   forwarded rs.  A taken branch/JR kills the instruction being captured
//!   into IF/ID.  J/JAL resolve in ID with no penalty.
//! * The ALU is: forwarding mux -> 3-bit carry-select slices -> group carry
//!   lookahead -> (sum select + logic ops) folded into the EX/MEM result
//!   register.  Subtract inverts B with carry-in 1; SLTU is the carry out,
//!   SLT comes from the sign bits.
//! * Data SRAM: CE# is a clock-generator strobe, WE# a registered level,
//!   OE# grounded; store data comes from a tri-state register enabled by a
//!   strobe.  Register file: CE_W is a strobe, R/W a registered level.
//!
//! Net naming: buses are `NAME{bit}`.  Stage prefixes: `IR`/`P4` (IF/ID),
//! `X*` (ID/EX), `M*` (EX/MEM), `W*` (MEM/WB).

use crate::as7c164a::{self, As7c164a};
use crate::cy7c131::{Cy7c131, Port};
use crate::galpack::{Eq, GalSpec, Mode, SLit, instantiate_all, lit, nlit, pack};
use crate::isa::Instr;
use crate::ds1100::{Ds1100, Grade};
use crate::uart16550::{BusTiming, Uart16550, UartPin, uart_pin_of};
use crate::netlist::{DS1100_IN, FastGate, Level, NetId, Netlist, ResetSupervisor, Rom, RomPin, Sim, Sram16, Sram16Pin, SramPin, Sram8kPin, Time, NS, ds1100_tap_pin, rom_pin_of, sram_pin_of, sram8k_pin_of, sram16_pin_of};

// ---------------------------------------------------------------------------
// Helpers

fn n(prefix: &str, i: usize) -> String {
    format!("{prefix}{i}")
}
fn bus(prefix: &str, lo: usize, hi: usize) -> Vec<String> {
    (lo..=hi).map(|i| n(prefix, i)).collect()
}
fn strs(v: &[String]) -> Vec<&str> {
    v.iter().map(String::as_str).collect()
}
fn l(s: &str) -> SLit {
    lit(s)
}
fn nl_(s: &str) -> SLit {
    nlit(s)
}

/// Instruction field bit -> IF/ID net.
fn ir(bit: usize) -> String {
    n("IR", bit)
}
/// PC+4 bit (in IF/ID) -> net.  Underscore so the diagram groups it as P4.
fn p4(bit: usize) -> String {
    format!("P4_{bit}")
}

// ---------------------------------------------------------------------------
// Decode (combinatorial, from opcode and funct).  Truth tables over the 12
// bits IR31..IR26, IR5..IR0.

#[derive(Clone, Copy)]
struct Dec {
    rtype_rw: bool, // R-type writing rd
    itype_rw: bool, // I-type writing rt (ALU immediates, LUI, LW)
    jal: bool,
    selimm: bool, // B register takes the immediate
    sext: bool,   // immediate sign-extended (else zero-extended)
    lui: bool,
    seljt: bool, // J or JAL
    uses_rt: bool, // rt is an ALU/compare operand (forward into B)
    store: bool,
    add: bool,
    sub: bool,
    and: bool,
    or: bool,
    xor: bool,
    nor: bool,
    slt: bool,
    sltu: bool,
    beq: bool,
    bne: bool,
    /// Branch on a condition of rs alone (BLEZ, BGTZ, BLTZ, BGEZ).
    brs: bool,
    /// ... including "rs == 0" (BLEZ, BGTZ).
    bz: bool,
    /// ... inverted (BGTZ, BGEZ).
    binv: bool,
    jr: bool,
    /// JAL or JALR: B takes the link (PC+8) and the ALU passes it through.
    link: bool,
    load: bool,
    /// Shift instruction: the result is the shifter output.
    sh: bool,
    /// ... right (SRL, SRA, SRLV, SRAV): amount complemented, mask reversed.
    shr: bool,
    /// ... arithmetic (SRA, SRAV): fill with the sign.
    sra: bool,
    /// ... constant amount from the shamt field (SLL, SRL, SRA).
    shimm: bool,
}

fn decode(word: u32) -> Option<Dec> {
    use crate::isa::Op::*;
    let i = Instr::decode(word)?;
    let op = i.op();
    let mut d = Dec {
        rtype_rw: false,
        itype_rw: false,
        jal: false,
        selimm: false,
        sext: false,
        lui: false,
        seljt: false,
        uses_rt: false,
        store: false,
        add: false,
        sub: false,
        and: false,
        or: false,
        xor: false,
        nor: false,
        slt: false,
        sltu: false,
        beq: false,
        bne: false,
        brs: false,
        bz: false,
        binv: false,
        jr: false,
        link: false,
        load: false,
        sh: false,
        shr: false,
        sra: false,
        shimm: false,
    };
    match op {
        Nop => {}
        Sll | Srl | Sra | Sllv | Srlv | Srav => {
            d.rtype_rw = true;
            d.uses_rt = true; // the shifted operand is rt, forwarded into B
            d.sh = true;
            d.shr = matches!(op, Srl | Sra | Srlv | Srav);
            d.sra = matches!(op, Sra | Srav);
            d.shimm = matches!(op, Sll | Srl | Sra);
        }
        Addu | Subu | And | Or | Xor | Nor | Slt | Sltu => {
            d.rtype_rw = true;
            d.uses_rt = true;
        }
        Jr => d.jr = true,
        Jalr => {
            d.rtype_rw = true;
            d.jr = true;
            d.link = true;
        }
        Addiu | Andi | Ori | Xori | Slti | Sltiu | Lui | Lw => {
            d.itype_rw = true;
            d.selimm = true;
        }
        Sw => {
            d.selimm = true;
            d.store = true;
        }
        Beq | Bne | Blez | Bgtz => d.uses_rt = true,
        Bltz | Bgez => {}
        J => d.seljt = true,
        Jal => {
            d.seljt = true;
            d.jal = true;
            d.link = true;
        }
    }
    d.brs = matches!(op, Blez | Bgtz | Bltz | Bgez);
    d.bz = matches!(op, Blez | Bgtz);
    d.binv = matches!(op, Bgtz | Bgez);
    d.sext = matches!(op, Addiu | Slti | Sltiu | Lw | Sw);
    d.lui = op == Lui;
    // XADD: the result is the adder output (SLT/SLTU use the adder but
    // produce only the compare bit).  XSUB: invert B, carry-in 1.
    d.add = matches!(op, Addu | Addiu | Subu | Lui | Lw | Sw | Jr | Nop);
    d.sub = matches!(op, Subu | Slt | Sltu | Slti | Sltiu);
    d.and = matches!(op, And | Andi);
    d.or = matches!(op, Or | Ori);
    d.xor = matches!(op, Xor | Xori);
    d.nor = op == Nor;
    d.slt = matches!(op, Slt | Slti);
    d.sltu = matches!(op, Sltu | Sltiu);
    d.beq = op == Beq;
    d.bne = op == Bne;
    d.load = op == Lw;
    Some(d)
}

/// Inputs of the decode tables: opcode then funct, as net names.
fn dec_inputs() -> Vec<String> {
    (26..=31).map(ir).chain((0..=5).map(ir)).collect()
}
/// Map a 12-bit table index (bit i = dec_inputs()[i]) to an instruction word.
fn dec_word(m: u32) -> u32 {
    let opcode = m & 0x3F;
    let funct = m >> 6 & 0x3F;
    // Opcode 0 / funct 0 is SLL; the all-zero word (NOP) is just
    // `sll $0, $0, 0`, whose write is suppressed by the r0 check in XRW.
    // Give it a non-zero rd so `Instr::decode` does not report NOP.
    opcode << 26 | funct | if opcode == 0 && funct == 0 { 1 << 11 } else { 0 }
}
/// ALU operation code carried in ID/EX as XOP2..0 (sub is add with XSUB).
fn alu_op(d: &Dec) -> u32 {
    if d.link { 1 } else if d.and { 2 } else if d.or { 3 } else if d.xor { 4 } else if d.nor { 5 } else if d.slt { 6 } else if d.sltu { 7 } else { 0 }
}
/// Literals selecting ALU operation `code` in the EX/MEM result terms.
fn op_lits(code: u32) -> Vec<SLit> {
    (0..3).map(|b| if code >> b & 1 == 1 { l(&format!("XOP{b}")) } else { nl_(&format!("XOP{b}")) }).collect()
}

fn dec_table(out: &str, mode: Mode, f: impl Fn(&Dec) -> bool) -> Eq {
    let ins = dec_inputs();
    Eq::table(out, mode, &strs(&ins), |m| decode(dec_word(m)).map(|d| f(&d)))
}

// ---------------------------------------------------------------------------
// Blocks.  Each returns equations; `pack` turns them into chips.

/// PC register (13 bits) with 4:1 mux: INC / JT / BT / AF, async reset.
fn pc_block() -> Vec<Eq> {
    let mut eqs: Vec<Eq> = (2..=14)
        .map(|i| {
            Eq::sop(
                &n("PC", i),
                Mode::Reg,
                vec![
                    vec![l("SELINC"), l(&n("INC", i))],
                    vec![l("SELJT"), l(&ir(i - 2))],
                    vec![l("SELBT"), l(&n("XBT", i))],
                    vec![l("SELAF"), l(&n("FA", i))],
                ],
            )
        })
        .collect();
    // Bit 15: the incrementer's carry out.  Not an address; it is the boot
    // copier's WRAP flag (the PC has run past 8K words), set for one
    // increment.
    eqs.push(Eq::sop("PC15", Mode::Reg, vec![vec![l("SELINC"), l("INC15")]]));
    eqs
}

/// PC + 1 (13 bits): two slices, the second taking the first's group
/// propagate as its carry-in.
fn inc_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    // Slice 0: bits 2..10 (9 bits), carry-in 1.
    let ins0: Vec<String> = bus("PC", 2, 10);
    for i in 2..=10 {
        let bit = i - 2;
        eqs.push(Eq::table(&n("INC", i), Mode::Comb, &strs(&ins0), move |m| Some(((m & 0x1FF) + 1) >> bit & 1 == 1)));
    }
    eqs.push(Eq::table("INCG0", Mode::Comb, &strs(&ins0), |m| Some(m & 0x1FF == 0x1FF)));
    // Slice 1: bits 11..14 with carry-in INCG0.
    let mut ins1 = bus("PC", 11, 14);
    ins1.push("INCG0".into());
    for i in 11..=14 {
        let bit = i - 11;
        eqs.push(Eq::table(&n("INC", i), Mode::Comb, &strs(&ins1), move |m| {
            let cin = m >> 4 & 1;
            Some(((m & 0xF) + cin) >> bit & 1 == 1)
        }));
    }
    eqs.push(Eq::table("INC15", Mode::Comb, &strs(&ins1), |m| Some(m == 0x1F)));
    eqs
}

/// IF/ID: instruction (killed to NOP on a taken branch) and PC+4.
fn ifid_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    for i in 0..32 {
        eqs.push(Eq::sop(&ir(i), Mode::Reg, vec![vec![l(&n("IM", i)), nl_("KILL")]]));
    }
    for i in 2..=14 {
        eqs.push(Eq::sop(&p4(i), Mode::Reg, vec![vec![l(&n("INC", i))]]));
    }
    eqs
}

/// Combinatorial decode signals used by the ID-stage registers' D terms.
fn dec_block() -> Vec<Eq> {
    vec![
        dec_table("RTYPERW", Mode::Comb, |d| d.rtype_rw),
        dec_table("ITYPERW", Mode::Comb, |d| d.itype_rw),
        dec_table("JAL", Mode::Comb, |d| d.jal),
        dec_table("SELIMM", Mode::Comb, |d| d.selimm),
        dec_table("SEXT", Mode::Comb, |d| d.sext),
        dec_table("LUI", Mode::Comb, |d| d.lui),
        dec_table("DSELJT", Mode::Comb, |d| d.seljt),
        dec_table("USESRT", Mode::Comb, |d| d.uses_rt),
        dec_table("STORE", Mode::Comb, |d| d.store),
        dec_table("LINK", Mode::Comb, |d| d.link),
        dec_table("SHIMM", Mode::Comb, |d| d.shimm),
        dec_table("LOAD", Mode::Comb, |d| d.load),
    ]
}

/// ID/EX control: ALU op and branch bits straight from the opcode/funct
/// tables (decode folded into the register), plus destination / write
/// enable, memory bits.
fn ctrl_block() -> Vec<Eq> {
    let mut eqs = vec![
        dec_table("XOP0", Mode::Reg, |d| alu_op(d) & 1 == 1),
        dec_table("XOP1", Mode::Reg, |d| alu_op(d) >> 1 & 1 == 1),
        dec_table("XOP2", Mode::Reg, |d| alu_op(d) >> 2 & 1 == 1),
        dec_table("XSUB", Mode::Reg, |d| d.sub),
        dec_table("XSH", Mode::Reg, |d| d.sh),
        dec_table("XSHR", Mode::Reg, |d| d.shr),
        dec_table("XSRA", Mode::Reg, |d| d.sra),
        dec_table("XBEQ", Mode::Reg, |d| d.beq),
        dec_table("XBNE", Mode::Reg, |d| d.bne),
        dec_table("XBRS", Mode::Reg, |d| d.brs),
        dec_table("XBZ", Mode::Reg, |d| d.bz),
        // BLTZ and BGEZ share opcode 1 (REGIMM) and differ in the rt field,
        // which the opcode/funct tables cannot see: invert when BGTZ
        // (opcode 7) or REGIMM with IR16 set.
        Eq::sop(
            "XBINV",
            Mode::Reg,
            vec![
                vec![nl_(&ir(31)), nl_(&ir(30)), nl_(&ir(29)), l(&ir(28)), l(&ir(27)), l(&ir(26))],
                vec![nl_(&ir(31)), nl_(&ir(30)), nl_(&ir(29)), nl_(&ir(28)), nl_(&ir(27)), l(&ir(26)), l(&ir(16))],
            ],
        ),
        dec_table("XJR", Mode::Reg, |d| d.jr),
        dec_table("XMR", Mode::Reg, |d| d.load),
        dec_table("XMW", Mode::Reg, |d| d.store),
    ];
    // Destination: rd for R-type, rt for I-type, 31 for JAL.
    for i in 0..5 {
        eqs.push(Eq::sop(
            &n("XDEST", i),
            Mode::Reg,
            vec![vec![l("RTYPERW"), l(&ir(11 + i))], vec![l("ITYPERW"), l(&ir(16 + i))], vec![l("JAL")]],
        ));
    }
    // Register write: class allows it and the destination is not r0.
    let mut rw = vec![vec![l("JAL")]];
    for i in 0..5 {
        rw.push(vec![l("RTYPERW"), l(&ir(11 + i))]);
        rw.push(vec![l("ITYPERW"), l(&ir(16 + i))]);
    }
    eqs.push(Eq::sop("XRW", Mode::Reg, rw));
    // rs / rt numbers travel along for the forwarding control.
    for i in 0..5 {
        eqs.push(Eq::sop(&n("XRS", i), Mode::Reg, vec![vec![l(&ir(21 + i))]]));
        eqs.push(Eq::sop(&n("XRT", i), Mode::Reg, vec![vec![l(&ir(16 + i))]]));
    }
    eqs
}

/// Register-file steer: a source equal to the register being written this
/// cycle (MEM/WB) is read from MEM/WB instead, and that bank's read port is
/// disabled.  STA/STB select the ID/EX mux; CERA_n/CERB_n go to the chips.
fn steer_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    for (src, sel, ce, field) in [("a", "STA", "CERA_n", 21), ("b", "STB", "CERB_n", 16)] {
        let _ = src;
        let mut ins: Vec<String> = (0..5).map(|i| ir(field + i)).collect();
        ins.extend(bus("WDEST", 0, 4));
        ins.push("WREG".into());
        let f = |m: u32| Some((m & 31) == (m >> 5 & 31) && m >> 10 & 1 == 1);
        eqs.push(Eq::table(sel, Mode::Comb, &strs(&ins), f));
        // CE is active low: high (port off) exactly when steered.
        eqs.push(Eq::table(ce, Mode::Comb, &strs(&ins), f));
    }
    eqs
}

/// Forwarding control, computed in ID and registered: for each operand,
/// whether the writer one ahead (now in EX, in EX/MEM next cycle) or two
/// ahead (now in MEM, in MEM/WB next cycle) produces it.  Both may be set;
/// the muxes give EX/MEM priority.  Written active-low because the
/// complement of a 5-bit equality is ten terms while the equality is 32.
fn fwdctl_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    for (field, gate, pre) in [(21usize, None, "XFA"), (16, Some("USESRT"), "XFB"), (16, Some("STORE"), "XFS")] {
        for (suffix, dest, rw) in [("E", "XDEST", "XRW"), ("M", "MDEST", "MRW")] {
            // !hit = !gate + !rw + (r == 0) + OR_i (r_i != dest_i)
            //        (+ "writer is a load" for the EX/MEM source: its EX/MEM
            //        value is the address, and the load delay slot sees the
            //        old register, as in the reference simulator)
            let mut terms: Vec<Vec<SLit>> = vec![vec![nl_(rw)]];
            if suffix == "E" {
                terms.push(vec![l("XMR")]);
            }
            if let Some(g) = gate {
                terms.push(vec![nl_(g)]);
            }
            terms.push((0..5).map(|i| nl_(&ir(field + i))).collect());
            for i in 0..5 {
                terms.push(vec![l(&ir(field + i)), nl_(&n(dest, i))]);
                terms.push(vec![nl_(&ir(field + i)), l(&n(dest, i))]);
            }
            eqs.push(Eq::sop(&format!("{pre}{suffix}"), Mode::Reg, terms).active_low());
        }
    }
    eqs
}

/// ID/EX operand A: register file, or MEM/WB when steered.  For constant
/// shifts (rs is r0, so RA is zero) bits 4:0 also take the shamt field, so
/// the shifter always finds its amount in A.
fn idex_a_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let mut terms = vec![vec![nl_("STA"), l(&n("RA", i))], vec![l("STA"), l(&n("WD", i))]];
            if i < 5 {
                terms.push(vec![l("SHIMM"), l(&ir(6 + i))]);
            }
            Eq::sop(&n("XA", i), Mode::Reg, terms)
        })
        .collect()
}

/// ID/EX operand B: register file / MEM/WB / immediate (sign, zero or
/// LUI-extended) / the link address for JAL and JALR.  While the jump is in
/// ID the incrementer holds PC+4 of its delay slot, i.e. PC+8: exactly the
/// link, with no adder.
fn idex_b_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let mut terms = vec![
                vec![nl_("SELIMM"), nl_("LINK"), nl_("STB"), l(&n("RB", i))],
                vec![nl_("SELIMM"), nl_("LINK"), l("STB"), l(&n("WD", i))],
            ];
            if i < 16 {
                terms.push(vec![l("SELIMM"), nl_("LUI"), l(&ir(i))]);
            } else {
                terms.push(vec![l("SELIMM"), l("SEXT"), l(&ir(15))]);
                terms.push(vec![l("SELIMM"), l("LUI"), l(&ir(i - 16))]);
            }
            if (2..=14).contains(&i) {
                terms.push(vec![l("LINK"), l(&n("INC", i))]);
            }
            Eq::sop(&n("XB", i), Mode::Reg, terms)
        })
        .collect()
}

/// ID/EX store data: rt from the register file or MEM/WB.
fn idex_sd_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            Eq::sop(
                &n("XSD", i),
                Mode::Reg,
                vec![vec![nl_("STB"), l(&n("RB", i))], vec![l("STB"), l(&n("WD", i))]],
            )
        })
        .collect()
}

/// Branch target adder, 13 bits: P4[14:2] + IR[12:0] (the low 13 bits of
/// the sign-extended offset in words).  3-bit carry-select slices, group
/// carries, and the final select folded into the ID/EX BT register.
fn bt_block() -> (Vec<Eq>, Vec<Eq>, Vec<Eq>) {
    let mut l1 = Vec::new();
    let mut l2 = Vec::new();
    let mut reg = Vec::new();
    // Groups: bits 2-4, 5-7, 8-10, 11-13, 14 (PC bit index).
    let groups: Vec<(usize, usize)> = vec![(2, 4), (5, 7), (8, 10), (11, 13), (14, 14)];
    for (k, &(lo, hi)) in groups.iter().enumerate() {
        let w = hi - lo + 1;
        let mut ins: Vec<String> = (lo..=hi).map(|i| p4(i)).collect();
        ins.extend((lo..=hi).map(|i| ir(i - 2)));
        let mask = (1u32 << w) - 1;
        for cin in 0..2u32 {
            for bit in 0..w {
                let name = format!("BS{cin}_{}", lo + bit);
                l1.push(Eq::table(&name, Mode::Comb, &strs(&ins), move |m| {
                    Some(((m & mask) + (m >> w & mask) + cin) >> bit & 1 == 1)
                }));
            }
        }
        if k + 1 < groups.len() {
            l1.push(Eq::table(&format!("BG{k}"), Mode::Comb, &strs(&ins), move |m| {
                Some(((m & mask) + (m >> w & mask)) >> w & 1 == 1)
            }));
            l1.push(Eq::table(&format!("BP{k}"), Mode::Comb, &strs(&ins), move |m| {
                Some(((m & mask) + (m >> w & mask) + 1) >> w & 1 == 1)
            }));
        }
    }
    // Carries into groups 1..4.
    for k in 1..groups.len() {
        let mut terms = Vec::new();
        for j in 0..k {
            let mut t = vec![l(&format!("BG{j}"))];
            for i in (j + 1)..k {
                t.push(l(&format!("BP{i}")));
            }
            terms.push(t);
        }
        l2.push(Eq::sop(&format!("BC{k}"), Mode::Comb, terms));
    }
    // Register with select.
    for (k, &(lo, hi)) in groups.iter().enumerate() {
        for i in lo..=hi {
            let terms = if k == 0 {
                vec![vec![l(&format!("BS0_{i}"))]]
            } else {
                vec![
                    vec![l(&format!("BC{k}")), l(&format!("BS1_{i}"))],
                    vec![nl_(&format!("BC{k}")), l(&format!("BS0_{i}"))],
                ]
            };
            reg.push(Eq::sop(&n("XBT", i), Mode::Reg, terms));
        }
    }
    (l1, l2, reg)
}

/// EX forwarding muxes.  FA = A operand; FB = B operand, inverted for
/// subtract.  Sources: ID/EX register, EX/MEM result (MR), MEM/WB data (WD).
fn fwd_mux_block() -> (Vec<Eq>, Vec<Eq>) {
    // Bits 4:0 carry the shift amount; a right shift by s is done as a
    // left rotation by ~s (+1 inside stage 2), so they are complemented
    // when XSHR.
    let fa = (0..32)
        .map(|i| {
            let src = [
                (vec![nl_("XFAE"), nl_("XFAM")], n("XA", i)),
                (vec![l("XFAE")], n("MR", i)),
                (vec![nl_("XFAE"), l("XFAM")], n("WD", i)),
            ];
            let mut terms = Vec::new();
            for (sel, d) in &src {
                if i < 5 {
                    let mut t = sel.clone();
                    t.push(nl_("XSHR"));
                    t.push(l(d));
                    terms.push(t);
                    let mut t = sel.clone();
                    t.push(l("XSHR"));
                    t.push(nl_(d));
                    terms.push(t);
                } else {
                    let mut t = sel.clone();
                    t.push(l(d));
                    terms.push(t);
                }
            }
            Eq::sop(&n("FA", i), Mode::Comb, terms)
        })
        .collect();
    let fb = (0..32)
        .map(|i| {
            let src = [
                (vec![nl_("XFBE"), nl_("XFBM")], n("XB", i)),
                (vec![l("XFBE")], n("MR", i)),
                (vec![nl_("XFBE"), l("XFBM")], n("WD", i)),
            ];
            let mut terms = Vec::new();
            for (sel, d) in &src {
                let mut t = sel.clone();
                t.push(nl_("XSUB"));
                t.push(l(d));
                terms.push(t);
                let mut t = sel.clone();
                t.push(l("XSUB"));
                t.push(nl_(d));
                terms.push(t);
            }
            Eq::sop(&n("FB", i), Mode::Comb, terms)
        })
        .collect();
    (fa, fb)
}

/// ALU level 1: 3-bit carry-select slices over FA/FB (groups 0..9 = bits
/// 3k..3k+2, group 10 = bits 30..31).
fn alu_l1_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    for k in 0..11 {
        let lo = 3 * k;
        let hi = (lo + 2).min(31);
        let w = hi - lo + 1;
        let mut ins: Vec<String> = (lo..=hi).map(|i| n("FA", i)).collect();
        ins.extend((lo..=hi).map(|i| n("FB", i)));
        let mask = (1u32 << w) - 1;
        for cin in 0..2u32 {
            for bit in 0..w {
                eqs.push(Eq::table(&format!("S{cin}_{}", lo + bit), Mode::Comb, &strs(&ins), move |m| {
                    Some(((m & mask) + (m >> w & mask) + cin) >> bit & 1 == 1)
                }));
            }
        }
        eqs.push(Eq::table(&format!("G{k}"), Mode::Comb, &strs(&ins), move |m| {
            Some(((m & mask) + (m >> w & mask)) >> w & 1 == 1)
        }));
        eqs.push(Eq::table(&format!("PP{k}"), Mode::Comb, &strs(&ins), move |m| {
            Some(((m & mask) + (m >> w & mask) + 1) >> w & 1 == 1)
        }));
    }
    eqs
}

/// ALU level 2: carries into groups 1..10, with carry-in XSUB into group 0.
fn alu_l2_block() -> Vec<Eq> {
    (1..=10)
        .map(|k| {
            let mut terms = Vec::new();
            for j in 0..k {
                let mut t = vec![l(&format!("G{j}"))];
                for i in (j + 1)..k {
                    t.push(l(&format!("PP{i}")));
                }
                terms.push(t);
            }
            // Carry-in propagated through every group.
            let mut t = vec![l("XSUB")];
            for i in 0..k {
                t.push(l(&format!("PP{i}")));
            }
            terms.push(t);
            Eq::sop(&format!("C{k}"), Mode::Comb, terms)
        })
        .collect()
}

/// Shifter stage 1: rotate FB left by FA[2:0].  Output i is an 8:1 mux of
/// FB[i..i-7]; six consecutive outputs share a 13-input window, so a chip
/// holds six of them.
fn shift1_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let terms = (0..8)
                .map(|k| {
                    let mut t: Vec<SLit> = (0..3).map(|b| if k >> b & 1 == 1 { l(&n("FA", b)) } else { nl_(&n("FA", b)) }).collect();
                    t.push(l(&n("FB", (i + 32 - k) % 32)));
                    t
                })
                .collect();
            Eq::sop(&n("SY", i), Mode::Comb, terms)
        })
        .collect()
}

/// Shifter mask decoder, in parallel with stage 1.  KEEPj says whether an
/// output in class j (bit index mod 8) whose block equals the amount's high
/// part keeps its rotated bit: for a left shift by s (t = s) when j >= t_lo,
/// for a right shift by s (t = ~s, rotation t+1) when j <= t_lo.  FILL is
/// the sign for SRA.
fn shift_mask_block() -> Vec<Eq> {
    let ins = ["FA0", "FA1", "FA2", "XSHR"];
    let mut eqs: Vec<Eq> = (0..8)
        .map(|j| {
            Eq::table(&format!("KEEP{j}"), Mode::Comb, &ins, move |m| {
                let tlo = m & 7;
                let right = m >> 3 & 1 == 1;
                Some(if right { j <= tlo } else { j >= tlo })
            })
        })
        .collect();
    eqs.push(Eq::sop("FILL", Mode::Comb, vec![vec![l("XSRA"), l("FB31")]]));
    eqs
}

/// Shifter stage 2: rotate SY left by 8*FA[4:3], plus one more position for
/// right shifts, with the shift mask and sign fill folded in.  Output i
/// (block b = i/8, class j = i%8) takes class j for left shifts and class
/// j-1 for right shifts, from block b-h for amount-high h.
fn shift2_block() -> Vec<Eq> {
    // Emitted class by class (j, j+8, j+16, j+24): the four outputs of one
    // class read the same eight SY nets and KEEPj, so a chip holds a class.
    (0..32)
        .map(|q| 8 * (q % 4) + q / 4)
        .map(|i| {
            let (b, j) = (i / 8, i % 8);
            let mut terms: Vec<Vec<SLit>> = Vec::new();
            let hi = |h: usize| -> Vec<SLit> {
                vec![if h & 1 == 1 { l("FA3") } else { nl_("FA3") }, if h >> 1 & 1 == 1 { l("FA4") } else { nl_("FA4") }]
            };
            for h in 0..4 {
                // Left shift (or rotation): source y[8*((b-h) mod 4) + j].
                let src_l = n("SY", 8 * ((b + 4 - h) % 4) + j);
                let mut t = hi(h);
                t.push(nl_("XSHR"));
                if b == h {
                    t.push(l(&format!("KEEP{j}")));
                    t.push(l(&src_l));
                    terms.push(t);
                } else if b > h {
                    t.push(l(&src_l));
                    terms.push(t);
                } // b < h: zero
                // Right shift: rotation t+1, source y[(i - 8h - 1) mod 32].
                let src_r = n("SY", (i + 32 - 8 * h - 1) % 32);
                let mut t = hi(h);
                t.push(l("XSHR"));
                if b == h {
                    t.push(l(&format!("KEEP{j}")));
                    t.push(l(&src_r));
                    terms.push(t);
                    // Sign fill where the mask clears.
                    let mut f = hi(h);
                    f.push(l("XSHR"));
                    f.push(nl_(&format!("KEEP{j}")));
                    f.push(l("FILL"));
                    terms.push(f);
                } else if b < h {
                    t.push(l(&src_r));
                    terms.push(t);
                } else {
                    // b > h: shifted out, fill.
                    let mut f = hi(h);
                    f.push(l("XSHR"));
                    f.push(l("FILL"));
                    terms.push(f);
                }
            }
            Eq::sop(&n("SH", i), Mode::Comb, terms)
        })
        .collect()
}

/// EX/MEM result register with the ALU's last level folded in.
fn exmem_result_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let k = (i / 3).min(10);
            let (s0, s1) = (format!("S0_{i}"), format!("S1_{i}"));
            let (fa, fb) = (n("FA", i), n("FB", i));
            // Sum with carry select (group 0's carry-in is XSUB).
            let c = if k == 0 { "XSUB".to_string() } else { format!("C{k}") };
            let with = |code: u32, extra: Vec<SLit>| {
                let mut t = op_lits(code);
                t.push(nl_("XSH"));
                t.extend(extra);
                t
            };
            let terms = vec![
                vec![l("XSH"), l(&n("SH", i))],
                with(0, vec![l(&c), l(&s1)]),
                with(0, vec![nl_(&c), l(&s0)]),
                with(1, vec![l(&fb)]),
                with(2, vec![l(&fa), l(&fb)]),
                with(3, vec![l(&fa)]),
                with(3, vec![l(&fb)]),
                with(4, vec![l(&fa), nl_(&fb)]),
                with(4, vec![nl_(&fa), l(&fb)]),
                with(5, vec![nl_(&fa), nl_(&fb)]),
            ];
            if i == 0 {
                // Bit 0 also carries SLT and SLTU, which pushes the hand-written
                // form past 16 terms; let the minimiser share terms, with the
                // control combinations that decode never produces as don't
                // cares (XSH implies XOP = 0; XOP 6/7 imply XSUB).
                let ins = [
                    "XOP0", "XOP1", "XOP2", "XSH", "XSUB", "S0_0", "S1_0", "FA0", "FB0", "SH0", "FA31", "FB31", "C10", "S0_31",
                    "S1_31", "G10", "PP10",
                ];
                let f = |m: u32| -> Option<bool> {
                    let bit = |b: usize| m >> b & 1 == 1;
                    let op = m & 7;
                    let (xsh, xsub) = (bit(3), bit(4));
                    let (s0, s1, fa, fb, sh) = (bit(5), bit(6), bit(7), bit(8), bit(9));
                    let (fa31, fb31, c10, s0_31, s1_31, g10, pp10) = (bit(10), bit(11), bit(12), bit(13), bit(14), bit(15), bit(16));
                    if xsh {
                        return if op == 0 { Some(sh) } else { None };
                    }
                    let cin = xsub;
                    let sum0 = if cin { s1 } else { s0 };
                    Some(match op {
                        0 => sum0,
                        1 => fb,
                        2 => fa & fb,
                        3 => fa | fb,
                        4 => fa ^ fb,
                        5 => !(fa | fb),
                        6 | 7 if !xsub => return None,
                        6 => {
                            // b31 as seen here is !fb31 (B inverted for subtract).
                            let sum31 = if c10 { s1_31 } else { s0_31 };
                            if fa31 == fb31 { fa31 } else { sum31 }
                        }
                        _ => !(g10 || (pp10 && c10)),
                    })
                };
                return Eq::table(&n("MR", i), Mode::Reg, &ins, f);
                // (bit 0 is not a data-memory address pin: no OE)
            }
            let eq = Eq::sop(&n("MR", i), Mode::Reg, terms);
            // Bits on the data-memory address pins give way to the boot
            // address buffers while the copier counts (off one cycle before
            // the buffers come on, back one cycle after they go off).
            if (2..20).contains(&i) { eq.with_oe(vec![nl_("BOOTCNT"), nl_("BOOTCNTD")]) } else { eq }
        })
        .collect()
}

/// EX/MEM store data with forwarding, driven onto the data bus during the
/// store's MEM cycle (the write cycle).  The memory's outputs were turned
/// off a cycle earlier (OEN), so the drivers can turn on from the edge.
fn exmem_sd_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            Eq::sop(
                &n("DQ", i),
                Mode::Reg,
                vec![
                    vec![nl_("XFSE"), nl_("XFSM"), l(&n("XSD", i))],
                    vec![l("XFSE"), l(&n("MR", i))],
                    vec![nl_("XFSE"), l("XFSM"), l(&n("WD", i))],
                ],
            )
            .with_oe(vec![l("MMW")])
        })
        .collect()
}

/// Reset synchroniser: two GAL registers on CLK turn the supervisor's
/// asynchronous release (RST_n, active low) into RESET, the active-high
/// asynchronous reset of every pipeline register, released 2 to 5.5 ns
/// after a clock edge.  Both stages are active-low so that the GAL's
/// power-up register clear leaves RESET asserted before the first clock.
/// The first stage is the synchroniser (`Eq::sync`).
fn rsync_block() -> Vec<Eq> {
    vec![
        Eq::sop("RS1", Mode::Reg, vec![vec![l("RST_n")]]).active_low().sync(),
        Eq::sop("RESET", Mode::Reg, vec![vec![nl_("RS1"), l("DONE")]]).active_low(),
    ]
}

/// Boot copier sequencer (see docs/boot.md).  While the CPU is held in
/// reset, copies the boot ROMs into instruction memory (the code phase)
/// and then into data memory region by region, one word every five
/// clocks: steps 0-2 let the ROM's access time pass after the address
/// changed, step 3 is the write cycle, step 4 holds the address past the
/// write end, and the PC (the address counter, released from reset early
/// through RESET_PC and otherwise held through HOLD) advances at the end
/// of step 4.  Instruction memory is written with a full-cycle pulse from
/// its registered CE2 (IMEN) and WE# (BOOTWI_n); data memory through the
/// store gate (MMWB) like a store.  WRAP is the PC bit just above the
/// region size and marks the end of a phase, which clears the PC (PCCLR).
/// PHASE counts data regions down from NPH (wired constant) to 0.  DONE
/// lets the reset synchroniser release the CPU.  SKIP (wired) ends the
/// copy at once for a preloaded machine.
///
/// Registers that must read "asserted" from the GAL's power-up clear are
/// active-low outputs (BOOT, CODE, BOOTWI_n).
fn bseq_block() -> Vec<Eq> {
    let run = |t: Vec<SLit>| -> Vec<SLit> {
        let mut v = vec![nl_("RS1"), l("BOOTCNT"), nl_("WRAP")];
        v.extend(t);
        v
    };
    // Step decode helpers (3-bit counter 0..4).
    let step = |v: u32| -> Vec<SLit> { (0..3).map(|b| if v >> b & 1 == 1 { l(&format!("STEP{b}")) } else { nl_(&format!("STEP{b}")) }).collect() };
    let mut eqs = vec![
        Eq::sop("BOOTCNT", Mode::Reg, vec![vec![nl_("RS1"), nl_("SKIP"), nl_("DONE"), nl_("WRAP")], vec![nl_("RS1"), nl_("SKIP"), nl_("DONE"), l("CODE")], vec![nl_("RS1"), nl_("SKIP"), nl_("DONE"), nl_("LAST")]]),
    ];
    // STEP next = STEP + 1 mod 5 while running and not wrapping, else 0.
    for b in 0..3 {
        let terms: Vec<Vec<SLit>> = (0..5u32).filter(|&v| ((v + 1) % 5) >> b & 1 == 1).map(|v| run(step(v))).collect();
        eqs.push(Eq::sop(&format!("STEP{b}"), Mode::Reg, terms));
    }
    let mut write_next = run(step(2)); // next cycle is step 3
    // The store gate fires for every store except an I/O store (its base
    // register has bit 31 set), plus the boot copier's data writes.
    eqs.push(Eq::sop("MMWB", Mode::Reg, vec![vec![l("XMW"), nl_(&n("FA", 31))], { write_next.push(nl_("CODE")); write_next.clone() }]));
    let mut wi = run(step(2));
    wi.push(l("CODE"));
    eqs.push(Eq::sop("BOOTWI_n", Mode::Reg, vec![wi.clone()]).active_low());
    eqs.push(Eq::sop("IMEN", Mode::Reg, vec![vec![l("DONE")], wi]));
    eqs.push(Eq::sop("PCCLR", Mode::Reg, vec![vec![nl_("RS1"), l("BOOTCNT"), l("WRAP")]]));
    eqs.push(Eq::sop("DONE", Mode::Reg, vec![vec![nl_("RS1"), l("DONE")], vec![nl_("RS1"), l("SKIP")], vec![nl_("RS1"), l("BOOTCNT"), l("WRAP"), nl_("CODE"), l("LAST")]]));
    eqs.push(Eq::sop("BOOT", Mode::Reg, vec![vec![l("DONE")]]).active_low());
    // ROM output enable (active low): off from DONE, a cycle before BOOT
    // lets instruction memory drive the bus, so the ROM's 25 ns turn-off
    // is over before the first fetch.
    eqs.push(Eq::sop("ROMOE_n", Mode::Comb, vec![vec![l("DONE")]]));
    eqs.push(Eq::sop("RESET_PC", Mode::Comb, vec![vec![l("RESET"), nl_("BOOTCNT")], vec![l("PCCLR")]]));
    // Phase state.  CODE is 1 from power-up (active-low register) until
    // the first wrap; PHASE loads NPH at that wrap and counts down.
    eqs.push(Eq::sop("CODE", Mode::Reg, vec![vec![nl_("RS1"), nl_("CODE")], vec![nl_("RS1"), l("BOOTCNT"), l("WRAP")]]).active_low());
    for i in 0..5 {
        let mut ins: Vec<String> = (0..5).map(|j| n("PHASE", j)).collect();
        ins.extend(["WRAP", "BOOTCNT", "CODE", "RS1", &n("NPH", i)].map(String::from));
        eqs.push(Eq::table(&n("PHASE", i), Mode::Reg, &strs(&ins), move |m| {
            let phase = m & 31;
            let (wrap, cnt, code, rs1, nph_i) = (m >> 5 & 1 == 1, m >> 6 & 1 == 1, m >> 7 & 1 == 1, m >> 8 & 1 == 1, m >> 9 & 1 == 1);
            if rs1 {
                return Some(false);
            }
            if cnt && wrap && code {
                return Some(nph_i);
            }
            let next = if cnt && wrap { phase.wrapping_sub(1) & 31 } else { phase };
            Some(next >> i & 1 == 1)
        }));
    }
    eqs.push(Eq::sop("LAST", Mode::Comb, vec![vec![nl_("CODE"), nl_("PHASE0"), nl_("PHASE1"), nl_("PHASE2"), nl_("PHASE3"), nl_("PHASE4")]]));
    eqs.push(Eq::sop("BOOTD", Mode::Comb, vec![vec![l("BOOTCNT"), nl_("CODE")]]));
    // BOOTCNT delayed one cycle: sequences the address-net hand-over.
    eqs.push(Eq::sop("BOOTCNTD", Mode::Reg, vec![vec![l("BOOTCNT")]]));
    eqs
}

/// Boot address buffers: the PC (word counter) and the region number onto
/// the data-memory address nets while BOOT.  Which BA bit lands on which
/// MR net is wiring (see `Cpu::build`).
fn badr_block() -> Vec<Eq> {
    // Enabled from one cycle after the copier starts counting until it
    // stops; the EX/MEM chips release the nets a cycle earlier and take
    // them back a cycle later, so the two never drive them together.
    let mut eqs: Vec<Eq> = (0..13).map(|j| Eq::sop(&n("BA", j), Mode::Comb, vec![vec![l(&n("PC", j + 2))]]).with_oe(vec![l("BOOTCNT"), l("BOOTCNTD")])).collect();
    eqs.extend((0..5).map(|i| Eq::sop(&n("BA", 13 + i), Mode::Comb, vec![vec![l(&n("PHASE", i))]]).with_oe(vec![l("BOOTCNT"), l("BOOTCNTD")])));
    eqs
}

/// Boot data buffers: ROM data (on the instruction bus) onto the data bus
/// during the data phases.
fn bdat_block() -> Vec<Eq> {
    (0..32).map(|i| Eq::sop(&n("DQ", i), Mode::Comb, vec![vec![l(&n("IM", i))]]).with_oe(vec![l("BOOTD")])).collect()
}

/// Data memory interlocks and output enable.
fn stall_block() -> Vec<Eq> {
    vec![
        // Hold the instruction in ID for one cycle when it is a load right
        // behind a store (the memory's outputs would still be off in its
        // MEM cycle) or a store right behind a load (the outputs must go
        // off a cycle before the store's write, while the load still needs
        // them).  PC and IF/ID keep their values, ID/EX takes a bubble;
        // EX, MEM and WB proceed, so no forwarding state is disturbed.  The
        // pair separates by one cycle, which ends the hold by itself.
        // ... and the boot copier holds the PC except in step 4.
        Eq::sop("HOLD", Mode::Comb, hold_terms()),
        // Data memory OE#, registered so it cannot glitch: high during a
        // store's EX cycle (set from the decode of a store in ID, unless a
        // load is in EX and still needs the outputs next cycle), its MEM
        // cycle (the write) and the cycle after (until the store-data
        // drivers have released the bus).
        // Written as the complement (active-low register) so that the
        // asynchronous reset leaves OE# high: the memory must not drive
        // the bus while the boot copier does.
        Eq::sop("OEN", Mode::Reg, vec![vec![nl_("STORE"), nl_("XMW"), nl_("MMW"), nl_("BOOT")], vec![l("XMR"), nl_("XMW"), nl_("MMW"), nl_("BOOT")]]).active_low(),

        // Data memory CE# (low = selected): off during the boot code phase
        // and during an I/O access (the UART has the bus).
        Eq::sop("DMEN_n", Mode::Comb, vec![vec![l("BOOTCNT"), l("CODE")], vec![l("MIO")]]),
        // Bus wait: an I/O access holds the whole pipeline for IO_CYCLES
        // clocks (the wait counter runs while MIO; the access ends when
        // it reaches its last value).  WAIT holds MEM/WB; HOLDW = HOLD |
        // WAIT holds PC and IF/ID.  ID/EX and EX/MEM need no hold input:
        // their inputs are functions of held registers, so they recapture
        // the same values every cycle.
        Eq::sop("WAIT", Mode::Comb, wait_terms()),
        Eq::sop("HOLDW", Mode::Comb, { let mut t = hold_terms(); t.extend(wait_terms()); t }),
    ]
}

/// The interlock and boot terms of HOLD.
fn hold_terms() -> Vec<Vec<SLit>> {
    vec![vec![l("STORE"), l("XMR")], vec![l("LOAD"), l("XMW")], vec![l("BOOTCNT"), nl_("STEP2")], vec![l("BOOTCNT"), l("STEP1")], vec![l("BOOTCNT"), l("STEP0")]]
}

/// WAIT = MIO & (CNT != IO_CYCLES - 1).
fn wait_terms() -> Vec<Vec<SLit>> {
    (0..IO_CNT_BITS)
        .map(|b| {
            let last = (IO_CYCLES - 1) >> b & 1 == 1;
            vec![l("MIO"), (n("CNT", b), !last)]
        })
        .collect()
}

/// Clock cycles an I/O access holds the pipeline (see docs/uart.md): the
/// UART's read cycle in FIFO mode wants 425 ns between reads, and the
/// pipeline is frozen anyway, so every access takes this long.
pub const IO_CYCLES: usize = 16;
const IO_CNT_BITS: usize = 4;

/// Bus-wait sequencer: the wait counter, the UART's strobes, chip select
/// and registered address (held for the strobe's hold times after the
/// pipeline moves on).  All registers reset synchronously (RESET is a
/// data input); see docs/uart.md for the cycle-by-cycle timing.
fn wseq_block() -> Vec<Eq> {
    let cnt: Vec<String> = (0..IO_CNT_BITS).map(|b| n("CNT", b)).collect();
    let mut ins: Vec<String> = cnt.clone();
    ins.extend(["MIO", "MMR", "MMW"].map(String::from));
    let ins = strs(&ins);
    let c = |m: u32| (m & 0xF) as usize;
    let mio = |m: u32| m >> 4 & 1 == 1;
    let mmr = |m: u32| m >> 5 & 1 == 1;
    let mmw = |m: u32| m >> 6 & 1 == 1;
    let mut eqs = Vec::new();
    // CNT <- MIO ? CNT + 1 mod 2^bits : 0.
    for b in 0..IO_CNT_BITS {
        eqs.push(Eq::table_pos(&cnt[b], Mode::Reg, &ins, move |m| Some(mio(m) && ((c(m) + 1) % IO_CYCLES) >> b & 1 == 1)));
    }
    // Chip select follows MIO one cycle late (registered), so it stays
    // for a cycle after the access ends: the strobe's hold time.
    eqs.push(Eq::sop("UCS_n", Mode::Reg, vec![vec![l("MIO")]]).active_low());
    // Read strobe: cycles 2 .. IO_CYCLES-1 of the access, i.e. registered
    // from CNT in 1 .. IO_CYCLES-2.  It ends after the edge on which
    // MEM/WB captures the data, and the address / chip select are held
    // a cycle beyond.
    eqs.push(Eq::table_pos("URD_n", Mode::Reg, &ins, move |m| Some(mio(m) && mmr(m) && (1..=IO_CYCLES - 2).contains(&c(m)))).active_low());
    // Write strobe: cycles 2 .. IO_CYCLES-2, ending a cycle before the
    // store-data drivers let go (data hold).
    eqs.push(Eq::table_pos("UWR_n", Mode::Reg, &ins, move |m| Some(mio(m) && mmw(m) && (1..=IO_CYCLES - 3).contains(&c(m)))).active_low());
    // Register select: MR[4:2] captured on the access's first edge (CNT
    // = 0) and held while the counter runs.
    let hold: Vec<Vec<SLit>> = (0..IO_CNT_BITS).map(|b| vec![l(&cnt[b])]).collect();
    for i in 0..3 {
        let mut terms = vec![{
            let mut t: Vec<SLit> = cnt.iter().map(|c| nl_(c)).collect();
            t.push(l(&n("MR", i + 2)));
            t
        }];
        for h in &hold {
            let mut t = h.clone();
            t.push(l(&n("UA", i)));
            terms.push(t);
        }
        eqs.push(Eq::sop(&n("UA", i), Mode::Reg, terms));
    }
    eqs
}

/// EX/MEM control.
fn exmem_ctrl_block() -> Vec<Eq> {
    let mut eqs = vec![
        Eq::sop("MRW", Mode::Reg, vec![vec![l("XRW")]]),
        Eq::sop("MMR", Mode::Reg, vec![vec![l("XMR")]]),
        Eq::sop("MMW", Mode::Reg, vec![vec![l("XMW")]]),
        // I/O access in MEM: a load or store whose base register (the
        // forwarded operand A) has bit 31 set.
        Eq::sop("MIO", Mode::Reg, vec![vec![l(&n("FA", 31)), l("XMR")], vec![l(&n("FA", 31)), l("XMW")]]),
    ];
    for i in 0..5 {
        eqs.push(Eq::sop(&n("MDEST", i), Mode::Reg, vec![vec![l(&n("XDEST", i))]]));
    }
    eqs
}

/// Branch comparator: NEQk = OR over 8 bits of FA != FB.
fn cmp_block() -> Vec<Eq> {
    (0..4)
        .map(|k| {
            let terms = (8 * k..8 * k + 8)
                .flat_map(|i| {
                    vec![vec![l(&n("FA", i)), nl_(&n("FB", i))], vec![nl_(&n("FA", i)), l(&n("FB", i))]]
                })
                .collect();
            Eq::sop(&format!("NEQ{k}"), Mode::Comb, terms)
        })
        .collect()
}

/// Next-PC selection and kill.
fn taken_block() -> Vec<Eq> {
    let ins = ["NEQ0", "NEQ1", "NEQ2", "NEQ3", "XBEQ", "XBNE", "XJR", "DSELJT", "FA31", "XBRS", "XBZ", "XBINV"];
    let neq = |m: u32| m & 15 != 0;
    let bit = |m: u32, b: u32| m >> b & 1 == 1;
    let taken = move |m: u32| {
        // rs-conditioned: sign, optionally OR'd with "rs == 0" (rt is r0,
        // so NEQ is rs != 0), optionally inverted.
        let cond = bit(m, 8) || (bit(m, 10) && !neq(m));
        (bit(m, 4) && !neq(m)) || (bit(m, 5) && neq(m)) || (bit(m, 9) && (cond != bit(m, 11)))
    };
    let jr = |m: u32| m >> 6 & 1 == 1;
    let jt = |m: u32| m >> 7 & 1 == 1;
    vec![
        Eq::table("SELBT", Mode::Comb, &ins, move |m| Some(taken(m))),
        Eq::table("SELAF", Mode::Comb, &ins, move |m| Some(jr(m) && !taken(m))),
        Eq::table("SELJT", Mode::Comb, &ins, move |m| Some(jt(m) && !jr(m) && !taken(m))),
        Eq::table("SELINC", Mode::Comb, &ins, move |m| Some(!jt(m) && !jr(m) && !taken(m))),
        Eq::table("KILL", Mode::Comb, &ins, move |m| Some(jr(m) || taken(m))),
    ]
}

/// MEM/WB: load data or ALU result, destination, write enable (both
/// polarities; the active-low one drives the register file's R/W).
fn memwb_block() -> Vec<Eq> {
    // Bits 8..31 of an I/O load are zero: the UART drives DQ[7:0] only.
    let mut eqs: Vec<Eq> = (0..32)
        .map(|i| {
            let mut dq = vec![l("MMR"), l(&n("DQ", i))];
            if i >= 8 {
                dq.push(nl_("MIO"));
            }
            Eq::sop(&n("WD", i), Mode::Reg, vec![dq, vec![nl_("MMR"), l(&n("MR", i))]])
        })
        .collect();
    for i in 0..5 {
        eqs.push(Eq::sop(&n("WDEST", i), Mode::Reg, vec![vec![l(&n("MDEST", i))]]));
    }
    // Polarity chosen so that reset (registers cleared) reads as "writing
    // r0": the steer then keeps the read ports off r0 while the write copy
    // (also reset) zeroes it.
    eqs.push(Eq::sop("WREG", Mode::Reg, vec![vec![nl_("MRW")]]).active_low());
    eqs
}

/// Write-port copies, two stages on delay-line taps (see docs/memory-timing.md).
///
/// Stage 1 (clock T3, 18 ns after the edge) samples the MEM-stage
/// destination / write flag / store flag while they are stable mid-cycle.
/// Stage 2 (clock T1, 6 ns after the edge) re-times them so that the
/// register-file write port sees the WB instruction's destination from
/// ~13 ns into its WB cycle until ~8 ns into the next: valid before the
/// write starts at the clock's falling edge, held past its end at the
/// rising edge (tHA = 2 ns).  MMWC likewise is the store flag delayed
/// ~6..12 ns, which times the store-data drivers into the gap between the
/// memory's outputs turning off and on.
///
/// Neither stage has an asynchronous reset: their sources are held at zero
/// by the pipeline's reset, the ATF22V10C powers up with registers cleared,
/// and RESET enters the write flag synchronously so that "in reset" reads
/// as "writing r0" (WC1W_n = 0) without any recovery-time relation between
/// the reset release and the tap clocks.
fn wcopy1_block() -> Vec<Eq> {
    let mut eqs: Vec<Eq> = (0..5).map(|i| Eq::sop(&n("WC1D", i), Mode::Reg, vec![vec![l(&n("MDEST", i))]])).collect();
    eqs.push(Eq::sop("WC1W_n", Mode::Reg, vec![vec![nl_("MRW"), nl_("RESET")]]));
    // Bus wait: the pipeline holds at the next edge, so the WB instruction
    // stays and the copy must too.  WAIT itself is combinational and would
    // miss the T3 window; MIO and the counter are registered (valid 5.5 ns
    // after the edge, 9.5 ns before the window), so the condition is
    // rebuilt here: hold while MIO and CNT != IO_CYCLES - 1.
    let not_wait: Vec<Vec<SLit>> = vec![
        vec![nl_("MIO")],
        (0..IO_CNT_BITS).map(|b| (n("CNT", b), (IO_CYCLES - 1) >> b & 1 == 1)).collect(),
    ];
    with_hold_sop(eqs, &not_wait, &wait_terms())
}

/// [`with_hold`] for a hold condition given as a sum of products:
/// `Q <- hold ? Q : f`, with `not_hold` the complement of `hold`, also as
/// a sum of products (each term of `f` is multiplied by each term of it).
pub fn with_hold_sop(eqs: Vec<Eq>, not_hold: &[Vec<SLit>], hold: &[Vec<SLit>]) -> Vec<Eq> {
    eqs.into_iter()
        .map(|mut e| {
            if e.mode != Mode::Reg {
                return e;
            }
            let mut terms: Vec<Vec<SLit>> = Vec::new();
            for t in &e.terms {
                for nh in not_hold {
                    let mut t = t.clone();
                    t.extend(nh.iter().cloned());
                    terms.push(t);
                }
            }
            for h in hold {
                let mut t = h.clone();
                t.push((e.out.clone(), !e.active_low));
                terms.push(t);
            }
            e.terms = terms;
            e
        })
        .collect()
}
fn wcopy2_block() -> Vec<Eq> {
    let mut eqs: Vec<Eq> = (0..5).map(|i| Eq::sop(&n("WDESTC", i), Mode::Reg, vec![vec![l(&n("WC1D", i))]])).collect();
    eqs.push(Eq::sop("WREGC_n", Mode::Reg, vec![vec![l("WC1W_n")]]));
    eqs
}

// ---------------------------------------------------------------------------
// Assembly

/// Gate every term of registered equations with `!HOLD`: the register
/// captures zeros (a bubble) during a hold.
pub fn with_bubble(eqs: Vec<Eq>, hold: &str) -> Vec<Eq> {
    eqs.into_iter()
        .map(|mut e| {
            if e.mode == Mode::Reg {
                for t in &mut e.terms {
                    t.push(nl_(hold));
                }
            }
            e
        })
        .collect()
}

/// Add a hold input to registered equations: `Q <- HOLD ? Q : f`.
pub fn with_hold(eqs: Vec<Eq>, hold: &str) -> Vec<Eq> {
    eqs.into_iter()
        .map(|mut e| {
            // MR0 (16 terms) has no room; it would need re-minimising with
            // HOLD as a table input.  Count it as unchanged.
            if e.mode != Mode::Reg || e.terms.len() >= 16 {
                return e;
            }
            let mut terms: Vec<Vec<SLit>> = e
                .terms
                .iter()
                .map(|t| {
                    let mut t = t.clone();
                    t.push(nl_(hold));
                    t
                })
                .collect();
            // Recirculate the pin's logical level (polarity-aware feedback).
            terms.push(vec![l(hold), (e.out.clone(), !e.active_low)]);
            e.terms = terms;
            e
        })
        .collect()
}

/// Every GAL of the CPU, packed.
pub fn gal_specs() -> Vec<GalSpec> {
    // Minimising the tables takes seconds; every CPU instance shares one
    // set of specs.
    static SPECS: std::sync::OnceLock<Vec<GalSpec>> = std::sync::OnceLock::new();
    SPECS.get_or_init(build_gal_specs).clone()
}

fn build_gal_specs() -> Vec<GalSpec> {
    let clk = Some("CLK");
    let rst = Some("RESET");
    let (bt1, bt2, btr) = bt_block();
    let (fa, fb) = fwd_mux_block();
    let mut v = Vec::new();
    let h = |eqs: Vec<Eq>| with_hold(eqs, "HOLDW");
    v.extend(pack("pc", clk, Some("RESET_PC"), h(pc_block())));
    v.extend(pack("inc", None, None, inc_block()));
    v.extend(pack("ifid", clk, rst, h(ifid_block())));
    v.extend(pack("dec", None, None, dec_block()));
    // ID/EX: a bubble on HOLD (the instruction stays in ID), held on WAIT
    // (the instruction in EX must not be replaced while MEM waits).
    let w = |eqs: Vec<Eq>| with_hold(eqs, "WAIT");
    v.extend(pack("ctl", clk, rst, w(with_bubble(ctrl_block(), "HOLD"))));
    v.extend(pack("steer", None, None, steer_block()));
    // No async reset: under reset the write flags are 0, so these settle
    // to "no hit" from their inputs after one clock.  An async reset would
    // leave them at "hit" (they are written active-low), which selects the
    // EX/MEM result, undriven during the boot copy, into the ALU.
    v.extend(pack("fwdc", clk, None, w(fwdctl_block())));
    v.extend(pack("xa", clk, None, w(idex_a_block())));
    v.extend(pack("xb", clk, None, w(idex_b_block())));
    v.extend(pack("xsd", clk, None, w(idex_sd_block())));
    v.extend(pack("bt1", None, None, bt1));
    v.extend(pack("bt2", None, None, bt2));
    v.extend(pack("xbt", clk, None, w(btr)));
    v.extend(pack("fa", None, None, fa));
    v.extend(pack("fb", None, None, fb));
    v.extend(pack("alu1", None, None, alu_l1_block()));
    v.extend(pack("alu2", None, None, alu_l2_block()));
    v.extend(pack("sh1", None, None, shift1_block()));
    v.extend(pack("shm", None, None, shift_mask_block()));
    v.extend(pack("sh2", None, None, shift2_block()));
    v.extend(pack("mr", clk, None, exmem_result_block()));
    // EX/MEM store data and control hold on WAIT too (the instruction in
    // EX must not replace the one waiting in MEM).  The result register
    // is not held: its bit 0 has no spare product term, and nothing reads
    // it during a wait (the UART address is captured on the first edge,
    // the data memory is deselected, and MEM/WB is held).
    v.extend(pack("msd", clk, None, w(exmem_sd_block())));
    // EX/MEM control feeds the tap-clocked write copies (wc1 reads MDEST
    // and MRW inside its 15 to 21 ns window).  An asynchronous clear
    // would land there when RESET is asserted mid-run (button press), so
    // this block resets synchronously: its outputs only ever move at CLK.
    v.extend(pack("mctl", clk, None, w(with_bubble(exmem_ctrl_block(), "RESET"))));
    v.extend(pack("stl", clk, rst, stall_block()));
    v.extend(pack("rsync", clk, None, rsync_block()));
    v.extend(pack("bseq", clk, None, bseq_block()));
    v.extend(pack("badr", None, None, badr_block()));
    v.extend(pack("bdat", None, None, bdat_block()));
    v.extend(pack("cmp", None, None, cmp_block()));
    v.extend(pack("nxt", None, None, taken_block()));
    v.extend(pack("wb", clk, rst, with_hold(memwb_block(), "WAIT")));
    v.extend(pack("wseq", clk, None, with_bubble(wseq_block(), "RESET")));
    v.extend(pack("wc1", Some("T3"), None, wcopy1_block()));
    v.extend(pack("wc2", Some("T1"), None, wcopy2_block()));
    v
}

/// The running CPU.
/// The delay line part used for the write-copy clocks: DS1100-30, taps at
/// 6, 12, 18, 24, 30 ns.  T1 (6) clocks the second copy stage, T3 (18) the
/// first.
pub const DELAY_LINE_TOTAL: u32 = 30;

/// Build options.
/// How the memories get their contents.
#[derive(Clone, Debug)]
pub enum Boot {
    /// Preloaded (the copier is skipped; the ROMs are present but idle).
    Preload,
    /// The boot copier fills them from the ROMs.  `code_words_log2` is the
    /// region size (13 on the board: 8K words; smaller in tests), and
    /// `data` is the data image, `data_regions` regions of that size.
    Copy { code_words_log2: u32, data_regions: u32, data: Vec<u32> },
}

#[derive(Clone, Debug)]
pub struct Build {
    /// Delay-line tolerance grade.
    pub grade: Grade,
    /// Data memory timing grade (default CY7C1041G-10, two 256K x 16 chips).
    pub dmem: as7c164a::Timing,
    /// Tap for the write-enable gate: (DS1100 total, tap index 0..5).  A
    /// total other than [`DELAY_LINE_TOTAL`] adds a second delay line.
    pub gate_tap: (u32, usize),
    /// Fast gate propagation delay range (ps).  Default: Diodes 74LVC1G00Q
    /// at 5 V, 0.5 to 5.5 ns over -40..+125 C (datasheet June 2020).
    pub gate_tpd: (Time, Time),
    /// Where in a clock cycle the reset supervisor releases (ns after a
    /// rising edge).  Any value must work; the tests sweep it.
    pub reset_phase_ns: f64,
    pub boot: Boot,
    /// UART crystal (Hz).  The board's is 14.7456 MHz; tests use a faster
    /// one so that characters take tens of cycles rather than thousands.
    pub uart_xin_hz: f64,
    /// Characters the terminal sends, from when the program first polls
    /// the line status, one per character time.
    pub uart_rx: Vec<u8>,
}

/// The board's UART crystal: 14.7456 MHz (3225 SMD, 12 pF), divisor 8
/// for 115200 baud.
pub const UART_XIN_HZ: f64 = 14_745_600.0;

impl Default for Build {
    fn default() -> Build {
        Build { grade: Grade::Commercial, dmem: as7c164a::Timing::cy7c1041g_10(), gate_tap: (40, 0), gate_tpd: (500, 5500), reset_phase_ns: 11.0, boot: Boot::Preload, uart_xin_hz: UART_XIN_HZ, uart_rx: Vec::new() }
    }
}

pub struct Cpu {
    pub sim: Sim,
    pub period: Time,
    pub cycles: u64,
    pub pc_trace: Vec<Option<u32>>,
    /// Delay-line tolerance grade the netlist was built with.
    pub grade: Grade,
    /// When RESET was released (ps).
    pub reset_release: Time,
    clk: NetId,
    mr_n: NetId,
    /// Cycles a full reset (boot copy included) may take.
    boot_budget: usize,
    pc: Vec<NetId>,
    imem: Vec<usize>,
    dmem: Vec<usize>,
    rf: Vec<usize>,
    pub gal_count: usize,
}

/// Time stamp of a chip warning (`t=123.500ns` inside the text).
fn warning_time(w: &str) -> Option<Time> {
    let i = w.find("t=")?;
    let rest = &w[i + 2..];
    let end = rest.find("ns")?;
    rest[..end].parse::<f64>().ok().map(|ns| (ns * NS as f64).round() as Time)
}

/// Clock cycles RESET is held with the clock running: enough for the
/// UART's 1 us master-reset pulse even without a boot copy.
pub const RESET_CYCLES: usize = 32;
/// How long the modelled supervisor keeps RESET# low after the button is
/// released (the MAX811L's 140 ms would be 4 million cycles; the length
/// does not matter to the CPU, only the release phase does, but the UART's
/// master reset wants 1 us).
pub const MR_TIMEOUT_NS: f64 = 1100.0;

pub const PH_RF: usize = 0;
pub const PH_CE: usize = 1;
pub const PH_SD: usize = 2;

impl Cpu {
    /// Build the netlist with `program` in instruction memory at address 0,
    /// default options.
    pub fn new(program: &[u32], period_ns: f64) -> Cpu {
        Cpu::build(program, period_ns, Build::default())
    }

    pub fn with_grade(program: &[u32], period_ns: f64, grade: Grade) -> Cpu {
        Cpu::build(program, period_ns, Build { grade, ..Build::default() })
    }

    pub fn build(program: &[u32], period_ns: f64, opt: Build) -> Cpu {
        let grade = opt.grade;
        let mut nl = Netlist::new();
        let specs = gal_specs();
        let gal_count = specs.len();
        instantiate_all(&mut nl, &specs);
        // Delay line on CLK: its taps clock the write-copy registers.
        let dl = nl.add_chip("dl0", Ds1100::new(DELAY_LINE_TOTAL, grade));
        let clk = nl.net("CLK");
        nl.connect(clk, dl, DS1100_IN);
        for k in 0..5 {
            let net = nl.net(&format!("T{}", k + 1));
            nl.connect(net, dl, ds1100_tap_pin(k));
        }
        // The data-memory write-enable gate: NAND of a tap and the store
        // flag.  A second delay line if the tap is from another part.
        let (gt_total, gt_k) = opt.gate_tap;
        let tap_net = if gt_total == DELAY_LINE_TOTAL {
            nl.net(&format!("T{}", gt_k + 1))
        } else {
            let dl1 = nl.add_chip("dl1", Ds1100::new(gt_total, grade));
            nl.connect(clk, dl1, DS1100_IN);
            for k in 0..5 {
                let net = nl.net(&format!("U{}", k + 1));
                nl.connect(net, dl1, ds1100_tap_pin(k));
            }
            nl.net(&format!("U{}", gt_k + 1))
        };
        // Reset supervisor: releases RST_n after the power-up settle and
        // RESET_CYCLES clocks, at the requested phase.
        let period = (period_ns * NS as f64).round() as Time;
        let release = (3 + RESET_CYCLES as Time) * period + Self::ns(opt.reset_phase_ns);
        let sup = nl.add_chip("rst0", ResetSupervisor::new(release, Self::ns(MR_TIMEOUT_NS)));
        let rst_n = nl.net("RST_n");
        nl.connect(rst_n, sup, 2);
        // MR#: the reset button, to ground, pulled up inside the part.
        // [`Cpu::press_reset`] pulls it low.
        let mr_n = nl.net("MR_n");
        nl.connect(mr_n, sup, 3);
        nl.pull(mr_n, Level::H);
        let gate = nl.add_chip("gate0", FastGate::new(opt.gate_tpd.0, opt.gate_tpd.1));
        nl.connect(tap_net, gate, 1);
        let mmwb = nl.net("MMWB");
        nl.connect(mmwb, gate, 2);
        let wen = nl.net("WEN");
        nl.connect(wen, gate, 4);

        let gnd = nl.net("GND");
        let vcc = nl.net("VCC");

        // UART (TL16C550C) on the data bus, byte lane 0; register select
        // from the bus-wait sequencer's held address; strobes and chip
        // select from it too; master reset from RESET (active high).
        // ADS# low (no address latch), RD2 / WR2 low, CS0 / CS1 high.
        {
            let mut u = Uart16550::new(BusTiming::tl16c550c(), opt.uart_xin_hz);
            u.core.send(&opt.uart_rx);
            let c = nl.add_chip("uart0", u);
            for i in 0..8u8 {
                let net = nl.net(&n("DQ", i as usize));
                nl.connect(net, c, uart_pin_of(UartPin::D(i)));
            }
            for i in 0..3u8 {
                let net = nl.net(&n("UA", i as usize));
                nl.connect(net, c, uart_pin_of(UartPin::A(i)));
            }
            for (net, p) in [("UCS_n", UartPin::Cs2N), ("URD_n", UartPin::Rd1N), ("UWR_n", UartPin::Wr1N), ("RESET", UartPin::Mr)] {
                let net = nl.net(net);
                nl.connect(net, c, uart_pin_of(p));
            }
            for p in [UartPin::Cs0, UartPin::Cs1] {
                nl.connect(vcc, c, uart_pin_of(p));
            }
            for p in [UartPin::AdsN, UartPin::Rd2, UartPin::Wr2] {
                nl.connect(gnd, c, uart_pin_of(p));
            }
        }
        nl.tie(gnd, Level::L);
        nl.tie(vcc, Level::H);

        // Boot: region size and wiring of the copier.
        let (k, nph, skip, data_regions, data_image): (u32, u32, bool, u32, Vec<u32>) = match &opt.boot {
            Boot::Preload => (13, 0, true, 0, Vec::new()),
            Boot::Copy { code_words_log2, data_regions, data } => (*code_words_log2, data_regions.saturating_sub(1), false, *data_regions, data.clone()),
        };
        let words = 1usize << k;

        // Instruction memory: 4 lanes, address = PC[14:2].  Preloaded, or
        // left unknown for the copier to fill.
        let mut imem = Vec::new();
        for lane in 0..4 {
            let mut chip = As7c164a::with_timing(as7c164a::Timing::is61c64al_10());
            if skip {
                for (w, &word) in program.iter().enumerate() {
                    chip.preload(w as u32, (word >> (8 * lane)) as u8);
                }
                for w in program.len()..8192 {
                    chip.preload(w as u32, 0);
                }
            }
            let c = nl.add_chip(&format!("imem{lane}"), chip);
            imem.push(c);
            for a in 0..13 {
                let net = nl.net(&n("PC", a + 2));
                nl.connect(net, c, sram8k_pin_of(Sram8kPin::A(a as u8)));
            }
            for b in 0..8 {
                let net = nl.net(&n("IM", 8 * lane + b));
                nl.connect(net, c, sram8k_pin_of(Sram8kPin::Dq(b as u8)));
            }
            nl.connect(gnd, c, sram8k_pin_of(Sram8kPin::CeN));
            let imen = nl.net("IMEN");
            nl.connect(imen, c, sram8k_pin_of(Sram8kPin::Ce2));
            let boot = nl.net("BOOT");
            nl.connect(boot, c, sram8k_pin_of(Sram8kPin::OeN));
            let wei = nl.net("BOOTWI_n");
            nl.connect(wei, c, sram8k_pin_of(Sram8kPin::WeN));
        }
        // Boot ROMs: one per byte lane on the instruction bus.  Address:
        // word index from the PC (k bits), then the region number, then
        // CODE on A18; the rest grounded.  Image: code at A18 = 1, data
        // region p at p << k.
        for lane in 0..4 {
            let mut rom = Rom::sst39sf040_70();
            for (w, &word) in program.iter().enumerate() {
                assert!(w < words, "program longer than the boot code region");
                rom.preload((1 << 18) | w as u32, (word >> (8 * lane)) as u8);
            }
            for w in program.len()..words {
                rom.preload((1 << 18) | w as u32, 0);
            }
            for p in 0..data_regions {
                for w in 0..words {
                    let v = data_image.get(p as usize * words + w).copied().unwrap_or(0);
                    rom.preload((p << k) | w as u32, (v >> (8 * lane)) as u8);
                }
            }
            let c = nl.add_chip(&format!("rom{lane}"), rom);
            for j in 0..19u32 {
                let net = if j < k {
                    nl.net(&n("PC", j as usize + 2))
                } else if j < k + 5 {
                    nl.net(&n("PHASE", (j - k) as usize))
                } else if j == 18 {
                    nl.net("CODE")
                } else {
                    gnd
                };
                nl.connect(net, c, rom_pin_of(RomPin::A(j as u8)));
            }
            for b in 0..8 {
                let net = nl.net(&n("IM", 8 * lane + b));
                nl.connect(net, c, rom_pin_of(RomPin::Dq(b as u8)));
            }
            nl.connect(gnd, c, rom_pin_of(RomPin::CeN));
            let romoe = nl.net("ROMOE_n");
            nl.connect(romoe, c, rom_pin_of(RomPin::OeN));
            nl.connect(vcc, c, rom_pin_of(RomPin::WeN));
        }
        // Copier wiring: WRAP is the PC bit above the region; the boot
        // address buffers land on the data-memory address nets (PC bits
        // below the region size, then the region number, then the PC's
        // always-zero upper bits); NPH and SKIP are wired constants.
        {
            let wrap = nl.net("WRAP");
            let pc_k = nl.net(&n("PC", k as usize + 2));
            nl.merge(pc_k, wrap);
            let mut target: Vec<usize> = (0..k as usize).map(|j| j + 2).collect();
            target.extend((0..5).map(|i| k as usize + 2 + i));
            target.extend((k as usize..13).map(|m| k as usize + 7 + (m - k as usize)));
            let mut ba_order: Vec<usize> = (0..k as usize).collect();
            ba_order.extend(13..18);
            ba_order.extend(k as usize..13);
            for (ba, mr) in ba_order.into_iter().zip(target) {
                let ba_net = nl.net(&n("BA", ba));
                let mr_net = nl.net(&n("MR", mr));
                nl.merge(mr_net, ba_net);
            }
            for i in 0..5 {
                let net = nl.net(&n("NPH", i));
                nl.tie(net, Level::from_bit(nph >> i & 1 == 1));
            }
            let sk = nl.net("SKIP");
            nl.tie(sk, Level::from_bit(skip));
        }
        // Data memory: two 256K x 16 chips (low / high half-word), always
        // selected with both byte enables on (word access only for now),
        // output-enabled except around a store (OEN); address MR[19:2];
        // WE# from the gate during the store's MEM cycle.  DQ is driven by
        // the SRAMs during loads and by the EX/MEM store-data drivers
        // during a store.
        let mut dmem = Vec::new();
        for half in 0..2 {
            let mut chip = Sram16::new(opt.dmem);
            for w in 0..(1u32 << 18) {
                chip.preload(w, 0);
            }
            let c = nl.add_chip(&format!("dmem{half}"), chip);
            dmem.push(c);
            for a in 0..18 {
                let net = nl.net(&n("MR", a + 2));
                nl.connect(net, c, sram16_pin_of(Sram16Pin::A(a as u8)));
            }
            for b in 0..16 {
                let net = nl.net(&n("DQ", 16 * half + b));
                nl.connect(net, c, sram16_pin_of(Sram16Pin::Io(b as u8)));
            }
            let dmen = nl.net("DMEN_n");
            nl.connect(dmen, c, sram16_pin_of(Sram16Pin::CeN));
            nl.connect(gnd, c, sram16_pin_of(Sram16Pin::BheN));
            nl.connect(gnd, c, sram16_pin_of(Sram16Pin::BleN));
            let oen = nl.net("OEN");
            nl.connect(oen, c, sram16_pin_of(Sram16Pin::OeN));
            nl.connect(wen, c, sram16_pin_of(Sram16Pin::WeN));
        }
        // Register file: bank 0 reads rs (IR25..21) -> RA, bank 1 reads rt
        // (IR20..16) -> RB; write port: address WDESTC (delayed copy), data
        // WD, CE = CLK (write in the low half), R/W = WREGC_n.
        let mut rf = Vec::new();
        for bank in 0..2 {
            for lane in 0..4 {
                let mut chip = Cy7c131::new();
                for r in 0..32 {
                    chip.preload(r, 0);
                }
                // r0 powers up as garbage; the reset sequence must zero it.
                chip.preload(0, 0xA5);
                let c = nl.add_chip(&format!("rf{bank}{lane}"), chip);
                rf.push(c);
                let (field, rdata, ce) = if bank == 0 { (21, "RA", "CERA_n") } else { (16, "RB", "CERB_n") };
                for i in 0..5 {
                    let net = nl.net(&ir(field + i));
                    nl.connect(net, c, sram_pin_of(SramPin::A(Port::Left, i as u8)));
                    let net = nl.net(&n("WDESTC", i));
                    nl.connect(net, c, sram_pin_of(SramPin::A(Port::Right, i as u8)));
                }
                // A5 of the write port is the inverted write enable: an idle
                // port lands on 32..63, which no read ever matches, so it
                // never arbitrates against a read.
                let wreg_n = nl.net("WREGC_n");
                nl.connect(wreg_n, c, sram_pin_of(SramPin::A(Port::Right, 5)));
                nl.connect(gnd, c, sram_pin_of(SramPin::A(Port::Left, 5)));
                for i in 6..10 {
                    nl.connect(gnd, c, sram_pin_of(SramPin::A(Port::Left, i)));
                    nl.connect(gnd, c, sram_pin_of(SramPin::A(Port::Right, i)));
                }
                for b in 0..8 {
                    let net = nl.net(&n(rdata, 8 * lane + b));
                    nl.connect(net, c, sram_pin_of(SramPin::Io(Port::Left, b as u8)));
                    let net = nl.net(&n("WD", 8 * lane + b));
                    nl.connect(net, c, sram_pin_of(SramPin::Io(Port::Right, b as u8)));
                }
                let cen = nl.net(ce);
                nl.connect(cen, c, sram_pin_of(SramPin::Ce(Port::Left)));
                nl.connect(vcc, c, sram_pin_of(SramPin::Rw(Port::Left)));
                nl.connect(gnd, c, sram_pin_of(SramPin::Oe(Port::Left)));
                nl.connect(clk, c, sram_pin_of(SramPin::Ce(Port::Right)));
                let rw = nl.net("WREGC_n");
                nl.connect(rw, c, sram_pin_of(SramPin::Rw(Port::Right)));
                nl.connect(vcc, c, sram_pin_of(SramPin::Oe(Port::Right)));
                for (p, name) in [
                    (SramPin::Busy(Port::Left), "BUSYL"),
                    (SramPin::Busy(Port::Right), "BUSYR"),
                    (SramPin::Int(Port::Left), "INTL"),
                    (SramPin::Int(Port::Right), "INTR"),
                ] {
                    let net = nl.net(&format!("{name}_{bank}{lane}"));
                    nl.pull(net, Level::H);
                    nl.connect(net, c, sram_pin_of(p));
                }
            }
        }
        let sim = nl.build();
        let mr_n = sim.net_id("MR_n");
        let pc = (2..=14).map(|i| sim.net_id(&n("PC", i))).collect();
        let mut cpu = Cpu {
            sim,
            period: (period_ns * NS as f64).round() as Time,
            cycles: 0,
            pc_trace: Vec::new(),
            clk,
            pc,
            imem,
            dmem,
            rf,
            gal_count,
            grade,
            reset_release: 0,
            mr_n,
            boot_budget: RESET_CYCLES + 8 + if skip { 0 } else { 5 * words * (1 + data_regions as usize) + 8 * (2 + data_regions as usize) },
        };
        // Power-on: the supervisor holds RST_n low, the synchroniser's
        // registers power up with RESET asserted, clock low.  Let every
        // combinational chain settle, then run the clock under reset (the
        // pipeline registers stay cleared; the write-back stage writes
        // r0 = 0).  The supervisor releases at `reset_phase_ns` into a
        // cycle; the synchroniser passes that on 2 to 5.5 ns after an edge
        // one or two cycles later.
        cpu.sim.schedule(0, cpu.clk, Level::L);
        cpu.sim.run_until(3 * cpu.period);
        cpu.wait_reset_release();
        cpu
    }

    /// Clock until RESET has been released (the boot copy, if any, has
    /// run), record the release time, then one more cycle so the first
    /// instruction is in flight before the caller starts counting.
    fn wait_reset_release(&mut self) {
        let reset = self.sim.net_id("RESET");
        for _ in 0..self.boot_budget {
            self.step();
            if self.sim.value(reset) == Level::L {
                break;
            }
        }
        assert_eq!(self.sim.value(reset), Level::L, "reset never released");
        self.reset_release = self
            .sim
            .history(reset)
            .iter()
            .rev()
            .find(|(_, l)| *l == Level::L)
            .map(|(t, _)| *t)
            .expect("reset release time");
        self.step();
        self.cycles = 0;
        self.pc_trace.clear();
    }

    /// Press the reset button: MR_n low from `phase_ns` into the current
    /// cycle for `hold_ns`, then run through the supervisor's timeout and
    /// the boot copy until RESET is released again.  Warnings are counted
    /// from the new release ([`Cpu::warnings`]); the ones before it go
    /// through [`Cpu::reset_warnings`].
    pub fn press_reset(&mut self, phase_ns: f64, hold_ns: f64) {
        let down = self.sim.now() + Self::ns(phase_ns);
        self.sim.schedule(down, self.mr_n, Level::L);
        self.sim.schedule(down + Self::ns(hold_ns), self.mr_n, Level::Z);
        let reset = self.sim.net_id("RESET");
        // Clock until the press has been seen (RESET asserted) ...
        for _ in 0..8 {
            self.step();
            if self.sim.value(reset) == Level::H {
                break;
            }
        }
        assert_eq!(self.sim.value(reset), Level::H, "button press not seen");
        // ... then until it is over.
        self.wait_reset_release();
    }

    /// Chip warnings from RESET release onwards.  Before that, registers
    /// without a reset capture whatever is on their inputs, which the model
    /// reports as unknown captures; [`Cpu::reset_warnings`] checks those.
    pub fn warnings(&self) -> Vec<String> {
        self.sim.warnings().into_iter().filter(|w| warning_time(w).is_none_or(|t| t >= self.reset_release)).collect()
    }

    /// Warnings raised while RESET was held, minus the expected unknown
    /// captures of data registers that have no reset.  Anything left is a
    /// real problem in the reset sequence (a glitch write, a bus conflict,
    /// a timing violation).
    pub fn reset_warnings(&self) -> Vec<String> {
        // The first three periods are the power-up settle: every input is
        // unknown until the chips have driven their pins once.
        let settle = 3 * self.period;
        self.sim
            .warnings()
            .into_iter()
            .filter(|w| warning_time(w).is_some_and(|t| t >= settle && t < self.reset_release))
            .filter(|w| !w.contains("CapturedX") && !w.contains("AddrUnknown"))
            .collect()
    }

    fn ns(t: f64) -> Time {
        (t * NS as f64).round() as Time
    }

    /// Run one clock cycle (rising edge now).
    pub fn step(&mut self) {
        let base = self.sim.now();
        let half = base + self.period / 2;
        self.sim.schedule(base, self.clk, Level::H);
        self.sim.schedule(half, self.clk, Level::L);
        // Sample the PC just before the next edge (what IF is fetching).
        self.sim.run_until(base + self.period - Self::ns(3.5));
        self.pc_trace.push(self.sim.read_bus(&self.pc).map(|w| w << 2));
        self.sim.run_until(base + self.period);
        self.cycles += 1;
    }

    pub fn run(&mut self, cycles: u64) {
        for _ in 0..cycles {
            self.step();
        }
    }

    /// Run until the PC (as sampled at the end of a cycle) equals `stop`
    /// for `settle` consecutive cycles, or `max` cycles pass.  Returns
    /// whether it stopped.
    pub fn run_until_pc(&mut self, stop: u32, max: u64) -> bool {
        for _ in 0..max {
            self.step();
            if self.pc_trace.last().copied().flatten() == Some(stop) {
                // Drain the pipeline.
                self.run(4);
                return true;
            }
        }
        false
    }

    /// Register contents from the register-file chips (both banks must
    /// agree and be known).
    pub fn reg(&self, r: u8) -> Option<u32> {
        let bank = |b: usize| -> Option<u32> {
            let mut w = 0u32;
            for lane in 0..4 {
                let chip = self.chip_sram(self.rf[b * 4 + lane]);
                w |= (chip.peek(r as u16)? as u32) << (8 * lane);
            }
            Some(w)
        };
        match (bank(0), bank(1)) {
            (Some(a), Some(b)) if a == b => Some(a),
            _ => None,
        }
    }
    pub fn regs(&self) -> Vec<Option<u32>> {
        (0..32).map(|r| self.reg(r)).collect()
    }
    /// Data memory word.
    /// Instruction memory word.
    pub fn imem_word(&self, addr: u32) -> Option<u32> {
        let mut w = 0u32;
        for lane in 0..4 {
            let chip = self.sim.chip(self.imem[lane]).downcast_ref::<As7c164a>().unwrap();
            w |= (chip.peek(addr >> 2)? as u32) << (8 * lane);
        }
        Some(w)
    }
    pub fn dmem_word(&self, addr: u32) -> Option<u32> {
        let mut w = 0u32;
        for half in 0..2 {
            let chip = self.sim.chip(self.dmem[half]).downcast_ref::<Sram16>().unwrap();
            w |= (chip.peek(addr >> 2)? as u32) << (16 * half);
        }
        Some(w)
    }
    fn chip_sram(&self, id: usize) -> &Cy7c131 {
        self.sim.chip(id).downcast_ref::<Cy7c131>().unwrap()
    }
    /// The UART chip model.
    pub fn uart(&self) -> &Uart16550 {
        let id = self.sim.chip_id("uart0");
        self.sim.chip(id).downcast_ref::<Uart16550>().expect("uart model")
    }
    /// Characters the UART has transmitted so far.
    pub fn uart_tx(&self) -> Vec<u8> {
        self.uart().core.tx.clone()
    }

    pub fn imem_chips(&self) -> &[usize] {
        &self.imem
    }
}

// ---------------------------------------------------------------------------
// Structure export (for diagrams)

/// One chip of the CPU with its pin-to-net map, block and stage.
pub struct ChipInfo {
    pub name: String,
    pub kind: &'static str,
    pub block: &'static str,
    pub stage: &'static str,
    /// (pin, net, is_output)
    pub pins: Vec<(usize, String, bool)>,
}

/// Block and stage of a chip, from its name prefix.
fn block_of(name: &str) -> (&'static str, &'static str) {
    let prefix: String = name.chars().take_while(|c| c.is_ascii_alphabetic()).collect();
    match prefix.as_str() {
        "pc" => ("PC", "IF"),
        "inc" => ("PC+4", "IF"),
        "imem" => ("Instruction memory", "IF"),
        "ifid" => ("IF/ID", "IF/ID"),
        "dec" => ("Decode", "ID"),
        "ctl" => ("ID/EX control", "ID/EX"),
        "steer" => ("Steer", "ID"),
        "fwdc" => ("Forwarding control", "ID/EX"),
        "rf" => ("Register file", "ID"),
        "bt" => ("Branch target adder", "ID"),
        "xa" => ("ID/EX A", "ID/EX"),
        "xb" => ("ID/EX B", "ID/EX"),
        "xsd" => ("ID/EX store data", "ID/EX"),
        "xbt" => ("ID/EX branch target", "ID/EX"),
        "fa" => ("Forward A", "EX"),
        "fb" => ("Forward B", "EX"),
        "alu" => ("ALU slices + carries", "EX"),
        "sh" => ("Shifter", "EX"),
        "shm" => ("Shifter", "EX"),
        "cmp" => ("Compare", "EX"),
        "nxt" => ("Next PC", "EX"),
        "mr" => ("EX/MEM result (ALU last level)", "EX/MEM"),
        "msd" => ("EX/MEM store data", "EX/MEM"),
        "mctl" => ("EX/MEM control", "EX/MEM"),
        "dmem" => ("Data memory", "MEM"),
        "wb" => ("MEM/WB", "MEM/WB"),
        "dl" => ("Delay line", "CLK"),
        "wc" => ("Write copies", "MEM/WB"),
        "stl" => ("Stall", "EX/MEM"),
        "wseq" => ("Bus wait", "MEM"),
        "uart" => ("UART", "MEM"),
        "rsync" => ("Reset sync", "IF"),
        "bseq" => ("Boot sequencer", "IF"),
        "badr" => ("Boot address", "IF"),
        "bdat" => ("Boot data", "MEM"),
        "rom" => ("Boot ROM", "IF"),
        "rst" => ("Reset supervisor", "IF"),
        "gate" => ("Write gate", "MEM"),
        _ => ("?", "?"),
    }
}

/// Every chip in the CPU with its wiring, as built by [`Cpu::new`].
pub fn chip_infos() -> Vec<ChipInfo> {
    let mut out = Vec::new();
    for spec in gal_specs() {
        let mut nl = Netlist::new();
        let (_, pins) = crate::galpack::instantiate(&mut nl, &spec);
        let outs: Vec<&str> = spec.eqs.iter().map(|e| e.out.as_str()).collect();
        let (block, stage) = block_of(&spec.name);
        out.push(ChipInfo {
            name: spec.name.clone(),
            kind: "ATF22V10C",
            block,
            stage,
            pins: pins.into_iter().map(|(p, n)| (p, n.clone(), outs.contains(&n.as_str()))).collect(),
        });
    }
    // SRAMs: replicate the wiring of Cpu::new.
    for lane in 0..4 {
        let mut pins = Vec::new();
        for a in 0..13 {
            pins.push((sram8k_pin_of(Sram8kPin::A(a as u8)), n("PC", a + 2), false));
        }
        for b in 0..8 {
            pins.push((sram8k_pin_of(Sram8kPin::Dq(b as u8)), n("IM", 8 * lane + b), true));
        }
        let (block, stage) = block_of("imem");
        out.push(ChipInfo { name: format!("imem{lane}"), kind: "IS61C64AL-10", block, stage, pins });
    }
    for half in 0..2 {
        let mut pins = Vec::new();
        for a in 0..18 {
            pins.push((sram16_pin_of(Sram16Pin::A(a as u8)), n("MR", a + 2), false));
        }
        for b in 0..16 {
            pins.push((sram16_pin_of(Sram16Pin::Io(b as u8)), n("DQ", 16 * half + b), true));
        }
        pins.push((sram16_pin_of(Sram16Pin::OeN), "OEN".into(), false));
        pins.push((sram16_pin_of(Sram16Pin::WeN), "WEN".into(), false));
        let (block, stage) = block_of("dmem");
        out.push(ChipInfo { name: format!("dmem{half}"), kind: "CY7C1041GN-10", block, stage, pins });
    }
    {
        let mut pins = vec![(DS1100_IN, "CLK".to_string(), false)];
        for k in 0..5 {
            pins.push((ds1100_tap_pin(k), format!("T{}", k + 1), true));
        }
        let (block, stage) = block_of("dl");
        out.push(ChipInfo { name: "dl0".into(), kind: "DS1100-30", block, stage, pins });
        let (block, stage) = block_of("rst");
        out.push(ChipInfo { name: "rst0".into(), kind: "MAX811LEUS+T", block, stage, pins: vec![(2, "RST_n".to_string(), true), (3, "MR_n".to_string(), false)] });
        let (block, stage) = block_of("uart");
        let mut pins = Vec::new();
        for i in 0..8u8 {
            pins.push((uart_pin_of(UartPin::D(i)), n("DQ", i as usize), true));
        }
        for i in 0..3u8 {
            pins.push((uart_pin_of(UartPin::A(i)), n("UA", i as usize), false));
        }
        for (net, p) in [("UCS_n", UartPin::Cs2N), ("URD_n", UartPin::Rd1N), ("UWR_n", UartPin::Wr1N), ("RESET", UartPin::Mr)] {
            pins.push((uart_pin_of(p), net.to_string(), false));
        }
        out.push(ChipInfo { name: "uart0".into(), kind: "TL16C550C", block, stage, pins });
        // Boot ROMs, wired for the board's 8K-word regions.
        for lane in 0..4 {
            let mut pins = Vec::new();
            for j in 0..19u32 {
                let net = if j < 13 { n("PC", j as usize + 2) } else if j < 18 { n("PHASE", (j - 13) as usize) } else { "CODE".to_string() };
                pins.push((rom_pin_of(RomPin::A(j as u8)), net, false));
            }
            for b in 0..8 {
                pins.push((rom_pin_of(RomPin::Dq(b as u8)), n("IM", 8 * lane + b), true));
            }
            pins.push((rom_pin_of(RomPin::OeN), "ROMOE_n".to_string(), false));
            let (block, stage) = block_of("rom");
            out.push(ChipInfo { name: format!("rom{lane}"), kind: "SST39SF040", block, stage, pins });
        }
        let (block, stage) = block_of("dl");
        let (gt_total, gt_k) = Build::default().gate_tap;
        if gt_total != DELAY_LINE_TOTAL {
            let mut pins = vec![(DS1100_IN, "CLK".to_string(), false)];
            for k in 0..5 {
                pins.push((ds1100_tap_pin(k), format!("U{}", k + 1), true));
            }
            out.push(ChipInfo { name: "dl1".into(), kind: "DS1100-40", block, stage, pins });
        }
        let tap = if gt_total == DELAY_LINE_TOTAL { format!("T{}", gt_k + 1) } else { format!("U{}", gt_k + 1) };
        let pins = vec![(1, tap, false), (2, "MMWB".to_string(), false), (4, "WEN".to_string(), true)];
        let (block, stage) = block_of("gate");
        out.push(ChipInfo { name: "gate0".into(), kind: "74LVC1G00Q", block, stage, pins });
    }
    for bank in 0..2 {
        for lane in 0..4 {
            let (field, rdata, ce) = if bank == 0 { (21, "RA", "CERA_n") } else { (16, "RB", "CERB_n") };
            let mut pins = Vec::new();
            for i in 0..5 {
                pins.push((sram_pin_of(SramPin::A(Port::Left, i as u8)), ir(field + i), false));
                pins.push((sram_pin_of(SramPin::A(Port::Right, i as u8)), n("WDESTC", i), false));
            }
            pins.push((sram_pin_of(SramPin::A(Port::Right, 5)), "WREGC_n".into(), false));
            for b in 0..8 {
                pins.push((sram_pin_of(SramPin::Io(Port::Left, b as u8)), n(rdata, 8 * lane + b), true));
                pins.push((sram_pin_of(SramPin::Io(Port::Right, b as u8)), n("WD", 8 * lane + b), false));
            }
            pins.push((sram_pin_of(SramPin::Ce(Port::Left)), ce.into(), false));
            pins.push((sram_pin_of(SramPin::Ce(Port::Right)), "CLK".into(), false));
            pins.push((sram_pin_of(SramPin::Rw(Port::Right)), "WREGC_n".into(), false));
            let (block, stage) = block_of("rf");
            out.push(ChipInfo { name: format!("rf{bank}{lane}"), kind: "CY7C131", block, stage, pins });
        }
    }
    out
}

/// The structure as JSON: `{"chips":[{name,kind,block,stage,pins:[[pin,net,out]]}]}`.
pub fn structure_json() -> String {
    let mut s = String::from("{\"chips\":[");
    for (i, c) in chip_infos().iter().enumerate() {
        if i > 0 {
            s.push(',');
        }
        s.push_str(&format!("{{\"name\":\"{}\",\"kind\":\"{}\",\"block\":\"{}\",\"stage\":\"{}\",\"pins\":[", c.name, c.kind, c.block, c.stage));
        for (j, (p, net, o)) in c.pins.iter().enumerate() {
            if j > 0 {
                s.push(',');
            }
            s.push_str(&format!("[{p},\"{net}\",{o}]"));
        }
        s.push_str("]}");
    }
    s.push_str("]}");
    s
}
