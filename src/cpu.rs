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

use crate::as7c164a::As7c164a;
use crate::cy7c131::{Cy7c131, Port};
use crate::galpack::{Eq, GalSpec, Mode, SLit, instantiate, instantiate_all, lit, nlit, pack};
use crate::isa::Instr;
use crate::ds1100::{Ds1100, Grade};
use crate::netlist::{DS1100_IN, Level, NetId, Netlist, Sim, SramPin, Sram8kPin, Time, NS, ds1100_tap_pin, sram_pin_of, sram8k_pin_of};

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
    (2..=14)
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
        .collect()
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
            }
            Eq::sop(&n("MR", i), Mode::Reg, terms)
        })
        .collect()
}

/// EX/MEM store data with forwarding, tri-stated onto the data bus during
/// stores (enabled by MMW and the PH_SD strobe).
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
            .with_oe(vec![l("MMW"), l("PH_SD")])
        })
        .collect()
}

/// EX/MEM control.
fn exmem_ctrl_block() -> Vec<Eq> {
    let mut eqs = vec![
        Eq::sop("MRW", Mode::Reg, vec![vec![l("XRW")]]),
        Eq::sop("MMR", Mode::Reg, vec![vec![l("XMR")]]),
        Eq::sop("MMW", Mode::Reg, vec![vec![l("XMW")]]),
        Eq::sop("MMW_n", Mode::Reg, vec![vec![l("XMW")]]).active_low(),
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
    let mut eqs: Vec<Eq> = (0..32)
        .map(|i| {
            Eq::sop(&n("WD", i), Mode::Reg, vec![vec![l("MMR"), l(&n("DQ", i))], vec![nl_("MMR"), l(&n("MR", i))]])
        })
        .collect();
    for i in 0..5 {
        eqs.push(Eq::sop(&n("WDEST", i), Mode::Reg, vec![vec![l(&n("MDEST", i))]]));
    }
    // Polarity chosen so that reset (registers cleared) means "writing r0
    // with WD = 0": the register file's r0 is zeroed by the reset sequence
    // itself, and the steer keeps the read ports off r0 meanwhile.
    eqs.push(Eq::sop("WREG", Mode::Reg, vec![vec![nl_("MRW")]]).active_low());
    eqs.push(Eq::sop("WREG_n", Mode::Reg, vec![vec![nl_("MRW")]]));
    eqs
}

// ---------------------------------------------------------------------------
// Assembly

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
    v.extend(pack("pc", clk, rst, pc_block()));
    v.extend(pack("inc", None, None, inc_block()));
    v.extend(pack("ifid", clk, rst, ifid_block()));
    v.extend(pack("dec", None, None, dec_block()));
    v.extend(pack("ctl", clk, rst, ctrl_block()));
    v.extend(pack("steer", None, None, steer_block()));
    v.extend(pack("fwdc", clk, rst, fwdctl_block()));
    v.extend(pack("xa", clk, None, idex_a_block()));
    v.extend(pack("xb", clk, None, idex_b_block()));
    v.extend(pack("xsd", clk, None, idex_sd_block()));
    v.extend(pack("bt1", None, None, bt1));
    v.extend(pack("bt2", None, None, bt2));
    v.extend(pack("xbt", clk, None, btr));
    v.extend(pack("fa", None, None, fa));
    v.extend(pack("fb", None, None, fb));
    v.extend(pack("alu1", None, None, alu_l1_block()));
    v.extend(pack("alu2", None, None, alu_l2_block()));
    v.extend(pack("sh1", None, None, shift1_block()));
    v.extend(pack("shm", None, None, shift_mask_block()));
    v.extend(pack("sh2", None, None, shift2_block()));
    v.extend(pack("mr", clk, None, exmem_result_block()));
    v.extend(pack("msd", clk, None, exmem_sd_block()));
    v.extend(pack("mctl", clk, rst, exmem_ctrl_block()));
    v.extend(pack("cmp", None, None, cmp_block()));
    v.extend(pack("nxt", None, None, taken_block()));
    v.extend(pack("wb", clk, rst, memwb_block()));
    v
}

/// The running CPU.
/// Where the memory strobes come from.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Strobes {
    /// Ideal testbench stimulus at the [`phases`] times.
    Ideal,
    /// A DS1100-`total` delay line on CLK feeding one combinational GAL
    /// (`clkgen0`) that shapes PH_RF / PH_CE / PH_SD from the taps.
    DelayLine { total: u32, grade: Grade, taps: StrobeTaps },
}

/// Which tap edges shape each strobe.  A strobe's active window is
/// `[fall, half + rise]` in nominal ns, where `fall` is the tap index whose
/// rising edge starts it and `rise` the tap whose falling edge ends it
/// (index 0 = CLK itself, 1..=5 = TAP1..TAP5).  Built as
/// `CLK & !T(fall) | !CLK & !T(rise)`, which is high outside the window.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct StrobeTaps {
    pub ce: (u8, u8),
    pub rf: (u8, u8),
    pub sd: (u8, u8),
}

impl StrobeTaps {
    /// Nearest taps to the ideal phases for a DS1100-`total`.
    pub fn nearest(total: u32) -> StrobeTaps {
        let step = total as f64 / 5.0;
        let tap = |ns: f64| ((ns / step).round() as u8).clamp(0, 5);
        StrobeTaps { ce: (tap(10.0), tap(14.0)), rf: (tap(15.0), tap(14.0)), sd: (tap(8.0), tap(15.0)) }
    }
}

pub struct Cpu {
    pub sim: Sim,
    pub period: Time,
    pub cycles: u64,
    pub pc_trace: Vec<Option<u32>>,
    pub strobes: Strobes,
    /// When RESET was released (ps).
    pub reset_release: Time,
    clk: NetId,
    ph_rf: NetId,
    ph_ce: NetId,
    ph_sd: NetId,
    pc: Vec<NetId>,
    imem: Vec<usize>,
    dmem: Vec<usize>,
    rf: Vec<usize>,
    pub gal_count: usize,
}

/// Clock-generator phases as (fall, rise) for the active-low strobes and
/// (rise, fall) for PH_SD, in ns after the rising edge, given the period.
///
/// * PH_RF: register-file write CE.  Falls after the steer has settled
///   (13 ns) plus tPS; rises at least tHA (2 ns) before the next MEM/WB
///   update (next edge + 2), and after a >= 12 ns pulse.
/// * PH_CE: data-memory CE#.  Load data is valid 15 ns after the fall and
///   must still be there at the next edge, so it rises 1 ns after it; the
///   EX/MEM address does not change before next edge + 2.
/// * PH_SD: store-data output enable.  Rises after a preceding load's
///   outputs are off (CE# up at next+1, tCHZ 7: 8 ns into the cycle);
///   falls so the data holds to the write end (tER min 3) yet is gone
///   before a following load's outputs turn on (10 + tCLZ 4).
pub fn phases(period_ns: f64) -> [(f64, f64); 3] {
    [(15.0, period_ns - 3.0), (10.0, period_ns + 1.0), (8.0, period_ns - 2.0)]
}
/// Strobe generator: one combinational GAL on CLK and the delay-line taps.
/// Each strobe is `CLK & !T(fall) | !CLK & !T(rise)`: with a 50% clock and
/// T(k) a delayed copy of CLK, that is high from the edge until T(fall)
/// rises and again from T(rise)'s falling edge (half period + rise) until
/// the next edge.  PH_SD is the same shape inverted (active high).
pub fn clkgen_spec(taps: StrobeTaps) -> GalSpec {
    let t = |k: u8| if k == 0 { "CLK".to_string() } else { format!("T{k}") };
    let shape = |(fall, rise): (u8, u8)| -> Vec<Vec<SLit>> {
        let mut v = Vec::new();
        if fall != 0 {
            v.push(vec![l("CLK"), nl_(&t(fall))]);
        }
        v.push(vec![nl_("CLK"), nl_(&t(rise))]);
        v
    };
    let eqs = vec![
        Eq::sop("PH_CE", Mode::Comb, shape(taps.ce)),
        Eq::sop("PH_RF", Mode::Comb, shape(taps.rf)),
        Eq::sop("PH_SD", Mode::Comb, shape(taps.sd)).active_low(),
    ];
    GalSpec { name: "clkgen0".into(), clk: None, ar: None, eqs }
}

/// Time stamp of a chip warning (`t=123.500ns` inside the text).
fn warning_time(w: &str) -> Option<Time> {
    let i = w.find("t=")?;
    let rest = &w[i + 2..];
    let end = rest.find("ns")?;
    rest[..end].parse::<f64>().ok().map(|ns| (ns * NS as f64).round() as Time)
}

/// Clock cycles RESET is held with the clock running.
pub const RESET_CYCLES: usize = 4;

pub const PH_RF: usize = 0;
pub const PH_CE: usize = 1;
pub const PH_SD: usize = 2;

impl Cpu {
    /// Build the netlist with `program` in instruction memory at address 0,
    /// strobes from ideal stimulus.
    pub fn new(program: &[u32], period_ns: f64) -> Cpu {
        Cpu::with_strobes(program, period_ns, Strobes::Ideal)
    }

    pub fn with_strobes(program: &[u32], period_ns: f64, strobes: Strobes) -> Cpu {
        let mut nl = Netlist::new();
        let specs = gal_specs();
        let mut gal_count = specs.len();
        instantiate_all(&mut nl, &specs);
        if let Strobes::DelayLine { total, grade, taps } = strobes {
            let dl = nl.add_chip("dl0", Ds1100::new(total, grade));
            let clk = nl.net("CLK");
            nl.connect(clk, dl, DS1100_IN);
            for k in 0..5 {
                let net = nl.net(&format!("T{}", k + 1));
                nl.connect(net, dl, ds1100_tap_pin(k));
            }
            let spec = clkgen_spec(taps);
            spec.fits().unwrap();
            instantiate(&mut nl, &spec);
            gal_count += 1;
        }

        let gnd = nl.net("GND");
        let vcc = nl.net("VCC");
        nl.tie(gnd, Level::L);
        nl.tie(vcc, Level::H);
        let clk = nl.net("CLK");
        let ph_rf = nl.net("PH_RF");
        let ph_ce = nl.net("PH_CE");
        let ph_sd = nl.net("PH_SD");

        // Instruction memory: 4 lanes, address = PC[14:2].
        let mut imem = Vec::new();
        for lane in 0..4 {
            let mut chip = As7c164a::new();
            for (w, &word) in program.iter().enumerate() {
                chip.preload(w as u16, (word >> (8 * lane)) as u8);
            }
            for w in program.len()..8192 {
                chip.preload(w as u16, 0);
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
            nl.connect(vcc, c, sram8k_pin_of(Sram8kPin::Ce2));
            nl.connect(gnd, c, sram8k_pin_of(Sram8kPin::OeN));
            nl.connect(vcc, c, sram8k_pin_of(Sram8kPin::WeN));
        }
        // Data memory: address = MR[14:2], CE# strobe, WE# = MMW_n, OE# = GND.
        let mut dmem = Vec::new();
        for lane in 0..4 {
            let mut chip = As7c164a::new();
            for w in 0..8192 {
                chip.preload(w, 0);
            }
            let c = nl.add_chip(&format!("dmem{lane}"), chip);
            dmem.push(c);
            for a in 0..13 {
                let net = nl.net(&n("MR", a + 2));
                nl.connect(net, c, sram8k_pin_of(Sram8kPin::A(a as u8)));
            }
            for b in 0..8 {
                let net = nl.net(&n("DQ", 8 * lane + b));
                nl.connect(net, c, sram8k_pin_of(Sram8kPin::Dq(b as u8)));
            }
            nl.connect(ph_ce, c, sram8k_pin_of(Sram8kPin::CeN));
            nl.connect(vcc, c, sram8k_pin_of(Sram8kPin::Ce2));
            nl.connect(gnd, c, sram8k_pin_of(Sram8kPin::OeN));
            let we = nl.net("MMW_n");
            nl.connect(we, c, sram8k_pin_of(Sram8kPin::WeN));
        }
        // Register file: bank 0 reads rs (IR25..21) -> RA, bank 1 reads rt
        // (IR20..16) -> RB; write port: WDEST / WD / CE_W strobe / R/W = WREG_n.
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
                    let net = nl.net(&n("WDEST", i));
                    nl.connect(net, c, sram_pin_of(SramPin::A(Port::Right, i as u8)));
                }
                // A5 of the write port is the inverted write enable: an idle
                // strobe lands on 32..63, which no read ever matches, so it
                // never arbitrates against a read.
                let wreg_n = nl.net("WREG_n");
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
                nl.connect(ph_rf, c, sram_pin_of(SramPin::Ce(Port::Right)));
                let rw = nl.net("WREG_n");
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
        let pc = (2..=14).map(|i| sim.net_id(&n("PC", i))).collect();
        let mut cpu = Cpu {
            sim,
            period: (period_ns * NS as f64).round() as Time,
            cycles: 0,
            pc_trace: Vec::new(),
            clk,
            ph_rf,
            ph_ce,
            ph_sd,
            pc,
            imem,
            dmem,
            rf,
            gal_count,
            strobes,
            reset_release: 0,
        };
        // Power-on: RESET asserted from the start (a supervisor holds it
        // through power-up), clock low.
        let reset = cpu.sim.net_id("RESET");
        cpu.sim.schedule(0, reset, Level::H);
        cpu.sim.schedule(0, cpu.clk, Level::L);
        if strobes == Strobes::Ideal {
            cpu.sim.schedule(0, cpu.ph_rf, Level::H);
            cpu.sim.schedule(0, cpu.ph_ce, Level::H);
            cpu.sim.schedule(0, cpu.ph_sd, Level::L);
        }
        // Power-up: let every combinational chain settle, then hold RESET
        // with the clock and strobes running (the pipeline registers stay
        // cleared; the write-back stage writes r0 = 0), and release RESET
        // early in a cycle so the reset path settles and recovers before
        // the next edge.
        cpu.sim.run_until(3 * cpu.period);
        for _ in 0..RESET_CYCLES {
            cpu.step();
        }
        let release = cpu.sim.now() + cpu.period / 4;
        cpu.sim.schedule(release, reset, Level::L);
        cpu.step();
        cpu.reset_release = release;
        cpu.cycles = 0;
        cpu.pc_trace.clear();
        cpu
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
        self.sim
            .warnings()
            .into_iter()
            .filter(|w| warning_time(w).is_some_and(|t| t < self.reset_release))
            .filter(|w| !w.contains("CapturedX") && !w.contains("AddrUnknown"))
            .collect()
    }

    fn ns(t: f64) -> Time {
        (t * NS as f64).round() as Time
    }

    /// Run one clock cycle (rising edge now).
    pub fn step(&mut self) {
        let base = self.sim.now();
        let t = |off: f64| base + Self::ns(off);
        let half = base + self.period / 2;
        let ph = phases(self.period as f64 / NS as f64);
        self.sim.schedule(base, self.clk, Level::H);
        self.sim.schedule(half, self.clk, Level::L);
        if self.strobes == Strobes::Ideal {
            self.sim.schedule(t(ph[PH_RF].0), self.ph_rf, Level::L);
            self.sim.schedule(t(ph[PH_RF].1), self.ph_rf, Level::H);
            self.sim.schedule(t(ph[PH_CE].0), self.ph_ce, Level::L);
            self.sim.schedule(t(ph[PH_CE].1), self.ph_ce, Level::H);
            self.sim.schedule(t(ph[PH_SD].0), self.ph_sd, Level::H);
            self.sim.schedule(t(ph[PH_SD].1), self.ph_sd, Level::L);
        }
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
    pub fn dmem_word(&self, addr: u32) -> Option<u32> {
        let mut w = 0u32;
        for lane in 0..4 {
            let chip = self.chip_sram8k(self.dmem[lane]);
            w |= (chip.peek((addr >> 2) as u16)? as u32) << (8 * lane);
        }
        Some(w)
    }
    fn chip_sram(&self, id: usize) -> &Cy7c131 {
        self.sim.chip(id).downcast_ref::<Cy7c131>().unwrap()
    }
    fn chip_sram8k(&self, id: usize) -> &As7c164a {
        self.sim.chip(id).downcast_ref::<As7c164a>().unwrap()
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
        "clkgen" | "dl" => ("Clock generator", "CLK"),
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
        out.push(ChipInfo { name: format!("imem{lane}"), kind: "AS7C164A", block, stage, pins });
        let mut pins = Vec::new();
        for a in 0..13 {
            pins.push((sram8k_pin_of(Sram8kPin::A(a as u8)), n("MR", a + 2), false));
        }
        for b in 0..8 {
            pins.push((sram8k_pin_of(Sram8kPin::Dq(b as u8)), n("DQ", 8 * lane + b), true));
        }
        pins.push((sram8k_pin_of(Sram8kPin::CeN), "PH_CE".into(), false));
        pins.push((sram8k_pin_of(Sram8kPin::WeN), "MMW_n".into(), false));
        let (block, stage) = block_of("dmem");
        out.push(ChipInfo { name: format!("dmem{lane}"), kind: "AS7C164A", block, stage, pins });
    }
    for bank in 0..2 {
        for lane in 0..4 {
            let (field, rdata, ce) = if bank == 0 { (21, "RA", "CERA_n") } else { (16, "RB", "CERB_n") };
            let mut pins = Vec::new();
            for i in 0..5 {
                pins.push((sram_pin_of(SramPin::A(Port::Left, i as u8)), ir(field + i), false));
                pins.push((sram_pin_of(SramPin::A(Port::Right, i as u8)), n("WDEST", i), false));
            }
            pins.push((sram_pin_of(SramPin::A(Port::Right, 5)), "WREG_n".into(), false));
            for b in 0..8 {
                pins.push((sram_pin_of(SramPin::Io(Port::Left, b as u8)), n(rdata, 8 * lane + b), true));
                pins.push((sram_pin_of(SramPin::Io(Port::Right, b as u8)), n("WD", 8 * lane + b), false));
            }
            pins.push((sram_pin_of(SramPin::Ce(Port::Left)), ce.into(), false));
            pins.push((sram_pin_of(SramPin::Ce(Port::Right)), "PH_RF".into(), false));
            pins.push((sram_pin_of(SramPin::Rw(Port::Right)), "WREG_n".into(), false));
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
