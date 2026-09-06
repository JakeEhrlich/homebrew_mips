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
use crate::galpack::{Eq, GalSpec, Mode, SLit, instantiate_all, lit, nlit, pack};
use crate::isa::Instr;
use crate::netlist::{Level, NetId, Netlist, Sim, SramPin, Sram8kPin, Time, NS, sram_pin_of, sram8k_pin_of};

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
    jr: bool,
    load: bool,
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
        jr: false,
        load: false,
    };
    match op {
        Nop => {}
        Addu | Subu | And | Or | Xor | Nor | Slt | Sltu => {
            d.rtype_rw = true;
            d.uses_rt = true;
        }
        Jr => d.jr = true,
        Addiu | Andi | Ori | Xori | Slti | Sltiu | Lui | Lw => {
            d.itype_rw = true;
            d.selimm = true;
        }
        Sw => {
            d.selimm = true;
            d.store = true;
        }
        Beq | Bne => d.uses_rt = true,
        J => d.seljt = true,
        Jal => {
            d.seljt = true;
            d.jal = true;
        }
    }
    d.sext = matches!(op, Addiu | Slti | Sltiu | Lw | Sw);
    d.lui = op == Lui;
    // XADD: the result is the adder output (SLT/SLTU use the adder but
    // produce only the compare bit).  XSUB: invert B, carry-in 1.
    d.add = matches!(op, Addu | Addiu | Subu | Lui | Lw | Sw | Jal | Jr | Nop);
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
    opcode << 26 | funct
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
        eqs.push(Eq::sop(&n("P4", i), Mode::Reg, vec![vec![l(&n("INC", i))]]));
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
    ]
}

/// ID/EX control: ALU op and branch bits straight from the opcode/funct
/// tables (decode folded into the register), plus destination / write
/// enable, memory bits.
fn ctrl_block() -> Vec<Eq> {
    let mut eqs = vec![
        dec_table("XADD", Mode::Reg, |d| d.add),
        dec_table("XSUB", Mode::Reg, |d| d.sub),
        dec_table("XAND", Mode::Reg, |d| d.and),
        dec_table("XOR_", Mode::Reg, |d| d.or),
        dec_table("XXOR", Mode::Reg, |d| d.xor),
        dec_table("XNOR", Mode::Reg, |d| d.nor),
        dec_table("XSLT", Mode::Reg, |d| d.slt),
        dec_table("XSLTU", Mode::Reg, |d| d.sltu),
        dec_table("XBEQ", Mode::Reg, |d| d.beq),
        dec_table("XBNE", Mode::Reg, |d| d.bne),
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

/// ID/EX operand A: register file / MEM/WB (steer) / PC+4 (JAL link).
fn idex_a_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let mut terms = vec![
                vec![nl_("JAL"), nl_("STA"), l(&n("RA", i))],
                vec![nl_("JAL"), l("STA"), l(&n("WD", i))],
            ];
            if (2..=14).contains(&i) {
                terms.push(vec![l("JAL"), l(&n("P4", i))]);
            }
            Eq::sop(&n("XA", i), Mode::Reg, terms)
        })
        .collect()
}

/// ID/EX operand B: register file / MEM/WB / immediate (sign, zero or
/// LUI-extended) / constant 4 (JAL link).
fn idex_b_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let mut terms = vec![
                vec![nl_("SELIMM"), nl_("JAL"), nl_("STB"), l(&n("RB", i))],
                vec![nl_("SELIMM"), nl_("JAL"), l("STB"), l(&n("WD", i))],
            ];
            if i < 16 {
                terms.push(vec![l("SELIMM"), nl_("LUI"), l(&ir(i))]);
            } else {
                terms.push(vec![l("SELIMM"), l("SEXT"), l(&ir(15))]);
                terms.push(vec![l("SELIMM"), l("LUI"), l(&ir(i - 16))]);
            }
            if i == 2 {
                terms.push(vec![l("JAL")]);
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
        let mut ins: Vec<String> = (lo..=hi).map(|i| n("P4", i)).collect();
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
    let fa = (0..32)
        .map(|i| {
            Eq::sop(
                &n("FA", i),
                Mode::Comb,
                vec![
                    vec![nl_("XFAE"), nl_("XFAM"), l(&n("XA", i))],
                    vec![l("XFAE"), l(&n("MR", i))],
                    vec![nl_("XFAE"), l("XFAM"), l(&n("WD", i))],
                ],
            )
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

/// EX/MEM result register with the ALU's last level folded in.
fn exmem_result_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let k = (i / 3).min(10);
            let (s0, s1) = (format!("S0_{i}"), format!("S1_{i}"));
            let (fa, fb) = (n("FA", i), n("FB", i));
            // Sum with carry select (group 0's carry-in is XSUB).
            let c = if k == 0 { "XSUB".to_string() } else { format!("C{k}") };
            let mut terms = vec![
                vec![l("XADD"), l(&c), l(&s1)],
                vec![l("XADD"), nl_(&c), l(&s0)],
                vec![l("XAND"), l(&fa), l(&fb)],
                vec![l("XOR_"), l(&fa)],
                vec![l("XOR_"), l(&fb)],
                vec![l("XXOR"), l(&fa), nl_(&fb)],
                vec![l("XXOR"), nl_(&fa), l(&fb)],
                vec![l("XNOR"), nl_(&fa), nl_(&fb)],
            ];
            if i == 0 {
                // SLTU: a < b  <=>  no carry out of a + !b + 1.
                // carry out = G10 + PP10 & C10.
                terms.push(vec![l("XSLTU"), nl_("G10"), nl_("PP10")]);
                terms.push(vec![l("XSLTU"), nl_("G10"), nl_("C10")]);
                // SLT (signed): signs differ -> a negative; else sign of a - b,
                // whose bit 31 is the selected sum bit 31 (b inverted, cin 1).
                // b31 as seen here is !FB31.
                terms.push(vec![l("XSLT"), l("FA31"), l("FB31")]); // a neg, b pos
                for (cval, s) in [(true, "S1_31"), (false, "S0_31")] {
                    let cl = if cval { l("C10") } else { nl_("C10") };
                    // same sign (FA31 != FB31 since FB is inverted): result negative
                    terms.push(vec![l("XSLT"), l("FA31"), nl_("FB31"), cl.clone(), l(s)]);
                    terms.push(vec![l("XSLT"), nl_("FA31"), l("FB31"), cl, l(s)]);
                }
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
    let ins = ["NEQ0", "NEQ1", "NEQ2", "NEQ3", "XBEQ", "XBNE", "XJR", "DSELJT"];
    let neq = |m: u32| m & 15 != 0;
    let taken = move |m: u32| (m >> 4 & 1 == 1 && !neq(m)) || (m >> 5 & 1 == 1 && neq(m));
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
    eqs.push(Eq::sop("WREG", Mode::Reg, vec![vec![l("MRW")]]));
    eqs.push(Eq::sop("WREG_n", Mode::Reg, vec![vec![l("MRW")]]).active_low());
    eqs
}

// ---------------------------------------------------------------------------
// Assembly

/// Every GAL of the CPU, packed.
pub fn gal_specs() -> Vec<GalSpec> {
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
    v.extend(pack("mr", clk, None, exmem_result_block()));
    v.extend(pack("msd", clk, None, exmem_sd_block()));
    v.extend(pack("mctl", clk, rst, exmem_ctrl_block()));
    v.extend(pack("cmp", None, None, cmp_block()));
    v.extend(pack("nxt", None, None, taken_block()));
    v.extend(pack("wb", clk, rst, memwb_block()));
    v
}

/// The running CPU.
pub struct Cpu {
    pub sim: Sim,
    pub period: Time,
    pub cycles: u64,
    pub pc_trace: Vec<Option<u32>>,
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
pub const PH_RF: usize = 0;
pub const PH_CE: usize = 1;
pub const PH_SD: usize = 2;

impl Cpu {
    /// Build the netlist with `program` in instruction memory at address 0.
    pub fn new(program: &[u32], period_ns: f64) -> Cpu {
        let mut nl = Netlist::new();
        let specs = gal_specs();
        let gal_count = specs.len();
        instantiate_all(&mut nl, &specs);

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
        };
        let reset = cpu.sim.net_id("RESET");
        cpu.sim.schedule(0, reset, Level::L);
        cpu.sim.schedule(0, cpu.clk, Level::L);
        cpu.sim.schedule(0, cpu.ph_rf, Level::H);
        cpu.sim.schedule(0, cpu.ph_ce, Level::H);
        cpu.sim.schedule(0, cpu.ph_sd, Level::L);
        // Power-up: let every combinational chain settle before the first
        // edge (the reset period in hardware).
        cpu.sim.run_until(3 * cpu.period);
        cpu
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
        self.sim.schedule(t(ph[PH_RF].0), self.ph_rf, Level::L);
        self.sim.schedule(t(ph[PH_RF].1), self.ph_rf, Level::H);
        self.sim.schedule(t(ph[PH_CE].0), self.ph_ce, Level::L);
        self.sim.schedule(t(ph[PH_CE].1), self.ph_ce, Level::H);
        self.sim.schedule(t(ph[PH_SD].0), self.ph_sd, Level::H);
        self.sim.schedule(t(ph[PH_SD].1), self.ph_sd, Level::L);
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
    pub fn warnings(&self) -> Vec<String> {
        self.sim.warnings()
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
