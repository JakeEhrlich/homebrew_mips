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
use crate::galpack::{Eq, GalSpec, Mode, SLit, lit, nlit, pack};
use crate::isa::Instr;
use crate::ds1100::{Ds1100, Grade};
use crate::uart16550::{BusTiming, Uart16550, UartPin, uart_pin_of};
use crate::board::{Board, ChipMeta, Column, Load, Model};
use crate::netlist::{DS1100_IN, FastGate, Level, NetId, Netlist, Passive, ResetSupervisor, Rom, RomPin, Sim, Sram16, Sram16Pin, SramPin, Sram8kPin, Time, NS, ds1100_tap_pin, rom_pin_of, sram_pin_of, sram8k_pin_of, sram16_pin_of};
use std::collections::{BTreeMap, HashMap};

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
    /// Byte-sized memory access (LB, LBU, SB).
    szb: bool,
    /// Halfword-sized (LH, LHU, SH).
    szh: bool,
    /// Signed narrow load (LB, LH): fill with the sign.
    lsx: bool,
    /// Narrow store (SB, SH).
    nstore: bool,
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
        szb: false,
        szh: false,
        lsx: false,
        nstore: false,
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
        Addiu | Andi | Ori | Xori | Slti | Sltiu | Lui | Lw | Lb | Lbu | Lh | Lhu => {
            d.itype_rw = true;
            d.selimm = true;
        }
        Sw | Sb | Sh => {
            d.selimm = true;
            d.store = true;
        }
        Beq | Bne | Blez | Bgtz => d.uses_rt = true,
        Bltz | Bgez => {}
        // Link branches: the branch of BLTZ / BGEZ plus the link of JAL
        // (r31 <- PC + 8 through operand B and the pass-B ALU op).
        Bltzal | Bgezal => {
            d.jal = true;
            d.link = true;
        }
        J => d.seljt = true,
        Jal => {
            d.seljt = true;
            d.jal = true;
            d.link = true;
        }
    }
    d.brs = matches!(op, Blez | Bgtz | Bltz | Bgez | Bltzal | Bgezal);
    d.bz = matches!(op, Blez | Bgtz);
    d.binv = matches!(op, Bgtz | Bgez | Bgezal);
    d.sext = matches!(op, Addiu | Slti | Sltiu | Lw | Sw | Lb | Lbu | Lh | Lhu | Sb | Sh);
    d.szb = matches!(op, Lb | Lbu | Sb);
    d.szh = matches!(op, Lh | Lhu | Sh);
    d.lsx = matches!(op, Lb | Lh);
    d.nstore = matches!(op, Sb | Sh);
    d.lui = op == Lui;
    // XADD: the result is the adder output (SLT/SLTU use the adder but
    // produce only the compare bit).  XSUB: invert B, carry-in 1.
    d.add = matches!(op, Addu | Addiu | Subu | Lui | Lw | Sw | Lb | Lbu | Lh | Lhu | Sb | Sh | Jr | Nop);
    d.sub = matches!(op, Subu | Slt | Sltu | Slti | Sltiu);
    d.and = matches!(op, And | Andi);
    d.or = matches!(op, Or | Ori);
    d.xor = matches!(op, Xor | Xori);
    d.nor = op == Nor;
    d.slt = matches!(op, Slt | Slti);
    d.sltu = matches!(op, Sltu | Sltiu);
    d.beq = op == Beq;
    d.bne = op == Bne;
    d.load = matches!(op, Lw | Lb | Lbu | Lh | Lhu);
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

/// A decode signal that is also set by the REGIMM link branches (BLTZAL
/// / BGEZAL: opcode 1 with IR20), which the opcode / funct tables cannot
/// see.  The table part is kept positive so the extra term can be OR'd.
fn dec_table_or_regimm_link(out: &str, mode: Mode, f: impl Fn(&Dec) -> bool) -> Eq {
    let ins = dec_inputs();
    let mut eq = Eq::table_pos(out, mode, &strs(&ins), |m| decode(dec_word(m)).map(|d| f(&d)));
    eq.terms.push(vec![nl_(&ir(31)), nl_(&ir(30)), nl_(&ir(29)), nl_(&ir(28)), nl_(&ir(27)), l(&ir(26)), l(&ir(20))]);
    eq
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
        dec_table_or_regimm_link("JAL", Mode::Comb, |d| d.jal),
        dec_table("SELIMM", Mode::Comb, |d| d.selimm),
        dec_table("SEXT", Mode::Comb, |d| d.sext),
        dec_table("LUI", Mode::Comb, |d| d.lui),
        dec_table("DSELJT", Mode::Comb, |d| d.seljt),
        dec_table("USESRT", Mode::Comb, |d| d.uses_rt),
        dec_table("STORE", Mode::Comb, |d| d.store),
        dec_table_or_regimm_link("LINK", Mode::Comb, |d| d.link),
        dec_table("SHIMM", Mode::Comb, |d| d.shimm),
        dec_table("LOAD", Mode::Comb, |d| d.load),
        // Byte-sized access: the ID/EX store data replicates the byte
        // into lane 1.  Narrow store: the forward-hold (see sf_block).
        dec_table("SZB", Mode::Comb, |d| d.szb),
        dec_table("NSTORE", Mode::Comb, |d| d.nstore),
    ]
}

/// Narrow-store forward hold.  A byte or halfword store replicates its
/// data across the lanes from the register file (ID/EX lane 1, EX/MEM
/// lanes 2 and 3); a value that would have to be forwarded from EX/MEM
/// or MEM/WB is not replicated, so such a store waits in ID until its
/// producer has reached WB and the steer supplies it.  SFE / SFM: the
/// narrow store in ID names, as rt, the destination of the instruction
/// in EX / in MEM.  Active-low sums (the complement of a 5-bit equality
/// is ten terms), like the forwarding control.
fn sf_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    for (name, dest, rw) in [("SFE", "XDEST", "XRW"), ("SFM", "MDEST", "MRW")] {
        let mut terms: Vec<Vec<SLit>> = vec![vec![nl_("NSTORE")], vec![nl_(rw)]];
        for i in 0..5 {
            terms.push(vec![l(&ir(16 + i)), nl_(&n(dest, i))]);
            terms.push(vec![nl_(&ir(16 + i)), l(&n(dest, i))]);
        }
        eqs.push(Eq::sop(name, Mode::Comb, terms).active_low());
    }
    eqs
}

/// ID/EX control: ALU op and branch bits straight from the opcode/funct
/// tables (decode folded into the register), plus destination / write
/// enable, memory bits.
fn ctrl_block() -> Vec<Eq> {
    let mut eqs = vec![
        // XOP0 is also set by the link branches (pass-B op code 1).
        dec_table_or_regimm_link("XOP0", Mode::Reg, |d| alu_op(d) & 1 == 1),
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
        // Access size (byte enables, load lane select), sign extension,
        // and "narrow" for the EX/MEM store-data lanes 2 and 3.
        dec_table("XSZB", Mode::Reg, |d| d.szb),
        dec_table("XSZH", Mode::Reg, |d| d.szh),
        dec_table("XLSX", Mode::Reg, |d| d.lsx),
        dec_table("XNAR", Mode::Reg, |d| d.szb || d.szh),
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

/// ID/EX store data: rt from the register file or MEM/WB.  Lane 1 takes
/// the low byte for a byte-sized access (SB puts its byte on every lane;
/// lanes 2 and 3 are filled from lanes 0 and 1 by the EX/MEM drivers).
fn idex_sd_block() -> Vec<Eq> {
    (0..32)
        .map(|i| {
            let terms = if (8..16).contains(&i) {
                vec![
                    vec![nl_("SZB"), nl_("STB"), l(&n("RB", i))],
                    vec![nl_("SZB"), l("STB"), l(&n("WD", i))],
                    vec![l("SZB"), nl_("STB"), l(&n("RB", i - 8))],
                    vec![l("SZB"), l("STB"), l(&n("WD", i - 8))],
                ]
            } else {
                vec![vec![nl_("STB"), l(&n("RB", i))], vec![l("STB"), l(&n("WD", i))]]
            };
            Eq::sop(&n("XSD", i), Mode::Reg, terms)
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
        // Group 0 has no carry in: only its cin = 0 sum and generate.
        for cin in 0..if k == 0 { 1 } else { 2u32 } {
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
            if k > 0 {
                l1.push(Eq::table(&format!("BP{k}"), Mode::Comb, &strs(&ins), move |m| {
                    Some(((m & mask) + (m >> w & mask) + 1) >> w & 1 == 1)
                }));
            }
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
        if k == 0 {
            // Group 0's carry-in is XSUB, known from the start of the
            // cycle: bit 0's sum is produced with it folded in (SUM0, four
            // terms), so the EX/MEM result bit 0 needs one term for it
            // instead of a carry-select pair; that term is what its hold
            // needs.  (Bits 1 and 2 would take 12 and 28 terms folded.)
            let mut ins0 = ins.clone();
            ins0.push("XSUB".into());
            eqs.push(Eq::table("SUM0", Mode::Comb, &strs(&ins0), move |m| {
                let cin = m >> (2 * w) & 1;
                Some(((m & mask) + (m >> w & mask) + cin) & 1 == 1)
            }));
        }
        for cin in 0..2u32 {
            for bit in 0..w {
                if k == 0 && bit == 0 {
                    continue;
                }
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
            let c = if k == 0 { "XSUB".to_string() } else { format!("C{k}") };
            let with = |code: u32, extra: Vec<SLit>| {
                let mut t = op_lits(code);
                t.push(nl_("XSH"));
                t.extend(extra);
                t
            };
            // Sum: bit 0 has its carry-in folded in (SUM0), the others
            // select by the group carry (XSUB for group 0).
            let sum: Vec<Vec<SLit>> = if i == 0 {
                vec![with(0, vec![l("SUM0")])]
            } else {
                vec![with(0, vec![l(&c), l(&s1)]), with(0, vec![nl_(&c), l(&s0)])]
            };
            let mut terms = vec![vec![l("XSH"), l(&n("SH", i))]];
            terms.extend(sum);
            terms.extend(vec![
                with(1, vec![l(&fb)]),
                with(2, vec![l(&fa), l(&fb)]),
                with(3, vec![l(&fa)]),
                with(3, vec![l(&fb)]),
                with(4, vec![l(&fa), nl_(&fb)]),
                with(4, vec![nl_(&fa), l(&fb)]),
                with(5, vec![nl_(&fa), nl_(&fb)]),
            ]);
            if i == 0 {
                // Bit 0 also carries SLT and SLTU, which pushes the hand-written
                // form past 16 terms; let the minimiser share terms, with the
                // control combinations that decode never produces as don't
                // cares (XSH implies XOP = 0; XOP 6/7 imply XSUB).
                let ins = [
                    "XOP0", "XOP1", "XOP2", "XSH", "SUM0", "FA0", "FB0", "SH0", "FA31", "FB31", "C10", "S0_31", "S1_31", "G10", "PP10",
                ];
                let f = |m: u32| -> Option<bool> {
                    let bit = |b: usize| m >> b & 1 == 1;
                    let op = m & 7;
                    let xsh = bit(3);
                    let (sum0, fa, fb, sh) = (bit(4), bit(5), bit(6), bit(7));
                    let (fa31, fb31, c10, s0_31, s1_31, g10, pp10) = (bit(8), bit(9), bit(10), bit(11), bit(12), bit(13), bit(14));
                    if xsh {
                        return if op == 0 { Some(sh) } else { None };
                    }
                    // Ops 6 and 7 (SLT, SLTU) always subtract, so the
                    // carries are those of A - B.
                    Some(match op {
                        0 => sum0,
                        1 => fb,
                        2 => fa & fb,
                        3 => fa | fb,
                        4 => fa ^ fb,
                        5 => !(fa | fb),
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
            // Lanes 2 and 3 of a narrow store repeat lanes 0 and 1 (which
            // ID/EX has already made "the byte" or "the halfword").
            let mut terms = if i >= 16 {
                vec![
                    vec![nl_("XFSE"), nl_("XFSM"), nl_("XNAR"), l(&n("XSD", i))],
                    vec![nl_("XFSE"), nl_("XFSM"), l("XNAR"), l(&n("XSD", i - 16))],
                ]
            } else {
                vec![vec![nl_("XFSE"), nl_("XFSM"), l(&n("XSD", i))]]
            };
            terms.push(vec![l("XFSE"), l(&n("MR", i))]);
            terms.push(vec![nl_("XFSE"), l("XFSM"), l(&n("WD", i))]);
            Eq::sop(&n("DQ", i), Mode::Reg, terms).with_oe(vec![l("MMW")])
        })
        .collect()
}

/// MEM-stage access control, registered at the EX/MEM edge: the data
/// memory's byte enables (active low; a narrow store leaves the other
/// lanes' enables high, everything else keeps all four low), the
/// MEM-stage copies of the size / sign flags and of the two low address
/// bits (the EX/MEM result register's, but forced to lane 0 and word
/// for an I/O access, whose byte arrives on lane 0 whatever the
/// address), and the selected element's sign bit for the load extension.
fn mem_access_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    // Address low bits: bit 0's sum (carry-in folded in), bit 1's group-0
    // sum with carry-in 0 (loads and stores add).
    let (a0, a1) = ("SUM0".to_string(), n("S0_", 1));
    for j in 0..4usize {
        let j1 = j >> 1 & 1 == 1;
        let mut terms = Vec::new();
        // Byte store not at lane j: the three other lanes.
        for a in 0..4usize {
            if a != j {
                terms.push(vec![l("XMW"), l("XSZB"), (a0.clone(), a & 1 == 1), (a1.clone(), a >> 1 & 1 == 1)]);
            }
        }
        // Halfword store in the other half.
        terms.push(vec![l("XMW"), l("XSZH"), (a1.clone(), !j1)]);
        eqs.push(Eq::sop(&format!("MBE{j}_n"), Mode::Reg, terms));
    }
    eqs.push(Eq::sop("MSZB", Mode::Reg, vec![vec![l("XSZB")]]));
    eqs.push(Eq::sop("MSZH", Mode::Reg, vec![vec![l("XSZH")]]));
    eqs.push(Eq::sop("MLSX", Mode::Reg, vec![vec![l("XLSX")]]));
    // Sign of the selected byte (MSZB) or halfword; the address bits are
    // the EX/MEM result's.
    eqs.push(Eq::sop(
        "LSGN",
        Mode::Comb,
        vec![
            vec![l("MSZB"), nl_("MR1"), nl_("MR0"), l(&n("DQ", 7))],
            vec![l("MSZB"), nl_("MR1"), l("MR0"), l(&n("DQ", 15))],
            vec![l("MSZB"), l("MR1"), nl_("MR0"), l(&n("DQ", 23))],
            vec![l("MSZB"), l("MR1"), l("MR0"), l(&n("DQ", 31))],
            vec![nl_("MSZB"), nl_("MR1"), l(&n("DQ", 15))],
            vec![nl_("MSZB"), l("MR1"), l(&n("DQ", 31))],
        ],
    ));
    eqs
}

/// MEM/WB load-data selects, combinational in MEM (one-hot per source):
/// which DQ byte lands in each byte of WD, and the sign fills.
fn mem_select_block() -> Vec<Eq> {
    let live = || vec![l("MMR")];
    let with = |extra: Vec<SLit>| {
        let mut t = live();
        t.extend(extra);
        t
    };
    vec![
        // Not a load: WD takes the ALU result.
        Eq::sop("MNR", Mode::Comb, vec![vec![nl_("MMR")]]),
        // Bits 7:0 from DQ lane 0 / 1 / 2 / 3.
        Eq::sop("ML0", Mode::Comb, vec![with(vec![nl_("MSZB"), nl_("MSZH")]), with(vec![l("MSZH"), nl_("MR1")]), with(vec![l("MSZB"), nl_("MR1"), nl_("MR0")])]),
        Eq::sop("ML1", Mode::Comb, vec![with(vec![l("MSZB"), nl_("MR1"), l("MR0")])]),
        Eq::sop("ML2", Mode::Comb, vec![with(vec![l("MSZH"), l("MR1")]), with(vec![l("MSZB"), l("MR1"), nl_("MR0")])]),
        Eq::sop("ML3", Mode::Comb, vec![with(vec![l("MSZB"), l("MR1"), l("MR0")])]),
        // Bits 15:8: own lane (word, or halfword from the low half), the
        // high half's low byte, or the byte's sign.  A byte device (BB8)
        // drives lane 0 only: its upper bytes read as zero.
        Eq::sop("MW8", Mode::Comb, vec![with(vec![nl_("MSZB"), nl_("MSZH"), nl_("BB8")]), with(vec![l("MSZH"), nl_("MR1")])]),
        Eq::sop("MH1", Mode::Comb, vec![with(vec![l("MSZH"), l("MR1")])]),
        Eq::sop("MSB", Mode::Comb, vec![with(vec![l("MSZB"), l("MLSX")])]),
        // Bits 31:16: own lane (word) or the sign.
        Eq::sop("MW16", Mode::Comb, vec![with(vec![nl_("MSZB"), nl_("MSZH"), nl_("BB8")])]),
        Eq::sop("MSN", Mode::Comb, vec![with(vec![l("MSZB"), l("MLSX")]), with(vec![l("MSZH"), l("MLSX")])]),
    ]
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
        // ... and off during an I/O load's MEM cycle (a device drives the
        // bus) and the cycle after (until the device has let go).
        Eq::sop("OEN", Mode::Reg, {
            let base = vec![vec![nl_("STORE"), nl_("XMW"), nl_("MMW"), nl_("BOOT")], vec![l("XMR"), nl_("XMW"), nl_("MMW"), nl_("BOOT")]];
            let mut terms = Vec::new();
            for t in base {
                for a in [nl_("XMR"), nl_(&n("FA", 31))] {
                    for b in [nl_("MMR"), nl_("MIO")] {
                        let mut u = t.clone();
                        u.push(a.clone());
                        u.push(b.clone());
                        terms.push(u);
                    }
                }
            }
            terms
        })
        .active_low(),

        // Data memory CE# (low = selected): off during the boot code phase
        // and during an I/O access (the UART has the bus).
        Eq::sop("DMEN_n", Mode::Comb, vec![vec![l("BOOTCNT"), l("CODE")], vec![l("MIO")]]),
        // The bus strobes (docs/bus.md): levels for the access's cycle.
        Eq::sop("BRD_n", Mode::Comb, vec![vec![l("MIO"), l("MMR")]]).active_low(),
        Eq::sop("BWR_n", Mode::Comb, vec![vec![l("MIO"), l("MMW")]]).active_low(),
    ]
}

/// The interlock and boot terms of HOLD.
fn hold_terms() -> Vec<Vec<SLit>> {
    vec![
        vec![l("STORE"), l("XMR")],
        vec![l("LOAD"), l("XMW")],
        vec![l("LOAD"), l("XMR"), l(&n("FA", 31))],
        // A narrow store whose data would have to be forwarded (sf_block).
        vec![l("SFE")],
        vec![l("SFM")],
        vec![l("BOOTCNT"), nl_("STEP2")],
        vec![l("BOOTCNT"), l("STEP1")],
        vec![l("BOOTCNT"), l("STEP0")],
    ]
}

/// A slow chip behind registers: the bridge that makes any device with
/// its own strobe timing look like a memory to the bus (docs/bus.md
/// section 5).  This instance carries the TL16C550 (docs/uart.md).
///
/// Registers (slot 0):
///
/// | offset | read | write |
/// |---|---|---|
/// | 0 | RDATA: the result of the last read command | CMD: bits 7:0 data, bits 10:8 the chip's register address, bit 15 = read; starts a cycle when not busy |
/// | 4 | STATUS: bit 0 BUSY | |
///
/// A command latches the address, data and direction (SBA, UD, SBRW),
/// raises BUSY (the 16550's chip select) and runs a cycle counter K:
/// reads hold RD# low for K = 1..3, latch the chip's data at the end of
/// K = 3 and stay busy to K = 13 (the 16550 wants 425 ns between FIFO
/// reads); writes hold WR# low for K = 1..2 with the data on the chip's
/// private bus UD until K = 4.  The chip's timing model checks all of it.
fn bridge_block() -> Vec<Eq> {
    let mut eqs = Vec::new();
    // Decode (combinational): slot 0, the accesses, the byte-device flag.
    eqs.push(Eq::sop("SLOT0", Mode::Comb, vec![vec![nl_(&n("MR", 23)), nl_(&n("MR", 24)), nl_(&n("MR", 25)), nl_(&n("MR", 26))]]));
    // Everything below is from bus signals only (the strobes, the
    // address, BB8), so the block moves to a card unchanged.
    eqs.push(Eq::sop("SBCMD", Mode::Comb, vec![vec![nl_("BWR_n"), l("SLOT0"), nl_(&n("MR", 3)), nl_(&n("MR", 2))]]));
    eqs.push(Eq::sop("SBRDD", Mode::Comb, vec![vec![nl_("BRD_n"), l("SLOT0"), nl_(&n("MR", 3)), nl_(&n("MR", 2))]]));
    eqs.push(Eq::sop("SBRDS", Mode::Comb, vec![vec![nl_("BRD_n"), l("SLOT0"), nl_(&n("MR", 3)), l(&n("MR", 2))]]));
    eqs.push(Eq::sop("START", Mode::Comb, vec![vec![l("SBCMD"), nl_("BUSY")]]));
    eqs.push(Eq::sop("UDOE", Mode::Comb, vec![vec![l("BUSY"), nl_("SBRW")]]));
    eqs.push(Eq::sop("BB8", Mode::Comb, vec![vec![nl_("BRD_n"), l("SLOT0")]]).with_oe(vec![nl_("BRD_n"), l("SLOT0")]));
    // Data latch: its outputs are the chip's private data bus UD, driven
    // during a write (the chip drives it during a read).
    for i in 0..8 {
        eqs.push(Eq::sop(&n("UD", i), Mode::Reg, vec![vec![l("START"), l(&n("DQ", i))], vec![nl_("START"), l(&n("UD", i))]]).with_oe(vec![l("UDOE")]));
    }
    // Address latch.
    for i in 0..3 {
        eqs.push(Eq::sop(&n("SBA", i), Mode::Reg, vec![vec![l("START"), l(&n("DQ", 8 + i))], vec![nl_("START"), l(&n("SBA", i))]]));
    }
    // Direction, busy, counter, strobes, the latch pulse.
    let k: Vec<String> = (0..4).map(|i| n("K", i)).collect();
    let mut ins: Vec<String> = k.clone();
    ins.extend(["BUSY", "SBRW", "START"].map(String::from));
    let ins = strs(&ins);
    let kv = |m: u32| (m & 15) as usize;
    let busy = |m: u32| m >> 4 & 1 == 1;
    let rw = |m: u32| m >> 5 & 1 == 1;
    let start = |m: u32| m >> 6 & 1 == 1;
    let last = |m: u32| if rw(m) { 13 } else { 4 };
    eqs.push(Eq::sop("SBRW", Mode::Reg, vec![vec![l("START"), l(&n("DQ", 15))], vec![nl_("START"), l("SBRW")]]));
    eqs.push(Eq::table_pos("BUSY", Mode::Reg, &ins, move |m| Some(start(m) || (busy(m) && kv(m) < last(m)))));
    for b in 0..4 {
        eqs.push(Eq::table_pos(&k[b], Mode::Reg, &ins, move |m| Some(busy(m) && !start(m) && (kv(m) + 1) >> b & 1 == 1)));
    }
    eqs.push(Eq::table_pos("URD_n", Mode::Reg, &ins, move |m| Some(busy(m) && rw(m) && (0..=2).contains(&kv(m)))).active_low());
    eqs.push(Eq::table_pos("UWR_n", Mode::Reg, &ins, move |m| Some(busy(m) && !rw(m) && (0..=1).contains(&kv(m)))).active_low());
    eqs.push(Eq::table_pos("LATCH", Mode::Comb, &ins, move |m| Some(busy(m) && rw(m) && kv(m) == 3)));
    // Result.
    for i in 0..8 {
        eqs.push(Eq::sop(&n("RDATA", i), Mode::Reg, vec![vec![l("LATCH"), l(&n("UD", i))], vec![nl_("LATCH"), l(&n("RDATA", i))]]));
    }
    // Bus read drivers: RDATA at offset 0, STATUS at offset 4, on lane 0,
    // enabled from the bus read strobe (a gate level after MIO: the data
    // memory's outputs are off by then, docs/bus.md section 4).
    for i in 0..8 {
        let mut terms = vec![vec![l("SBRDD"), l(&n("RDATA", i))]];
        if i == 0 {
            terms.push(vec![l("SBRDS"), l("BUSY")]);
        }
        eqs.push(Eq::sop(&n("DQ", i), Mode::Comb, terms).with_oe(vec![nl_("BRD_n"), l("SLOT0")]));
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
        // The bus write qualifier: an I/O store in MEM, registered, so a
        // memory card can shape its write pulse from it and a delay-line
        // tap the way the board does (docs/bus.md section 1).
        Eq::sop("BWEQ", Mode::Reg, vec![vec![l(&n("FA", 31)), l("XMW")]]),
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
    // Load data by lane, from the one-hot selects (mem_select_block):
    // bits 7:0 from any DQ byte, bits 15:8 from their own lane or the
    // high half or the sign, bits 31:16 from their own lane or the
    // sign.  Bits 8..31 from a byte device are zero (the selects see BB8).
    let mut eqs: Vec<Eq> = (0..32)
        .map(|i| {
            let mut terms = vec![vec![l("MNR"), l(&n("MR", i))]];
            let sels: Vec<&str> = if i < 8 {
                terms.push(vec![l("ML0"), l(&n("DQ", i))]);
                terms.push(vec![l("ML1"), l(&n("DQ", i + 8))]);
                terms.push(vec![l("ML2"), l(&n("DQ", i + 16))]);
                terms.push(vec![l("ML3"), l(&n("DQ", i + 24))]);
                vec!["MNR", "ML0", "ML1", "ML2", "ML3"]
            } else if i < 16 {
                terms.push(vec![l("MW8"), l(&n("DQ", i))]);
                terms.push(vec![l("MH1"), l(&n("DQ", i + 16))]);
                terms.push(vec![l("MSB"), l("LSGN")]);
                vec!["MNR", "MW8", "MH1", "MSB"]
            } else {
                terms.push(vec![l("MW16"), l(&n("DQ", i))]);
                terms.push(vec![l("MSN"), l("LSGN")]);
                vec!["MNR", "MW16", "MSN"]
            };
            let _ = &sels;
            Eq::sop(&n("WD", i), Mode::Reg, terms)
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
    eqs
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
    let h = |eqs: Vec<Eq>| with_hold(eqs, "HOLD");
    v.extend(pack("pc", clk, Some("RESET_PC"), h(pc_block())));
    v.extend(pack("inc", None, None, inc_block()));
    v.extend(pack("ifid", clk, rst, h(ifid_block())));
    v.extend(pack("dec", None, None, dec_block()));
    // ID/EX: a bubble on HOLD (the instruction stays in ID).
    v.extend(pack("ctl", clk, rst, with_bubble(ctrl_block(), "HOLD")));
    v.extend(pack("steer", None, None, steer_block()));
    // No async reset: under reset the write flags are 0, so these settle
    // to "no hit" from their inputs after one clock.  An async reset would
    // leave them at "hit" (they are written active-low), which selects the
    // EX/MEM result, undriven during the boot copy, into the ALU.
    v.extend(pack("fwdc", clk, None, fwdctl_block()));
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
    // EX/MEM control feeds the tap-clocked write copies (wc1 reads MDEST
    // and MRW inside its 15 to 21 ns window).  An asynchronous clear
    // would land there when RESET is asserted mid-run (button press), so
    // this block resets synchronously: its outputs only ever move at CLK.
    v.extend(pack("mctl", clk, None, with_bubble(exmem_ctrl_block(), "RESET")));
    v.extend(pack("stl", clk, rst, stall_block()));
    v.extend(pack("rsync", clk, None, rsync_block()));
    v.extend(pack("bseq", clk, None, bseq_block()));
    v.extend(pack("badr", None, None, badr_block()));
    v.extend(pack("bdat", None, None, bdat_block()));
    v.extend(pack("cmp", None, None, cmp_block()));
    v.extend(pack("nxt", None, None, taken_block()));
    v.extend(pack("wb", clk, rst, memwb_block()));
    v.extend(pack("macc", clk, None, mem_access_block()));
    v.extend(pack("msel", None, None, mem_select_block()));
    v.extend(pack("sf", None, None, sf_block()));
    v.extend(pack("sbr", clk, None, with_bubble(bridge_block(), "RESET")));
    v.extend(pack("wc1", Some("T3"), None, wcopy1_block()));
    v.extend(pack("wc2", Some("T1"), None, wcopy2_block()));
    v
}

/// Design parameters the *wiring* depends on: what differs between the
/// board and a test-sized variant of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Params {
    /// Boot region size: 2^k words (13 on the board).
    pub code_words_log2: u32,
    /// Data regions copied at boot (NPH + 1; 32 on the board).
    pub data_regions: u32,
    /// SKIP wired high: no boot copy, memories preloaded (test boards).
    pub skip: bool,
    /// Tap for the write-enable gate: (DS1100 total, tap index 0..5).  A
    /// total other than [`DELAY_LINE_TOTAL`] adds a second delay line.
    pub gate_tap: (u32, usize),
}

impl Params {
    /// The board as built.
    pub fn board() -> Params {
        Params { code_words_log2: 13, data_regions: 32, skip: false, gate_tap: (40, 0) }
    }
    fn from_build(opt: &Build) -> Params {
        let (code_words_log2, data_regions, skip) = match &opt.boot {
            Boot::Preload => (13, 0, true),
            Boot::Copy { code_words_log2, data_regions, .. } => (*code_words_log2, *data_regions, false),
        };
        Params { code_words_log2, data_regions, skip, gate_tap: opt.gate_tap }
    }
}

/// The chip map's columns.
pub fn layout() -> Vec<Column> {
    let col = |title: &str, width: u32, blocks: &[&str], reg: bool| Column { title: title.into(), width, blocks: blocks.iter().map(|b| b.to_string()).collect(), reg };
    vec![
        col("IF", 300, &["PC", "PC+4", "Instruction memory", "Boot ROM", "Boot sequencer", "Boot address", "Reset supervisor", "Reset sync"], false),
        col("", 240, &["IF/ID"], true),
        col("ID", 330, &["Decode", "Steer", "Narrow-store hold", "Register file", "Branch target adder"], false),
        col("", 240, &["ID/EX control", "Forwarding control", "ID/EX A", "ID/EX B", "ID/EX store data", "ID/EX branch target"], true),
        col("EX", 330, &["Forward A", "Forward B", "ALU slices + carries", "Shifter", "Compare", "Next PC"], false),
        col("", 240, &["EX/MEM result (ALU last level)", "EX/MEM store data", "EX/MEM control", "Access size", "Stall"], true),
        col("MEM", 240, &["Data memory", "Load lane select", "Boot data", "Write gate", "Delay line", "Serial bridge", "UART", "Serial port"], false),
        col("", 240, &["MEM/WB", "Write copies"], true),
        col("WB", 170, &[], false),
    ]
}

fn meta(part: &str, package: &str, name: &str, role: Option<String>, model: Model) -> ChipMeta {
    let (block, stage) = block_of(name);
    ChipMeta { part: part.into(), package: package.into(), block: block.into(), stage: stage.into(), role, model, lcsc: None }
}

/// The board file for these parameters.
pub fn board(p: &Params) -> Board {
    let nl = build_netlist(p);
    let mut params = BTreeMap::new();
    params.insert("code_words_log2".to_string(), serde_json::json!(p.code_words_log2));
    params.insert("data_regions".to_string(), serde_json::json!(p.data_regions));
    params.insert("skip".to_string(), serde_json::json!(p.skip));
    params.insert("gate_tap".to_string(), serde_json::json!([p.gate_tap.0, p.gate_tap.1]));
    nl.export(
        "crag",
        "MIPS-I five-stage pipeline: ATF22V10C logic, CY7C131 register file, IS61C64AL instruction memory, CY7C1041GN data memory, SST39SF040 boot ROMs, DS1100 delay lines, MAX811L reset, TL16C550 UART.",
        34.0,
        params,
        layout(),
    )
}

/// Every chip and every connection, with board metadata; no stimulus.
pub fn build_netlist(p: &Params) -> Netlist {
    let mut nl = Netlist::new();
    let specs = gal_specs();
    for spec in specs.iter() {
        let (id, pins) = crate::galpack::instantiate(&mut nl, spec);
        let model = Model::Gal { clk: spec.clk.clone(), ar: spec.ar.clone(), eqs: spec.eqs.clone(), pins };
        nl.set_meta(id, meta("ATF22V10C-7PX", "DIP-24", &spec.name, None, model));
    }
    let clk = nl.net("CLK");
    nl.set_net_role(clk, "clk");
    let reset = nl.net("RESET");
    nl.set_net_role(reset, "reset");
    let gnd = nl.net("GND");
    let vcc = nl.net("VCC");
    nl.tie(gnd, Level::L);
    nl.tie(vcc, Level::H);
    // Delay line on CLK: its taps clock the write-copy registers.
    let dl = nl.add_chip("dl0", Ds1100::new(DELAY_LINE_TOTAL, Grade::Commercial));
    nl.set_meta(dl, meta("DS1100Z-30", "SOIC-8", "dl0", Some("dl:0".into()), Model::Ds1100 { total_ns: DELAY_LINE_TOTAL }));
    nl.connect(clk, dl, DS1100_IN);
    for k in 0..5 {
        let net = nl.net(&format!("T{}", k + 1));
        nl.connect(net, dl, ds1100_tap_pin(k));
    }
    // The data-memory write-enable gate: NAND of a tap and the store
    // flag.  A second delay line if the tap is from another part.
    let (gt_total, gt_k) = p.gate_tap;
    let tap_net = if gt_total == DELAY_LINE_TOTAL {
        nl.net(&format!("T{}", gt_k + 1))
    } else {
        let dl1 = nl.add_chip("dl1", Ds1100::new(gt_total, Grade::Commercial));
        nl.set_meta(dl1, meta(&format!("DS1100U-{gt_total}"), "uSOP-8", "dl1", Some("dl:1".into()), Model::Ds1100 { total_ns: gt_total }));
        nl.connect(clk, dl1, DS1100_IN);
        for k in 0..5 {
            let net = nl.net(&format!("U{}", k + 1));
            nl.connect(net, dl1, ds1100_tap_pin(k));
        }
        nl.net(&format!("U{}", gt_k + 1))
    };
    // Reset supervisor; MR# is the reset button (pulled up in the part).
    let sup = nl.add_chip("rst0", ResetSupervisor::new(0, 0));
    nl.set_meta(sup, meta("MAX811LEUS+T", "SOT-143", "rst0", Some("supervisor".into()), Model::Supervisor));
    let rst_n = nl.net("RST_n");
    nl.connect(rst_n, sup, 2);
    let mr_n = nl.net("MR_n");
    nl.connect(mr_n, sup, 3);
    nl.pull(mr_n, Level::H);
    nl.set_net_role(mr_n, "reset_button");
    let gate = nl.add_chip("gate0", FastGate::new(500, 5500));
    nl.set_meta(gate, meta("74LVC1G00QSE-7", "SOT-353", "gate0", Some("gate".into()), Model::Gate));
    nl.connect(tap_net, gate, 1);
    let mmwb = nl.net("MMWB");
    nl.connect(mmwb, gate, 2);
    let wen = nl.net("WEN");
    nl.connect(wen, gate, 4);

    // UART (TL16C550D) on the data bus, byte lane 0; register select
    // from the bus-wait sequencer's held address; strobes and chip
    // select from it too; master reset from RESET (active high).
    // ADS# low (no address latch), RD2 / WR2 low, CS0 / CS1 high.
    // Modem loop-back: RTS# -> CTS#, DTR# -> DSR# + DCD#, RI# high;
    // BAUDOUT -> RCLK.
    {
        // The bus byte-device flag: driven by whichever device is
        // selected, pulled down.
        let bb8 = nl.net("BB8");
        nl.pull(bb8, Level::L);
        nl.set_net_role(bb8, "bus:bb8");
        // The UART (TL16C550D) behind the bridge: data on the private bus
        // UD, address from the bridge's latch, chip select CS0 = BUSY,
        // strobes from the bridge, master reset from RESET.  CS1 high,
        // CS2# low, ADS# low, RD2 / WR2 low.  Modem lines: RTS# and CTS#
        // through the transceiver's second pair (hardware flow control,
        // auto mode in the chip); DTR# -> DSR# + DCD#, RI# high;
        // BAUDOUT -> RCLK.
        let c = nl.add_chip("uart0", Uart16550::new(BusTiming::tl16c550c(), UART_XIN_HZ));
        nl.set_meta(c, meta("TL16C550DPTR", "LQFP-48", "uart0", Some("uart".into()), Model::Uart { xin_hz: UART_XIN_HZ }));
        for i in 0..8u8 {
            let net = nl.net(&n("UD", i as usize));
            nl.connect(net, c, uart_pin_of(UartPin::D(i)));
        }
        for i in 0..3u8 {
            let net = nl.net(&n("SBA", i as usize));
            nl.connect(net, c, uart_pin_of(UartPin::A(i)));
        }
        for (net, p) in [("BUSY", UartPin::Cs0), ("URD_n", UartPin::Rd1N), ("UWR_n", UartPin::Wr1N), ("RESET", UartPin::Mr), ("SIN", UartPin::Sin), ("SOUT", UartPin::Sout)] {
            let net = nl.net(net);
            nl.connect(net, c, uart_pin_of(p));
        }
        nl.connect(vcc, c, uart_pin_of(UartPin::Cs1));
        for p in [UartPin::Cs2N, UartPin::AdsN, UartPin::Rd2, UartPin::Wr2] {
            nl.connect(gnd, c, uart_pin_of(p));
        }
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
        let xin = nl.net("XIN");
        let xout = nl.net("XOUT");
        nl.connect(xin, c, uart_pin_of(UartPin::Xin));
        nl.connect(xout, c, uart_pin_of(UartPin::Xout));
        // Crystal and load capacitors.
        let x = nl.add_chip("x1", Passive::new(vec![(1, "1".into()), (2, "2".into())]));
        nl.set_meta(x, meta("X3225147456MOB4SI", "3225", "x1", None, Model::Passive));
        nl.connect(xin, x, 1);
        nl.connect(xout, x, 2);
        for (name, net) in [("c5", xin), ("c6", xout)] {
            let cap = nl.add_chip(name, Passive::new(vec![(1, "1".into()), (2, "2".into())]));
            nl.set_meta(cap, meta("18pF 0603 C0G", "0603", name, None, Model::Passive));
            nl.connect(net, cap, 1);
            nl.connect(gnd, cap, 2);
        }
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

    let k = p.code_words_log2;
    let nph = p.data_regions.saturating_sub(1);
    let skip = p.skip;

    // Instruction memory: 4 lanes, address = PC[14:2].
    for lane in 0..4 {
        let c = nl.add_chip(&format!("imem{lane}"), As7c164a::with_timing(as7c164a::Timing::is61c64al_10()));
        nl.set_meta(c, meta("IS61C64AL-10TLI", "TSOP-28", &format!("imem{lane}"), Some(format!("imem:{lane}")), Model::Sram8k { timing: "IS61C64AL-10".into() }));
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
    // CODE on A18; the rest grounded.
    for lane in 0..4 {
        let c = nl.add_chip(&format!("rom{lane}"), Rom::sst39sf040_70());
        nl.set_meta(c, meta("SST39SF040-70-4C-PHE", "DIP-32", &format!("rom{lane}"), Some(format!("rom:{lane}")), Model::Rom { tacc_ns: 70, toe_ns: 35, tdf_ns: 25 }));
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
    // Data memory: two 256K x 16 chips (low / high half-word), both byte
    // enables on (word access only for now), deselected during boot code
    // and I/O (DMEN_n), output-enabled except around a store (OEN);
    // address MR[19:2]; WE# from the gate during the store's MEM cycle.
    for half in 0..2 {
        let c = nl.add_chip(&format!("dmem{half}"), Sram16::new(as7c164a::Timing::cy7c1041g_10()));
        nl.set_meta(c, meta("CY7C1041GN-10ZSXI", "TSOP-44", &format!("dmem{half}"), Some(format!("dmem:{half}")), Model::Sram16 { timing: "CY7C1041G-10".into() }));
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
        // Byte enables: lane 2*half (low byte) and 2*half + 1 (high byte).
        let ble = nl.net(&format!("MBE{}_n", 2 * half));
        nl.connect(ble, c, sram16_pin_of(Sram16Pin::BleN));
        let bhe = nl.net(&format!("MBE{}_n", 2 * half + 1));
        nl.connect(bhe, c, sram16_pin_of(Sram16Pin::BheN));
        let oen = nl.net("OEN");
        nl.connect(oen, c, sram16_pin_of(Sram16Pin::OeN));
        nl.connect(wen, c, sram16_pin_of(Sram16Pin::WeN));
    }
    // Register file: bank 0 reads rs (IR25..21) -> RA, bank 1 reads rt
    // (IR20..16) -> RB; write port: address WDESTC (delayed copy), data
    // WD, CE = CLK (write in the low half), R/W = WREGC_n.
    for bank in 0..2 {
        for lane in 0..4 {
            let name = format!("rf{bank}{lane}");
            let c = nl.add_chip(&name, Cy7c131::new());
            nl.set_meta(c, meta("CY7C131-15JXC", "PLCC-52", &name, Some(format!("rf:{bank}{lane}")), Model::DualPort1k { timing: "CY7C131-15".into() }));
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
            for (p, pname) in [
                (SramPin::Busy(Port::Left), "BUSYL"),
                (SramPin::Busy(Port::Right), "BUSYR"),
                (SramPin::Int(Port::Left), "INTL"),
                (SramPin::Int(Port::Right), "INTR"),
            ] {
                let net = nl.net(&format!("{pname}_{bank}{lane}"));
                nl.pull(net, Level::H);
                nl.connect(net, c, sram_pin_of(p));
            }
        }
    }
    nl
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
    /// Physical fuzz (see docs/fuzz.md).  0 = off: ideal wires, a 50 %
    /// clock, zeroed memories.  Otherwise the seed for: a propagation
    /// delay on every chip pin drawn from 0..`pin_delay_ns`, a clock duty
    /// cycle drawn from `clock_duty` and a jitter of up to
    /// `clock_jitter_ns` on every edge, and random contents in every
    /// memory and register (mirror them into the reference with
    /// [`fuzz_image`]).
    pub fuzz_seed: u64,
    pub pin_delay_ns: f64,
    pub clock_duty: (f64, f64),
    pub clock_jitter_ns: f64,
    /// Extra propagation delay on particular pins, (chip, pin, ns), on
    /// top of the fuzz's.  The slack analysis (`slack`) uses it one bus
    /// at a time, on the pins that listen from an earlier stage.
    pub pin_delays: Vec<(String, usize, f64)>,
}

/// A small deterministic generator for the fuzz.
pub struct Lcg(pub u64);
impl Lcg {
    pub fn next(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 33) as u32
    }
    /// Uniform in [0, 1) (`next` yields 31 bits).
    pub fn unit(&mut self) -> f64 {
        self.next() as f64 / 2147483648.0
    }
}

/// The random power-up contents the fuzz gives the machine, for the
/// reference simulator: data memory words and the registers (r0 is
/// zeroed by reset).
pub struct FuzzImage {
    pub dmem: Vec<u32>,
    pub regs: [u32; 32],
}

pub fn fuzz_image(seed: u64) -> FuzzImage {
    let mut r = Lcg(seed ^ 0x5eed_0000_0000_0000);
    let dmem = (0..crate::iss::DMEM_WORDS).map(|_| r.next()).collect();
    let mut regs = [0u32; 32];
    for x in regs.iter_mut().skip(1) {
        *x = r.next();
    }
    FuzzImage { dmem, regs }
}

/// The board's UART crystal: 14.7456 MHz (3225 SMD, 12 pF), divisor 8
/// for 115200 baud.
pub const UART_XIN_HZ: f64 = 14_745_600.0;

impl Default for Build {
    fn default() -> Build {
        Build { grade: Grade::Commercial, dmem: as7c164a::Timing::cy7c1041g_10(), gate_tap: (40, 0), gate_tpd: (500, 5500), reset_phase_ns: 11.0, boot: Boot::Preload, uart_xin_hz: UART_XIN_HZ, uart_rx: Vec::new(), fuzz_seed: 0, pin_delay_ns: 0.0, clock_duty: (0.5, 0.5), clock_jitter_ns: 0.0, pin_delays: Vec::new() }
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
    /// Fuzz: (generator, duty range, jitter ns).
    clock_fuzz: Option<(Lcg, (f64, f64), f64)>,
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
        let params = Params::from_build(&opt);
        let board = board(&params);
        Cpu::from_board(&board, program, period_ns, &opt)
    }

    /// Instantiate a board file, load the program and images, and run the
    /// power-on reset (and the boot copy, if the board does one).
    pub fn from_board(board: &Board, program: &[u32], period_ns: f64, opt: &Build) -> Cpu {
        let period = (period_ns * NS as f64).round() as Time;
        let load = Load {
            grade: opt.grade,
            gate_tpd: opt.gate_tpd,
            reset_release: (3 + RESET_CYCLES as Time) * period + Self::ns(opt.reset_phase_ns),
            mr_timeout: Self::ns(MR_TIMEOUT_NS),
            uart_xin_hz: Some(opt.uart_xin_hz),
            dmem_timing: Some(opt.dmem),
        };
        let mut nl = board.instantiate(&load);
        let k = board.param_u64("code_words_log2").expect("code_words_log2") as u32;
        let words = 1usize << k;
        let skip = board.param_bool("skip").unwrap_or(false);
        let data_regions = board.param_u64("data_regions").unwrap_or(0) as u32;
        let data_image: Vec<u32> = match &opt.boot {
            Boot::Copy { data, .. } => data.clone(),
            Boot::Preload => Vec::new(),
        };
        // Instruction memory: preloaded, or left unknown for the copier.
        let mut imem: Vec<(String, usize)> = nl.chips_with_role("imem:").into_iter().map(|(id, r)| (r, id)).collect();
        imem.sort();
        let imem: Vec<usize> = imem.into_iter().map(|(_, id)| id).collect();
        for (lane, &id) in imem.iter().enumerate() {
            let chip = nl.chip_mut::<As7c164a>(id);
            if skip {
                for (w, &word) in program.iter().enumerate() {
                    chip.preload(w as u32, (word >> (8 * lane)) as u8);
                }
                for w in program.len()..8192 {
                    chip.preload(w as u32, 0);
                }
            }
        }
        // Boot ROM images: code at A18 = 1, data region p at p << k.
        let mut roms: Vec<(String, usize)> = nl.chips_with_role("rom:").into_iter().map(|(id, r)| (r, id)).collect();
        roms.sort();
        for (lane, (_, id)) in roms.into_iter().enumerate() {
            let rom = nl.chip_mut::<Rom>(id);
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
        }
        // Data memory and the register file start zeroed (r0 as garbage,
        // which reset must fix), or, under the fuzz, with the random
        // contents of `fuzz_image` (the reference gets the same).
        let image = (opt.fuzz_seed != 0).then(|| fuzz_image(opt.fuzz_seed));
        let mut dmem: Vec<(String, usize)> = nl.chips_with_role("dmem:").into_iter().map(|(id, r)| (r, id)).collect();
        dmem.sort();
        let dmem: Vec<usize> = dmem.into_iter().map(|(_, id)| id).collect();
        for (half, &id) in dmem.iter().enumerate() {
            let chip = nl.chip_mut::<Sram16>(id);
            let mut r = Lcg(opt.fuzz_seed ^ 0xd0e5);
            for w in 0..(1u32 << 18) {
                let word = match &image {
                    Some(im) => im.dmem.get(w as usize).copied().unwrap_or_else(|| r.next()),
                    None => 0,
                };
                chip.preload(w, (word >> (16 * half)) as u16);
            }
        }
        let mut rf: Vec<(String, usize)> = nl.chips_with_role("rf:").into_iter().map(|(id, r)| (r, id)).collect();
        rf.sort();
        let rf: Vec<usize> = rf.into_iter().map(|(_, id)| id).collect();
        for (i, &id) in rf.iter().enumerate() {
            let lane = i % 4;
            let chip = nl.chip_mut::<Cy7c131>(id);
            for r in 0..32 {
                let v = image.as_ref().map_or(0, |im| im.regs[r as usize]);
                chip.preload(r, (v >> (8 * lane)) as u8);
            }
            chip.preload(0, 0xA5);
        }
        if let Some(id) = nl.chips_with_role("uart").first().map(|(id, _)| *id) {
            nl.chip_mut::<Uart16550>(id).core.send(&opt.uart_rx);
        }
        let gal_count = board.chips.iter().filter(|c| matches!(c.model, Model::Gal { .. })).count();
        let mut sim = nl.build();
        if (opt.fuzz_seed != 0 && opt.pin_delay_ns > 0.0) || !opt.pin_delays.is_empty() {
            let mut r = Lcg(opt.fuzz_seed ^ 0xde1a);
            let max = if opt.fuzz_seed != 0 { opt.pin_delay_ns } else { 0.0 };
            let extra: HashMap<(&str, usize), Time> = opt.pin_delays.iter().map(|(c, p, d)| ((c.as_str(), *p), Self::ns(*d))).collect();
            sim.set_pin_delays(|chip, pin| Self::ns(r.unit() * max) + extra.get(&(chip, pin)).copied().unwrap_or(0));
        }
        let clk = sim.net_id("CLK");
        let mr_n = sim.net_id("MR_n");
        let pc = (2..=14).map(|i| sim.net_id(&n("PC", i))).collect();
        let mut cpu = Cpu {
            sim,
            period,
            cycles: 0,
            pc_trace: Vec::new(),
            clk,
            pc,
            imem,
            dmem,
            rf,
            gal_count,
            grade: opt.grade,
            reset_release: 0,
            mr_n,
            boot_budget: RESET_CYCLES + 8 + if skip { 0 } else { 5 * words * (1 + data_regions as usize) + 8 * (2 + data_regions as usize) },
            clock_fuzz: (opt.fuzz_seed != 0).then(|| (Lcg(opt.fuzz_seed ^ 0xc10c), opt.clock_duty, opt.clock_jitter_ns)),
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
        // The clock: ideal, or with the fuzz's duty cycle and jitter.
        let (rise, fall) = match &mut self.clock_fuzz {
            None => (base, base + self.period / 2),
            Some((r, duty, jitter)) => {
                let d = duty.0 + r.unit() * (duty.1 - duty.0);
                let j1 = Self::ns(r.unit() * *jitter);
                let j2 = Self::ns(r.unit() * *jitter);
                (base + j1, base + (self.period as f64 * d) as Time + j2)
            }
        };
        self.sim.schedule(rise, self.clk, Level::H);
        self.sim.schedule(fall, self.clk, Level::L);
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
        "sbr" => ("Serial bridge", "MEM"),
        "uart" => ("UART", "MEM"),
        "macc" => ("Access size", "EX/MEM"),
        "msel" => ("Load lane select", "MEM"),
        "sf" => ("Narrow-store hold", "ID"),
        "rsync" => ("Reset sync", "IF"),
        "bseq" => ("Boot sequencer", "IF"),
        "badr" => ("Boot address", "IF"),
        "bdat" => ("Boot data", "MEM"),
        "rom" => ("Boot ROM", "IF"),
        "rst" => ("Reset supervisor", "IF"),
        "gate" => ("Write gate", "MEM"),
        "x" | "c" | "xcvr" | "j" => ("Serial port", "MEM"),
        _ => ("?", "?"),
    }
}
