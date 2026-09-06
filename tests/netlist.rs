//! Netlist-level tests: chip models wired by pin.

use mips32::cy7c131::{Cy7c131, Port};
use mips32::gal22v10::{Config, Gal22v10, Lit, OlmcConfig, Term, pin_olmc};
use mips32::netlist::*;

const L: Level = Level::L;
const H: Level = Level::H;
const X: Level = Level::X;

fn ns(n: f64) -> Time {
    (n * NS as f64).round() as Time
}

/// pin23 = !pin2
fn inverter() -> Gal22v10 {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([Lit::npin(2)])]);
    Gal22v10::new(c)
}

#[test]
fn two_gals_in_series_and_vcd() {
    let mut nl = Netlist::new();
    let a = nl.add_chip("A", inverter());
    let b = nl.add_chip("B", inverter());
    let (n_in, n_mid, n_out) = (nl.net("in"), nl.net("mid"), nl.net("out"));
    nl.connect(n_in, a, 2);
    nl.connect(n_mid, a, 23);
    nl.connect(n_mid, b, 2);
    nl.connect(n_out, b, 23);
    let mut sim = nl.build();
    sim.schedule(0, n_in, L);
    sim.run_until(ns(20.0));
    assert_eq!(sim.value(n_mid), H);
    assert_eq!(sim.value(n_out), L);
    sim.schedule(ns(100.0), n_in, H);
    sim.run_until(ns(102.9));
    assert_eq!(sim.value(n_mid), H);
    sim.run_until(ns(103.0));
    assert_eq!(sim.value(n_mid), X);
    sim.run_until(ns(107.5));
    assert_eq!(sim.value(n_mid), L);
    assert_eq!(sim.value(n_out), X);
    sim.run_until(ns(115.0));
    assert_eq!(sim.value(n_out), H);
    assert_eq!(sim.history(n_out), vec![(0, X), (ns(15.0), L), (ns(106.0), X), (ns(115.0), H)]);
    assert!(sim.warnings().is_empty(), "{:?}", sim.warnings());
    let vcd = sim.vcd();
    assert!(vcd.contains("$var wire 1 n2 out $end"));
    assert!(vcd.contains("#115000\n1n2"));
}

#[test]
fn bus_conflict_and_pull_up() {
    let mut nl = Netlist::new();
    let a = nl.add_chip("A", inverter());
    let n_in = nl.net("in");
    let n_out = nl.net("out");
    let n_od = nl.net("open_drain");
    nl.connect(n_in, a, 2);
    nl.connect(n_out, a, 23);
    nl.pull(n_od, H);
    let mut sim = nl.build();
    sim.schedule(0, n_in, L);
    sim.run_until(ns(10.0));
    assert_eq!(sim.value(n_out), H);
    assert_eq!(sim.value(n_od), H); // nobody drives it: pull-up wins
    sim.schedule(ns(10.0), n_od, L);
    sim.run_until(ns(11.0));
    assert_eq!(sim.value(n_od), L);
    // Testbench fights the GAL output.
    sim.schedule(ns(20.0), n_out, L);
    sim.run_until(ns(21.0));
    assert_eq!(sim.value(n_out), X);
    assert!(sim.warnings().iter().any(|w| w.contains("out: drive conflict")), "{:?}", sim.warnings());
}

// ---------------------------------------------------------------------------
// The steer register file as a netlist: 8 x CY7C131 + 1 x ATF22V10C.
//
// GAL pins:  rd0..4 = 2..6   rs0..4 = 7..11   we = 13
//            rt0..4 = 14,15,16,22,23 (OLMCs 9,8,7,1,0 as inputs)
//            CE_R_A_n = 20 (OLMC 3)   CE_R_B_n = 19 (OLMC 4), both active low.
//   /CE_R_x = !we + OR_i (a_i ^ b_i)      -> 11 product terms
//   i.e. CE_R_x is high (read disabled) exactly when the source register is
//   the one being written this cycle.

fn steer_gal() -> Gal22v10 {
    let mut c = Config::empty();
    let rd: Vec<Lit> = (2..=6).map(Lit::pin).collect();
    let rs: Vec<Lit> = (7..=11).map(Lit::pin).collect();
    let rt: Vec<Lit> = [14u8, 15, 16, 22, 23].iter().map(|&p| c.out(pin_olmc(p))).collect();
    let we = Lit::pin(13);
    let neg = |l: Lit| Lit { neg: !l.neg, ..l };
    let steer = |a: &[Lit], b: &[Lit]| -> Vec<Term> {
        let mut terms = vec![Term::new([neg(we)])];
        for i in 0..5 {
            terms.push(Term::new([a[i], neg(b[i])]));
            terms.push(Term::new([neg(a[i]), b[i]]));
        }
        terms
    };
    c.olmc[3] = OlmcConfig::comb(steer(&rs, &rd)).active_low();
    c.olmc[4] = OlmcConfig::comb(steer(&rt, &rd)).active_low();
    c.validate().unwrap();
    Gal22v10::new(c)
}

struct RegFileNets {
    rs: Vec<NetId>,
    rt: Vec<NetId>,
    rd: Vec<NetId>,
    wdata: Vec<NetId>,
    rs_data: Vec<NetId>,
    rt_data: Vec<NetId>,
    we: NetId,
    ce_w: NetId,
}

fn build_regfile() -> (Sim, RegFileNets) {
    let mut nl = Netlist::new();
    let gnd = nl.net("GND");
    let vcc = nl.net("VCC");
    nl.tie(gnd, L);
    nl.tie(vcc, H);
    let rs = nl.bus("rs", 5);
    let rt = nl.bus("rt", 5);
    let rd = nl.bus("rd", 5);
    let wdata = nl.bus("wdata", 32);
    let rs_data = nl.bus("rs_data", 32);
    let rt_data = nl.bus("rt_data", 32);
    let we = nl.net("we");
    let ce_w = nl.net("CE_W_n");
    let ce_r = [nl.net("CE_R_A_n"), nl.net("CE_R_B_n")];

    // Steer GAL.
    let g = nl.add_chip("steer", steer_gal());
    for i in 0..5 {
        nl.connect(rd[i], g, 2 + i);
        nl.connect(rs[i], g, 7 + i);
        nl.connect(rt[i], g, [14, 15, 16, 22, 23][i]);
    }
    nl.connect(we, g, 13);
    nl.connect(ce_r[0], g, 20);
    nl.connect(ce_r[1], g, 19);

    // SRAMs: bank 0 reads rs, bank 1 reads rt; left = read, right = write.
    for (bank, &ce_r_bank) in ce_r.iter().enumerate() {
        for lane in 0..4 {
            let mut chip = Cy7c131::new();
            for r in 0..32 {
                chip.preload(r, 0);
            }
            let c = nl.add_chip(&format!("sram{bank}{lane}"), chip);
            let raddr = if bank == 0 { &rs } else { &rt };
            let rdata = if bank == 0 { &rs_data } else { &rt_data };
            for i in 0..5 {
                nl.connect(raddr[i], c, sram_pin_of(SramPin::A(Port::Left, i as u8)));
                nl.connect(rd[i], c, sram_pin_of(SramPin::A(Port::Right, i as u8)));
            }
            for i in 5..10 {
                nl.connect(gnd, c, sram_pin_of(SramPin::A(Port::Left, i)));
                nl.connect(gnd, c, sram_pin_of(SramPin::A(Port::Right, i)));
            }
            for i in 0..8 {
                nl.connect(rdata[8 * lane + i], c, sram_pin_of(SramPin::Io(Port::Left, i as u8)));
                nl.connect(wdata[8 * lane + i], c, sram_pin_of(SramPin::Io(Port::Right, i as u8)));
            }
            nl.connect(ce_r_bank, c, sram_pin_of(SramPin::Ce(Port::Left)));
            nl.connect(vcc, c, sram_pin_of(SramPin::Rw(Port::Left)));
            nl.connect(gnd, c, sram_pin_of(SramPin::Oe(Port::Left)));
            nl.connect(ce_w, c, sram_pin_of(SramPin::Ce(Port::Right)));
            nl.connect(gnd, c, sram_pin_of(SramPin::Rw(Port::Right)));
            nl.connect(vcc, c, sram_pin_of(SramPin::Oe(Port::Right)));
            for (p, name) in [
                (SramPin::Busy(Port::Left), "BUSY_L"),
                (SramPin::Busy(Port::Right), "BUSY_R"),
                (SramPin::Int(Port::Left), "INT_L"),
                (SramPin::Int(Port::Right), "INT_R"),
            ] {
                let n = nl.net(&format!("{name}_{bank}{lane}"));
                nl.pull(n, H);
                nl.connect(n, c, sram_pin_of(p));
            }
        }
    }
    let sim = nl.build();
    (sim, RegFileNets { rs, rt, rd, wdata, rs_data, rt_data, we, ce_w })
}

#[derive(Clone, Copy, Debug)]
struct Cycle {
    rs: u8,
    rt: u8,
    rd: u8,
    wdata: u32,
    we: bool,
}

fn random_program(seed: u64, n: usize) -> Vec<Cycle> {
    let mut x = seed | 1;
    let mut next = || {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        x
    };
    (0..n)
        .map(|_| {
            let r = next();
            let rd = (r >> 10 & 31) as u8;
            Cycle {
                rs: if r & 7 == 0 { rd } else { (r & 31) as u8 },
                rt: if r >> 3 & 7 == 0 { rd } else { (r >> 5 & 31) as u8 },
                rd,
                wdata: (next() & 0xFFFF_FFFF) as u32,
                we: r >> 15 & 3 != 0,
            }
        })
        .collect()
}

/// Run the program with a 34ns cycle.  Pipeline-register outputs land at
/// edge + 5.5 (worst-case GAL tCO).  CE_W pulses from `ce_w_fall` for 14ns
/// when writing.  Returns (sampled rs, sampled rt) at edge + 30.5 (3.5ns
/// setup for the capturing register).
fn run(sim: &mut Sim, n: &RegFileNets, cycles: &[Cycle], ce_w_fall: f64) -> Vec<(Option<u32>, Option<u32>)> {
    const T: f64 = 34.0;
    sim.schedule(0, n.we, L);
    sim.schedule(0, n.ce_w, H);
    sim.schedule_bus(0, &n.rs, 0);
    sim.schedule_bus(0, &n.rt, 0);
    sim.schedule_bus(0, &n.rd, 0);
    sim.schedule_bus(0, &n.wdata, 0);
    let mut out = Vec::new();
    for (k, c) in cycles.iter().enumerate() {
        let base = (k as f64 + 1.0) * T;
        let we = c.we && c.rd != 0;
        sim.schedule_bus(ns(base + 5.5), &n.rs, c.rs as u32);
        sim.schedule_bus(ns(base + 5.5), &n.rt, c.rt as u32);
        sim.schedule_bus(ns(base + 5.5), &n.rd, c.rd as u32);
        sim.schedule_bus(ns(base + 5.5), &n.wdata, c.wdata);
        sim.schedule(ns(base + 5.5), n.we, Level::from_bit(we));
        if we {
            sim.schedule(ns(base + ce_w_fall), n.ce_w, L);
            sim.schedule(ns(base + ce_w_fall + 14.0), n.ce_w, H);
        }
        sim.run_until(ns(base + T - 3.5));
        let fwd = |r: u8, sram: Option<u32>| if we && r == c.rd { Some(c.wdata) } else { sram };
        out.push((fwd(c.rs, sim.read_bus(&n.rs_data)), fwd(c.rt, sim.read_bus(&n.rt_data))));
    }
    out
}

fn reference(cycles: &[Cycle]) -> Vec<(u32, u32)> {
    let mut regs = [0u32; 32];
    cycles
        .iter()
        .map(|c| {
            if c.we && c.rd != 0 {
                regs[c.rd as usize] = c.wdata;
            }
            (regs[c.rs as usize], regs[c.rt as usize])
        })
        .collect()
}

#[test]
fn steer_regfile_netlist_matches_reference() {
    let cycles = random_program(0xC0FFEE, 400);
    let (mut sim, nets) = build_regfile();
    let got = run(&mut sim, &nets, &cycles, 16.0);
    let want = reference(&cycles);
    for (i, (g, w)) in got.iter().zip(&want).enumerate() {
        assert_eq!(*g, (Some(w.0), Some(w.1)), "cycle {i}: {:?}", cycles[i]);
    }
    assert!(sim.warnings().is_empty(), "{:#?}", sim.warnings());
    // Leave a waveform behind for inspection.
    std::fs::create_dir_all("target").ok();
    std::fs::write("target/regfile_steer.vcd", sim.vcd()).unwrap();
}

/// The one ordering the design depends on: CE_W must not fall until the
/// steer has settled (13.5ns).  Dropping it with the addresses (5.5) puts
/// both ports on the same register within tPS whenever rs == rd, and the
/// model refuses to say the write happened.  The hand-wired driver in
/// tests/regfile.rs could not see this because it moved CE_R itself.
#[test]
fn ce_w_falling_with_addresses_is_ambiguous() {
    let cycles = [
        Cycle { rs: 1, rt: 2, rd: 3, wdata: 0x11111111, we: true },
        Cycle { rs: 7, rt: 2, rd: 7, wdata: 0x77777777, we: true },
        Cycle { rs: 7, rt: 7, rd: 8, wdata: 0x88888888, we: true },
    ];
    let (mut sim, nets) = build_regfile();
    let got = run(&mut sim, &nets, &cycles, 5.5);
    let w = sim.warnings();
    assert!(w.iter().any(|s| s.contains("ArbitrationAmbiguous")), "{:#?}", w);
    assert_eq!(got[2].0, None, "{:?}", got); // r7 unknowable afterwards
    assert_eq!(got[0], (Some(0), Some(0))); // no conflict in cycle 0: fine

    // Falling right at the steer's settle time is still too early to be
    // provable (the steered CE_R is X until then); 14 is the first safe value.
    for (fall, ok) in [(13.0, false), (14.0, true)] {
        let (mut sim, nets) = build_regfile();
        run(&mut sim, &nets, &cycles, fall);
        assert_eq!(sim.warnings().is_empty(), ok, "fall={fall}: {:#?}", sim.warnings());
    }
}
