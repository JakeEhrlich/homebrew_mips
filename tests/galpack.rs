//! Equation packing and instantiation, checked by simulating the result.

use mips32::galpack::*;
use mips32::netlist::*;

fn ns(n: f64) -> Time {
    (n * NS as f64).round() as Time
}

/// A 3-bit carry-select adder slice from truth tables: s^0, s^1 for both
/// carry-ins, plus group generate and propagate.  Packs into one chip.
fn adder_slice(prefix: &str) -> Vec<Eq> {
    let a: Vec<String> = (0..3).map(|i| format!("a{i}")).collect();
    let b: Vec<String> = (0..3).map(|i| format!("b{i}")).collect();
    let ins: Vec<&str> = a.iter().chain(b.iter()).map(String::as_str).collect();
    let sum = |m: u32, cin: u32| (m & 7) + (m >> 3 & 7) + cin;
    let mut eqs = Vec::new();
    for cin in 0..2u32 {
        for bit in 0..3 {
            eqs.push(Eq::table(&format!("{prefix}s{bit}_{cin}"), Mode::Comb, &ins, move |m| Some(sum(m, cin) >> bit & 1 == 1)));
        }
    }
    eqs.push(Eq::table(&format!("{prefix}G"), Mode::Comb, &ins, move |m| Some(sum(m, 0) >> 3 & 1 == 1)));
    eqs.push(Eq::table(&format!("{prefix}P"), Mode::Comb, &ins, move |m| Some(sum(m, 1) >> 3 & 1 == 1)));
    eqs
}

#[test]
fn adder_slice_packs_into_one_chip_and_adds() {
    let specs = pack("add", None, None, adder_slice("x"));
    assert_eq!(specs.len(), 1, "{specs:#?}");
    let mut nl = Netlist::new();
    instantiate_all(&mut nl, &specs);
    let mut sim = nl.build();
    let a: Vec<NetId> = (0..3).map(|i| sim.net_id(&format!("a{i}"))).collect();
    let b: Vec<NetId> = (0..3).map(|i| sim.net_id(&format!("b{i}"))).collect();
    let s0: Vec<NetId> = (0..3).map(|i| sim.net_id(&format!("xs{i}_0"))).collect();
    let s1: Vec<NetId> = (0..3).map(|i| sim.net_id(&format!("xs{i}_1"))).collect();
    let (g, p) = (sim.net_id("xG"), sim.net_id("xP"));
    let mut t = 0.0;
    for av in 0..8u32 {
        for bv in 0..8u32 {
            sim.schedule_bus(ns(t), &a, av);
            sim.schedule_bus(ns(t), &b, bv);
            sim.run_until(ns(t + 7.5));
            assert_eq!(sim.read_bus(&s0), Some((av + bv) & 7), "a={av} b={bv}");
            assert_eq!(sim.read_bus(&s1), Some((av + bv + 1) & 7), "a={av} b={bv}");
            assert_eq!(sim.value(g).bit(), Some((av + bv) >> 3 & 1 == 1));
            assert_eq!(sim.value(p).bit(), Some((av + bv + 1) >> 3 & 1 == 1));
            t += 10.0;
        }
    }
    assert!(sim.warnings().is_empty(), "{:?}", sim.warnings());
}

/// A 32-bit register with a 2:1 mux folded into every bit: 6 bits per chip
/// (13 inputs, one of them on a spare I/O pin), clocked, with reset.
#[test]
fn register_with_mux_packs_six_bits_per_chip() {
    let mut eqs = Vec::new();
    for i in 0..32 {
        eqs.push(Eq::sop(
            &format!("q{i}"),
            Mode::Reg,
            vec![vec![lit("sel"), lit(&format!("x{i}"))], vec![nlit("sel"), lit(&format!("y{i}"))]],
        ));
    }
    let specs = pack("r", Some("clk"), Some("rst"), eqs);
    assert_eq!(specs.len(), 6, "{}", specs.len()); // 6+6+6+6+6+2
    for s in &specs {
        s.fits().unwrap();
    }
    let mut nl = Netlist::new();
    instantiate_all(&mut nl, &specs);
    let mut sim = nl.build();
    let x: Vec<NetId> = (0..32).map(|i| sim.net_id(&format!("x{i}"))).collect();
    let y: Vec<NetId> = (0..32).map(|i| sim.net_id(&format!("y{i}"))).collect();
    let q: Vec<NetId> = (0..32).map(|i| sim.net_id(&format!("q{i}"))).collect();
    let (clk, sel, rst) = (sim.net_id("clk"), sim.net_id("sel"), sim.net_id("rst"));
    sim.schedule(0, clk, Level::L);
    sim.schedule(0, rst, Level::L);
    sim.schedule(0, sel, Level::H);
    sim.schedule_bus(0, &x, 0xDEAD_BEEF);
    sim.schedule_bus(0, &y, 0x1234_5678);
    sim.run_until(ns(20.0));
    assert_eq!(sim.read_bus(&q), Some(0)); // power-up
    sim.schedule(ns(20.0), clk, Level::H);
    sim.run_until(ns(25.5));
    assert_eq!(sim.read_bus(&q), Some(0xDEAD_BEEF));
    sim.schedule(ns(30.0), clk, Level::L);
    sim.schedule(ns(30.0), sel, Level::L);
    sim.schedule(ns(40.0), clk, Level::H);
    sim.run_until(ns(45.5));
    assert_eq!(sim.read_bus(&q), Some(0x1234_5678));
    // Async reset.
    sim.schedule(ns(50.0), rst, Level::H);
    sim.run_until(ns(60.0));
    assert_eq!(sim.read_bus(&q), Some(0));
    assert!(sim.warnings().is_empty(), "{:?}", sim.warnings());
}

#[test]
fn table_picks_cheaper_polarity() {
    // 5-bit equality needs 32 terms; inequality needs 10: table() must pick
    // the active-low form.
    let ins: Vec<String> = (0..10).map(|i| format!("i{i}")).collect();
    let ins: Vec<&str> = ins.iter().map(String::as_str).collect();
    let eq = Eq::table("eq", Mode::Comb, &ins, |m| Some((m & 31) == (m >> 5 & 31)));
    assert!(eq.active_low);
    assert_eq!(eq.terms.len(), 10);
}

#[test]
fn packing_refuses_what_cannot_fit() {
    // 17 product terms in one output: no macrocell has that.
    let terms: Vec<Vec<SLit>> = (0..17).map(|i| vec![lit(&format!("i{i}"))]).collect();
    let spec = GalSpec { name: "bad".into(), clk: None, ar: None, eqs: vec![Eq::sop("o", Mode::Comb, terms)] };
    assert!(spec.fits().is_err());
    // 20 inputs and 4 outputs is 24 pins.
    let terms: Vec<Vec<SLit>> = vec![(0..20).map(|i| lit(&format!("i{i}"))).collect()];
    let eqs: Vec<Eq> = (0..4).map(|k| Eq::sop(&format!("o{k}"), Mode::Comb, terms.clone())).collect();
    let spec = GalSpec { name: "bad".into(), clk: None, ar: None, eqs };
    assert!(spec.fits().is_err());
}
