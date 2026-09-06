//! Datasheet tests for the ATF22V10C-7 (DIP) model.

use mips32::gal22v10::*;

const L: Level = Level::L;
const H: Level = Level::H;
const Z: Level = Level::Z;
const X: Level = Level::X;

fn ns(n: f64) -> Time {
    (n * NS as f64).round() as Time
}

struct Bench {
    g: Gal22v10,
    ext: [Level; 25],
}

impl Bench {
    fn new(cfg: Config) -> Self {
        Bench { g: Gal22v10::new(cfg), ext: [Z; 25] }
    }
    fn at(&mut self, t: f64, f: impl FnOnce(&mut [Level; 25])) {
        f(&mut self.ext);
        self.g.set_inputs(ns(t), self.ext);
    }
    fn pin(&self, p: u8, t: f64) -> Level {
        self.g.drive_pin_at(p, ns(t))
    }
    fn no_warnings(&self) {
        assert!(self.g.warnings().is_empty(), "warnings: {:#?}", self.g.warnings());
    }
    fn kinds(&self) -> Vec<WarningKind> {
        self.g.warnings().iter().map(|w| w.kind.clone()).collect()
    }
}

/// pin23 = pin2 & pin3  (combinatorial, active high)
fn and_gate() -> Config {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([Lit::pin(2), Lit::pin(3)])]);
    c
}

#[test]
fn combinatorial_tpd_window() {
    let mut b = Bench::new(and_gate());
    // Never-driven inputs: output unknown.
    assert_eq!(b.pin(23, 0.0), X);
    b.at(0.0, |e| {
        e[2] = L;
        e[3] = H;
    });
    assert_eq!(b.pin(23, 2.9), X);
    assert_eq!(b.pin(23, 7.5), L);
    // a rises at 100: X from 103 to 107.5, then H.
    b.at(100.0, |e| e[2] = H);
    assert_eq!(b.pin(23, 102.9), L);
    assert_eq!(b.pin(23, 103.0), X);
    assert_eq!(b.pin(23, 107.4), X);
    assert_eq!(b.pin(23, 107.5), H);
    // Two changes 2ns apart: settle is 7.5 after the LAST one.
    b.at(200.0, |e| e[2] = L);
    b.at(202.0, |e| e[2] = H);
    assert_eq!(b.pin(23, 207.5), X);
    assert_eq!(b.pin(23, 209.5), H);
    b.no_warnings();
}

/// An input change that does not alter the function value still opens an X
/// window (no hazard guarantee).
#[test]
fn combinatorial_unchanged_value_still_glitches() {
    let mut b = Bench::new(and_gate());
    b.at(0.0, |e| {
        e[2] = L;
        e[3] = L;
    });
    assert_eq!(b.pin(23, 50.0), L);
    b.at(50.0, |e| e[3] = H); // output stays L logically
    assert_eq!(b.pin(23, 53.0), X);
    assert_eq!(b.pin(23, 57.5), L);
}

#[test]
fn active_low_output() {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([Lit::pin(2), Lit::pin(3)])]).active_low();
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[2] = H;
        e[3] = H;
    });
    assert_eq!(b.pin(23, 10.0), L);
    b.at(10.0, |e| e[3] = L);
    assert_eq!(b.pin(23, 17.5), H);
}

/// Input keeper: releasing an input leaves its last level in place.
#[test]
fn input_keeper_holds_level() {
    let mut b = Bench::new(and_gate());
    b.at(0.0, |e| {
        e[2] = H;
        e[3] = H;
    });
    assert_eq!(b.pin(23, 10.0), H);
    b.at(10.0, |e| e[2] = Z);
    assert_eq!(b.pin(23, 30.0), H); // still H, no X window
    b.at(30.0, |e| e[2] = X);
    assert_eq!(b.pin(23, 40.0), X);
    b.no_warnings();
}

/// Two-level logic through pin feedback: pin23 = a & b, pin22 = pin23 & c.
#[test]
fn feedback_costs_two_tpd() {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([Lit::pin(2), Lit::pin(3)])]);
    let fb0 = c.out(0);
    c.olmc[1] = OlmcConfig::comb(vec![Term::new([fb0, Lit::pin(4)])]);
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[2] = L;
        e[3] = H;
        e[4] = H;
    });
    assert_eq!(b.pin(22, 20.0), L);
    b.at(100.0, |e| e[2] = H);
    assert_eq!(b.pin(23, 107.5), H);
    assert_eq!(b.pin(22, 105.9), L);
    assert_eq!(b.pin(22, 106.0), X); // 2 * tPD(min)
    assert_eq!(b.pin(22, 114.9), X);
    assert_eq!(b.pin(22, 115.0), H); // 2 * tPD(max)
    b.no_warnings();
}

/// pin23.d = pin2 (registered), clock on pin 1.
fn dff() -> Config {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::reg(vec![Term::new([Lit::pin(2)])]);
    c
}

#[test]
fn registered_tco_and_power_up() {
    let mut b = Bench::new(dff());
    // Power-up: Q = 0 -> pin low (active high) even before any clock.
    assert_eq!(b.pin(23, 0.0), L);
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = H;
    });
    b.at(20.0, |e| e[1] = H); // rising edge, D stable for 20 >= tS
    assert_eq!(b.pin(23, 21.9), L);
    assert_eq!(b.pin(23, 22.0), X); // tCO min 2
    assert_eq!(b.pin(23, 25.4), X);
    assert_eq!(b.pin(23, 25.5), H); // tCO max 5.5 (DIP)
    assert_eq!(b.g.q(0), H);
    // Data changes after the edge don't matter until the next edge.
    b.at(30.0, |e| e[2] = L);
    b.at(40.0, |e| e[1] = L);
    assert_eq!(b.pin(23, 60.0), H);
    b.at(60.0, |e| e[1] = H);
    assert_eq!(b.pin(23, 65.5), L);
    b.no_warnings();
}

#[test]
fn registered_output_does_not_glitch_when_unchanged() {
    let mut b = Bench::new(dff());
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = H;
    });
    b.at(20.0, |e| e[1] = H);
    assert_eq!(b.pin(23, 25.5), H);
    b.at(40.0, |e| e[1] = L);
    b.at(60.0, |e| e[1] = H); // D still H: no X window at all
    for t in [60.0, 62.0, 63.0, 65.0, 70.0] {
        assert_eq!(b.pin(23, t), H, "t={t}");
    }
    b.no_warnings();
}

#[test]
fn setup_violation_makes_q_unknown() {
    let mut b = Bench::new(dff());
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = L;
    });
    b.at(17.0, |e| e[2] = H); // 3ns before the edge < tS 3.5
    b.at(20.0, |e| e[1] = H);
    assert_eq!(b.g.q(0), X);
    assert_eq!(b.pin(23, 30.0), X);
    assert_eq!(b.kinds(), vec![WarningKind::Setup { olmc: 0, changed_at: ns(17.0) }]);

    // Exactly tS is fine.
    let mut b = Bench::new(dff());
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = L;
    });
    b.at(16.5, |e| e[2] = H);
    b.at(20.0, |e| e[1] = H);
    assert_eq!(b.g.q(0), H);
    b.no_warnings();
}

#[test]
fn data_change_in_same_snapshot_as_edge_is_a_violation() {
    let mut b = Bench::new(dff());
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = L;
    });
    b.at(20.0, |e| {
        e[1] = H;
        e[2] = H;
    });
    assert_eq!(b.g.q(0), X);
    assert!(matches!(b.kinds()[0], WarningKind::Setup { .. } | WarningKind::Hold { .. }));
}

#[test]
fn clock_width_violation() {
    let mut b = Bench::new(dff());
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = H;
    });
    b.at(20.0, |e| e[1] = H);
    b.at(22.0, |e| e[1] = L); // high for 2 < tW 3
    assert!(b.kinds().contains(&WarningKind::ClockWidth { width: ns(2.0) }));
    assert_eq!(b.g.q(0), X);
}

/// Toggle flip-flop through registered feedback: Q.d = !Q.
/// Internal fMAX = 1/(tS + tCF) = 1/6ns.
#[test]
fn registered_feedback_toggle_and_internal_fmax() {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::reg(vec![]); // placeholder to get polarity right
    let nq = c.nout(0); // logical !Q
    c.olmc[0] = OlmcConfig::reg(vec![Term::new([nq])]);
    let mut b = Bench::new(c);
    b.at(0.0, |e| e[1] = L);
    let mut t = 0.0;
    for i in 0..8 {
        t += 3.0;
        b.at(t, |e| e[1] = H); // period 6ns = tS + tCF exactly
        t += 3.0;
        b.at(t, |e| e[1] = L);
        let expect = if i % 2 == 0 { H } else { L };
        assert_eq!(b.g.q(0), expect, "edge {i}");
    }
    b.no_warnings();
    // Feedback timing: pin changes tCO after the edge, feedback tCF.
    let mut b = Bench::new({
        let mut c = Config::empty();
        c.olmc[0] = OlmcConfig::reg(vec![]);
        let nq = c.nout(0);
        c.olmc[0] = OlmcConfig::reg(vec![Term::new([nq])]);
        c
    });
    b.at(0.0, |e| e[1] = L);
    b.at(10.0, |e| e[1] = H);
    assert_eq!(b.pin(23, 15.5), H);
    b.at(20.0, |e| e[1] = L);
    b.at(25.5, |e| e[1] = H); // 5.5ns after previous edge... period 15.5 fine
    assert_eq!(b.g.q(0), L);
    b.no_warnings();
    // Registered -> combinatorial in the same chip: pin22 = Q0 (logical).
    // Output moves tCF(min)+tPD(min) after the edge, settles at tCF+tPD max.
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::reg(vec![]);
    let (q, nq) = (c.out(0), c.nout(0));
    c.olmc[0] = OlmcConfig::reg(vec![Term::new([nq])]);
    c.olmc[1] = OlmcConfig::comb(vec![Term::new([q])]);
    let mut b = Bench::new(c);
    b.at(0.0, |e| e[1] = L);
    assert_eq!(b.pin(22, 10.0), L);
    b.at(20.0, |e| e[1] = H);
    assert_eq!(b.pin(22, 24.9), L);
    assert_eq!(b.pin(22, 25.0), X); // 2 + 3
    assert_eq!(b.pin(22, 29.9), X);
    assert_eq!(b.pin(22, 30.0), H); // 2.5 + 7.5
    b.no_warnings();
}

/// Output enable term: pin23 = pin2 when pin3, else high-Z and usable as an
/// input.
#[test]
fn output_enable_and_bus_conflict() {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([Lit::pin(2)])]).with_oe(Oe::Term(Term::new([Lit::pin(3)])));
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[2] = H;
        e[3] = L;
    });
    assert_eq!(b.pin(23, 20.0), Z);
    b.at(20.0, |e| e[3] = H);
    assert_eq!(b.pin(23, 22.9), Z);
    assert_eq!(b.pin(23, 23.0), X); // tEA min 3
    assert_eq!(b.pin(23, 27.5), H); // tEA max 7.5
    b.at(40.0, |e| e[3] = L);
    assert_eq!(b.pin(23, 43.0), X);
    assert_eq!(b.pin(23, 47.5), Z); // tER max 7.5
    b.no_warnings();
    // External driver on the pin while enabled -> conflict.
    b.at(60.0, |e| e[3] = H);
    b.at(80.0, |e| e[23] = L);
    assert!(b.kinds().contains(&WarningKind::BusConflict { pin: 23 }));
}

/// An I/O pin used as an input (OLMC disabled) feeds other OLMCs through its
/// feedback column.
#[test]
fn io_pin_as_input() {
    let mut c = Config::empty();
    // OLMC 9 (pin 14) is an input; pin23 = pin14 & pin2.
    let in14 = c.out(9);
    c.olmc[0] = OlmcConfig::comb(vec![Term::new([in14, Lit::pin(2)])]);
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[2] = H;
        e[14] = L;
    });
    assert_eq!(b.pin(23, 10.0), L);
    b.at(10.0, |e| e[14] = H);
    assert_eq!(b.pin(23, 13.0), X);
    assert_eq!(b.pin(23, 17.5), H);
    b.no_warnings();
}

#[test]
fn async_reset() {
    let mut c = dff();
    c.ar = Some(Term::new([Lit::pin(3)]));
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = H;
        e[3] = L;
    });
    b.at(20.0, |e| e[1] = H);
    assert_eq!(b.pin(23, 30.0), H);
    b.at(40.0, |e| e[3] = H); // reset asserted
    assert_eq!(b.pin(23, 42.9), H);
    assert_eq!(b.pin(23, 43.0), X); // tAP min 3
    assert_eq!(b.pin(23, 50.0), L); // tAP max 10
    b.at(60.0, |e| e[3] = L); // width 20 >= tAW 7
    assert_eq!(b.pin(23, 80.0), L);
    b.no_warnings();
    // Clock edge too soon after release (< tAR 5 after the release settles).
    b.at(80.0, |e| e[1] = L);
    b.at(100.0, |e| e[3] = H);
    b.at(120.0, |e| e[3] = L); // AR settles low at 130
    b.at(132.0, |e| e[1] = H);
    assert_eq!(b.g.q(0), X);
    assert!(b.kinds().contains(&WarningKind::AsyncResetRecovery));
}

#[test]
fn async_reset_width_violation() {
    let mut c = dff();
    c.ar = Some(Term::new([Lit::pin(3)]));
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = H;
        e[3] = L;
    });
    b.at(20.0, |e| e[1] = H);
    b.at(40.0, |e| e[3] = H);
    b.at(45.0, |e| e[3] = L); // 5 < tAW 7
    b.g.advance(ns(60.0));
    assert!(b.kinds().contains(&WarningKind::AsyncResetWidth { width: ns(5.0) }));
    assert_eq!(b.g.q(0), X);
}

#[test]
fn sync_preset() {
    let mut c = dff();
    c.sp = Some(Term::new([Lit::pin(3)]));
    let mut b = Bench::new(c);
    b.at(0.0, |e| {
        e[1] = L;
        e[2] = L;
        e[3] = H;
    });
    assert_eq!(b.pin(23, 10.0), L);
    b.at(20.0, |e| e[1] = H); // SP high at the edge -> Q = 1 regardless of D
    assert_eq!(b.g.q(0), H);
    assert_eq!(b.pin(23, 25.5), H);
    b.at(30.0, |e| {
        e[1] = L;
        e[3] = L;
    });
    b.at(50.0, |e| e[1] = H); // SP released 20 >= tSPR: loads D = L
    assert_eq!(b.g.q(0), L);
    b.no_warnings();
    b.at(60.0, |e| e[1] = L);
    b.at(78.0, |e| e[3] = H); // 2ns before the edge < tSP
    b.at(80.0, |e| e[1] = H);
    assert_eq!(b.g.q(0), X);
    assert!(b.kinds().contains(&WarningKind::SyncPresetSetup));
}

#[test]
fn product_term_budget_is_enforced() {
    let mut c = Config::empty();
    c.olmc[0] = OlmcConfig::comb((0..9).map(|_| Term::new([Lit::pin(2)])).collect());
    assert!(c.validate().is_err());
    c.olmc[0] = OlmcConfig::comb((0..8).map(|_| Term::new([Lit::pin(2)])).collect());
    assert!(c.validate().is_ok());
}

// ---------------------------------------------------------------------------
// The register-file steer comparator, as it would actually be programmed.
//
//   rd[0..5]  pins 2..6      rs[0..5] pins 7..11     we pin 13
//   rt[0..5]  pins 14..18 (OLMC 9..5 as inputs)
//   CE_R_A_n  pin 20 (OLMC 3, 14 PTs)   = we & (rs == rd)     (active low: L = read enabled)
//   CE_R_B_n  pin 19 (OLMC 4, 16 PTs)   = we & (rt == rd)
//
// Equality needs 2^5 product terms, so the OLMC computes the complement:
//   /CE = !we + OR_i (a_i & !b_i + !a_i & b_i)   -> 11 terms, active-low output.

fn steer_config() -> Config {
    let mut c = Config::empty();
    let rd: Vec<Lit> = (2..=6).map(Lit::pin).collect();
    let rs: Vec<Lit> = (7..=11).map(Lit::pin).collect();
    let rt: Vec<Lit> = (14..=18).map(|p| c.out(pin_olmc(p))).collect();
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
    c
}

fn set_bits(e: &mut [Level; 25], pins: impl IntoIterator<Item = u8>, v: u8) {
    for (i, p) in pins.into_iter().enumerate() {
        e[p as usize] = Level::from_bit(v >> i & 1 == 1);
    }
}

#[test]
fn steer_comparator_function_and_timing() {
    let cfg = steer_config();
    cfg.validate().unwrap();
    let mut b = Bench::new(cfg);
    let mut x = 0x2545F491u32;
    let mut t = 0.0;
    for _ in 0..500 {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        let rd = (x & 31) as u8;
        let rs = if x >> 5 & 3 == 0 { rd } else { (x >> 8 & 31) as u8 };
        let rt = if x >> 13 & 3 == 0 { rd } else { (x >> 16 & 31) as u8 };
        let we = x >> 21 & 1 == 1;
        b.at(t, |e| {
            set_bits(e, 2..=6, rd);
            set_bits(e, 7..=11, rs);
            set_bits(e, 14..=18, rt);
            e[13] = Level::from_bit(we);
        });
        // Not guaranteed before tPD(max)...
        assert!(matches!(b.pin(20, t + 7.4), X | L | H));
        // ...and exactly what the steer logic wants after it.
        let want = |r: u8| if we && r == rd { H } else { L };
        assert_eq!(b.pin(20, t + 7.5), want(rs), "rs={rs} rd={rd} we={we}");
        assert_eq!(b.pin(19, t + 7.5), want(rt), "rt={rt} rd={rd} we={we}");
        t += 34.0;
    }
    b.no_warnings();
}
