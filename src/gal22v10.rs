//! Behavioural + timing model of the Atmel/Microchip ATF22V10C (-7, DIP).
//!
//! Source: Atmel datasheet 0735U (July 2010), saved in `docs/`.
//!
//! # Fabric (fuse-level view, 24-pin DIP)
//!
//! ```text
//!  1 CLK/IN   2..11 IN   12 GND   13 IN   14..23 I/O (OLMC 9..0)   24 VCC
//! ```
//! The AND array has 22 inputs (44 columns: true and complement of each).
//! Array input `i` is pin `i/2 + 1` for even `i < 21`, the feedback of OLMC
//! `i/2` for odd `i`, and pin 13 for `i = 21`.  OLMC `k` drives pin `23 - k`
//! and owns `PT_COUNT[k]` product terms plus one output-enable term.  Row 0 of
//! the array is the global asynchronous reset term, the last row the global
//! synchronous preset term.
//!
//! Each OLMC is either combinatorial (pin = SoP, optionally inverted; the
//! array sees the *pin*) or registered (D = SoP, pin = Q optionally inverted;
//! the array sees Q-bar).  Outputs are active low when `active_low` is set.
//!
//! # Timing model
//!
//! Internal discrete-event simulation.  Any change on an array input makes
//! every combinatorial output that structurally depends on it `X` from
//! `tPD(min)` until `tPD(max)` after the *latest* such change, then the
//! value computed from the settled inputs.  That is deliberately pessimistic
//! (it assumes any input change may glitch the output even if the function
//! value is unchanged) because the AND-OR array gives no hazard guarantee.
//! Registers sample on the rising edge of pin 1 and demand `tS` before and
//! `tH` after it on every array input in the D cone, else Q becomes `X`.
//! Register outputs never glitch: an unchanged Q stays stable.  Feedback from
//! a combinatorial OLMC is the pin, so a two-level function in one chip costs
//! two `tPD`s, exactly as the datasheet specifies ("input *or feedback* to
//! combinatorial output").  Registered feedback is `tCF` after the clock.
//!
//! Input pins have keepers ("latch feature"): an undriven input holds its
//! previous level.  A never-driven input is `X`.
//!
//! As with the SRAM model, any violation forces the affected state to `X`
//! and records a [`Warning`], so a clean run is one with no warnings.

use std::collections::BTreeMap;
use std::fmt;

pub use crate::cy7c131::{Level, Time, NS};

pub const ARRAY_INPUTS: usize = 22;
pub const OLMCS: usize = 10;
/// Product terms per OLMC (OLMC 0 = pin 23 ... OLMC 9 = pin 14).
pub const PT_COUNT: [usize; OLMCS] = [8, 10, 12, 14, 16, 16, 14, 12, 10, 8];

/// Pin driven by OLMC `k`.
pub const fn olmc_pin(k: usize) -> u8 {
    23 - k as u8
}
/// OLMC driving pin `p` (14..=23).
pub const fn pin_olmc(p: u8) -> usize {
    23 - p as usize
}

/// What feeds array input `i`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ArraySrc {
    Pin(u8),
    Fb(usize),
}
pub const fn array_src(i: usize) -> ArraySrc {
    if i == 21 {
        ArraySrc::Pin(13)
    } else if i.is_multiple_of(2) {
        ArraySrc::Pin((i / 2 + 1) as u8)
    } else {
        ArraySrc::Fb(i / 2)
    }
}
/// Array input carrying dedicated input pin `p` (1..=11, 13).
pub const fn pin_array_input(p: u8) -> usize {
    if p == 13 { 21 } else { (p as usize - 1) * 2 }
}
/// Array input carrying OLMC `k`'s feedback.
pub const fn fb_array_input(k: usize) -> usize {
    2 * k + 1
}

/// One literal in a product term: array input `input`, complemented if `neg`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Lit {
    pub input: usize,
    pub neg: bool,
}

impl Lit {
    /// Dedicated input pin, true polarity.
    pub const fn pin(p: u8) -> Lit {
        Lit { input: pin_array_input(p), neg: false }
    }
    pub const fn npin(p: u8) -> Lit {
        Lit { input: pin_array_input(p), neg: true }
    }
    /// Raw feedback column of OLMC `k` (pin level for combinatorial, Q-bar
    /// for registered).  See [`Config::out`] for the polarity-aware form.
    pub const fn fb(k: usize) -> Lit {
        Lit { input: fb_array_input(k), neg: false }
    }
    pub const fn nfb(k: usize) -> Lit {
        Lit { input: fb_array_input(k), neg: true }
    }
}

/// A product term.  No literals = always true.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct Term(pub Vec<Lit>);

impl Term {
    pub fn new(lits: impl IntoIterator<Item = Lit>) -> Term {
        Term(lits.into_iter().collect())
    }
    pub const fn always() -> Term {
        Term(Vec::new())
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum Oe {
    Always,
    Never,
    Term(Term),
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct OlmcConfig {
    pub registered: bool,
    pub active_low: bool,
    pub oe: Oe,
    /// Sum of products.  Empty = constant false.
    pub terms: Vec<Term>,
}

impl OlmcConfig {
    /// Unused OLMC: output disabled, so the pin is an input.
    pub fn input() -> OlmcConfig {
        OlmcConfig { registered: false, active_low: false, oe: Oe::Never, terms: vec![] }
    }
    pub fn comb(terms: Vec<Term>) -> OlmcConfig {
        OlmcConfig { registered: false, active_low: false, oe: Oe::Always, terms }
    }
    pub fn reg(terms: Vec<Term>) -> OlmcConfig {
        OlmcConfig { registered: true, active_low: false, oe: Oe::Always, terms }
    }
    pub fn active_low(mut self) -> OlmcConfig {
        self.active_low = true;
        self
    }
    pub fn with_oe(mut self, oe: Oe) -> OlmcConfig {
        self.oe = oe;
        self
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Config {
    pub olmc: [OlmcConfig; OLMCS],
    /// Global asynchronous reset term (`None` = never).
    pub ar: Option<Term>,
    /// Global synchronous preset term (`None` = never).
    pub sp: Option<Term>,
}

impl Config {
    /// All OLMCs as inputs, no AR/SP.
    pub fn empty() -> Config {
        Config { olmc: std::array::from_fn(|_| OlmcConfig::input()), ar: None, sp: None }
    }
    /// Literal for the *logical* value of OLMC `k`'s output (the level on
    /// its pin when enabled), taking mode and polarity into account.
    pub fn out(&self, k: usize) -> Lit {
        let c = &self.olmc[k];
        // Combinatorial: array sees the pin.  Registered: array sees Q-bar,
        // and pin = Q ^ active_low, so pin = !fb ^ active_low.
        let neg = c.registered && !c.active_low;
        Lit { input: fb_array_input(k), neg }
    }
    pub fn nout(&self, k: usize) -> Lit {
        let l = self.out(k);
        Lit { neg: !l.neg, ..l }
    }
    /// Check product-term budgets.
    pub fn validate(&self) -> Result<(), String> {
        for (k, c) in self.olmc.iter().enumerate() {
            if c.terms.len() > PT_COUNT[k] {
                return Err(format!("OLMC {k} (pin {}) uses {} product terms, has {}", olmc_pin(k), c.terms.len(), PT_COUNT[k]));
            }
            for t in c.terms.iter().chain(match &c.oe {
                Oe::Term(t) => Some(t),
                _ => None,
            }) {
                for l in &t.0 {
                    if l.input >= ARRAY_INPUTS {
                        return Err(format!("OLMC {k}: literal refers to array input {}", l.input));
                    }
                }
            }
        }
        Ok(())
    }
}

/// AC parameters, picoseconds.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    pub tpd_min: Time,
    pub tpd_max: Time,
    pub tco_min: Time,
    pub tco_max: Time,
    /// Clock to internal feedback.  The datasheet gives only a max; the min
    /// is assumed equal to tCO min.
    pub tcf_min: Time,
    pub tcf_max: Time,
    pub ts: Time,
    pub th: Time,
    pub tw: Time,
    pub tea_min: Time,
    pub tea_max: Time,
    pub ter_min: Time,
    pub ter_max: Time,
    pub tap_min: Time,
    pub tap_max: Time,
    pub taw: Time,
    pub tar: Time,
    pub tsp: Time,
    pub tspr: Time,
}

impl Timing {
    /// ATF22V10C-7PX (DIP): datasheet 0735U table 4.3 with note 2.
    pub const fn atf22v10c_7_dip() -> Timing {
        Timing {
            tpd_min: 3 * NS,
            tpd_max: 7_500,
            tco_min: 2 * NS,
            tco_max: 5_500,
            tcf_min: 2 * NS,
            tcf_max: 2_500,
            ts: 3_500,
            th: 0,
            tw: 3 * NS,
            tea_min: 3 * NS,
            tea_max: 7_500,
            ter_min: 3 * NS,
            ter_max: 7_500,
            tap_min: 3 * NS,
            tap_max: 10 * NS,
            taw: 7 * NS,
            tar: 5 * NS,
            tsp: 4_500,
            tspr: 5 * NS,
        }
    }
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Warning {
    pub time: Time,
    pub kind: WarningKind,
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub enum WarningKind {
    /// A D-cone input of this OLMC changed less than tS before the clock edge.
    Setup { olmc: usize, changed_at: Time },
    /// ... or within tH after it.
    Hold { olmc: usize, changed_at: Time },
    /// Clock high or low phase shorter than tW.
    ClockWidth { width: Time },
    /// Clock pin went X/Z.
    ClockUnknown,
    /// Register captured an unknown D (some cone input was X at the edge).
    CapturedX { olmc: usize },
    /// Asynchronous reset asserted for less than tAW.
    AsyncResetWidth { width: Time },
    /// Clock edge less than tAR after asynchronous reset release.
    AsyncResetRecovery,
    /// SP term changed less than tSP / tSPR before the edge.
    SyncPresetSetup,
    /// Chip and external driver both driving an I/O pin to different levels.
    BusConflict { pin: u8 },
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "t={:.3}ns {:?}", self.time as f64 / NS as f64, self.kind)
    }
}

// ---------------------------------------------------------------------------

fn and3(a: Level, b: Level) -> Level {
    use Level::*;
    match (a, b) {
        (L, _) | (_, L) => L,
        (H, H) => H,
        _ => X,
    }
}
fn or3(a: Level, b: Level) -> Level {
    use Level::*;
    match (a, b) {
        (H, _) | (_, H) => H,
        (L, L) => L,
        _ => X,
    }
}
fn not3(a: Level) -> Level {
    match a {
        Level::L => Level::H,
        Level::H => Level::L,
        _ => Level::X,
    }
}
fn xor_pol(a: Level, active_low: bool) -> Level {
    if active_low { not3(a) } else { a }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Ev {
    /// Combinatorial SoP output of OLMC may be changing.
    OutX(usize),
    /// Re-evaluate the SoP output; ignored if a later settle is pending.
    OutSettle(usize, Time),
    OeX(usize),
    OeSettle(usize, Time),
    ArX,
    ArSettle(Time),
    /// Registered output pin reaches its new value / feedback reaches it.
    RegPin(usize, Level),
    RegFb(usize, Level),
}

#[derive(Clone, Debug)]
struct Olmc {
    q: Level,
    /// SoP evaluation result for combinatorial mode (L/H/X).
    sop: Level,
    sop_settle: Time,
    /// Value driven to the pin before OE (comb: sop^pol; reg: q^pol).
    out: Level,
    oe: Level,
    oe_settle: Time,
    /// Most recent change time of any array input in the D/SoP cone.
    cone_changed: Time,
    /// Most recent change time of any array input in the OE cone.
    /// Feedback value presented to the array.
    fb: Level,
}

/// The chip.
#[derive(Clone, Debug)]
pub struct Gal22v10 {
    cfg: Config,
    tm: Timing,
    now: Time,
    /// External drive on each pin (index = pin number; 0, 12, 24 unused).
    ext: [Level; 25],
    /// Keeper: last known level on each pin.
    keeper: [Level; 25],
    /// Array input values and when they last changed.
    arr: [Level; ARRAY_INPUTS],
    arr_since: [Time; ARRAY_INPUTS],
    /// Value each array input had before its change at `arr_since` (used for
    /// the hazard analysis of simultaneous changes).
    arr_prev: [Level; ARRAY_INPUTS],
    /// Structural dependency: which OLMCs' SoP / OE, and AR / SP, use input i.
    sop_users: Vec<Vec<usize>>,
    oe_users: Vec<Vec<usize>>,
    ar_uses: Vec<bool>,
    sp_uses: Vec<bool>,
    olmc: Vec<Olmc>,
    clk: Level,
    clk_since: Time,
    last_edge: Option<Time>,
    ar: Level,
    ar_since: Time,
    /// AR term as the array sees it (undelayed) and when it last went H.
    ar_in: Level,
    ar_high_since: Time,
    ar_settle: Time,
    sp_changed: Time,
    queue: BTreeMap<(Time, u64), Ev>,
    seq: u64,
    warnings: Vec<Warning>,
}

impl Gal22v10 {
    pub fn new(cfg: Config) -> Self {
        Self::with_timing(cfg, Timing::atf22v10c_7_dip())
    }

    pub fn with_timing(cfg: Config, tm: Timing) -> Self {
        cfg.validate().expect("invalid GAL configuration");
        let mut sop_users = vec![Vec::new(); ARRAY_INPUTS];
        let mut oe_users = vec![Vec::new(); ARRAY_INPUTS];
        let mut ar_uses = vec![false; ARRAY_INPUTS];
        let mut sp_uses = vec![false; ARRAY_INPUTS];
        for (k, c) in cfg.olmc.iter().enumerate() {
            for t in &c.terms {
                for l in &t.0 {
                    if !sop_users[l.input].contains(&k) {
                        sop_users[l.input].push(k);
                    }
                }
            }
            if let Oe::Term(t) = &c.oe {
                for l in &t.0 {
                    if !oe_users[l.input].contains(&k) {
                        oe_users[l.input].push(k);
                    }
                }
            }
        }
        if let Some(t) = &cfg.ar {
            for l in &t.0 {
                ar_uses[l.input] = true;
            }
        }
        if let Some(t) = &cfg.sp {
            for l in &t.0 {
                sp_uses[l.input] = true;
            }
        }
        let olmc = cfg
            .olmc
            .iter()
            .map(|c| {
                // Power-up: registers low.  Combinatorial SoP unknown until
                // inputs are known.
                let q = Level::L;
                let sop = Level::X;
                let out = if c.registered { xor_pol(q, c.active_low) } else { Level::X };
                let oe = match c.oe {
                    Oe::Always => Level::H,
                    Oe::Never => Level::L,
                    Oe::Term(_) => Level::X,
                };
                Olmc { q, sop, sop_settle: 0, out, oe, oe_settle: 0, cone_changed: 0, fb: Level::X }
            })
            .collect();
        let mut g = Gal22v10 {
            cfg,
            tm,
            now: 0,
            ext: [Level::Z; 25],
            keeper: [Level::X; 25],
            arr: [Level::X; ARRAY_INPUTS],
            arr_since: [0; ARRAY_INPUTS],
            arr_prev: [Level::X; ARRAY_INPUTS],
            sop_users,
            oe_users,
            ar_uses,
            sp_uses,
            olmc,
            clk: Level::X,
            clk_since: 0,
            last_edge: None,
            ar: Level::L,
            ar_since: 0,
            ar_in: Level::L,
            ar_high_since: 0,
            ar_settle: 0,
            sp_changed: 0,
            queue: BTreeMap::new(),
            seq: 0,
            warnings: Vec::new(),
        };
        g.ar = if g.cfg.ar.is_some() { Level::X } else { Level::L };
        // Registered feedbacks are known at power-up (Q = 0 -> Q-bar = 1).
        for k in 0..OLMCS {
            if g.cfg.olmc[k].registered {
                g.olmc[k].fb = Level::H;
                g.arr[fb_array_input(k)] = Level::H;
            }
        }
        // Combinatorial outputs that depend only on registered feedback are
        // already known; everything else evaluates to X for now.
        for k in 0..OLMCS {
            if !g.cfg.olmc[k].registered {
                let v = g.eval_sop(&g.cfg.olmc[k].terms);
                g.olmc[k].sop = v;
                g.olmc[k].out = xor_pol(v, g.cfg.olmc[k].active_low);
            }
        }
        for k in 0..OLMCS {
            g.refresh_io_pin(k);
        }
        g
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }
    pub fn timing(&self) -> &Timing {
        &self.tm
    }
    pub fn now(&self) -> Time {
        self.now
    }
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }
    pub fn take_warnings(&mut self) -> Vec<Warning> {
        std::mem::take(&mut self.warnings)
    }
    fn warn(&mut self, kind: WarningKind) {
        self.warnings.push(Warning { time: self.now, kind });
    }

    fn schedule(&mut self, t: Time, ev: Ev) {
        self.seq += 1;
        self.queue.insert((t, self.seq), ev);
    }

    /// Earliest pending internal event strictly after `t`, if any.
    pub fn next_event(&self, t: Time) -> Option<Time> {
        self.queue.keys().map(|k| k.0).find(|&k| k > t)
    }

    /// Process internal events up to and including time `t`.
    pub fn advance(&mut self, t: Time) {
        assert!(t >= self.now, "advance() time went backwards ({t} < {})", self.now);
        while let Some((&(et, seq), &ev)) = self.queue.iter().next() {
            if et > t {
                break;
            }
            self.queue.remove(&(et, seq));
            self.now = et;
            self.process(ev);
        }
        self.now = t;
    }

    /// The level the chip drives on each pin (index = pin number).  `Z` for
    /// inputs and disabled outputs.
    pub fn drive(&self) -> [Level; 25] {
        let mut d = [Level::Z; 25];
        for k in 0..OLMCS {
            d[olmc_pin(k) as usize] = self.pin_drive(k);
        }
        d
    }
    /// Level the chip drives on pin `p` right now.
    pub fn drive_pin(&self, p: u8) -> Level {
        if (14..=23).contains(&p) { self.pin_drive(pin_olmc(p)) } else { Level::Z }
    }
    /// Chip-driven pin levels at a future time `t` assuming no further input
    /// changes (clones the model; use `advance` + `drive` in hot loops).
    pub fn drive_at(&self, t: Time) -> [Level; 25] {
        let mut g = self.clone();
        g.advance(t);
        g.drive()
    }
    pub fn drive_pin_at(&self, p: u8, t: Time) -> Level {
        self.drive_at(t)[p as usize]
    }
    /// Register contents (L/H/X) of OLMC `k`.
    pub fn q(&self, k: usize) -> Level {
        self.olmc[k].q
    }

    fn pin_drive(&self, k: usize) -> Level {
        let o = &self.olmc[k];
        match o.oe {
            Level::H => o.out,
            Level::L => Level::Z,
            _ => Level::X,
        }
    }

    /// Apply external pin levels at time `t`.  `ext[p]` is what the outside
    /// world drives on pin `p` (`Z` = nothing).  Pins 0, 12 and 24 ignored.
    pub fn set_inputs(&mut self, t: Time, ext: [Level; 25]) {
        self.advance(t);
        let old = self.ext;
        self.ext = ext;
        // Clock first, so a data change in the same snapshot counts as
        // "at the edge" for the setup check.
        let clk_new = self.pin_level(1);
        let clk_old = self.clk;
        if clk_new != clk_old {
            self.clock_transition(clk_old, clk_new);
        }
        for p in (1..=11).chain([13]) {
            if old[p as usize] != ext[p as usize] {
                let lvl = self.pin_level(p);
                self.set_array_input(pin_array_input(p), lvl);
            }
        }
        for k in 0..OLMCS {
            let p = olmc_pin(k) as usize;
            if old[p] != ext[p] {
                self.refresh_io_pin(k);
            }
        }
    }

    /// Level on a dedicated input pin as seen by the array (keeper applied).
    fn pin_level(&mut self, p: u8) -> Level {
        let e = self.ext[p as usize];
        match e {
            Level::L | Level::H => {
                self.keeper[p as usize] = e;
                e
            }
            Level::Z => self.keeper[p as usize],
            Level::X => Level::X,
        }
    }

    /// Resolve an I/O pin (chip drive vs external) and update the
    /// combinatorial feedback if applicable.
    fn refresh_io_pin(&mut self, k: usize) {
        let p = olmc_pin(k);
        let d = self.pin_drive(k);
        let e = self.ext[p as usize];
        let lvl = match (d, e) {
            (Level::Z, Level::Z) => self.keeper[p as usize],
            (Level::Z, e) => e,
            (d, Level::Z) => d,
            (d, e) if d == e => d,
            _ => {
                self.warn(WarningKind::BusConflict { pin: p });
                Level::X
            }
        };
        if matches!(lvl, Level::L | Level::H) {
            self.keeper[p as usize] = lvl;
        }
        if !self.cfg.olmc[k].registered {
            self.olmc[k].fb = lvl;
            self.set_array_input(fb_array_input(k), lvl);
        }
    }

    /// An array input takes a new value at `self.now`; fan out.
    /// Can the function `f` glitch as a consequence of the array inputs that
    /// changed at `self.now`?  Every combination of old/new values of those
    /// inputs is tried; the output is hazard-free if it is L for all of them,
    /// or H for all of them with one product term true throughout.
    fn hazard_free(&self, terms: &[Term]) -> bool {
        let t = self.now;
        let changed: Vec<usize> = (0..ARRAY_INPUTS).filter(|&j| self.arr_since[j] == t).collect();
        if changed.len() > 6 {
            return false;
        }
        let mut arr = self.arr;
        let combos = 1usize << changed.len();
        let (mut seen_l, mut seen_h) = (false, false);
        let mut covering: Vec<bool> = vec![true; terms.len()];
        for c in 0..combos {
            for (b, &j) in changed.iter().enumerate() {
                arr[j] = if c >> b & 1 == 1 { self.arr_prev[j] } else { self.arr[j] };
            }
            let mut any_h = false;
            for (ti, term) in terms.iter().enumerate() {
                let mut v = Level::H;
                for l in &term.0 {
                    let a = arr[l.input];
                    v = and3(v, if l.neg { not3(a) } else { a });
                }
                match v {
                    Level::H => any_h = true,
                    Level::L => covering[ti] = false,
                    _ => return false,
                }
            }
            if any_h {
                seen_h = true;
            } else {
                seen_l = true;
            }
            if seen_l && seen_h {
                return false; // value differs between combinations
            }
        }
        seen_l || covering.iter().any(|&c| c)
    }

    /// An array input takes a new value at `self.now`; fan out.
    fn set_array_input(&mut self, i: usize, v: Level) {
        if self.arr[i] == v {
            return;
        }
        if self.arr_since[i] != self.now {
            self.arr_prev[i] = self.arr[i];
        }
        self.arr[i] = v;
        self.arr_since[i] = self.now;
        let t = self.now;
        let tm = self.tm;
        for &k in &self.sop_users[i].clone() {
            if self.hazard_free(&self.cfg.olmc[k].terms) {
                continue; // D / output provably unaffected
            }
            self.olmc[k].cone_changed = t;
            if self.cfg.olmc[k].registered {
                if let Some(e) = self.last_edge
                    && t >= e
                    && t <= e + tm.th
                {
                    self.warn(WarningKind::Hold { olmc: k, changed_at: t });
                    self.set_q(k, Level::X);
                }
            } else {
                self.olmc[k].sop_settle = t + tm.tpd_max;
                self.schedule(t + tm.tpd_min, Ev::OutX(k));
                self.schedule(t + tm.tpd_max, Ev::OutSettle(k, t + tm.tpd_max));
            }
        }
        for &k in &self.oe_users[i].clone() {
            if let Oe::Term(term) = &self.cfg.olmc[k].oe
                && self.hazard_free(std::slice::from_ref(term))
            {
                continue;
            }
            let settle = t + tm.tea_max.max(tm.ter_max);
            self.olmc[k].oe_settle = settle;
            self.schedule(t + tm.tea_min.min(tm.ter_min), Ev::OeX(k));
            self.schedule(settle, Ev::OeSettle(k, settle));
        }
        if self.ar_uses[i] && !self.hazard_free(std::slice::from_ref(self.cfg.ar.as_ref().unwrap())) {
            self.ar_settle = t + tm.tap_max;
            self.schedule(t + tm.tap_min, Ev::ArX);
            self.schedule(t + tm.tap_max, Ev::ArSettle(t + tm.tap_max));
            // Pulse-width check on the term as the array sees it.
            let ar_in = self.cfg.ar.as_ref().map_or(Level::L, |term| self.eval_term(term));
            if ar_in != self.ar_in {
                if self.ar_in == Level::H && t - self.ar_high_since < tm.taw {
                    self.warn(WarningKind::AsyncResetWidth { width: t - self.ar_high_since });
                    self.unknown_all_regs();
                }
                if ar_in == Level::H {
                    self.ar_high_since = t;
                }
                self.ar_in = ar_in;
            }
        }
        if self.sp_uses[i] {
            self.sp_changed = t;
        }
    }

    fn eval_term(&self, t: &Term) -> Level {
        let mut v = Level::H;
        for l in &t.0 {
            let a = self.arr[l.input];
            v = and3(v, if l.neg { not3(a) } else { a });
        }
        v
    }
    fn eval_sop(&self, terms: &[Term]) -> Level {
        let mut v = Level::L;
        for t in terms {
            v = or3(v, self.eval_term(t));
        }
        v
    }

    fn process(&mut self, ev: Ev) {
        match ev {
            Ev::OutX(k) => {
                self.olmc[k].sop = Level::X;
                self.set_out(k, Level::X);
            }
            Ev::OutSettle(k, when) => {
                if when < self.olmc[k].sop_settle {
                    return;
                }
                let v = self.eval_sop(&self.cfg.olmc[k].terms);
                self.olmc[k].sop = v;
                let out = xor_pol(v, self.cfg.olmc[k].active_low);
                self.set_out(k, out);
            }
            Ev::OeX(k) => {
                self.olmc[k].oe = Level::X;
                self.refresh_io_pin(k);
            }
            Ev::OeSettle(k, when) => {
                if when < self.olmc[k].oe_settle {
                    return;
                }
                let v = match &self.cfg.olmc[k].oe {
                    Oe::Always => Level::H,
                    Oe::Never => Level::L,
                    Oe::Term(t) => self.eval_term(t),
                };
                self.olmc[k].oe = v;
                self.refresh_io_pin(k);
            }
            Ev::ArX => self.set_ar(Level::X),
            Ev::ArSettle(when) => {
                if when < self.ar_settle {
                    return;
                }
                let v = self.cfg.ar.as_ref().map_or(Level::L, |t| self.eval_term(t));
                self.set_ar(v);
            }
            Ev::RegPin(k, v) => self.set_out(k, v),
            Ev::RegFb(k, v) => {
                self.olmc[k].fb = v;
                self.set_array_input(fb_array_input(k), v);
            }
        }
    }

    /// Set the pre-OE output value of OLMC `k` and propagate to the pin.
    fn set_out(&mut self, k: usize, v: Level) {
        if self.olmc[k].out == v {
            return;
        }
        self.olmc[k].out = v;
        self.refresh_io_pin(k);
    }

    fn set_ar(&mut self, v: Level) {
        let old = self.ar;
        if old == v {
            return;
        }
        let t = self.now;
        self.ar = v;
        self.ar_since = t;
        match v {
            Level::H => {
                for k in 0..OLMCS {
                    if self.cfg.olmc[k].registered {
                        self.set_q(k, Level::L);
                    }
                }
            }
            Level::X if old != Level::H => {
                // Reset may engage at any moment in the tAP window.
                for k in 0..OLMCS {
                    if self.cfg.olmc[k].registered {
                        self.set_q(k, Level::X);
                    }
                }
            }
            // Releasing (H -> X -> L): registers stay reset either way.
            _ => {}
        }
    }

    /// Asynchronously force Q (reset or unknown): pin and feedback follow at
    /// once (the tAP window has already been spent getting here).
    fn set_q(&mut self, k: usize, v: Level) {
        if self.olmc[k].q == v {
            return;
        }
        self.olmc[k].q = v;
        let out = xor_pol(v, self.cfg.olmc[k].active_low);
        self.set_out(k, out);
        let fb = not3(v);
        self.olmc[k].fb = fb;
        self.set_array_input(fb_array_input(k), fb);
    }

    fn clock_transition(&mut self, old: Level, new: Level) {
        let t = self.now;
        let tm = self.tm;
        let width = t - self.clk_since;
        self.clk = new;
        self.clk_since = t;
        let any_reg = self.cfg.olmc.iter().any(|c| c.registered);
        if !any_reg {
            return;
        }
        match (old, new) {
            (Level::L, Level::H) => {
                if width < tm.tw {
                    self.warn(WarningKind::ClockWidth { width });
                    self.unknown_all_regs();
                    return;
                }
                self.rising_edge();
            }
            (Level::H, Level::L) => {
                if width < tm.tw {
                    self.warn(WarningKind::ClockWidth { width });
                    self.unknown_all_regs();
                }
            }
            (_, Level::X) | (_, Level::Z) => {
                self.warn(WarningKind::ClockUnknown);
                self.unknown_all_regs();
            }
            (Level::X, _) | (Level::Z, _) => {
                // Clock became known; treat a new high as a possible edge.
                if new == Level::H {
                    self.unknown_all_regs();
                }
            }
            _ => {}
        }
    }

    fn unknown_all_regs(&mut self) {
        for k in 0..OLMCS {
            if self.cfg.olmc[k].registered {
                self.set_q(k, Level::X);
            }
        }
    }

    fn rising_edge(&mut self) {
        let t = self.now;
        let tm = self.tm;
        self.last_edge = Some(t);
        // Reset dominates.
        if self.ar == Level::H {
            return;
        }
        let ar_bad = self.ar == Level::X || (self.ar_since + tm.tar > t && self.ar_since > 0);
        if self.ar == Level::L && self.ar_since > 0 && self.ar_since + tm.tar > t {
            self.warn(WarningKind::AsyncResetRecovery);
        }
        // Synchronous preset.
        let sp = match &self.cfg.sp {
            None => Level::L,
            Some(term) => {
                if self.sp_changed + tm.tsp.max(tm.tspr) > t {
                    self.warn(WarningKind::SyncPresetSetup);
                    Level::X
                } else {
                    self.eval_term(term)
                }
            }
        };
        let mut new_q = [Level::L; OLMCS];
        #[allow(clippy::needless_range_loop)]
        for k in 0..OLMCS {
            let c = &self.cfg.olmc[k];
            if !c.registered {
                continue;
            }
            let cone_changed = self.olmc[k].cone_changed;
            let mut v = if ar_bad {
                Level::X
            } else if sp == Level::H {
                Level::H
            } else if sp == Level::X {
                Level::X
            } else {
                self.eval_sop(&c.terms)
            };
            let setup_bad = cone_changed + tm.ts > t && cone_changed > 0;
            if setup_bad {
                self.warn(WarningKind::Setup { olmc: k, changed_at: cone_changed });
                v = Level::X;
            } else if v == Level::X && !ar_bad && sp != Level::X {
                self.warn(WarningKind::CapturedX { olmc: k });
            }
            new_q[k] = v;
        }
        #[allow(clippy::needless_range_loop)]
        for k in 0..OLMCS {
            if !self.cfg.olmc[k].registered {
                continue;
            }
            let v = new_q[k];
            let old = self.olmc[k].q;
            self.olmc[k].q = v;
            if v == old {
                continue; // flip-flop outputs don't glitch
            }
            let out = xor_pol(v, self.cfg.olmc[k].active_low);
            self.schedule(t + tm.tco_min, Ev::RegPin(k, Level::X));
            self.schedule(t + tm.tco_max, Ev::RegPin(k, out));
            self.schedule(t + tm.tcf_min, Ev::RegFb(k, Level::X));
            self.schedule(t + tm.tcf_max, Ev::RegFb(k, not3(v)));
        }
    }
}
