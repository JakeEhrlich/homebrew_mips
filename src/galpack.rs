//! From named equations to ATF22V10C configurations and netlist
//! connections.
//!
//! An [`Eq`] describes one output net as a sum of products over other named
//! nets (or as a truth table, minimised by [`crate::qm`] with the polarity
//! chosen to use fewer terms).  [`pack`] greedily fills 22V10s with
//! equations, respecting the pin count and each macrocell's product-term
//! budget, and [`instantiate`] turns a packed chip into a `Gal22v10` wired
//! into a [`Netlist`] by net name.

use std::collections::BTreeSet;

use crate::gal22v10::{Config, Gal22v10, Lit, Oe, OlmcConfig, PT_COUNT, Term, olmc_pin, pin_array_input};
use crate::netlist::{ChipId, Netlist};
use crate::qm;

/// `(net, positive)`.
pub type SLit = (String, bool);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    Comb,
    Reg,
}

#[derive(Clone, Debug)]
pub struct Eq {
    pub out: String,
    pub mode: Mode,
    /// Pin is the complement of the sum of products.
    pub active_low: bool,
    pub terms: Vec<Vec<SLit>>,
    /// Output-enable product term (`None` = always enabled).
    pub oe: Option<Vec<SLit>>,
    /// Synchroniser stage (see `gal22v10::OlmcConfig::sync`).
    pub sync: bool,
}

pub fn lit(net: &str) -> SLit {
    (net.to_string(), true)
}
pub fn nlit(net: &str) -> SLit {
    (net.to_string(), false)
}

impl Eq {
    /// Direct sum of products, active high.
    pub fn sop(out: &str, mode: Mode, terms: Vec<Vec<SLit>>) -> Eq {
        Eq { out: out.to_string(), mode, active_low: false, terms, oe: None, sync: false }
    }
    /// From a truth table over `inputs` (bit `i` of the argument is input
    /// `i`).  Picks whichever of f / !f needs fewer terms.
    pub fn table(out: &str, mode: Mode, inputs: &[&str], f: impl Fn(u32) -> Option<bool>) -> Eq {
        let n = inputs.len();
        let pos = qm::minimize(n, &f);
        let neg = qm::minimize(n, |m| f(m).map(|b| !b));
        let (terms, active_low) = if neg.len() < pos.len() { (neg, true) } else { (pos, false) };
        let terms = terms
            .into_iter()
            .map(|c| c.into_iter().map(|(i, p)| (inputs[i].to_string(), p)).collect())
            .collect();
        Eq { out: out.to_string(), mode, active_low, terms, oe: None, sync: false }
    }
    /// From a truth table, as the positive sum of products (the pin is
    /// `f`, and a cleared register reads as `f = 0`).  Use for registers
    /// whose reset value matters, and before `.active_low()`.
    pub fn table_pos(out: &str, mode: Mode, inputs: &[&str], f: impl Fn(u32) -> Option<bool>) -> Eq {
        let terms = qm::minimize(inputs.len(), &f)
            .into_iter()
            .map(|c| c.into_iter().map(|(i, p)| (inputs[i].to_string(), p)).collect())
            .collect();
        Eq { out: out.to_string(), mode, active_low: false, terms, oe: None, sync: false }
    }
    /// Mark as a synchroniser stage.
    pub fn sync(mut self) -> Eq {
        self.sync = true;
        self
    }
    pub fn with_oe(mut self, oe: Vec<SLit>) -> Eq {
        self.oe = Some(oe);
        self
    }
    pub fn active_low(mut self) -> Eq {
        self.active_low = !self.active_low;
        self
    }
    fn inputs(&self) -> BTreeSet<String> {
        self.terms.iter().flatten().chain(self.oe.iter().flatten()).map(|l| l.0.clone()).collect()
    }
}

/// One packed chip.
#[derive(Clone, Debug)]
pub struct GalSpec {
    pub name: String,
    pub clk: Option<String>,
    /// Asynchronous reset net (active high) for registered outputs.
    pub ar: Option<String>,
    pub eqs: Vec<Eq>,
}

impl GalSpec {
    /// Nets this chip needs on input pins (referenced, not produced here).
    pub fn external_inputs(&self) -> BTreeSet<String> {
        let outs: BTreeSet<String> = self.eqs.iter().map(|e| e.out.clone()).collect();
        let mut ins: BTreeSet<String> = self.eqs.iter().flat_map(|e| e.inputs()).filter(|n| !outs.contains(n)).collect();
        if let Some(ar) = &self.ar {
            ins.insert(ar.clone());
        }
        ins
    }
    fn needs_clock(&self) -> bool {
        self.eqs.iter().any(|e| e.mode == Mode::Reg)
    }
    /// Does everything fit on one 22V10?
    pub fn fits(&self) -> Result<(), String> {
        if self.eqs.len() > 10 {
            return Err(format!("{}: {} outputs", self.name, self.eqs.len()));
        }
        // Pins 1..11 and 13 are inputs; pin 1 is the clock when needed.
        let dedicated = if self.needs_clock() { 11 } else { 12 };
        let ins = self.external_inputs().len();
        let spare_io = 10 - self.eqs.len();
        if ins > dedicated + spare_io {
            return Err(format!("{}: {} inputs, {} outputs", self.name, ins, self.eqs.len()));
        }
        // Product terms: largest equations to largest macrocells.
        let mut need: Vec<usize> = self.eqs.iter().map(|e| e.terms.len()).collect();
        need.sort_unstable_by(|a, b| b.cmp(a));
        let mut have = PT_COUNT.to_vec();
        have.sort_unstable_by(|a, b| b.cmp(a));
        for (n, h) in need.iter().zip(&have) {
            if n > h {
                return Err(format!("{}: an output needs {n} product terms, largest free macrocell has {h}", self.name));
            }
        }
        Ok(())
    }
}

/// Greedily pack equations into chips named `prefix0`, `prefix1`, ...
/// Equations are taken in order, so put related bits next to each other.
pub fn pack(prefix: &str, clk: Option<&str>, ar: Option<&str>, eqs: Vec<Eq>) -> Vec<GalSpec> {
    let mut chips: Vec<GalSpec> = Vec::new();
    let new_chip = |i: usize| GalSpec {
        name: format!("{prefix}{i}"),
        clk: clk.map(str::to_string),
        ar: ar.map(str::to_string),
        eqs: vec![],
    };
    let mut cur = new_chip(0);
    for eq in eqs {
        let mut trial = cur.clone();
        trial.eqs.push(eq.clone());
        if trial.fits().is_ok() {
            cur = trial;
        } else {
            let single = GalSpec { eqs: vec![eq.clone()], ..new_chip(chips.len() + 1) };
            single.fits().unwrap_or_else(|e| panic!("equation {} fits no chip: {e}", eq.out));
            chips.push(cur);
            cur = single;
        }
    }
    chips.push(cur);
    // Rename sequentially (a chip may have been skipped over above).
    for (i, c) in chips.iter_mut().enumerate() {
        c.name = format!("{prefix}{i}");
        if c.eqs.iter().all(|e| e.mode == Mode::Comb) {
            c.clk = None;
            c.ar = None;
        }
    }
    chips
}

/// Build the chip and wire it into the netlist.  Returns the chip id and the
/// pin assignment `(pin, net)`.
pub fn instantiate(nl: &mut Netlist, spec: &GalSpec) -> (ChipId, Vec<(usize, String)>) {
    spec.fits().unwrap();
    // Outputs: largest term count to largest macrocell.
    let mut order: Vec<usize> = (0..spec.eqs.len()).collect();
    order.sort_by_key(|&i| std::cmp::Reverse(spec.eqs[i].terms.len()));
    let mut olmcs_by_size: Vec<usize> = (0..10).collect();
    olmcs_by_size.sort_by_key(|&k| std::cmp::Reverse(PT_COUNT[k]));
    let mut olmc_of: Vec<Option<usize>> = vec![None; spec.eqs.len()];
    let mut used_olmc = [false; 10];
    for (rank, &ei) in order.iter().enumerate() {
        let k = olmcs_by_size[rank];
        olmc_of[ei] = Some(k);
        used_olmc[k] = true;
    }
    // Inputs: clock on pin 1, then dedicated pins, then spare I/O pins
    // (smallest macrocells first).
    let mut pins: Vec<(usize, String)> = Vec::new();
    let mut dedicated: Vec<usize> = (2..=11).chain([13]).collect();
    if spec.needs_clock() {
        pins.push((1, spec.clk.clone().expect("registered outputs need a clock net")));
    } else {
        dedicated.insert(0, 1);
    }
    let mut spare: Vec<usize> = (0..10).filter(|&k| !used_olmc[k]).collect();
    spare.sort_by_key(|&k| PT_COUNT[k]);
    let mut input_array: std::collections::BTreeMap<String, usize> = Default::default();
    for name in spec.external_inputs() {
        if let Some(p) = dedicated.first().copied() {
            dedicated.remove(0);
            pins.push((p, name.clone()));
            input_array.insert(name, pin_array_input(p as u8));
        } else {
            let k = spare.remove(0);
            pins.push((olmc_pin(k) as usize, name.clone()));
            input_array.insert(name, crate::gal22v10::fb_array_input(k));
        }
    }
    // Build the config.
    let mut cfg = Config::empty();
    for (ei, eq) in spec.eqs.iter().enumerate() {
        let k = olmc_of[ei].unwrap();
        cfg.olmc[k] = OlmcConfig {
            registered: eq.mode == Mode::Reg,
            active_low: eq.active_low,
            oe: Oe::Always,
            terms: vec![],
            sync: eq.sync,
        };
    }
    let resolve = |cfg: &Config, l: &SLit| -> Lit {
        if let Some(&ai) = input_array.get(&l.0) {
            Lit { input: ai, neg: !l.1 }
        } else {
            let ei = spec.eqs.iter().position(|e| e.out == l.0).expect("unknown signal");
            let base = cfg.out(olmc_of[ei].unwrap());
            Lit { neg: base.neg ^ !l.1, ..base }
        }
    };
    for (ei, eq) in spec.eqs.iter().enumerate() {
        let k = olmc_of[ei].unwrap();
        let terms: Vec<Term> = eq.terms.iter().map(|t| Term::new(t.iter().map(|l| resolve(&cfg, l)))).collect();
        let oe = match &eq.oe {
            None => Oe::Always,
            Some(t) => Oe::Term(Term::new(t.iter().map(|l| resolve(&cfg, l)))),
        };
        cfg.olmc[k].terms = terms;
        cfg.olmc[k].oe = oe;
    }
    if let Some(ar) = &spec.ar {
        cfg.ar = Some(Term::new([resolve(&cfg, &lit(ar))]));
    }
    for (ei, eq) in spec.eqs.iter().enumerate() {
        pins.push((olmc_pin(olmc_of[ei].unwrap()) as usize, eq.out.clone()));
    }
    let chip = nl.add_chip(&spec.name, Gal22v10::new(cfg));
    for (p, net) in &pins {
        let n = nl.net(net);
        nl.connect(n, chip, *p);
    }
    (chip, pins)
}

/// Instantiate every chip of a packed block.
pub fn instantiate_all(nl: &mut Netlist, specs: &[GalSpec]) -> Vec<ChipId> {
    specs.iter().map(|s| instantiate(nl, s).0).collect()
}
