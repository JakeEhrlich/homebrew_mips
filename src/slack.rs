//! Trace slack: how much delay each bus that runs backwards through the
//! pipeline can carry before the machine misbehaves.
//!
//! The physical fuzz (`docs/fuzz.md`) puts a random delay on every pin at
//! once.  This asks a sharper question, one bus at a time: with ideal
//! wires everywhere else, a divided clock and binned delay lines, what is
//! the largest delay on every listening pin of this bus for which the
//! soak programs still run clean?  The answer is that bus's slack, and
//! at roughly 6.5 ps per mm in FR4 it is a trace length.
//!
//! A backward edge is a net driven by a chip in one pipeline stage and
//! read by a chip in an earlier one: forwarding, write-back, branch
//! resolution, stalls.  They are found from the board file, so a new
//! block that feeds back is included by construction.
use crate::asm::assemble;
use crate::board::{Board, PinKind};
use crate::cpu::{Boot, Build, Cpu};
use crate::ds1100::Grade;
use crate::iss::Cpu as Iss;
use std::collections::BTreeMap;

/// A bus (nets sharing a name up to the trailing index) with at least one
/// backward listener.
#[derive(Clone, Debug)]
pub struct Edge {
    pub bus: String,
    pub nets: Vec<String>,
    /// The listening pins in an earlier stage: (chip, pin).  The delay
    /// goes on these and nowhere else.
    pub pins: Vec<(String, usize)>,
    /// Driving blocks, "block (stage)".
    pub from: Vec<String>,
    /// Listening blocks in an earlier stage than the latest driver.
    pub to: Vec<String>,
}

fn rank(stage: &str) -> Option<usize> {
    ["IF", "IF/ID", "ID", "ID/EX", "EX", "EX/MEM", "MEM", "MEM/WB"].iter().position(|s| *s == stage)
}

fn bus_of(net: &str) -> &str {
    net.trim_end_matches(|c: char| c.is_ascii_digit())
}

/// The buses with a backward listener, sorted by name.
pub fn backward_edges(board: &Board) -> Vec<Edge> {
    let mut drivers: BTreeMap<&str, Vec<(&str, &str)>> = BTreeMap::new();
    let mut listeners: BTreeMap<&str, Vec<(&str, &str, &str, usize)>> = BTreeMap::new();
    for c in &board.chips {
        for p in &c.pins {
            if matches!(p.kind, PinKind::Out | PinKind::Bidir) {
                drivers.entry(&p.net).or_default().push((&c.block, &c.stage));
            }
            if matches!(p.kind, PinKind::In | PinKind::Bidir) {
                listeners.entry(&p.net).or_default().push((&c.block, &c.stage, &c.name, p.pin));
            }
        }
    }
    let mut edges: BTreeMap<String, Edge> = BTreeMap::new();
    for (net, ds) in &drivers {
        let Some(latest) = ds.iter().filter_map(|(_, s)| rank(s)).max() else { continue };
        let back: Vec<_> = listeners.get(net).map(|l| l.iter().filter(|(_, s, _, _)| rank(s).is_some_and(|r| r < latest)).collect()).unwrap_or_default();
        if back.is_empty() {
            continue;
        }
        let e = edges.entry(bus_of(net).to_string()).or_insert_with(|| Edge { bus: bus_of(net).to_string(), nets: Vec::new(), pins: Vec::new(), from: Vec::new(), to: Vec::new() });
        e.nets.push(net.to_string());
        for (b, s) in ds {
            let t = format!("{b} ({s})");
            if !e.from.contains(&t) {
                e.from.push(t);
            }
        }
        for (b, s, chip, pin) in back {
            let t = format!("{b} ({s})");
            if !e.to.contains(&t) {
                e.to.push(t);
            }
            e.pins.push((chip.to_string(), *pin));
        }
    }
    edges.into_values().collect()
}

/// Every backward listener at once: the board where all the back traces
/// are the same length.
pub fn all_edges(edges: &[Edge]) -> Edge {
    let mut all = Edge { bus: "ALL".into(), nets: Vec::new(), pins: Vec::new(), from: vec!["every backward edge".into()], to: vec!["at once".into()] };
    for e in edges {
        all.nets.extend(e.nets.iter().cloned());
        all.pins.extend(e.pins.iter().cloned());
    }
    all
}

/// A program the slack is measured against: assembled words, stop
/// address and the reference's final state.
pub struct Program {
    pub name: String,
    pub words: Vec<u32>,
    pub stop: u32,
    pub regs: [u32; 32],
    pub dmem: Vec<u32>,
    pub budget: u64,
}

impl Program {
    pub fn new(name: &str, src: &str) -> Program {
        let p = assemble(src, 0).unwrap_or_else(|e| panic!("{name}: assembler {e:?}"));
        let stop = p.labels["stop"];
        let mut iss = Iss::new();
        iss.load_program(0, &p.words);
        let retired = iss.run_until(stop, 200_000).unwrap_or_else(|e| panic!("{name}: reference fault {e:?}"));
        assert_eq!(iss.pc, stop, "{name}: reference did not reach stop");
        let dmem = (0..1024).step_by(4).map(|a| iss.load_word(a)).collect();
        Program { name: name.into(), words: p.words, stop, regs: iss.regs, dmem, budget: retired * 3 + 300 }
    }
}

/// The build the slack is measured under: ideal wires, a 50 % clock, no
/// jitter, binned delay lines, zeroed memories.
pub fn build(pins: &[(String, usize)], delay_ns: f64) -> Build {
    Build { boot: Boot::Preload, grade: Grade::Room, pin_delays: pins.iter().map(|(c, p)| (c.clone(), *p, delay_ns)).collect(), ..Build::default() }
}

/// Run one program with `delay_ns` on `pins`; the error
/// says what went wrong first.  The machine is the preloaded build (the
/// board with the boot copy skipped): the same chips and nets.
pub fn check(prog: &Program, pins: &[(String, usize)], delay_ns: f64) -> Result<(), String> {
    let period: f64 = std::env::var("SLACK_PERIOD_NS").ok().and_then(|v| v.parse().ok()).unwrap_or(34.0);
    let mut cpu = Cpu::build(&prog.words, period, build(pins, delay_ns));
    let reached = cpu.run_until_pc(prog.stop, prog.budget);
    if let Some(w) = cpu.reset_warnings().first() {
        return Err(format!("{}: during reset: {w}", prog.name));
    }
    if let Some(w) = cpu.warnings().first() {
        return Err(format!("{}: {w}", prog.name));
    }
    if !reached {
        return Err(format!("{}: did not reach stop in {} cycles, no chip complained", prog.name, prog.budget));
    }
    for r in 0..32 {
        if cpu.reg(r) != Some(prog.regs[r as usize]) {
            return Err(format!("{}: register {r} = {:?}, expected {:#x}", prog.name, cpu.reg(r), prog.regs[r as usize]));
        }
    }
    for (i, v) in prog.dmem.iter().enumerate() {
        let a = 4 * i as u32;
        if cpu.dmem_word(a) != Some(*v) {
            return Err(format!("{}: dmem[{a:#x}] = {:?}, expected {v:#x}", prog.name, cpu.dmem_word(a)));
        }
    }
    Ok(())
}

fn check_all(progs: &[Program], edge: &Edge, delay_ns: f64) -> Result<(), String> {
    let t = std::time::Instant::now();
    let r = progs.iter().try_for_each(|p| check(p, &edge.pins, delay_ns));
    if std::env::var_os("SLACK_VERBOSE").is_some() {
        eprintln!("  {} at {delay_ns:.2} ns: {} ({:.0} s)", edge.bus, if r.is_ok() { "pass" } else { "fail" }, t.elapsed().as_secs_f64());
    }
    r
}

/// The result of a bisection: the largest delay that passed, and the
/// first failure one step above it (`None` if `hi` itself passed).
#[derive(Clone, Debug)]
pub struct Slack {
    pub bus: String,
    pub pass_ns: f64,
    pub fail: Option<(f64, String)>,
}

/// Bisect the delay on `edge` between 0 and `hi_ns` to within `step_ns`.
pub fn max_delay(progs: &[Program], edge: &Edge, hi_ns: f64, step_ns: f64) -> Slack {
    if let Err(e) = check_all(progs, edge, 0.0) {
        return Slack { bus: edge.bus.clone(), pass_ns: f64::NAN, fail: Some((0.0, e)) };
    }
    let (mut lo, mut hi) = (0.0, hi_ns);
    let mut fail = match check_all(progs, edge, hi) {
        Ok(()) => return Slack { bus: edge.bus.clone(), pass_ns: hi, fail: None },
        Err(e) => e,
    };
    while hi - lo > step_ns + 1e-9 {
        let mid = ((lo + hi) / 2.0 / step_ns).round() * step_ns;
        let mid = if mid <= lo || mid >= hi { (lo + hi) / 2.0 } else { mid };
        match check_all(progs, edge, mid) {
            Ok(()) => lo = mid,
            Err(e) => {
                hi = mid;
                fail = e;
            }
        }
    }
    Slack { bus: edge.bus.clone(), pass_ns: lo, fail: Some((hi, fail)) }
}
