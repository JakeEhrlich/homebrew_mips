//! A small netlist simulator: chip models wired together by pin.
//!
//! * A [`Chip`] is anything with numbered pins that accepts externally driven
//!   levels, reports the levels it drives, and can say when its outputs next
//!   change on their own.
//! * A net joins chip pins, optional testbench drivers, ties and pull
//!   resistors.  Multiple strong drivers resolve to their common level, or to
//!   `X` (with a warning) when they disagree.
//! * The simulation is event driven.  At each event time the nets are settled
//!   iteratively (chips may respond with zero delay, e.g. an SRAM output
//!   going `X` the instant its address changes), then time jumps to the next
//!   testbench stimulus or chip-internal event.
//! * Every net-level change is traced and can be written as a VCD file.
//!
//! Each chip sees, on a pin it drives itself, only what *other* drivers put on
//! the net; the chip models deal with their own output vs. the outside world.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use crate::cy7c131::{self, Bus, Cy7c131, Inputs, PortInputs};
use crate::gal22v10::Gal22v10;
pub use crate::cy7c131::{Level, Time, NS};

/// A chip model with 1-based pin numbers.
pub trait Chip {
    /// Highest pin number.
    fn pin_count(&self) -> usize;
    fn pin_name(&self, pin: usize) -> String;
    /// Externally driven level on every pin (`ext[pin]`, index 0 unused).
    fn set_inputs(&mut self, t: Time, ext: &[Level]);
    /// Levels the chip drives at time `t` (`out[pin]`; `Z` for inputs).
    fn drive(&mut self, t: Time, out: &mut [Level]);
    /// Next time strictly after `t` at which an output changes by itself.
    fn next_event(&self, t: Time) -> Option<Time>;
    fn warnings(&self) -> Vec<String>;
}

// ---------------------------------------------------------------------------
// Chip impl: ATF22V10C (24-pin DIP)

impl Chip for Gal22v10 {
    fn pin_count(&self) -> usize {
        24
    }
    fn pin_name(&self, pin: usize) -> String {
        match pin {
            1 => "CLK/IN1".into(),
            2..=11 => format!("IN{pin}"),
            12 => "GND".into(),
            13 => "IN13".into(),
            14..=23 => format!("IO{pin}"),
            24 => "VCC".into(),
            _ => format!("?{pin}"),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut a = [Level::Z; 25];
        a[..25].copy_from_slice(&ext[..25]);
        Gal22v10::set_inputs(self, t, a);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        self.advance(t);
        let d = Gal22v10::drive(self);
        out[..25].copy_from_slice(&d);
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        Gal22v10::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        Gal22v10::warnings(self).iter().map(|w| w.to_string()).collect()
    }
}

// ---------------------------------------------------------------------------
// Chip impl: CY7C131 (52-pin PLCC)

/// PLCC pin functions, from the datasheet's "PLCC Top View".
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SramPin {
    Ce(cy7c131::Port),
    Rw(cy7c131::Port),
    Busy(cy7c131::Port),
    Int(cy7c131::Port),
    Oe(cy7c131::Port),
    A(cy7c131::Port, u8),
    Io(cy7c131::Port, u8),
    Nc,
    Gnd,
    Vcc,
}

pub fn sram_pin(pin: usize) -> SramPin {
    use cy7c131::Port::{Left as L, Right as R};
    match pin {
        1 => SramPin::Ce(L),
        2 => SramPin::Rw(L),
        3 => SramPin::Busy(L),
        4 => SramPin::Int(L),
        5 | 25 | 35 | 47 => SramPin::Nc,
        6 => SramPin::Oe(L),
        7..=16 => SramPin::A(L, (pin - 7) as u8),
        17..=24 => SramPin::Io(L, (pin - 17) as u8),
        26 => SramPin::Gnd,
        27..=34 => SramPin::Io(R, (pin - 27) as u8),
        36..=45 => SramPin::A(R, (45 - pin) as u8),
        46 => SramPin::Oe(R),
        48 => SramPin::Int(R),
        49 => SramPin::Busy(R),
        50 => SramPin::Rw(R),
        51 => SramPin::Ce(R),
        52 => SramPin::Vcc,
        _ => SramPin::Nc,
    }
}

/// Pin number for a given function (inverse of [`sram_pin`]).
pub fn sram_pin_of(f: SramPin) -> usize {
    (1..=52).find(|&p| sram_pin(p) == f).expect("no such pin")
}

impl Chip for Cy7c131 {
    fn pin_count(&self) -> usize {
        52
    }
    fn pin_name(&self, pin: usize) -> String {
        let s = |p: cy7c131::Port| if p == cy7c131::Port::Left { "L" } else { "R" };
        match sram_pin(pin) {
            SramPin::Ce(p) => format!("CE{}", s(p)),
            SramPin::Rw(p) => format!("RW{}", s(p)),
            SramPin::Busy(p) => format!("BUSY{}", s(p)),
            SramPin::Int(p) => format!("INT{}", s(p)),
            SramPin::Oe(p) => format!("OE{}", s(p)),
            SramPin::A(p, i) => format!("A{i}{}", s(p)),
            SramPin::Io(p, i) => format!("IO{i}{}", s(p)),
            SramPin::Nc => "NC".into(),
            SramPin::Gnd => "GND".into(),
            SramPin::Vcc => "VCC".into(),
        }
    }
    fn set_inputs(&mut self, t: Time, ext: &[Level]) {
        let mut inp = Inputs { l: PortInputs::default(), r: PortInputs::default() };
        for (pin, &v) in ext.iter().enumerate().take(53).skip(1) {
            match sram_pin(pin) {
                SramPin::Ce(p) => inp.port_mut(p).ce_n = v,
                SramPin::Rw(p) => inp.port_mut(p).rw_n = v,
                SramPin::Oe(p) => inp.port_mut(p).oe_n = v,
                SramPin::A(p, i) => inp.port_mut(p).addr[i as usize] = v,
                SramPin::Io(p, i) => inp.port_mut(p).data[i as usize] = v,
                _ => {}
            }
        }
        Cy7c131::set_inputs(self, t, inp);
    }
    fn drive(&mut self, t: Time, out: &mut [Level]) {
        let o = self.outputs(t);
        for (pin, slot) in out.iter_mut().enumerate().take(53).skip(1) {
            *slot = match sram_pin(pin) {
                SramPin::Busy(p) => o.port(p).busy_n,
                SramPin::Int(p) => o.port(p).int_n,
                SramPin::Io(p, i) => match o.port(p).data {
                    Bus::Z => Level::Z,
                    Bus::X => Level::X,
                    Bus::V(v) => Level::from_bit(v >> i & 1 == 1),
                },
                _ => Level::Z,
            };
        }
    }
    fn next_event(&self, t: Time) -> Option<Time> {
        Cy7c131::next_event(self, t)
    }
    fn warnings(&self) -> Vec<String> {
        Cy7c131::warnings(self).iter().map(|w| w.to_string()).collect()
    }
}

// ---------------------------------------------------------------------------
// Netlist

pub type NetId = usize;
pub type ChipId = usize;

#[derive(Clone, Debug)]
struct Net {
    name: String,
    /// (chip, pin) connections.
    pins: Vec<(ChipId, usize)>,
    /// Constant driver (VCC / GND).
    tie: Level,
    /// Weak pull (resistor): L, H or Z for none.
    pull: Level,
}

/// Builder.
#[derive(Default)]
pub struct Netlist {
    chips: Vec<(String, Box<dyn Chip>)>,
    nets: Vec<Net>,
    by_name: BTreeMap<String, NetId>,
}

impl Netlist {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add_chip(&mut self, name: &str, chip: impl Chip + 'static) -> ChipId {
        self.chips.push((name.to_string(), Box::new(chip)));
        self.chips.len() - 1
    }
    /// Get or create a net by name.
    pub fn net(&mut self, name: &str) -> NetId {
        if let Some(&id) = self.by_name.get(name) {
            return id;
        }
        let id = self.nets.len();
        self.nets.push(Net { name: name.to_string(), pins: vec![], tie: Level::Z, pull: Level::Z });
        self.by_name.insert(name.to_string(), id);
        id
    }
    /// Create `width` nets named `prefix0`, `prefix1`, ...
    pub fn bus(&mut self, prefix: &str, width: usize) -> Vec<NetId> {
        (0..width).map(|i| self.net(&format!("{prefix}{i}"))).collect()
    }
    pub fn connect(&mut self, net: NetId, chip: ChipId, pin: usize) {
        assert!(pin >= 1 && pin <= self.chips[chip].1.pin_count(), "pin {pin} out of range");
        self.nets[net].pins.push((chip, pin));
    }
    /// Constant driver on a net (VCC or GND).
    pub fn tie(&mut self, net: NetId, level: Level) {
        self.nets[net].tie = level;
    }
    /// Pull resistor on a net.
    pub fn pull(&mut self, net: NetId, level: Level) {
        self.nets[net].pull = level;
    }
    pub fn build(self) -> Sim {
        let n = self.nets.len();
        let mut pin_net: Vec<Vec<Option<NetId>>> =
            self.chips.iter().map(|(_, c)| vec![None; c.pin_count() + 1]).collect();
        for (id, net) in self.nets.iter().enumerate() {
            for &(c, p) in &net.pins {
                assert!(pin_net[c][p].is_none(), "chip {} pin {} on two nets", self.chips[c].0, p);
                pin_net[c][p] = Some(id);
            }
        }
        let drives = self.chips.iter().map(|(_, c)| vec![Level::Z; c.pin_count() + 1]).collect();
        let fed = self.chips.iter().map(|(_, c)| vec![Level::Z; c.pin_count() + 1]).collect();
        let mut sim = Sim {
            chips: self.chips,
            nets: self.nets,
            by_name: self.by_name,
            pin_net,
            drives,
            fed,
            ext: vec![Level::Z; n],
            value: vec![Level::Z; n],
            stim: BTreeMap::new(),
            seq: 0,
            now: 0,
            trace: Vec::new(),
            conflicts: Vec::new(),
        };
        sim.settle();
        sim
    }
}

/// A built, running netlist.
pub struct Sim {
    chips: Vec<(String, Box<dyn Chip>)>,
    nets: Vec<Net>,
    by_name: BTreeMap<String, NetId>,
    pin_net: Vec<Vec<Option<NetId>>>,
    /// Last levels each chip drove.
    drives: Vec<Vec<Level>>,
    /// Last external levels fed to each chip.
    fed: Vec<Vec<Level>>,
    /// Testbench driver per net.
    ext: Vec<Level>,
    /// Resolved net values.
    value: Vec<Level>,
    stim: BTreeMap<(Time, u64), (NetId, Level)>,
    seq: u64,
    now: Time,
    trace: Vec<(Time, NetId, Level)>,
    conflicts: Vec<(Time, NetId)>,
}

fn resolve(drivers: impl Iterator<Item = Level>, pull: Level) -> Level {
    let mut v = Level::Z;
    for d in drivers {
        v = match (v, d) {
            (a, Level::Z) => a,
            (Level::Z, b) => b,
            (a, b) if a == b => a,
            _ => Level::X,
        };
    }
    if v == Level::Z { pull } else { v }
}

impl Sim {
    pub fn now(&self) -> Time {
        self.now
    }
    pub fn net_id(&self, name: &str) -> NetId {
        *self.by_name.get(name).unwrap_or_else(|| panic!("no net {name}"))
    }
    pub fn net_name(&self, id: NetId) -> &str {
        &self.nets[id].name
    }
    /// Resolved level on a net right now.
    pub fn value(&self, net: NetId) -> Level {
        self.value[net]
    }
    /// Read a bus (bit i = nets[i]); `None` if any bit is not L/H.
    pub fn read_bus(&self, nets: &[NetId]) -> Option<u32> {
        let mut v = 0u32;
        for (i, &n) in nets.iter().enumerate() {
            v |= (self.value[n].bit()? as u32) << i;
        }
        Some(v)
    }

    /// Testbench: drive a net to `level` at time `t` (>= now).  `Z` releases.
    pub fn schedule(&mut self, t: Time, net: NetId, level: Level) {
        assert!(t >= self.now, "schedule() in the past");
        self.seq += 1;
        self.stim.insert((t, self.seq), (net, level));
    }
    pub fn schedule_bus(&mut self, t: Time, nets: &[NetId], value: u32) {
        for (i, &n) in nets.iter().enumerate() {
            self.schedule(t, n, Level::from_bit(value >> i & 1 == 1));
        }
    }
    pub fn schedule_bus_z(&mut self, t: Time, nets: &[NetId]) {
        for &n in nets {
            self.schedule(t, n, Level::Z);
        }
    }

    /// Time of the next thing that will happen after `now`.
    pub fn next_event(&self) -> Option<Time> {
        let s = self.stim.keys().next().map(|k| k.0);
        let c = self.chips.iter().filter_map(|(_, c)| c.next_event(self.now)).min();
        match (s, c) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// Advance to `t`, processing everything on the way.
    pub fn run_until(&mut self, t: Time) {
        assert!(t >= self.now);
        loop {
            let next = self.next_event();
            match next {
                Some(nt) if nt <= t => {
                    self.now = nt;
                    while let Some((&(st, seq), &(net, lvl))) = self.stim.iter().next() {
                        if st > nt {
                            break;
                        }
                        self.stim.remove(&(st, seq));
                        self.ext[net] = lvl;
                    }
                    self.settle();
                }
                _ => {
                    self.now = t;
                    self.settle();
                    return;
                }
            }
        }
    }

    /// Recompute nets and chip inputs at `now` until nothing changes.
    fn settle(&mut self) {
        let t = self.now;
        for iter in 0..1000 {
            // Feed every chip what the rest of the world drives on its pins.
            let mut any_input_change = false;
            for c in 0..self.chips.len() {
                let mut ext = vec![Level::Z; self.chips[c].1.pin_count() + 1];
                for (p, slot) in ext.iter_mut().enumerate().skip(1) {
                    if let Some(n) = self.pin_net[c][p] {
                        *slot = self.resolve_excluding(n, c, p);
                    }
                }
                if ext != self.fed[c] || iter == 0 {
                    self.fed[c] = ext.clone();
                    self.chips[c].1.set_inputs(t, &ext);
                    any_input_change = true;
                }
            }
            // Collect drives.
            let mut any_drive_change = false;
            for c in 0..self.chips.len() {
                let mut out = vec![Level::Z; self.chips[c].1.pin_count() + 1];
                self.chips[c].1.drive(t, &mut out);
                if out != self.drives[c] {
                    self.drives[c] = out;
                    any_drive_change = true;
                }
            }
            self.recompute_values();
            if !any_drive_change && !any_input_change {
                break;
            }
            if iter == 999 {
                panic!("netlist did not settle at t={t}");
            }
        }
    }

    fn resolve_excluding(&self, net: NetId, chip: ChipId, pin: usize) -> Level {
        let n = &self.nets[net];
        let others = n.pins.iter().filter(|&&(c, p)| !(c == chip && p == pin)).map(|&(c, p)| self.drives[c][p]);
        resolve(others.chain([n.tie, self.ext[net]]), n.pull)
    }

    fn recompute_values(&mut self) {
        let t = self.now;
        for id in 0..self.nets.len() {
            let n = &self.nets[id];
            let strong: Vec<Level> =
                n.pins.iter().map(|&(c, p)| self.drives[c][p]).chain([n.tie, self.ext[id]]).collect();
            let v = resolve(strong.iter().copied(), n.pull);
            // Contention: two different definite levels.
            let has_l = strong.contains(&Level::L);
            let has_h = strong.contains(&Level::H);
            if has_l && has_h && self.conflicts.last() != Some(&(t, id)) {
                self.conflicts.push((t, id));
            }
            if v != self.value[id] {
                self.value[id] = v;
                self.trace.push((t, id, v));
            }
        }
    }

    /// All warnings: chip warnings prefixed by chip name, plus net conflicts.
    pub fn warnings(&self) -> Vec<String> {
        let mut w: Vec<String> = self
            .chips
            .iter()
            .flat_map(|(name, c)| c.warnings().into_iter().map(move |s| format!("{name}: {s}")))
            .collect();
        for &(t, n) in &self.conflicts {
            w.push(format!("t={:.3}ns net {}: drive conflict", t as f64 / NS as f64, self.nets[n].name));
        }
        w
    }

    /// Value history of one net: (time, level) at each change.
    pub fn history(&self, net: NetId) -> Vec<(Time, Level)> {
        self.trace.iter().filter(|e| e.1 == net).map(|e| (e.0, e.2)).collect()
    }

    /// Whole trace as a VCD document (timescale 1ps).
    pub fn vcd(&self) -> String {
        let mut s = String::new();
        writeln!(s, "$timescale 1ps $end").unwrap();
        writeln!(s, "$scope module netlist $end").unwrap();
        let code = |id: NetId| format!("n{id}");
        for (id, n) in self.nets.iter().enumerate() {
            writeln!(s, "$var wire 1 {} {} $end", code(id), n.name.replace(' ', "_")).unwrap();
        }
        writeln!(s, "$upscope $end").unwrap();
        writeln!(s, "$enddefinitions $end").unwrap();
        let ch = |l: Level| match l {
            Level::L => '0',
            Level::H => '1',
            Level::Z => 'z',
            Level::X => 'x',
        };
        let mut last: Option<Time> = None;
        writeln!(s, "#0").unwrap();
        writeln!(s, "$dumpvars").unwrap();
        for id in 0..self.nets.len() {
            let initial = self.trace.iter().find(|e| e.1 == id && e.0 == 0).map(|e| e.2).unwrap_or(Level::Z);
            writeln!(s, "{}{}", ch(initial), code(id)).unwrap();
        }
        writeln!(s, "$end").unwrap();
        for &(t, id, v) in &self.trace {
            if t == 0 {
                continue;
            }
            if last != Some(t) {
                writeln!(s, "#{t}").unwrap();
                last = Some(t);
            }
            writeln!(s, "{}{}", ch(v), code(id)).unwrap();
        }
        s
    }
}
