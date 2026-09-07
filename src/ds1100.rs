//! Conservative timing model of the DS1100 5-tap silicon delay line
//! (Maxim/Dallas, datasheet 091202, saved under `docs/DS1100.pdf`).
//!
//! Five taps reproduce the input after 20%, 40%, ..., 100% of the total
//! delay given by the part number (DS1100-20 ... -500).  Both edges are
//! delayed alike.  The model is conservative in the usual way:
//!
//! * every input transition arrives at tap `k` as an `X` window of
//!   `[nominal - tol, nominal + tol]` around the nominal delay, then the new
//!   level.  The taps' drift is really correlated (note 3: "if TAP1 slows
//!   down, all other taps also slow down"), but treating them independently
//!   only makes the model more pessimistic;
//! * an input `X` propagates as `X` through the same windows;
//! * an input pulse (high or low) shorter than the tap-1 delay violates
//!   `tWI` and is reported as a warning (the datasheet gives no behaviour
//!   for it); the outputs are unknown from that pulse on.
//!
//! Tolerances (AC table): total delay <= 40 ns: +-2 ns at 25 C / 5 V,
//! +-3 ns over 0..70 C, +-4 ns over -40..85 C.  Total > 40 ns: +-5%, +-8%,
//! +-13%.

use crate::cy7c131::{Level, NS, Time};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Grade {
    /// Initial accuracy at +25 C, 5.0 V (a lab bench).
    Room,
    /// Over 0..70 C and 4.75..5.25 V.
    Commercial,
    /// Over -40..85 C.
    Industrial,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WarningKind {
    /// Input pulse shorter than the tap-1 delay (tWI).
    PulseTooShort,
}

#[derive(Clone, Debug)]
pub struct Warning {
    pub kind: WarningKind,
    pub t: Time,
    pub detail: String,
}

/// The five nominal tap delays in ns for a DS1100-`total` part.
pub fn tap_delays_ns(total: u32) -> [f64; 5] {
    let step = total as f64 / 5.0;
    [step, 2.0 * step, 3.0 * step, 4.0 * step, 5.0 * step]
}

/// Tolerance in ps for a tap of a given nominal delay.
pub fn tolerance(total: u32, grade: Grade) -> Time {
    if total <= 40 {
        let ns = match grade {
            Grade::Room => 2.0,
            Grade::Commercial => 3.0,
            Grade::Industrial => 4.0,
        };
        (ns * NS as f64) as Time
    } else {
        let pct = match grade {
            Grade::Room => 0.05,
            Grade::Commercial => 0.08,
            Grade::Industrial => 0.13,
        };
        (pct * total as f64 * NS as f64) as Time
    }
}

#[derive(Clone, Debug)]
pub struct Ds1100 {
    /// Nominal tap delays in ps.
    pub taps: [Time; 5],
    /// +- tolerance in ps (same absolute value for every tap of a part).
    pub tol: Time,
    /// Input transitions `(time, new level)`, oldest first.
    edges: Vec<(Time, Level)>,
    input: Level,
    /// True once a pulse-width violation made the outputs untrustworthy.
    broken: bool,
    warnings: Vec<Warning>,
}

impl Ds1100 {
    /// A DS1100-`total` (20, 25, 30, 35, 40, 45, 50, 60, 75, 100, ... 500).
    pub fn new(total: u32, grade: Grade) -> Ds1100 {
        let d = tap_delays_ns(total);
        let taps = [0, 1, 2, 3, 4].map(|k| (d[k] * NS as f64).round() as Time);
        Ds1100 { taps, tol: tolerance(total, grade), edges: Vec::new(), input: Level::Z, broken: false, warnings: Vec::new() }
    }

    pub fn set_input(&mut self, t: Time, level: Level) {
        // Undriven counts as unknown; a driven level that matches is no edge.
        let level = if level == Level::Z { Level::X } else { level };
        if level == self.input {
            return;
        }
        // Pulse width: the previous level must have lasted at least tap 1
        // (the datasheet's "20% of tap 5").  Only definite levels count.
        if let Some(&(t0, prev)) = self.edges.last() {
            if prev != Level::X && t - t0 < self.taps[0] && !self.broken {
                self.warnings.push(Warning {
                    kind: WarningKind::PulseTooShort,
                    t,
                    detail: format!("input pulse of {} ps < tWI {} ps", t - t0, self.taps[0]),
                });
                self.broken = true;
            }
        }
        self.input = level;
        self.edges.push((t, level));
        // Forget edges that no tap can still be reporting.
        let horizon = t.saturating_sub(self.taps[4] + self.tol + 1);
        let keep = self.edges.iter().position(|&(e, _)| e >= horizon).unwrap_or(self.edges.len());
        // Keep one edge before the horizon so the steady level is known.
        let keep = keep.saturating_sub(1);
        self.edges.drain(..keep);
    }

    /// Level of tap `k` (0-based) at time `t`.
    pub fn tap(&self, k: usize, t: Time) -> Level {
        if self.broken {
            return Level::X;
        }
        let d = self.taps[k];
        // Walk back: the newest edge whose window has fully passed gives the
        // level; any edge whose window contains t makes the output X.
        let mut level = Level::X;
        let mut found = false;
        for &(e, l) in self.edges.iter().rev() {
            let (lo, hi) = ((e + d).saturating_sub(self.tol), e + d + self.tol);
            if t > lo && t < hi {
                return Level::X;
            }
            if t >= hi && !found {
                level = l;
                found = true;
                // Older edges can only have earlier windows; done.
                break;
            }
        }
        if !found {
            // Before the first edge has propagated: whatever the input was
            // before we knew it, i.e. unknown.
            return Level::X;
        }
        level
    }

    /// Next time strictly after `t` at which some tap changes.
    pub fn next_event(&self, t: Time) -> Option<Time> {
        if self.broken {
            return None;
        }
        let mut next: Option<Time> = None;
        for &(e, _) in &self.edges {
            for &d in &self.taps {
                for c in [(e + d).saturating_sub(self.tol), e + d + self.tol] {
                    if c > t && next.is_none_or(|n| c < n) {
                        next = Some(c);
                    }
                }
            }
        }
        next
    }

    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cy7c131::NS;

    fn ns(x: f64) -> Time {
        (x * NS as f64) as Time
    }

    #[test]
    fn taps_of_a_dash_25() {
        let d = Ds1100::new(25, Grade::Room);
        assert_eq!(d.taps, [ns(5.0), ns(10.0), ns(15.0), ns(20.0), ns(25.0)]);
        assert_eq!(d.tol, ns(2.0));
        assert_eq!(Ds1100::new(50, Grade::Commercial).tol, ns(4.0));
        assert_eq!(Ds1100::new(40, Grade::Industrial).tol, ns(4.0));
    }

    #[test]
    fn edge_propagates_as_a_window() {
        let mut d = Ds1100::new(25, Grade::Room);
        d.set_input(ns(0.0), Level::L);
        // Before anything has propagated the taps are unknown.
        assert_eq!(d.tap(0, ns(1.0)), Level::X);
        assert_eq!(d.tap(0, ns(7.0)), Level::L);
        d.set_input(ns(100.0), Level::H);
        // Tap 3 (15 ns): X from 113 to 117, high from 117.
        assert_eq!(d.tap(2, ns(112.9)), Level::L);
        assert_eq!(d.tap(2, ns(113.5)), Level::X);
        assert_eq!(d.tap(2, ns(116.9)), Level::X);
        assert_eq!(d.tap(2, ns(117.0)), Level::H);
        // Tap 5 (25 ns) is still low while tap 1 is already high.
        assert_eq!(d.tap(4, ns(120.0)), Level::L);
        assert_eq!(d.tap(0, ns(120.0)), Level::H);
        assert_eq!(d.next_event(ns(100.0)), Some(ns(103.0)));
        assert_eq!(d.next_event(ns(103.0)), Some(ns(107.0)));
        assert!(d.warnings().is_empty());
    }

    #[test]
    fn short_pulse_is_flagged() {
        let mut d = Ds1100::new(25, Grade::Room);
        d.set_input(ns(0.0), Level::L);
        d.set_input(ns(100.0), Level::H);
        d.set_input(ns(104.0), Level::L); // 4 ns < tap 1 (5 ns)
        assert_eq!(d.warnings().len(), 1);
        assert_eq!(d.warnings()[0].kind, WarningKind::PulseTooShort);
        assert_eq!(d.tap(0, ns(200.0)), Level::X);
    }

    #[test]
    fn clock_through_taps() {
        // A 34 ns clock: every tap is a clean delayed copy.
        let mut d = Ds1100::new(30, Grade::Commercial);
        let mut t = 0;
        for i in 0..22 {
            d.set_input(t, if i % 2 == 0 { Level::H } else { Level::L });
            t += ns(17.0);
        }
        assert!(d.warnings().is_empty());
        // Tap 5 = 30 ns +- 3: the rising edge at 340 shows at 370 +- 3
        // (history is only kept as far back as the taps need, so queries
        // must not lag the input by more than that).
        assert_eq!(d.tap(4, ns(366.0)), Level::L);
        assert_eq!(d.tap(4, ns(369.0)), Level::X);
        assert_eq!(d.tap(4, ns(373.0)), Level::H);
        assert_eq!(d.tap(4, ns(383.0)), Level::H);
        assert_eq!(d.tap(4, ns(391.0)), Level::L);
    }
}
