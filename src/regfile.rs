//! 32 x 32-bit, 2-read / 1-write register file built from eight CY7C131s.
//!
//! ```text
//!            bank A (rs)                    bank B (rt)
//!   chip 0..3 = byte lanes 0..3      chip 4..7 = byte lanes 0..3
//!
//!   LEFT port  = read  port: A0-4 = rs (bank A) / rt (bank B), A5-9 = GND,
//!                            R/W = VCC, OE = GND, CE = CE_R (phase signal)
//!   RIGHT port = write port: A0-4 = rd, A5-9 = GND, I/O = wdata byte,
//!                            OE = VCC, R/W = RW_W, CE = CE_W
//! ```
//! All eight write ports are wired in parallel (same rd, same data byte per
//! lane).  Nothing but the chips ever drives the read buses, and the chips
//! never drive the write bus (OE tied high), so there is no bus contention to
//! reason about.
//!
//! The read and write ports must never be enabled at the same time on the same
//! register (the chip would arbitrate and either BUSY the read or inhibit the
//! write), so the surrounding logic drives CE_W during the first part of the
//! cycle and CE_R during the second part.  A read in the same cycle as a write
//! to the same register therefore returns the *new* value, which is exactly
//! what the classic 5-stage pipeline's ID/WB overlap needs.
//!
//! Register 0: the write-enable logic outside this module must suppress writes
//! to rd = 0, and the chips are preloaded with zero there.
//!
//! This module is a hand-wired composition of chip models (a netlist in code);
//! the mapping is trivial enough that a generic netlist simulator would add
//! nothing yet.

use crate::cy7c131::{Bus, Cy7c131, Inputs, Level, PortInputs, Time, Warning};

/// Signals driven into the register file by the surrounding logic.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RegFileInputs {
    pub rs: [Level; 5],
    pub rt: [Level; 5],
    pub rd: [Level; 5],
    pub wdata: [Level; 32],
    /// Write-port chip enable (active low), all eight chips.
    pub ce_w_n: Level,
    /// Write-port R/W (low = write), all eight chips.
    pub rw_w_n: Level,
    /// Read-port chip enable (active low), all eight chips.
    pub ce_r_n: Level,
}

impl RegFileInputs {
    /// Everything deselected, addresses/data zero.
    pub fn idle() -> Self {
        RegFileInputs {
            rs: reg_levels(0),
            rt: reg_levels(0),
            rd: reg_levels(0),
            wdata: word_levels(0),
            ce_w_n: Level::H,
            rw_w_n: Level::H,
            ce_r_n: Level::H,
        }
    }
}

pub fn reg_levels(r: u8) -> [Level; 5] {
    std::array::from_fn(|i| Level::from_bit(r >> i & 1 == 1))
}
pub fn word_levels(w: u32) -> [Level; 32] {
    std::array::from_fn(|i| Level::from_bit(w >> i & 1 == 1))
}

/// What the register file drives back.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct RegFileOutputs {
    /// Byte lanes 0..3 of the rs read bus.
    pub rs_data: [Bus; 4],
    /// Byte lanes 0..3 of the rt read bus.
    pub rt_data: [Bus; 4],
    /// Any chip's read-port BUSY not released (L or X).
    pub read_busy: bool,
    /// Any chip's write-port BUSY not released (L or X).
    pub write_busy: bool,
}

impl RegFileOutputs {
    /// The rs word if every lane is driving a definite value.
    pub fn rs_word(&self) -> Option<u32> {
        word(&self.rs_data)
    }
    pub fn rt_word(&self) -> Option<u32> {
        word(&self.rt_data)
    }
}

fn word(lanes: &[Bus; 4]) -> Option<u32> {
    let mut w = 0u32;
    for (i, b) in lanes.iter().enumerate() {
        match b {
            Bus::V(v) => w |= (*v as u32) << (8 * i),
            _ => return None,
        }
    }
    Some(w)
}

#[derive(Clone, Debug)]
pub struct RegFile {
    /// chips[bank * 4 + lane]
    pub chips: [Cy7c131; 8],
}

impl RegFile {
    /// All registers preloaded to zero.
    pub fn new() -> Self {
        let mut chips: [Cy7c131; 8] = std::array::from_fn(|_| Cy7c131::new());
        for c in &mut chips {
            for r in 0..32 {
                c.preload(r, 0);
            }
        }
        RegFile { chips }
    }

    /// Preload a register in all chips (both banks).
    pub fn preload(&mut self, r: u8, value: u32) {
        for bank in 0..2 {
            for lane in 0..4 {
                self.chips[bank * 4 + lane].preload(r as u16, (value >> (8 * lane)) as u8);
            }
        }
    }

    /// Current contents as the chips hold them; `None` if any lane is unknown
    /// or the two banks disagree.
    pub fn peek(&self, r: u8) -> Option<u32> {
        let bank = |b: usize| -> Option<u32> {
            let mut w = 0u32;
            for lane in 0..4 {
                w |= (self.chips[b * 4 + lane].peek(r as u16)? as u32) << (8 * lane);
            }
            Some(w)
        };
        match (bank(0), bank(1)) {
            (Some(a), Some(b)) if a == b => Some(a),
            _ => None,
        }
    }

    pub fn set_inputs(&mut self, t: Time, i: RegFileInputs) {
        for bank in 0..2 {
            let raddr = if bank == 0 { i.rs } else { i.rt };
            for lane in 0..4 {
                let read = PortInputs {
                    addr: pad_addr(raddr),
                    ce_n: i.ce_r_n,
                    rw_n: Level::H,
                    oe_n: Level::L,
                    data: [Level::Z; 8],
                };
                let write = PortInputs {
                    addr: pad_addr(i.rd),
                    ce_n: i.ce_w_n,
                    rw_n: i.rw_w_n,
                    oe_n: Level::H,
                    data: std::array::from_fn(|b| i.wdata[8 * lane + b]),
                };
                self.chips[bank * 4 + lane].set_inputs(t, Inputs { l: read, r: write });
            }
        }
    }

    pub fn outputs(&self, t: Time) -> RegFileOutputs {
        let outs: Vec<_> = self.chips.iter().map(|c| c.outputs(t)).collect();
        RegFileOutputs {
            rs_data: std::array::from_fn(|lane| outs[lane].l.data),
            rt_data: std::array::from_fn(|lane| outs[4 + lane].l.data),
            read_busy: outs.iter().any(|o| o.l.busy_n != Level::Z),
            write_busy: outs.iter().any(|o| o.r.busy_n != Level::Z),
        }
    }

    pub fn next_event(&self, t: Time) -> Option<Time> {
        self.chips.iter().filter_map(|c| c.next_event(t)).min()
    }

    /// Warnings from every chip, tagged with the chip index.
    pub fn warnings(&self) -> Vec<(usize, Warning)> {
        self.chips
            .iter()
            .enumerate()
            .flat_map(|(i, c)| c.warnings().iter().map(move |w| (i, w.clone())))
            .collect()
    }
}

impl Default for RegFile {
    fn default() -> Self {
        Self::new()
    }
}

fn pad_addr(r: [Level; 5]) -> [Level; crate::cy7c131::ADDR_BITS] {
    std::array::from_fn(|i| if i < 5 { r[i] } else { Level::L })
}
