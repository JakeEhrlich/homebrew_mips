# Memory timing: how the two SRAMs are written, and what the PCB must guarantee

This is the reference for the board design. Every number here is either a
datasheet value or a margin the simulator computes from datasheet values; the
places where the simulator cannot see the board (clock skew, trace delay) are
called out as PCB requirements with the margin they must fit into.

Status: simulated clean at 32 to 40 ns clock period, for delay-line tolerance
grades room, commercial (0..70 C) and industrial, with every CPU test program
(`cargo test --release`, `tests/memory_timing.rs`, `tests/cpu.rs`).

## 1. The problem this solves

Both memories are asynchronous SRAMs. A write is a pulse whose two edges must
each land inside a window bounded by other events:

- the start must come after address and data are stable, and, on the register
  file, after the steer logic has taken the read ports off the register being
  written (otherwise the dual-port arbitration inhibits the write);
- the end must come before the address changes again (address hold), and the
  pulse between them must be at least the part's minimum width.

The original design shaped these pulses from a delay line through a GAL. That
cannot work at 34 ns: an edge shaped that way lands anywhere in an 8.5 to
10.5 ns window (tap tolerance plus the GAL's 3 to 7.5 ns propagation range),
while the target windows for the pulse ends are 2 to 3 ns wide, because they
sit between "the last useful instant of this cycle" and "the first thing that
changes in the next" (a registered output, 2 ns after the edge). The search in
`tests/strobes.rs` (git history, commit 836fa5c) confirmed no tap assignment
closes at any period.

The fix is to stop generating pulses and use the one precise edge the machine
has, the clock itself, as the write enable. The rest of this document is the
consequences.

## 2. The scheme in one paragraph

Both SRAM write ports have their chip enable driven directly by CLK. The write
is the clock's low half (17 ns) and ends exactly at the rising edge. The
address and write flag for each write port come from a copy register clocked
by a delay-line tap, so they are valid before the low half starts and hold
until well after the edge that ends the write. Write data comes from an
ordinary edge-clocked register, which is enough because both parts have zero
data hold. Reads never share a port with writes: the register file already had
separate read ports, and the data memory becomes dual-port for the same
reason. Nothing else in the machine changed.

## 3. Parts

| Function | Part | Notes |
|---|---|---|
| Register file | 8 x CY7C131-15 (1K x 8 dual-port) | unchanged |
| Instruction memory | 4 x AS7C164A-15 (8K x 8) | unchanged, read only, no strobes |
| Data memory | 4 x IDT7006S15 or CY7C006-15 (16K x 8 dual-port, PLCC-68) | **new**; 64 KB; timing modelled with the CY7C131-15 numbers, to be confirmed against the 7006 datasheet before layout |
| Delay line | 1 x DS1100-30 (taps 6, 12, 18, 24, 30 ns) | **new**; taps 1 and 3 used |
| Logic | 145 x ATF22V10C-7 | was 132; +4 write copies, +4 write-data copy, +1 stall, +4 from adding a hold input to pipeline registers |

Total 162 chips.

The AS7C164A-15 and IS61C256AH-12 grades are both in the simulator
(`as7c164a::Timing`); data memory does not use them any more.

## 4. The clock

- Period 34 ns nominal (clean down to 32 in simulation). Oscillator socketed.
- Duty cycle matters now: the falling edge is the write start, and it must be
  at least 14 ns after the rising edge for the register file steer (section 5).
  Use a 45/55 % or better oscillator, or an exact divide-by-two from a doubled
  oscillator in a GAL.
- The clock drives every GAL clock pin, both SRAM write-port enables (8 + 4
  pins) and the delay line input.

### 4.1 Skew budget

Every hold-type margin in the machine has the form "N ns minus clock skew",
where skew is the arrival difference between the clock at the relevant SRAM
enable pin and at the GAL clock pins whose outputs it races. N is:

| Race | N | Where |
|---|---|---|
| GAL to GAL hold (tCO min 2, tH 0), everywhere | 2 | whole pipeline, unchanged |
| Data hold at both write ports (tHD 0), WD / WSD change 2 ns after the edge | 2 | MEM/WB chips vs SRAM enable |
| Read data hold at the data memory read port (tOHA 0 after address change) | 2 | EX/MEM address chips vs MEM/WB capture |
| Address hold at both write ports (tHA 2), copies change 5 ns after the edge at commercial grade | 3 | copy chips vs SRAM enable |
| Write start after steer (steer settled at 13, write starts at 17) | 4 | steer chip vs SRAM enable (helped, not hurt, by early SRAM clock) |

The board must keep skew under about 1 ns between the SRAM enable pins and the
GAL clock pins named above. Concretely:

1. One clock buffer family with a specified output-to-output skew (0.25 to
   0.5 ns class, e.g. a 74FCT3807-type 1:10 driver at 5 V).
2. The buffer output that clocks the EX/MEM and MEM/WB register chips also
   drives the four data-memory write-port enables; the output that clocks the
   MEM/WB chips also drives the eight register-file write-port enables. Only
   intra-device skew then applies to the races above.
3. Trace lengths of those enable nets matched to the corresponding GAL clock
   nets within about 3 cm (6.5 ps/mm on FR4).
4. Bias: if anything, make the SRAM enable traces the shorter ones. An early
   SRAM edge ends the write earlier (more address hold) and starts it earlier
   (eats into the 4 ns steer margin, the largest one).
5. Fast edges on the clock tree (series termination, no stubs), so input
   threshold differences between TTL-level SRAM pins and the GAL pins add
   under 0.3 ns.

### 4.2 The two knobs

Hold margins do not grow with a slower clock; setup margins do. So the board
carries two independent adjustments:

- **Tap on the GAL clock tree** (jumper: 0, 6, 12 ns from a spare delay line
  or the DS1100's T1/T2): delays every GAL clock relative to the SRAM enables.
  Adds the tap delay to every hold margin above, costs the same in setup.
- **Oscillator socket**: a slower clock buys back setup margin.

Between them any skew of a few ns can be absorbed on the bench.

## 5. Register file write

Signals at the eight CY7C131 write ports (right port):

| Pin | Net | Source |
|---|---|---|
| CE_R | CLK | clock buffer |
| R/W_R, A5_R | WREGC_n | copy stage 2 (chip wc2*), low = write; high parks the port at 32..63 |
| A0-4_R | WDESTC[4:0] | copy stage 2 |
| IO_R | WD[31:0] | MEM/WB register (edge clocked) |
| OE_R | VCC | port never drives |

Copy stages (`cpu::wcopy1_block`, `wcopy2_block`):

- Stage 1 clocked by T3 (CLK + 18 ns +-3): samples MDEST / MRW (the MEM-stage
  destination and write flag), which are valid from 5.5 ns and stable to
  36 ns. Setup margin 15 - 5.5 - 3.5 = 6 ns.
- Stage 2 clocked by T1 (CLK + 6 ns +-3): samples stage 1 in the next cycle.
  Stage 1 is valid by 26.5 ns and does not change until 17 ns into the
  following cycle; stage 2's edge is at 3 to 9 ns. Setup margin 7 ns, hold
  margin 8 ns.
- Stage 2 output: valid by 9 + 5.5 = 14.5 ns, changes no earlier than 3 + 2 =
  5 ns after the next edge.

Timing of the write itself (CY7C131-15 datasheet values):

| Requirement | Value | Have | Margin |
|---|---|---|---|
| Steer has read ports off the target register before the write port selects | ~13 ns (IR 5.5 + tPD 7.5) | falls at 17 | 4 - skew |
| tSA address setup before write start | 0 | address valid 14.5, write starts 17 | 2.5 |
| tPWE write pulse | 12 | 17 | 5 |
| tSCE CE low to write end | 12 | 17 | 5 |
| tSD data setup to write end | 10 | WD valid at 5.5 | 18.5 |
| tHA address hold after write end | 2 | changes at 39 (cycle 34 + 5) | 3 - skew |
| tHD data hold | 0 | WD changes at 36 | 2 - skew |
| Arbitration: write port deselects at the edge, read ports re-address 2 ns later | | | 2 - skew |

Nothing about the steer, forwarding or write-back changed: the write still
completes inside the instruction's own WB cycle.

## 6. Data memory

Four 16K x 8 dual-port SRAMs, one per byte lane. A13 of the write port is the
idle park (addresses 8K..16K are never read), so the board wires it exactly
like the register file's A5.

| Pin | Net | Source |
|---|---|---|
| Read port A0-12 | MR[14:2] | EX/MEM result |
| Read port A13 | GND | |
| Read port CE | MMR_n | EX/MEM control; low only during a load's MEM cycle |
| Read port R/W, OE | VCC, GND | read, always output-enabled |
| Read port IO | DQ[31:0] | to MEM/WB |
| Write port CE | CLK | clock buffer |
| Write port R/W, A13 | DWE_n | copy stage 2, low = write; high parks at 8K..16K |
| Write port A0-12 | DA[12:0] | copy stage 2 |
| Write port IO | WSD[31:0] | WB-stage copy of the store data (chips wsd*) |
| Write port OE | VCC | port never drives |
| BUSY, INT (both ports) | pulled up | unused |

The write happens in the store's WB cycle: stage 1 samples MR and MMW at T3
of the MEM cycle, stage 2 re-times them at T1 of the WB cycle, and WSD
captures the forwarded store data at the edge that begins the WB cycle. The
write is the low half of that cycle. Margins are the same as the register
file's (same copy stages, same datasheet numbers), except there is no steer
and no bus turnaround: the write port has its own data pins and never drives.

Loads: read port selected and addressed 2 to 5.5 ns into the MEM cycle, data
at 5.5 + 15 = 20.5 ns, captured at 34 with 3.5 setup: 10 ns margin. Hold:
address and select change at 36, tOHA 0: 2 - skew.

### 6.1 Load-after-store interlock

A load in MEM during a store's WB cycle would select the read port while the
write port is active. If the addresses matched, the arbitration would inhibit
the write (the read port, selected earlier, wins). The read port is selected
only during loads precisely so that no other instruction can cause this. For
the remaining case, a load immediately behind a store, the pipeline holds one
cycle (`cpu::stall_block`): PC, IF/ID, ID/EX and MEM/WB keep their values,
EX/MEM takes a bubble (the store has already been captured by the copies), and
the load enters MEM after the write has ended. `HELD` limits it to one cycle.
Cost: one extra cycle per store-then-load pair. A compiler or rewriter that
separates them avoids it.

This is also the machinery a bus-wait for slow peripherals will reuse, with
one addition: a wait must hold EX/MEM rather than bubble it.

## 7. Reset

- RESET is asserted from power-on (a supervisor) and released synchronously:
  the board needs a one-flop synchroniser on the main clock, so the release
  lands 2 to 5.5 ns after an edge. The simulator models the release at 5.5 ns.
- Every pipeline register with a reset uses it asynchronously (GAL AR).
  Recovery: release + AR path + 5 ns must precede the next edge; at 34 ns it
  does with about 13 ns to spare.
- The copy registers have **no** asynchronous reset: their sources are held at
  zero by the pipeline's reset, the ATF22V10C powers up cleared, and RESET
  enters the write-flag copy as ordinary synchronous data
  (`WC1W_n = !MRW & !RESET`). This avoids any recovery-time relation between
  the reset release and the tap clocks. Setup of RESET at stage 1: 6 ns.
- r0: while RESET is held the MEM/WB destination is 0, the data is 0, and the
  write flag reads "write" (polarity chosen for that), so the register file's
  r0 is written with zero on every clock during reset. The simulator preloads
  r0 with garbage to prove it. Hold RESET for at least four clocks after the
  clock is stable.

## 8. Delay line

DS1100-30 (8-pin DIP, 5 V). Input: CLK. Taps used: T1 (6 ns) and T3 (18 ns).
Tolerance per tap: +-2 ns at 25 C, +-3 ns over 0..70 C, +-4 ns over -40..85 C.
Both are modelled as independent unknown windows, which is more pessimistic
than the part (all taps drift together). Datasheet note 9: "at or near
maximum frequency the delay accuracy can vary and will be application
sensitive (decoupling, layout)". The input pulse width (17 ns) is far above
the 6 ns minimum. Give it its own decoupling capacitor and a short, clean
input trace from the clock buffer.

Each tap clocks two or three GAL clock pins (the copy chips). Those are
ordinary GAL clock inputs; the copy chips' hold and setup margins (section 5)
already include the full tap tolerance, so no matching is needed on the tap
nets beyond keeping them short.

## 9. What the simulation covers and what it does not

Covered, from datasheets, with the conservative unknown-window model:
GAL propagation, setup, hold, clock-to-output and reset recovery ranges; SRAM
access, write pulse, setup and hold times and dual-port arbitration; delay
line tap tolerance including the uncertain-edge clocking of the copy chips;
reset sequence and r0 initialisation; every program in the test suite at
32 to 40 ns.

Not covered:

- Clock skew and trace delay. All margins above are quoted "minus skew"; the
  board has to meet section 4.1. A clock-buffer model with per-net delays is
  the planned next step so the simulator can check this from the layout.
- IDT7006 timing: modelled with the CY7C131-15 numbers (same family, same
  nominal grade). Confirm tSA, tHA, tPWE, tSCE, tSD, tHD, tACE, tOHA and the
  arbitration parameters from the 7006 datasheet and update
  `cy7c131::Timing` before layout.
- Physical pin numbers of the 16K dual-port (PLCC-68): `netlist::dp16k_pin`
  is a logical map; replace it with the real one for the netlist export.
- Oscillator duty cycle: the simulator uses exactly 50 %.

## 10. PCB checklist

- [ ] Clock buffer chosen; skew spec recorded; tree drawn with which output
      feeds which chips (section 4.1 item 2).
- [ ] Enable nets of both SRAM write ports length-matched to their GAL clock
      nets; SRAM side never longer.
- [ ] Jumper for the GAL clock-tree tap (0 / 6 / 12 ns).
- [ ] Oscillator socket; 45/55 duty or divide-by-two.
- [ ] Reset supervisor plus one-flop synchroniser on CLK.
- [ ] DS1100-30 with local decoupling; T1 and T3 to the copy chips.
- [ ] Data memory write-port A13 and register-file write-port A5 wired to the
      write flags (park addresses), read-port A13 grounded.
- [ ] BUSY and INT pins of every dual-port pulled up.
- [ ] IDT7006 datasheet numbers entered and simulation re-run.
- [ ] Bench: scope CLK at one SRAM enable pin and one GAL clock pin of each
      group in section 4.1, record skew and edge rate, set the tap jumper.
