# Memory timing: how the two SRAMs are written, and what the PCB must guarantee

This is the reference for the board design. Every number here is either a
datasheet value or a margin the simulator computes from datasheet values; the
places where the simulator cannot see the board (clock skew, trace delay) are
called out as PCB requirements with the margin they must fit into.

Status: simulated clean with every CPU test program (`cargo test --release`,
`tests/memory_timing.rs`, `tests/cpu.rs`) at commercial-grade delay-line
tolerance (0..70 C) and the 74LVC1G00Q's full 0.5 to 5.5 ns range: from
33 ns with the CY7C1041G-10 data SRAM, from 37 ns with the IS61C256AH-12,
from 38 ns with the AS7C164A-15. The register file is clean from 32 ns.

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

Register file: the write port's chip enable is driven directly by CLK. The
write is the clock's low half (17 ns) and ends exactly at the rising edge. The
port's address and write flag come from a copy register clocked by a
delay-line tap, so they are valid before the low half starts and hold until
well after the edge that ends the write; the data comes from the ordinary
edge-clocked write-back register (zero data hold). Reads use the other port.

Data memory (single-port): the chip stays selected, so reads hold their data
past the edge. The write is a pulse on WE# made from one delay-line tap ANDed
with the store flag in a single fast gate, during the store's own MEM cycle,
so the address is the EX/MEM result register as before and the pulse ends a
few ns before the edge that changes it. The memory's output enable is a
registered signal raised a cycle early so the bus is free when the store-data
drivers turn on. Two one-cycle interlocks in ID (a load right behind a store,
a store right behind a load) keep loads away from the cycles where the outputs
are off.

## 3. Parts

| Function | Part | Notes |
|---|---|---|
| Register file | 8 x CY7C131-15 (1K x 8 dual-port) | unchanged |
| Instruction memory | 4 x IS61C64AL-10 (8K x 8, 5 V, 10 ns) | read only, no strobes; fetch data at 15.5 ns instead of 20.5. Timing modelled with the IS61C256AH-10 column (same ISSI family); confirm against the 61C64AL datasheet |
| Data memory | 2 x CY7C1041GN-10 (256K x 16, 5 V, 10 ns, TSOP II-44) | 1 MB; byte enables BHE# / BLE# driven by the access-size chip (section 6.4). Datasheet 001-91368 saved as `docs/CY7C1041G_datasheet.pdf`; timing from its 10 ns column. Alternatives in the simulator: 4 x IS61C256AH-12 (33 ns), 4 x AS7C164A-15 (37 ns) |
| Delay lines | DS1100-30 (taps 6, 12, 18, 24, 30 ns) for the register-file copies; DS1100-40 (8, 16, 24, 32, 40 ns) tap 1 for the write gate | **new** |
| Write gate | 1 x Diodes 74LVC1G00Q (2-input NAND, SOT-25 / SOT-353) | **new**; 0.5 to 5.5 ns at 5 V over -40..+125 C (datasheet June 2020), which is what the simulator uses |
| Reset supervisor | 1 x MAX811LEUS+T (SOT-143, 4.63 V threshold, 140 ms minimum timeout, debounced MR# input) | **new**; reset button from MR# to ground. The 4.75 V minimum of the GALs and delay lines sits above the threshold: that only matters for a brownout that stalls between 4.63 and 4.75 V, which ends in a hang, not damage. Set the 5 V rail to about 5.1 V and sense the supervisor at the far end of the plane |
| Boot ROM | 4 x SST39SF040 class (512K x 8, 5 V) | **new**; see docs/boot.md |
| UART | 1 x TL16C550D (LQFP-48) with SP3232 RS-232 transceiver and 14.7456 MHz crystal | **new**; see docs/uart.md |
| Logic | 160 x ATF22V10C-7 | was 132; +2 write copies, +1 stall / output enable, +1 reset synchroniser, +9 boot copier, -1 forwarding control repack, +1 hold / bubble, +1 bus wait sequencer, +6 from the wait's hold inputs (forwarding control +2, operand B +1, branch target +1, stall +1, MEM/WB +1), -1 after the board-file completeness check found ten dead ID/EX outputs (the rs / rt numbers) and three dead branch-adder intermediates, +10 byte / halfword access (section 6.4) |

Total 182 chips plus the gate and the RS-232 transceiver.  The
authoritative list is `boards/crag/netlist.json` (see boards/README.md).

## 4. The clock

- Period 34 ns nominal (clean down to 32 in simulation). Oscillator socketed.
- Duty cycle matters now: the falling edge is the write start, and it must be
  at least 14 ns after the rising edge for the register file steer (section 5).
  Use a 45/55 % or better oscillator, or an exact divide-by-two from a doubled
  oscillator in a GAL.
- The clock drives every GAL clock pin, the eight register-file write-port
  enables and the two delay-line inputs.

### 4.1 Skew budget

Every hold-type margin in the machine has the form "N ns minus clock skew",
where skew is the arrival difference between the clock at the relevant SRAM
enable pin and at the GAL clock pins whose outputs it races. N is:

| Race | N | Where |
|---|---|---|
| GAL to GAL hold (tCO min 2, tH 0), everywhere | 2 | whole pipeline, unchanged |
| Register-file data hold (tHD 0): WD changes 2 ns after the edge | 2 | MEM/WB chips vs register-file CE |
| Register-file address hold (tHA 2): copies change 5 ns after the edge at commercial grade | 3 | copy chips vs register-file CE |
| Register-file write start after steer (steer settled at 13, write starts at 17) | 4 | steer chip vs register-file CE (helped by an early SRAM clock) |
| Data memory write end (gate) before MR changes at 36 | 3.5 | delay-line input vs EX/MEM chips |
| Data memory read hold (tOHA 2 after the address changes at 36) | 2 | EX/MEM chips vs MEM/WB capture |

The board must keep skew under about 1 ns between the register-file enable
pins, the delay-line input, and the GAL clock pins named above. Concretely:

1. One clock buffer family with a specified output-to-output skew (0.25 to
   0.5 ns class, e.g. a 74FCT3807-type 1:10 driver at 5 V).
2. The buffer output that clocks the MEM/WB chips also drives the eight
   register-file write-port enables; the output that clocks the EX/MEM chips
   also feeds the two delay lines. Only intra-device skew then applies to the
   races above.
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

Two 256K x 16 single-port SRAMs (dmem0 = bits 15:0, dmem1 = bits 31:16),
selected except during boot code and I/O, byte enables low except for the
lanes a narrow store leaves alone. Pins:

| Pin | Net | Source |
|---|---|---|
| A0-17 | MR[19:2] | EX/MEM result: load address, or store address during the store's MEM cycle |
| I/O0-15 | DQ[15:0] / DQ[31:16] | driven by the SRAMs during loads (and whenever OE# is low), by the EX/MEM store-data drivers (chips msd*) during a store's MEM cycle |
| CE# | DMEN_n | high during the boot code phase and an I/O access |
| BLE#, BHE# | MBE0_n, MBE1_n (dmem0); MBE2_n, MBE3_n (dmem1) | registered at the EX/MEM edge with the address (chip macc0): high for the lanes a byte or halfword store does not write, low otherwise (they act like CE per byte: tDBE 4.5, tHZBE 6, tBW 7) |
| OE# | OEN | registered (chip stl0): high during a store's EX cycle, its MEM cycle and the cycle after |
| WE# | WEN | the gate: NAND(U1, MMW), U1 = CLK delayed 8 ns (DS1100-40 tap 1), MMW = store in MEM |

### 6.1 The write pulse

Nominal: WE# falls at 8 ns plus the gate delay, rises at 25 ns (the tap's
falling edge, half a period after its rising edge) plus the gate delay.
Windows at commercial grade (tap +-3 ns, gate 0.5 to 5.5 ns), 34 ns period:

| Event | Window | Constraint | Margin |
|---|---|---|---|
| Write start | 5.5 to 16.5 | address (MR) valid at 5.5, tAS 0; data (msd) valid at 5.5 | 0 (the simulator accepts coincidence; an early SRAM-side clock helps here) |
| Write end | 22.5 to 33.5 | before MR changes at 36 (tHA 0) | 2.5 - skew |
| Pulse width | >= 22.5 - 16.5 = 6 at 34 ns; grows by half the period increase | tPWE 7 (CY7C1041G-10), 8 (IS61C256AH-12), 10 (AS7C164A-15) | the simulator's model of the pulse is finer than this arithmetic (the two edges share the tap's drift direction is *not* assumed; the write-end check uses the SRAM's own rules): clean from 33 / 37 / 38 ns |
| Data setup to write end | msd valid 5.5, end >= 22.5 | tSD 5 / 7 / 8 | 12 |

A gate with a narrower delay range buys clock period directly: every
nanosecond of range costs half a nanosecond of pulse.

The pulse width is the binding constraint and the reason the two parts have
different operating points: half a period minus twice the tap tolerance minus
the gate's delay range must exceed the part's minimum pulse.

### 6.2 Reads and bus turnaround

Loads: address at 5.5, data at 15.5 (10 ns part), captured at the edge with
3.5 ns setup: 15 ns of margin. Data holds 3 ns after the address changes at
36 (tOHA): 2 - skew, the machine's standard hold margin.

Around a store the bus changes hands once each way, and both handovers are
kept a full cycle apart from any read:

- OEN goes high at 2 to 5.5 ns into the store's EX cycle (it is registered
  from the decode of a store in ID), so the SRAM outputs are off by 12.5 ns
  of that cycle, long before the store-data drivers turn on at 5 to 13 ns of
  the MEM cycle.
- OEN stays high through the cycle after the MEM cycle, so the outputs come
  back only at 2 to 12.5 ns of the second cycle after the store, by which
  time the drivers have been off for a full cycle.

Being registered, OEN cannot glitch at the EX-to-MEM handover; a
combinational OR of the two store flags did, and the simulator caught it as a
bus conflict.

### 6.3 Interlocks

A load in ID right behind a store in EX, or a store in ID right behind a load
in EX, is held in ID for one cycle (`cpu::stall_block`): PC and IF/ID keep
their values and ID/EX takes a bubble, while EX, MEM and WB proceed. The
first case keeps the load out of the cycle where the outputs are still off;
the second lets the load finish before OEN goes high for the store. Cost:
one cycle per adjacent load/store pair; instructions that separate them avoid
it. One consequence: a store in a load's delay slot that stores the loaded
register sees the new value rather than the old one. MIPS I leaves that read
undefined and the test programs avoid it.

This is also the machinery the bus wait (docs/uart.md) and the narrow-store
hold below reuse.

### 6.4 Byte and halfword access

LB, LBU, LH, LHU, SB, SH.  Little-endian lanes: byte a of a word is DQ
bits 8a+7..8a.  Ten GALs (150 -> 160): +1 ID/EX store data, +2 EX/MEM
store data, +4 MEM/WB, +1 access size (macc0), +1 load lane select
(msel0), +1 narrow-store hold (sf0).

**Stores replicate, the byte enables select.**  A byte store puts its byte
on all four lanes and a halfword store its halfword on both halves, so no
lane ever has to be chosen from the address on the data path.  Lane 1 is
filled in ID/EX (`XSD[15:8]` = the low byte when SZB), lanes 2 and 3 by
the EX/MEM drivers from lanes 0 and 1 (`XNAR`).  The byte enables
`MBE[3:0]_n` are registered at the EX/MEM edge from the ALU's group-0
sum bits (the address bits 1:0), the size and the store flag, so they
have exactly the address's timing (valid 2 to 5.5 ns into the MEM
cycle, held until 2 ns after the next edge): the same margins as CE#
against tBW = 7 ns and the write pulse.  A lane whose enable is high is
deselected for the whole pulse.  Loads, boot writes and I/O keep all
four low.

**Loads pick the lane in MEM/WB.**  The write-back register's D-terms
take, per byte: bits 7:0 from any of the four DQ bytes (one-hot selects
`ML0..3`), bits 15:8 from their own lane, the high half's low byte or
the sign, bits 31:16 from their own lane or the sign.  The selects
(msel0) are combinational in MEM from the registered MEM-stage copies
of size / sign / address bits 1:0 (`MSZB`, `MSZH`, `MLSX`, `MA0`,
`MA1`, chip macc0) and fold the bus wait: with no select active the
register recirculates.  The sign, `LSGN`, is the selected element's top
bit, muxed once in macc0 and shared by the 24 upper bits: its path is
DQ valid (15.5 ns) + 7.5 + 3.5 setup = 26.5 ns, 7.5 ns of margin (a
GAL's setup time is measured from the input pin, so the register's own
array is not a second level).  An I/O access is forced to word, lane 0
(the copies are gated by FA31), so a narrow load of the UART returns
its byte zero-extended whatever the address.

**A narrow store whose data would be forwarded waits.**  Replication
happens on the register-file path only; a value arriving from EX/MEM or
MEM/WB is a plain word.  So a byte or halfword store in ID that names,
as rt, the destination of the instruction in EX or in MEM is held there
(`SFE`, `SFM`, chip sf0: the fwdc comparison, combinational) until the
producer reaches WB and the steer supplies the value: one or two cycles,
only for that pattern.  The alternative, replicating in the EX/MEM
drivers, costs eight more chips (nine data inputs per lane-3 bit).
Timing of the hold: IR 5.5 + NSTORE decode 7.5 + SFE 7.5 + HOLDW 7.5 +
setup 3.5 = 31.5 ns at 34.

Unaligned addresses are undefined (the low address bits select the lane
regardless; nothing traps).  The reference simulator does the same.
Test: `tests/cpu.rs` `bytes_and_halfwords` (every alignment, signed and
unsigned, the three forward distances, a store behind a load).

## 7. Reset

Modelled as chips now, not stimulus:

- **Supervisor** (`rst0`, MAX811LEUS+T, active-low RESET# on pin 2): holds
  RST_n low from power-on for its reset timeout (140 ms minimum, starting
  once the rail passes 4.63 V), then releases at an instant unrelated to
  the clock. The timeout only has to exceed the oscillator's start-up
  time. The rail itself reaches 5 V a few milliseconds after the regulator
  starts (the decoupling is some hundreds of microfarads, charged at the
  regulator's current-limit surplus), so give the regulator headroom over
  the board's 10 A rather than sizing it at the load. MR# (pin 3) is the
  reset button: debounced in the part, RESET# low while pressed and for
  another timeout after release. The model takes MR# as an input (wired
  high) and folds the post-release timeout into `release`; a manual reset
  restarts the boot copier, so its length is immaterial.
- **Reset asserted mid-run** (the button) is the one case where an
  asynchronous clear happens while clocks are running. The pipeline
  registers clear about 10 ns after RESET rises, 15 ns into the cycle,
  which is inside the T3 window of the first write-copy chip (wc1 reads
  MDEST and MRW). So the EX/MEM control chips (`mctl`) reset
  synchronously instead: every term is gated with `!RESET`, and their
  outputs only ever move at a CLK edge. The write-back stage sees the
  clear one cycle later, which the reset sequence does not care about.
  Every other AR-cleared register feeds CLK-clocked chips only, where the
  clear has 18 ns of setup to the next edge.
- **Test**: `tests/boot.rs` `reset_button_recopies_and_reruns` presses
  the button at three phases of a cycle while the program is running,
  checks no chip warns through the reset, that both memories are
  recopied (a data word the program had overwritten is restored, one
  outside the copied regions is kept), and that the program runs again
  to the reference result.
- **Synchroniser** (`rsync0`, one GAL on CLK, no async reset of its own):
  two registers in series, RST_n -> RS1 -> RESET. Both are active-low
  outputs of registers that the ATF22V10C clears at power-up, so RESET is
  asserted before the first clock edge without any help. RESET falls 2 to
  5.5 ns after a clock edge, one or two cycles after the supervisor
  releases. Every other register with a reset uses RESET asynchronously,
  and its recovery (release + AR path + 5 ns) precedes the next edge with
  about 13 ns to spare at 34 ns. The tap-clocked copies take RESET as
  synchronous data (`WC1W_n = !MRW & !RESET`), setup margin 6 ns.
- **The one non-datasheet assumption**, flagged in the model
  (`gal22v10::OlmcConfig::sync`, `Eq::sync`): the first synchroniser stage,
  when RST_n moves inside its setup window, captures one of its two legal
  values rather than an unknown, settling within the clock-to-output time.
  That is what a flip-flop's metastability MTBF certifies, and it is the
  reason a two-stage synchroniser is used at all. The model takes the old
  value, which is the pessimistic outcome (release one edge later).
- **Test**: `tests/reset.rs` sweeps the supervisor's release across every
  nanosecond of a clock period and checks, for each phase, that RESET
  falls 2 to 5.5 ns after an edge, that no chip warns during or after
  reset, and that a program runs with r0 = 0 and correct results.
- r0: while RESET is held the MEM/WB destination is 0, the data is 0, and
  the write flag reads "write" (polarity chosen for that), so the register
  file's r0 is written with zero on every clock during reset. The simulator
  preloads r0 with garbage to prove it.

## 8. Delay line

Two DS1100 (8-pin DIP, 5 V), both fed by CLK. DS1100-30: taps T1 (6 ns) and
T3 (18 ns) clock the register-file copies. DS1100-40: tap U1 (8 ns) is the
write gate's timing input; its rising edge starts the data-memory write
pulse and its falling edge ends it.
Tolerance per tap: +-2 ns at 25 C, +-3 ns over 0..70 C, +-4 ns over -40..85 C.
Both are modelled as independent unknown windows, which is more pessimistic
than the part (all taps drift together). Datasheet note 9: "at or near
maximum frequency the delay accuracy can vary and will be application
sensitive (decoupling, layout)". The input pulse width (17 ns) is far above
the 6 ns minimum. Give it its own decoupling capacitor and a short, clean
input trace from the clock buffer.

T1 and T3 each clock one GAL (the copy chips); U1 feeds one gate input. The
margins in sections 5 and 6 include the full tap tolerance, so no matching is
needed on the tap nets beyond keeping them short.

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
- The gate: Diodes 74LVC1G00Q, 0.5 to 5.5 ns at 5 V from its datasheet
  (`Build::gate_tpd`).  A different vendor's part needs its own numbers.
- Oscillator duty cycle: the simulator uses exactly 50 %. The data-memory
  write pulse is half a period wide before tolerances, so a short high half
  eats directly into it (see 4).

## 10. PCB checklist

- [ ] Clock buffer chosen; skew spec recorded; tree drawn with which output
      feeds which chips (section 4.1 item 2).
- [ ] Enable nets of both SRAM write ports length-matched to their GAL clock
      nets; SRAM side never longer.
- [ ] Jumper for the GAL clock-tree tap (0 / 6 / 12 ns).
- [ ] Oscillator socket; 45/55 duty or divide-by-two.
- [ ] MAX811LEUS+T: RESET# (pin 2) into the rsync GAL's RST_n pin, MR#
      (pin 3) to the reset button and a 0.1 uF decoupling capacitor at VCC;
      no pull-up needed on MR# (internal). Sense point at the far end of
      the 5 V plane from the regulator; rail trimmed to about 5.1 V.
- [ ] DS1100-30 and DS1100-40 with local decoupling; T1 / T3 to the copy
      chips, U1 to the gate.
- [ ] Gate placed next to the delay line and the SRAMs; WEN to all four WE#.
- [ ] Register-file write-port A5 wired to the write flag copy (park address).
- [ ] BUSY and INT pins of the register-file chips pulled up.
- [ ] Data SRAM byte enables from macc0 (MBE0_n / MBE1_n to dmem0's BLE# /
      BHE#, MBE2_n / MBE3_n to dmem1's); same length as the address lines.
- [ ] IS61C64AL datasheet numbers entered and simulation re-run.
- [ ] Bench: scope CLK at one SRAM enable pin and one GAL clock pin of each
      group in section 4.1, record skew and edge rate, set the tap jumper.
