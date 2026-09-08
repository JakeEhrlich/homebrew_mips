# Back-edge slack

The pipeline runs left to right on the board, so the signals that run
right to left, from a later stage to an earlier one, are the long traces:
forwarding, write-back, branch resolution, the stall.  This is how much
delay each of them can carry, one bus at a time, measured on the model.

`mips32 slack crag` finds every net driven by a chip in one stage and
read by a chip in an earlier one, from the board file, and for each such
bus bisects the largest delay on its backward-listening pins for which
two soak programs still run clean, with ideal wires everywhere else, a
50 % clock, no jitter and binned delay lines, at 34 ns.  `ALL` puts the
same delay on every backward pin at once: the board where all the back
traces are the same length.  Lengths are at 6.5 ps per mm, an outer-layer
trace on FR4, before loading.

## At 34 ns

| Bus | Nets | From | To | Max delay | About | Sets it |
|---|---|---|---|---|---|---|
| MR | 32 | EX/MEM result | Forward A, Forward B | 2.5 ns | 385 mm | branch resolution (below) |
| WD | 32 | MEM/WB | Forward A/B, ID/EX A/B/store data, register file write port | 2.5 ns | 385 mm | branch resolution |
| SELBT, SELINC, KILL | 1 each | Next PC | PC, IF/ID | 2.5 ns | 385 mm | branch resolution |
| HOLD | 1 | Stall | PC, IF/ID, ID/EX control | 2.5 ns | 385 mm | narrow-store hold (below) |
| WDEST, WREG | 5, 1 | MEM/WB | Steer | 2.5 ns | 385 mm | steer to register-file enable to ID/EX |
| WDESTC, WREGC_n | 5, 1 | Write copies | Register file | 3.5 ns | 538 mm | write address before the copy's write pulse |
| DQ | 4 | data bus | Access size | 7.5 ns | 1150 mm | |
| FA | 14 | Forward A | PC (jump register), boot sequencer | over 8 ns | over 1200 mm | |
| SELJT, SELAF | 1 each | Next PC | PC | over 8 ns | | decided by ID/EX registers, not the compare |
| XBT, IR | 13 each | ID/EX target, IF/ID | PC | over 8 ns | | register to register |
| MDEST, MRW, XDEST, XRW, XMW | | EX/MEM and ID/EX control | forwarding control, narrow-store hold, boot | over 8 ns | | |
| **ALL** | 139 | every backward edge | at once | **1.25 ns** | **190 mm** | branch resolution, crossed twice |

The first failure in every 2.5 ns row is the same: a setup violation at
the PC register 3.25 ns before the edge.  Two paths are three GAL levels
deep between registers and share the 2.5 ns of slack the stages have at
34 ns:

- **Branch resolution.**  EX/MEM result (5.5 ns after the edge), forward
  mux (7.5), compare (7.5), next-PC (7.5), PC setup (3.5): 31.5 ns.  It
  crosses a backward edge twice, once as MR or WD into the forward mux
  and once as SELBT, SELINC or KILL back into PC and IF/ID.  That is why
  `ALL` is half of the single-bus number: two back traces of equal
  length each take half the slack.
- **Narrow-store hold.**  Instruction register (5.5), decode (7.5),
  narrow-store hold (7.5), stall (7.5), PC setup (3.5): the same 31.5,
  and HOLD fans out to 14 pins in three blocks.
- **Steer.**  MEM/WB destination (5.5), steer (7.5), register-file
  enable to output, ID/EX setup: the ID stage's critical path, entered
  through WDEST and WREG.

Everything else has a register at both ends or one level between and
does not care about length at this clock.

## What it means for the layout

- A back trace's budget is not its own.  It is the slack of the path it
  is on, shared with that path's forward hops and with the clock skew
  between the registers at its ends.  At 34 ns that is 2.5 ns for the
  branch, hold and steer paths, 3.5 ns for the write copies, and
  effectively unlimited for the rest.
- 100 mm is 0.65 ns.  Back traces of 100 to 150 mm on MR, WD, the
  selects, HOLD, WDEST and WREG are affordable at 34 ns only if the
  forward hops on the same paths (forward mux to compare, compare to
  next-PC, decode to narrow-store hold to stall) are short and the
  clock skew between EX/MEM, EX and IF is small: with two 150 mm back
  traces on the branch path, 1.95 ns of the 2.5 are gone.
- Route MR and WD to the forward muxes, and SELBT, SELINC, KILL and
  HOLD to PC and IF/ID, before anything else; XBT, IR, FA, the control
  buses and the write-back destination copies can take the long way.
- The write copies (WDESTC, WREGC_n) do not scale with the clock: their
  3.5 ns is the gap between the copy's address and its write pulse from
  the delay-line taps.  Keep the write-copy chips next to the register
  file.
- Air is faster than FR4 (about 3.5 ps per mm for a wire a few
  millimetres above the board against 6.5 in the substrate), so an
  overhead wire roughly halves a back trace's delay.  A 2 ns longer
  period gives the branch path more than that on every back trace at
  once (next section), so the wire is not worth it.

## Longer periods

The 2.5 ns is clock period and grows with it; the write copies' 3.5 ns
does not.

| Period | MR alone | SELBT alone | All back edges at once | About, all at once |
|---|---|---|---|---|
| 34 ns | 2.5 ns | 2.5 ns | 1.25 ns | 190 mm |
| 36 ns | 4.5 ns | 4.5 ns | 2.25 ns | 345 mm |
| 40 ns | 8.5 ns | 8.5 ns | 4.25 ns | 650 mm |

At 36 ns every back trace on the board can be 300 mm with nothing else
counted; at 40 ns the back traces stop being the question and the
data-memory write path (docs/fuzz.md finding 4) is what is left.

## Not in the model

Loading.  The GAL timings are datasheet numbers at the test load; each
input on a net adds a few picofarads and the wide back buses have 7 to 9
loads (WD 9, MR 7, HOLD 14).  The model has no per-net capacitance yet,
so the lengths above are optimistic by that amount; the trace-delay
knob is the place to charge it.

## Running it

```
mips32 slack crag                # every backward bus, 0..8 ns, 0.25 ns steps, 2 soak programs
mips32 slack crag 12 1           # up to 12 ns, one program
SLACK_BUS=ALL,MR SLACK_PERIOD_NS=40 mips32 slack crag
SLACK_VERBOSE=1 ...              # every check, with its time
```

About a minute per bus per program; buses run in parallel.  The table
is markdown on stdout, progress on stderr.  `tests/board.rs` checks that
the backward-edge finder still sees the known set.
