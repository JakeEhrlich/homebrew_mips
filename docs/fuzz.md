# Physical fuzz

The simulation is exact and the board is not.  `tests/fuzz.rs` runs the
soak programs with the things a board adds, drawn at random from a
seed, so that a margin the ideal model hides is found before a PCB is:

| Knob | What it models | Default in the test |
|---|---|---|
| `pin_delay_ns` | a propagation delay on every chip pin, drawn from 0 to the maximum: trace length, connector, input loading.  Per pin, so the clock reaches every chip at its own time and skew is real | 0.3 ns (`FUZZ_DELAY_NS`) |
| `gate_tap` | the write gate's delay line and tap | the DS1100-40's 8 ns (`FUZZ_GATE_TAP`) |
| `clock_duty` | the clock's duty cycle, drawn per cycle from the range | 0.49 to 0.51 (`FUZZ_DUTY`): a divide-by-two flop's |
| `clock_jitter_ns` | up to this much added to every edge, independently | 0.3 ns (`FUZZ_JITTER_NS`) |
| `fuzz_seed` | random power-up contents in the data memory and the register file (the reference simulator gets the same image, `cpu::fuzz_image`) | on |
| `grade` | the delay lines' tolerance: room (binned, +-2 ns), commercial (+-3 ns), industrial (+-4 ns) | room (`FUZZ_GRADE`) |
| period | the clock period | 34 ns (`FUZZ_PERIOD_NS`) |

Everything else the model already covers conservatively: every GAL,
SRAM and delay-line parameter is a datasheet min/max window inside which
the signal is unknown, and any capture of an unknown is a failure.

## Findings

Each of these was invisible to the ideal simulation and found by the
fuzz on its first runs (2026-09-07).

**1. The clock's duty cycle sets the width of the data-memory write
pulse.**  The pulse is a delayed copy of the clock's high half gated by
the store flag, so a 45 % clock shortens it below the CY7C1041G's 7 ns
minimum (the model reports a 5 ns pulse) and the write is lost.  The
oscillator must not drive the clock directly: divide a 2x oscillator by
two, which gives 50 % within a flop's own asymmetry.  The fuzz's duty
range is that of a divided clock.  `docs/memory-timing.md` section 4
already listed this as a knob; it is a requirement.

**2. The write gate can glitch at the end of a store.**  The 8 ns tap of
a commercial-grade DS1100-40 may rise anywhere from 5 to 11 ns after
the edge, and the store flag it is gated with settles between 2 and
5.5 ns.  In the overlap the NAND can produce a runt low pulse on WE#
while the memory is selected: the model marks the addressed cell
unknown, and that unknown then walks through a load into the register
file and every dependent instruction.  The ideal simulation never saw
it because both events fell on the same instant and were evaluated
together; 10 ps of jitter separates them.  With the delay line binned to
+-2 ns the window starts at 6 ns and the overlap is gone.  Binning the
delay lines (and the gate) was planned for margin; it is now required
for correctness, or the gating must change.

**3. Trace delays.**  With binned delay lines and a divided clock at
34 ns, the design tolerates 0.5 ns of delay on every pin, drawn
independently (so up to 0.5 ns of clock skew between any two chips on
top of the data paths), and fails at 0.75 ns.  The first path to give
way is the EX stage: the ALU's carry chain into the EX/MEM result
register (setup violations on the `mr` chips), which shares the ID
stage's 2.5 ns of slack.  Three hops of delay and the clock skew
between the register that starts the path and the one that ends it
consume it.

| Period | Clock | Delay lines | Gate tap | Max pin delay | Result |
|---|---|---|---|---|---|
| 34 ns | divided | binned | 8 ns | 0.5 ns | pass |
| 34 ns | divided | binned | 8 ns | 0.75 ns | fail: EX/MEM result setup, then everything |
| 40 ns | divided | binned | 8 ns | 1.0 ns | one of two programs fails: write starts before the address arrives |
| 40 ns | 45..55 % | commercial | 8 ns | 2.0 ns | fail: the same |
| 40 ns | 45..55 % | commercial | 10 ns (DS1100-50) | 1.0 ns | fail: the same |
| 40 ns | 45..55 % | commercial | 10 ns (DS1100-50) | 2.0 ns | fail: the same |

(An earlier version of this table reported twice these delays: the
fuzz's generator drew from half its range.)

**4. Slowing the clock does not buy trace-delay margin on the write
path.**  At 40 ns the ID and EX stages have 8.5 ns of slack and never
fail; what fails is the data-memory write, and its margins do not
scale with the period.  The write pulse lives between "address valid"
(5.5 ns after the edge plus the address's own delay) and "address
changes" (2 ns after the next edge), its start is the tap's earliest
rise plus the gate's minimum, and its width is the clock's high half
minus twice the tap tolerance minus the gate's 5 ns spread.  The start
has zero margin against the address by design, so the first time the
address's pin delay exceeds the strobe's, the write begins on the old
address; and a 45 % clock at commercial grade leaves a 7 ns pulse with
nothing to spare.  A DS1100-50's 10 ns tap does not help: its
commercial tolerance is 8 %, +-4 ns, so its earliest rise is 6 ns, no
later than the -40's.

What would: bin the delay lines and the gate (each nanosecond of
tolerance removed is a nanosecond of pulse or of start margin), take
the clock from a divider, and, to be free of the duty cycle, end the
pulse on a second tap rather than the clock's fall (U1 to U3 through a
three-input NAND such as the 74LVC1G10): a 16 ns window minus the
tolerances, at any duty.  That change is not built; it is the next
thing to model if the write path's skew budget needs to reach a
connector.

## Running it

```
cargo test --release --test fuzz                       # 4 programs at the defaults
FUZZ_PROGRAMS=20 FUZZ_SEED=7 FUZZ_DELAY_NS=0.6 cargo test --release --test fuzz
FUZZ_GRADE=commercial FUZZ_JITTER_NS=0 cargo test --release --test fuzz
```

A failure names the seed and the knob values, and the first warnings
say which chip captured what.  `Build` carries the same knobs for any
test, and `pin_delays` puts a delay on chosen pins: `docs/slack.md` uses
it to measure each backward bus on its own.
