# grit: the first board (proposal)

The smallest machine that runs real code out of the parts chosen for
crag: ATF22V10C GALs, SST39SF040 flash, AS7C164A 8K x 8 SRAM, the
TL16C550 and SP3232 serial pair, the MAX811L.  Its purpose is to learn
the physical side: programming GALs from the manifest, executing out of
a socketed flash, SRAM strobes from GAL logic, the serial link, a board
through JLCPCB, and above all to be debuggable: one thing per clock,
every control line on a pin, every table in a socket.

Status: modelled and simulated (`src/grit.rs`, `tests/grit.rs`,
`boards/grit/netlist.json`).  Every GAL fits, and programs run on the
netlist with no timing complaint from any chip, through reset from any
clock phase, the reset button, and the UART both ways.

## 1. The idea in one paragraph

A classic microprogrammed machine.  A small instruction set (three
registers A, B, PC; loads, stores, add, and, nor, jumps) lives in the
program flash.  Each instruction is executed by a sequence of
microwords read from a microcode ROM, the other pair of flash chips,
addressed by the instruction's opcode, a step counter and the ALU's
equal flag.  A microword is one bit per control line, latched into a
pipeline register at every clock edge, so every control line changes
only at edges and each clock does one transfer that settles before the
next.  Hold times and bus turnarounds are met by the order of the
microwords, not by strobes.  16 GALs and 10 other chips; the program,
the microcode and every GAL are reprogrammable in their sockets.

## 2. Programmer's view

**Word.**  16 bits.  A, B and the PC are 16 bits; addresses are 16-bit
word addresses (address n is the n-th 16-bit word, the PC counts by
one, an immediate is the next word).  No byte operations.

**Registers.**  A is the address register and the ALU's first operand:
its outputs are the address bus whenever the PC is not fetching.  B is
the second operand and the value a store writes.  The PC addresses the
program flash.  A third latch T is not in the programmer's model:
memory data bound for A or the PC lands there first, so that no
register ever changes the address bus at the edge that ends a strobe;
every instruction may clobber it.

**Instructions.**  One word; the opcode is bits 15:11.  Immediates are
the following word.

| Op | Instruction | Does | Clocks |
|---|---|---|---|
| 0 | RESET | fetch from the PC (what runs out of reset, PC = 0) | 2 |
| 1 | LDA imm | A = next word | 6 |
| 2 | LDB imm | B = next word | 6 |
| 3 | LDA (A) | A = mem[A] | 9 |
| 4 | LDB (A) | B = mem[A] | 7 |
| 5 | STB (A) | mem[A] = B | 8 |
| 6 | ADDA | A = A + B | 7 |
| 7 | ADDB | B = A + B | 7 |
| 8 | ANDA | A = A and B | 7 |
| 9 | ANDB | B = A and B | 7 |
| 10 | NORA | A = not (A or B) | 7 |
| 11 | NORB | B = not (A or B) | 7 |
| 12 | MOVAB | A = B | 7 |
| 13 | JMP imm | PC = next word | 8 |
| 14 | JEQ imm | if A == B then PC = next word, else skip it | 10 or 6 |
| 15 | NOP | | 3 |
| 16 | HALT | never fetches; the step counter shows it | |

Any load may read the UART; the bus treats it exactly as a memory.

Registers r0..r31 of the MIPS-like macro layer are SRAM words at
0x8000 + n.  `lw rt, (rs)` is `LDA &rs; LDA (A); LDB (A); LDA &rt;
STB (A)`; `addu rd, rs, rt` is `LDA &rt; LDB (A); LDA &rs; LDA (A);
ADDB; LDA &rd; STB (A)`; `beq rs, rt, target` is `LDA &rt; LDB (A);
LDA &rs; LDA (A); JEQ target`.  About 200 000 macro-instructions a
second at 4.6 MHz.

**Memory map.**

| Address | What |
|---|---|
| 0x0000 .. 0x7FFF | program flash, 32 K words (64 KB of the 512 KB); A15..A18 on a 4-way jumper, so one chip pair holds 16 programs |
| 0x8000 .. 0x801F | registers r0 .. r31 |
| 0x8020 .. 0x9FFF | SRAM, the other 8 160 words (0xA000 .. 0xBFFF repeats it) |
| 0xC000 + n | TL16C550 register n on the low byte; the high byte of a read is garbage, mask it |

The microcode ROM is not in this space; it has its own wires.

**Reset.**  The MAX811L holds reset; the PC, IR and step counter clear;
opcode 0's microcode fetches from address 0.

## 3. The microcode

**Address**, 10 bits: {opcode[4:0], NEL, step[3:0]}.  NEL is the ALU's
"A differs from B", latched at the end of every word in which A is on
the bus (the ALU only sees A then) and held otherwise; a conditional
instruction is two microcode sequences and the hardware chooses between
them with that address bit.  The ROM's other address pins are jumpers:
microcode variants without touching the program.

**Word**, 16 bits, one per control line, eleven used:

| Bit | Line | For the whole clock |
|---|---|---|
| 0 | PCDRV | the PC drives Addr |
| 11 | ADRV | A drives Addr (and so the ALU sees A) |
| 12 | TLD | T loads D at the ending edge |
| 13 | TDRV | T drives D |
| 1 | MEMRD | the memory selected by Addr drives D (flash, SRAM or UART, by Addr's top bits) |
| 2 | ALUOE | the ALU drives D |
| 3 | WE | the memory selected by Addr takes D |
| 4 | ALD | A copies D at the ending edge |
| 5 | BLD | B copies D at the ending edge |
| 6 | PCLD | the PC copies D at the ending edge |
| 7 | PCINC | PC += 1 at the ending edge |
| 8 | IRLD | IR copies D[15:11] and the step counter clears at the ending edge: the fetch |
| 9, 10 | F0, F1 | ALU function: 00 add, 01 and, 10 nor, 11 pass B |
| 14, 15 | spare | debug outputs, a halt flag |

**Pipeline.**  In clock k the ROM presents the word for step k while the
pipeline register executes the word for step k-1.  So the word after a
fetch is always executed before the new instruction's step 0, and every
sequence ends `... PCDRV MEMRD IRLD; nop`.

**The ordering rules**, which replace all clock-phase logic (the
microcode generator checks them):

1. The driver of D never changes between consecutive words, and neither
   does the driver of Addr: an idle word sits between a MEMRD word and
   an ALUOE word, and between a PCDRV word and an ADRV word.  The
   outgoing chip gets a whole clock to float.  Nobody drives Addr in
   the idle word; pull-downs on Addr15 and Addr14 keep the selects
   defined (the flash, with its OE# high).
2. An access at A (MEMRD, WE or ALUOE with ADRV) is preceded by a word
   with only ADRV: the address, and the ALU's operand, are valid a clock
   before anything strobes or samples them.
3. A word that loads B, T or the IR from memory, or writes, is followed
   by the same word without the load: the address and the data outlive
   the capture by a clock, so no clock skew between the chips can turn
   the capture into a hold violation.  The word after a fetch is that
   hold word, `PCDRV MEMRD`, so an instruction that starts by taking
   the address bus from the PC begins with an idle word.
4. The one bus rule: a register is never loaded at the end of a strobe
   while it drives the address bus.  Memory data for A or the PC lands
   in T, and A or the PC takes it from T in a word with no strobe.  So
   every device on the bus, memory or UART, sees the address stable
   from a word before its strobe to a word after.
5. A conditional instruction's two variants differ only at steps whose
   ROM read happens while NEL is fresh: word 1 has ADRV (NEL latches at
   its end), word 2 has not, the variants differ from step 3.

**The sequences.**  Each line is one clock.

```
FETCH   = PCDRV MEMRD IRLD      HOLD = PCDRV MEMRD
RESET     FETCH ; HOLD
LDA imm   PCINC ; PCDRV MEMRD ALD ; PCDRV MEMRD ; PCINC ; FETCH ; HOLD
LDB imm   PCINC ; PCDRV MEMRD BLD ; PCDRV MEMRD ; PCINC ; FETCH ; HOLD
LDA (A)   nop ; ADRV ; ADRV MEMRD TLD ; ADRV MEMRD ; nop ; TDRV ALD ; PCINC ; FETCH ; HOLD
LDB (A)   nop ; ADRV ; ADRV MEMRD BLD ; ADRV MEMRD ; PCINC ; FETCH ; HOLD
STB (A)   nop ; ADRV ; ADRV ALUOE passB WE ; ADRV ALUOE passB ; nop ; PCINC ; FETCH ; HOLD
ADDA      nop ; ADRV ; ADRV ALUOE add ALD ; nop ; PCINC ; FETCH ; HOLD
ADDB      nop ; ADRV ; ADRV ALUOE add BLD ; nop ; PCINC ; FETCH ; HOLD
MOVAB     nop ; ADRV ; ADRV ALUOE passB ALD ; nop ; PCINC ; FETCH ; HOLD
JMP imm   PCINC ; PCDRV MEMRD TLD ; PCDRV MEMRD ; nop ; TDRV PCLD ; nop ; FETCH ; HOLD
JEQ imm   nop ; ADRV ; PCINC ; then NEL=0: PCDRV MEMRD TLD ; PCDRV MEMRD ; nop ; TDRV PCLD ; nop ; FETCH ; HOLD
                                 NEL=1: PCINC ; FETCH ; HOLD
NOP       PCINC ; FETCH ; HOLD
HALT      nop x 16 (the counter wraps and it repeats)
```

ANDA/ANDB/NORA/NORB are ADDA/ADDB with F changed.  Rules 2, 3 and 4
were each found by the model, not by thought: the first version turned
A onto the bus and started the write strobe in the same word, and the
SRAM saw its address change 8 ns into the write; the second passed the
soak but, under 3 ns of clock skew, a literal read into B lost its
data before B sampled it; the third held the strobe through the word
after a load into A, and the random programs then read the UART by
accident whenever the loaded value happened to be a UART address.

## 4. The chips

**Buses.**  D[15:0]: driven by the flash, SRAM, UART (low byte) or the
ALU, T, one per clock; read by A, B, T, the PC and the IR.  Addr[15:0]:
the A latch, or the PC when PCDRV.  The microcode ROM's address and
data are private wires from the sequencer and to the pipeline
register.

| Chip | Clock | Reset | Inputs | Outputs |
|---|---|---|---|---|
| Sequencer: IR/step | CLK | sync | D11..D15, IRLD | IR0..IR4 (opcode), STEP0..STEP3 |
| Sequencer: flags | CLK | sync | RST_n, ADRV, NE3 | RS1, RESET, NEL |
| MIR-low, MIR-high | CLK | sync | M0..M7, M8..M15 (the ROM's data) | the control lines, one per bit |
| PC-low | CLK | sync | D0..D7, PCLD, PCINC, PCDRV | Addr0..Addr7 (enabled by PCDRV), CO |
| PC-high | CLK | sync | D8..D15, CO, PCLD, PCDRV | Addr8..Addr15 (enabled by PCDRV) |
| A-low, A-high | CLK | none | D0..D7 / D8..D15, ALD, ADRV | Addr0..Addr7 / Addr8..Addr15, enabled by ADRV |
| B-low, B-high | CLK | none | D0..D7 / D8..D15, BLD | B0..B7 / B8..B15 |
| T-low, T-high | CLK | none | D0..D7 / D8..D15, TLD, TDRV | T0..T7 / T8..T15, on the data bus, enabled by TDRV |
| ALU 0, 1, 2, 3 | none | none | four bits of Addr (that is, of A) and of B, carry and NE in (slices 1..3), F1, F0, ALUOE | four sums (enabled by ALUOE), carry out, NE0..NE3 out; NE3 is A differs from B |

Sixteen GALs.  The latches are `Q := LD & D + !LD & Q` per bit; the PC
is a 16-bit counter (PCINC) with a load (PCLD wins); the step counter
clears on IRLD and counts otherwise; the pipeline register is `Q := M`;
the ALU slices are 4-bit ripple adders with the function folded into
each sum term and a not-equal chain beside the carry chain.  "sync"
reset: RESET is a factor of every product term rather than the 22V10's
asynchronous reset, so the register clears at the next edge and RESET
is an ordinary input with an ordinary setup time (the asynchronous
reset would race the very clock edge RESET itself comes from).  RESET
is high at power-up (a zero register behind an active-low pin).  The
flags chip is the sequencer's second GAL: the two-stage reset
synchroniser (RS1, then RESET) and NEL, the condition, which is NE3
latched at the end of every word in which A drives the bus and held
otherwise.  The ALU is purely combinational, and no GAL makes a clock.

| Net | Driven by | Read by | Meaning |
|---|---|---|---|
| CLK | oscillator | every GAL | 4 MHz; every flop is on its rising edge |
| XIN, XOUT | crystal | UART | 1.8432 MHz, the UART's own clock; divisor 1 is 115200 baud |
| RST_n | MAX811L | flags | reset, active low |
| RESET | flags | IR/step, MIR, PC, NEL, UART MR | reset synchronised to CLK, active high |
| IR0..4, STEP0..3, NEL | IR/step, flags | microcode ROM address | |
| NE0..NE3 | ALU 0..3 | the next slice; NE3 the flags chip | not-equal so far up the slices |
| M0..M15 | microcode ROM | MIR | the next microword |
| PCDRV, MEMRD_n, ALUOE, WE_n, ALD, BLD, PCLD, PCINC, IRLD, F1, F0, ADRV, TLD, TDRV | MIR | as in section 3; MEMRD_n and WE_n are the active-low pins the memories want | |
| CO | PC-low | PC-high | carry into Addr8 |

**The memories.**

| Chip | Address pins | Data | Selects | Strobes |
|---|---|---|---|---|
| program flash, 2 x SST39SF040 | A0..A14 from Addr0..Addr14; A15..A18 jumpers | chip 0 D0..7, chip 1 D8..15 | CE# = Addr15 | OE# = MEMRD_n, WE# = VCC |
| microcode ROM, 2 x SST39SF040 | A0..A3 = STEP0..3, A4 = NEL, A5..A9 = IR0..4, A10..A12 jumpers, the rest GND | chip 0 M0..7, chip 1 M8..15 | CE# = GND | OE# = GND, WE# = VCC |
| 2 x AS7C164A | A0..A12 from Addr0..Addr12 | as the program flash | CE2 = Addr15, CE1# = Addr14 | OE# = MEMRD_n, WE# = WE_n |
| TL16C550 | A0..A2 from Addr0..Addr2 | D0..7 | CS0 = Addr14, CS1 = Addr15, CS2# = GND, ADS# = GND | RD1# = MEMRD_n, WR1# = WE_n, RD2 = WR2 = GND; MR = RESET |

UART: its own 1.8432 MHz crystal on XIN/XOUT with two 18 pF loads (the
16550's clock only feeds its baud generator; its bus side is the
strobes, so it neither knows nor cares what CLK is), BAUDOUT# to RCLK,
DTR# looped to DSR# and DCD#, RI# high, SOUT/SIN and RTS#/CTS# through
the SP3232's two pairs to a 5-pin header.  Divisor 1 is 115200 baud.

## 5. Timing

One clock is 250 ns.  Every flop is on the rising edge of CLK; the only
signal that is not a flop output or a bus is CLK itself, straight from
the oscillator.  Within a clock:

- the IR, step and NEL change within about 6 ns of the edge; the
  microcode ROM answers within 70 more; the pipeline register samples
  at 250 with 3.5 ns of setup: 170 ns of margin.  A 150 ns ROM would do.
- a fetch or literal: the PC drives Addr from about 6 ns, the flash
  answers by 80, the IR or latch samples at 250.
- a memory read: the SRAM answers by 25 ns, the UART by 55.
- a write: WE_n is the whole clock.  The ALU has driven D since about
  8 ns, A has driven Addr since an earlier edge, and both stay for the
  next clock (rule 2).  Setup, width and hold are all a clock or more
  against tens of nanoseconds needed.
- the ALU ripple settles in about 120 ns; A or B loaded at one edge give
  a valid sum well before the next.

**Bus turnaround** is rule 1: a driver always has a whole clock to
float before the next takes the bus.  The model is asked to tolerate
nothing.

**Clock rate.**  4 MHz because every margin above is already a hundred
nanoseconds or more, and a slower can is a drop-in if the scope says
so.  The UART's frequency is its own business.

## 6. Parts

| Part | Count | Role |
|---|---|---|
| ATF22V10C-7 (DIP-24, socketed) | 16 | section 4 |
| SST39SF040 (DIP-32, socketed) | 4 | program, microcode |
| AS7C164A 8K x 8 (DIP-28) | 2 | registers and data |
| TL16C550 (LQFP-48) | 1 | serial |
| SP3232 (SSOP-16) + 5 x 100 nF | 1 | RS-232, two pairs |
| 4 MHz oscillator | 1 | CLK |
| 1.8432 MHz crystal + 2 x 18 pF | 1 | the UART's XIN/XOUT |
| MAX811L + button | 1 | reset |
| headers: serial, program bank (4), microcode bank (3) | | |

26 chips.

## 7. Choices made along the way

1. Registers in SRAM: no register-file chips.  Which is why T exists:
   with two latches and one of them the address, a memory value bound
   for the address register has nowhere else to land.
2. A small ISA plus a microcode ROM instead of MIPS I in hardware or
   microcode as the program: a real assembler target, immediates from
   the program stream, and a microcode table that is small, fixed and
   dumpable.
3. The A latch is the address register: one net to the ALU and Addr.
4. One bit per control line; results into A or B are separate opcodes
   at no hardware cost.
5. Chip selects from Addr15 and Addr14 with wires; one read strobe and
   one write strobe for all three memories.
6. The condition code is a microcode address bit.
7. No strobe timing inside a clock: hold times and turnarounds by
   microword order.
8. Reprogrammable after the board exists: program, microcode, every GAL.
   Only the buses, selects and strobe wiring are fixed.
9. Cheap to add later: `or`, `sltu`, byte access for the UART,
   in-socket flash programming (WE# from a spare word bit), a register-
   index field in the instruction word.

## 8. Verification

| What | Where | Result |
|---|---|---|
| Every GAL fits its chip; the microcode obeys its rules | `tests/grit.rs` | pass |
| Hand-written programs: arithmetic and stores, loads from RAM and flash, a countdown loop, NOR and jumps, serial both ways | `tests/grit.rs` | pass, state matches the reference |
| Reset released at nine phases of the clock; the reset button mid-run | `tests/grit.rs` | pass |
| Random programs (`grit::soak`): registers, pointers, flash constants, ALU through A and B, forward JEQ, nested bounded loops, against the reference | `tests/grit_soak.rs` | 120 programs, about 60 000 instructions, no chip complaint, every register and data word matching |
| The same under the physical fuzz: a delay on every pin (clock pins included, so it is clock skew too), CLK duty 45 to 55 %, jitter on every edge, random power-up SRAM | `tests/grit_soak.rs` | passes with every pin delayed by up to 1, 2 and 3 ns (up to 450 mm of trace; jitter up to 2 ns); fails at 6 ns |

**The 6 ns failure, diagnosed.**  With every pin delayed by up to 6 ns
the clock pins are too, so two chips can see the same edge up to 6 ns
apart.  In the failing draw the PC-low chip's clock arrived 1.6 ns
after the edge and PC-high's 4.05 ns after.  At a JMP's load, PC-low
changed its address bits 2 ns (the GAL's minimum clock-to-output)
after its own edge, the flash's data followed at once (its output hold
is 0 ns by datasheet), and PC-high, still 0.15 ns short of its own
edge, sampled data that was already changing: a hold violation, and a
real one.  The same margin, minimum clock-to-output minus clock skew,
governs every flop-to-flop path on the board; the memory paths add
nothing to it because the flash holds its outputs for 0 ns after an
address change (the SRAM adds its 3 ns).  So the board's one timing
rule is: **clock skew between any two GALs under 2 ns**, which is
300 mm of trace difference and needs no care on a board this size.
Data traces are free.  With the clock skew on its own knob:

| Data delay per pin | Clock skew per pin | Result |
|---|---|---|
| 6, 20 and 40 ns | 0 | pass |
| 50 ns | 0 | setup failure on the not-equal path into NEL |
| 1 ns | 1.5 ns | pass |
| 1 ns | 2.5 ns | pass in 3 random draws (the critical pair happened not to exceed 2) |
| directed: PC-high's clock alone | 1.0, 1.8 ns | clean |
| directed: PC-high's clock alone | 2.2, 3.0 ns | hold violation, as computed |

`clock_skew_limit` in `tests/grit_soak.rs` is the directed sweep; it
is the number to keep in mind when routing CLK: every GAL within about
250 mm of trace of every other on the clock net, which a star or a
short daisy chain gives for free.

**Memories and output hold.**  The model's hold analysis credits a
memory's data with its output hold after an address change (3 ns for
the AS7C164A, 0 for the flash), on top of the 2 ns from the register
that changed the address.  A future board that loads a latch from the
flash at the edge that also changes the address, as JMP does here,
has exactly the 2 ns and no more.

The fuzz knobs are `GRIT_FUZZ_DELAY_NS`, `GRIT_FUZZ_JITTER_NS`,
`GRIT_FUZZ_DUTY`, `GRIT_FUZZ_PROGRAMS`, `GRIT_FUZZ_SEED`; the soak's
are `GRIT_SOAK_PROGRAMS` and `GRIT_SOAK_SEED`.

## 9. What the simulation found and taught

Building it in the netlist changed three things, none of them in the
block diagram.

- **The store strobe came a word too early.**  The first microcode
  turned A onto the address bus and asserted WE in the same word; the
  SRAM model reported its address changing 8 ns into the write, because
  a GAL's output enable takes up to 10 ns to turn on.  Hence rule 2.
- **A memory read's data must outlive the capture.**  With the address
  or the strobe dropping at the same edge that loads B, the flash's
  data (output hold 0 ns) can vanish 2 ns after the edge, which is
  exactly the clock skew a pair of chips can have.  The fuzz found it
  at 3 ns of per-pin delay; hence rule 3.
- **A read at a changing address is a read of something else.**  Holding
  the strobe through the word after a load into A meant a one-clock
  read at whatever address was loaded; the random programs loaded UART
  addresses and read the UART by accident.  Hence rule 4.
- **And the same thing at nanosecond scale.**  Even with the strobe
  dropped in the next word, the address change and the strobe's end
  come from two chips at the same edge, each 2 to 5.5 ns after it, in
  either order.  A value loaded into A that looks like a UART address
  can select the UART for a few nanoseconds with the read strobe still
  low: a runt read.  Ideal traces hid it; 20 ns of trace mismatch
  showed it.  Two wrong cures came first, a select bit for the UART and
  then separate I/O strobes; both treated the UART differently from
  the memories, when the fault was the machine changing an address
  register at the end of its own strobe.  The T latch is the right one
  (rule 4): the bus then has one protocol for everything on it, which
  is what PISC gets from a register file with more than one register
  to land memory data in.
- **The condition had to be latched.**  NE is only meaningful while A
  drives the bus (the ALU's A inputs are the address bus), so the ROM's
  condition bit is NEL, latched at the end of ADRV words, and JEQ is
  laid out so that its decision is read while NEL is fresh (rule 4).
- **Reset is synchronous.**  See section 4.
- **The review round.**  The reset synchroniser had been tucked into
  B-low and a divide-by-two into ALU 0, to save chips; both were
  "clever", so the sequencer got a second GAL for its flags (RS1,
  RESET, NEL) and the divider went away with the reason for it: the
  UART's clock only feeds its baud generator, so it runs on its own
  1.8432 MHz crystal and CLK is a plain 4 MHz can.  Addr became a full
  16-bit word address (it had been bits 15:1 of a byte address, with
  A0 going to the ALU alone), which doubles the flash a bank holds and
  removes a thing to explain.

And two things changed in the model, because a clock that comes out of
a GAL is not the ideal clock crag's tests use:

- **Same-clock inputs.**  A GAL output has a 2 to 5.5 ns window after
  the edge in which it is unknown; when the clock itself has that
  window, every flop-to-flop path on the board looked like a setup or
  hold violation, since the model treated each chip's edge as
  independently uncertain.  The simulator now works out which nets are
  synchronous to which clock (registered outputs, and anything
  combinational or memory-like that depends only on them) and tells
  each GAL; a change on such an input inside its clock window is taken
  as a consequence of the same physical edge, after it, and the capture
  uses the value from before.
- **Unchanged flops do not blink.**  A register whose D equals its Q on
  definite inputs cannot change whichever instant in the window the
  edge falls on, so its pin no longer goes unknown for the window.
  Before this, WE_n blinked at every edge and the SRAM reported a
  possible runt write on each.

Both are refinements of what the model already assumed about the
chip's own feedbacks, applied across chips.
