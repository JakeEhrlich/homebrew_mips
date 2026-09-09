# grit: the first board (proposal)

The smallest machine that runs real code out of the parts chosen for
crag: ATF22V10C GALs, SST39SF040 flash, AS7C164A 8K x 8 SRAM, the
TL16C550 and SP3232 serial pair, the MAX811L.  Its purpose is to learn
the physical side: programming GALs from the manifest, executing out of
a socketed flash, SRAM strobes from GAL logic, the serial link, a board
through JLCPCB, and above all to be debuggable: one thing per clock,
every control line on a pin, every table in a socket.

Status: proposal, reviewed component by component.  Nothing is modelled
yet.

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
microwords, not by strobes.  13 GALs and 10 other chips; the program,
the microcode and every GAL are reprogrammable in their sockets.

## 2. Programmer's view

**Word.**  16 bits.  A, B and the PC are 16 bits, memory is 16-bit words
at even byte addresses, addresses are 16 bits.  No byte operations.

**Registers.**  A is the address register and the ALU's first operand:
its outputs are the address bus whenever the PC is not fetching.  B is
the second operand and the value a store writes.  The PC addresses the
program flash.

**Instructions.**  One word; the opcode is bits 15:11.  Immediates are
the following word.

| Op | Instruction | Does | Clocks |
|---|---|---|---|
| 0 | RESET | fetch from the PC (what runs out of reset, PC = 0) | 2 |
| 1 | LDA imm | A = next word | 5 |
| 2 | LDB imm | B = next word | 5 |
| 3 | LDA (A) | A = mem[A] | 5 |
| 4 | LDB (A) | B = mem[A] | 5 |
| 5 | STB (A) | mem[A] = B | 6 |
| 6 | ADDA | A = A + B | 5 |
| 7 | ADDB | B = A + B | 5 |
| 8 | ANDA | A = A and B | 5 |
| 9 | ANDB | B = A and B | 5 |
| 10 | NORA | A = not (A or B) | 5 |
| 11 | NORB | B = not (A or B) | 5 |
| 12 | MOVAB | A = B | 5 |
| 13 | JMP imm | PC = next word | 5 |
| 14 | JEQ imm | if A == B then PC = next word, else skip it | 5 or 4 |
| 15 | NOP | | 3 |
| 16 | HALT | never fetches; the step counter shows it | |

Registers r0..r31 of the MIPS-like macro layer are SRAM words at
0x8000 + 2n.  `lw rt, (rs)` is `LDA &rs; LDA (A); LDB (A); LDA &rt;
STB (A)`; `addu rd, rs, rt` is `LDA &rt; LDB (A); LDA &rs; LDA (A);
ADDB; LDA &rd; STB (A)`; `beq rs, rt, target` is `LDA &rt; LDB (A);
LDA &rs; LDA (A); JEQ target`.  About 200 000 macro-instructions a
second at 4.6 MHz.

**Memory map.**

| Address | What |
|---|---|
| 0x0000 .. 0x7FFF | program flash, 32 KB of the 512 KB; A16..A18 on a 3-way jumper, so one chip pair holds 8 programs |
| 0x8000 .. 0x803F | registers r0 .. r31 |
| 0x8040 .. 0xBFFF | SRAM, 16 KB |
| 0xC000 + 2n | TL16C550 register n on the low byte; the high byte of a read is garbage, mask it |

The microcode ROM is not in this space; it has its own wires.

**Reset.**  The MAX811L holds reset; the PC, IR and step counter clear;
opcode 0's microcode fetches from address 0.

## 3. The microcode

**Address**, 10 bits: {opcode[4:0], NE, step[3:0]}.  NE is the ALU's
"A differs from B", so a conditional instruction is two microcode
sequences and the hardware chooses between them with an address bit.
The ROM's other address pins are jumpers: microcode variants without
touching the program.

**Word**, 16 bits, one per control line, eleven used:

| Bit | Line | For the whole clock |
|---|---|---|
| 0 | PCDRV | the PC drives Addr; otherwise A drives Addr |
| 1 | MEMRD | the memory selected by Addr drives D (flash, SRAM or UART, by Addr's top bits) |
| 2 | ALUOE | the ALU drives D |
| 3 | WE | the memory selected by Addr takes D |
| 4 | ALD | A copies D at the ending edge |
| 5 | BLD | B copies D at the ending edge |
| 6 | PCLD | the PC copies D at the ending edge |
| 7 | PCINC | PC += 2 at the ending edge |
| 8 | IRLD | IR copies D[15:11] and the step counter clears at the ending edge: the fetch |
| 9, 10 | F1, F0 | ALU function: 00 add, 01 and, 10 nor, 11 pass B |
| 11 .. 15 | spare | debug outputs, a halt flag |

**Pipeline.**  In clock k the ROM presents the word for step k while the
pipeline register executes the word for step k-1.  So the word after a
fetch is always executed before the new instruction's step 0, and every
sequence ends `... PCDRV MEMRD IRLD; nop`.

**Two ordering rules**, which replace all clock-phase logic:

1. The driver of D never changes between consecutive words: an idle
   word sits between a MEMRD word and an ALUOE word, or between either
   and a fetch.  The outgoing chip gets a whole clock to float.
2. After a MEMRD or WE word comes a word with the same Addr driver (not
   PCDRV) and, after WE, ALUOE still set: the address and data outlive
   the strobe by a clock, which is every hold time on the board.

**The sequences.**  Each line is one clock.

```
RESET     PCDRV MEMRD IRLD ; nop
LDA imm   PCINC ; PCDRV MEMRD ALD ; PCINC ; PCDRV MEMRD IRLD ; nop
LDB imm   PCINC ; PCDRV MEMRD BLD ; PCINC ; PCDRV MEMRD IRLD ; nop
LDA (A)   MEMRD ALD ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
LDB (A)   MEMRD BLD ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
STB (A)   ALUOE F=passB WE ; ALUOE F=passB ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
ADDA      ALUOE F=add ALD ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
ADDB      ALUOE F=add BLD ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
MOVAB     ALUOE F=passB ALD ; nop ; PCINC ; PCDRV MEMRD IRLD ; nop
JMP imm   PCINC ; PCDRV MEMRD PCLD ; nop ; PCDRV MEMRD IRLD ; nop
JEQ imm   NE=0: as JMP.   NE=1: PCINC ; PCINC ; PCDRV MEMRD IRLD ; nop
NOP       PCINC ; PCDRV MEMRD IRLD ; nop
HALT      nop x 16 (the counter wraps and it repeats)
```

The nop after MEMRD ALD in `LDA (A)` is rule 1 (the SRAM lets go
before the flash drives) and rule 2 at once.  ANDA/ANDB/NORA/NORB are
ADDA/ADDB with F changed.  The second word of STB holds the address in
A and the data on D for a clock after the write pulse.

## 4. The chips

**Buses.**  D[15:0]: driven by the flash, SRAM, UART (low byte) or the
ALU, one per clock; read by A, B, the PC and the IR.  Addr[15:1]: the A
latch, or the PC when PCDRV.  The microcode ROM's address and data are
private wires from the IR/step chip and to the pipeline register.

| Chip | Clock | Reset | Inputs | Outputs |
|---|---|---|---|---|
| IR/step | CLK | RESET | D11..D15, IRLD | IR0..IR4 (opcode), STEP0..STEP3 |
| MIR-low, MIR-high | CLK | RESET | M0..M7, M8..M15 (the ROM's data) | the control lines, one per bit |
| PC-low | CLK | RESET | D1..D8, PCLD, PCINC, PCDRV | PC1..PC8 (enabled by PCDRV), CO |
| PC-high | CLK | RESET | D9..D15, CO, PCLD, PCDRV | PC9..PC15 (enabled by PCDRV) |
| A-low, A-high | CLK | none | D0..D7 / D8..D15, ALD, PCDRV | A0..A7 / A8..A15; A1..A15 enabled by not PCDRV, A0 always |
| B-low | CLK | none | D0..D7, BLD, RST_n | B0..B7, RS1, RESET |
| B-high | CLK | none | D8..D15, BLD | B8..B15 |
| ALU 0 | CLK2X | none | A0..3, B0..3, F1, F0, ALUOE | S0..3 (enabled by ALUOE), carry out, NE out, CLK (the divide by two) |
| ALU 1, 2, 3 | none | none | the next four bits of A and B, carry and NE in, F1, F0, ALUOE | four sums, carry out, NE out; ALU 3's NE out is NE |

Thirteen GALs.  The latches are `Q := LD & D + !LD & Q` per bit; the PC
is a 15-bit counter (PCINC) with a load (PCLD wins); the step counter
clears on IRLD and counts otherwise; the pipeline register is `Q := M`;
the ALU slices are 4-bit ripple adders with the function folded into
each sum term and a not-equal chain beside the carry chain.

| Net | Driven by | Read by | Meaning |
|---|---|---|---|
| CLK2X | oscillator | ALU 0, UART XIN | 9.216 MHz |
| CLK | ALU 0 | every other GAL | 4.608 MHz; every flop is on its rising edge |
| RST_n | MAX811L | B-low | reset, active low |
| RESET | B-low | IR/step, MIR, PC, UART MR | reset synchronised to CLK, active high |
| IR0..4, STEP0..3, NE | IR/step, ALU 3 | microcode ROM address | |
| M0..M15 | microcode ROM | MIR | the next microword |
| PCDRV, MEMRD_n, ALUOE, WE_n, ALD, BLD, PCLD, PCINC, IRLD, F1, F0 | MIR | as in section 3; MEMRD_n and WE_n are the active-low pins the memories want | |
| CO | PC-low | PC-high | carry into PC9 |

**The memories.**

| Chip | Address pins | Data | Selects | Strobes |
|---|---|---|---|---|
| program flash, 2 x SST39SF040 | A0..A13 from Addr1..Addr14; A14, A15 to GND; A16..A18 jumpers | chip 0 D0..7, chip 1 D8..15 | CE# = Addr15 | OE# = MEMRD_n, WE# = VCC |
| microcode ROM, 2 x SST39SF040 | A0..A3 = STEP0..3, A4 = NE, A5..A9 = IR0..4, A10..A18 jumpers or GND | chip 0 M0..7, chip 1 M8..15 | CE# = GND | OE# = GND, WE# = VCC |
| 2 x AS7C164A | A0..A12 from Addr1..Addr13 | as the program flash | CE2 = Addr15, CE1# = Addr14 | OE# = MEMRD_n, WE# = WE_n |
| TL16C550 | A0..A2 from Addr1..Addr3 | D0..7 | CS0 = Addr14, CS1 = Addr15, CS2# = GND, ADS# = GND | RD1# = MEMRD_n, WR1# = WE_n, RD2 = WR2 = GND; MR = RESET |

UART: XIN from the oscillator, XOUT open, BAUDOUT# to RCLK, DTR# looped
to DSR# and DCD#, RI# high, SOUT/SIN and RTS#/CTS# through the SP3232's
two pairs to a 5-pin header.  Divisor 5 is 115200 baud.

## 5. Timing

One clock is 217 ns.  Every flop is on the rising edge of CLK; the only
signal that is not a flop output or a bus is CLK itself.  Within a
clock:

- the IR, step and NE change within about 6 ns of the edge; the
  microcode ROM answers within 70 more; the pipeline register samples
  at 217 with 3.5 ns of setup: 137 ns of margin.  A 150 ns ROM would do.
- a fetch or literal: the PC drives Addr from about 6 ns, the flash
  answers by 80, the IR or latch samples at 217.
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

**Clock rate.**  4.608 MHz because 9.216 MHz is the UART's frequency
and every margin above is already tens of nanoseconds.  A divide by
four is one more term if the scope says so.

## 6. Parts

| Part | Count | Role |
|---|---|---|
| ATF22V10C-7 (DIP-24, socketed) | 13 | section 4 |
| SST39SF040 (DIP-32, socketed) | 4 | program, microcode |
| AS7C164A 8K x 8 (DIP-28) | 2 | registers and data |
| TL16C550 (LQFP-48) | 1 | serial |
| SP3232 (SSOP-16) + 5 x 100 nF | 1 | RS-232, two pairs |
| 9.216 MHz oscillator | 1 | CLK2X and the UART's XIN |
| MAX811L + button | 1 | reset |
| headers: serial, program bank, microcode bank | | |

23 chips.

## 7. Choices made along the way

1. Registers in SRAM: no register-file chips.
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
