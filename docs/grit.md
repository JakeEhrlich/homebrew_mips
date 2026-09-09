# grit: the first board (proposal)

The smallest machine that runs real code out of the parts chosen for
crag: ATF22V10C GALs, SST39SF040 flash, AS7C164A 8K x 8 SRAM, the
TL16C550 and SP3232 serial pair, the MAX811L.  Its purpose is to learn
the physical side: programming GALs from the manifest, executing out of
a socketed flash, SRAM strobes from GAL logic, the serial link, a board
through JLCPCB.  Speed is not a goal.

Status: proposal, reviewed component by component.  Nothing is modelled
yet.

## 1. The idea in one paragraph

A microcoded 16-bit machine.  Every flash word is a microinstruction: a
horizontal word with one bit per control line, executed in two clocks,
fetch then execute.  There is no instruction decoder and no state
machine; the microinstruction register is the whole of control.  The
32 registers are the first 32 words of the SRAM, addressed like any
memory.  One data bus, one address bus, one read strobe per region,
one write strobe.  12 GALs and 8 other chips.

## 2. Programmer's view

**Word.**  16 bits.  Latches are 16 bits, memory is 16-bit words at even
byte addresses, addresses are 16 bits.  No byte operations.

**The machine.**  Two latches A and B feeding a 16-bit ALU; A's outputs
are also the address bus.  A program counter.  A flash for code and
constants, an SRAM for data and registers, a UART.  Everything is
joined by the data bus D.

**The microword.**  One bit per control line; the bottom six are spare.

| Bit | Signal | In the execute clock |
|---|---|---|
| 15 | PCDRV | the next word is data: the PC keeps the address bus, the flash drives D, the PC steps past the word |
| 14 | MEMRD | memory at A drives D |
| 13 | ALUOE | the ALU drives D |
| 12 | WR | memory at A takes D (a quarter-clock write pulse) |
| 11 | ALD | A loads D at the end of the clock |
| 10 | BLD | B loads D |
| 9 | PCLD | the PC loads D |
| 8 | PCEQ | the PC loads D if A equals B |
| 7:6 | F1, F0 | ALU function: 00 add, 01 and, 10 nor, 11 pass B |
| 5:0 | | unused |

Rules the assembler enforces: at most one of PCDRV, MEMRD, ALUOE (one
driver on D); MEMRD and WR never together.

**Memory map.**

| Address | What |
|---|---|
| 0x0000 .. 0x7FFF | flash, 32 KB of the 512 KB; A16..A18 on a 3-way jumper, so one chip holds 8 programs |
| 0x8000 .. 0x803F | registers r0 .. r31, r_n at 0x8000 + 2n |
| 0x8040 .. 0xBFFF | SRAM, 16 KB |
| 0xC000 + 2n | TL16C550 register n on the low byte; the high byte of a read is garbage, mask it |

**Immediates.**  A microop with PCDRV set is followed by a data word; in its
execute clock the flash presents that word on D and whatever LD bits are
set load it.  `B = 0x1234` is `0x8400, 0x1234`: two words, two clocks.
A register's address is an immediate like any other: `A = &r5` is
`0x8800, 0x800A`.

**Reset.**  The MAX811L holds reset; the PC starts at flash address 0.
Programs go into the flash with the programmer, in the DIP-32 socket.

**MIPS-like macros.**  A macro assembler gives the program MIPS syntax;
the binary is microcode.

| Macro | Microops | Words | Clocks |
|---|---|---|---|
| `lw rt, (rs)` | A=&rs; A=mem; B=mem; A=&rt; mem=B | 7 | 10 |
| `sw rt, (rs)` | A=&rt; B=mem; A=&rs; A=mem; mem=B | 7 | 10 |
| `addu rd, rs, rt` (also and, nor) | A=&rt; B=mem; A=&rs; A=mem; B=ALU; A=&rd; mem=B | 10 | 14 |
| `addiu rt, rs, imm` (also andi) | A=&rs; A=mem; B=imm; B=ALU; A=&rt; mem=B | 9 | 12 |
| `beq rs, rt, target` | A=&rt; B=mem; A=&rs; A=mem; PC=target if EQ | 8 | 10 |
| `j target` | PC=target | 2 | 2 |

About 0.4 million macro-instructions a second at 4.6 MHz, and about
1600 of them per flash bank.  A one-word "point A at register n" field
in the microword would halve both numbers; it is a pure addition to the
A latch (two flag bits, five index bits, four product terms) and is
left out until code size bites.

## 3. The chips

**Buses.**  D[15:0], the data bus: driven by the flash, the SRAM, the
UART's low byte or the ALU, one per clock; loaded by A, B, the PC and
the microinstruction register.  A[15:1], the address bus: driven by the
A latch, except in the second half of a fetch clock or a literal read,
when the PC drives it.  The memories decode A15 and A14 on their own
chip-select pins, so control has no chip selects: one read strobe for
the flash, one for the SRAM and UART, one write strobe for both.

**The twelve GALs.**  "Clock" is pin 1; "reset" is the chip's
asynchronous-reset term.

| Chip | Clock | Reset | Inputs | Outputs |
|---|---|---|---|---|
| glue | CLK2X | none | WR, PCDRV_n, PCLD, PCEQ, NE | CLK (divide by two), WE_n, PCDRIVE, LOAD |
| MIR | CLK | RESET | D6 to D15, EXEC | PCDRV_n, MEMRD_n, ALUOE, WR, ALD, BLD, PCLD, PCEQ, F1, F0: loaded from the word at a fetch edge, cleared at an execute edge; PCDRV_n is also active in every fetch clock |
| PC-low | CLK | RESET | D1 to D8, LOAD, PCDRIVE | PC1 to PC8 (enabled by PCDRIVE), CO |
| PC-high | CLK | RESET | D9 to D15, CO, LOAD, PCDRIVE | PC9 to PC15 (enabled by PCDRIVE) |
| A-low | CLK | none | D0 to D7, ALD, PCDRIVE | A0 (always on), A1 to A7 (enabled by not PCDRIVE) |
| A-high | CLK | none | D8 to D15, ALD, PCDRIVE | A8 to A15 (enabled by not PCDRIVE) |
| B-low | CLK | none | D0 to D7, BLD, RST_n | B0 to B7, RS1, RESET |
| B-high | CLK | RESET | D8 to D15, BLD | B8 to B15, EXEC |
| ALU 0 | none | none | A0..3, B0..3, F1, F0, ALUOE | S0..3 (enabled by ALUOE), C1, C2, C3, COUT0, NE0 |
| ALU 1, 2 | none | none | the next four bits of A and B, the carry and NE from below, F1, F0, ALUOE | four sums, C1, C2, C3, carry out, NE out |
| ALU 3 | none | none | A12..15, B12..15, COUT2, NE2, F1, F0, ALUOE | S12..15, C1, C2, C3, NE |

| Signal | Made by | Meaning |
|---|---|---|
| CLK2X | oscillator | 9.216 MHz; also the UART's XIN |
| CLK | glue | 4.608 MHz, every other GAL's clock |
| RST_n | MAX811L | active low |
| RS1, RESET | B-low | two-stage synchroniser; RESET is the async reset of PC, MIR, B-high and the UART's MR |
| EXEC | B-high | 1 in an execute clock, 0 in a fetch clock; toggles every edge, reset to 0 |
| PCDRV_n | MIR | low in every fetch clock and in an execute clock whose word set bit 15 (a literal); the flash's OE# |
| MEMRD_n | MIR | low in an execute clock with MEMRD; the SRAM's and UART's OE# |
| PCDRIVE | glue | PCDRV and CLK low: the PC has the address bus, and the PC counts at the edge |
| WE_n | glue | low while WR, CLK high and CLK2X low: quarter two of a writing execute clock |
| LOAD | glue | PCLD, or PCEQ and not NE |
| NE | ALU 3 | A differs from B |

The latches are one equation per bit, `Q := LD & D + !LD & Q`.  The PC
is a 15-bit counter that steps by one word when PCDRIVE is high at the
edge and loads D when LOAD is high (LOAD wins).  The ALU slices are
4-bit ripple adders with the function select folded into each sum
term, a carry between slices and a not-equal chain beside it.  The
microinstruction register's flops are `Q := !EXEC & Dk`, except PCDRV
which is `!EXEC & D15 + EXEC`.

**The memories.**

| Chip | Address pins | Data | Selects | Strobes |
|---|---|---|---|---|
| 2 x SST39SF040, DIP-32 | A0..A13 from bus A1..A14; A14, A15 to GND; A16..A18 to jumpers | chip 0 D0..7, chip 1 D8..15 | CE# = A15 | OE# = PCDRV_n, WE# = VCC |
| 2 x AS7C164A, DIP-28 | A0..A12 from bus A1..A13 | as above | CE2 = A15, CE1# = A14 | OE# = MEMRD_n, WE# = WE_n |
| TL16C550, LQFP-48 | A0..A2 from bus A1..A3 | D0..7 | CS0 = A14, CS1 = A15, CS2# = GND, ADS# = GND | RD1# = MEMRD_n, WR1# = WE_n, RD2 = WR2 = GND; MR = RESET |

UART: XIN from the oscillator, XOUT open, BAUDOUT# to RCLK, DTR# looped
to DSR# and DCD#, RI# high, SOUT/SIN and RTS#/CTS# through the SP3232's
two pairs to a 5-pin header.  Divisor 5 is 115200 baud.

## 4. Timing

One clock is 217 ns, in quarters of 54.  Every flop in the machine is on
the rising edge of CLK.  Two things happen inside a clock, both derived
from CLK2X transitions so that neither can glitch:

- **WE_n** is quarter two.  It starts when CLK2X falls with WR and CLK
  already high, ends when CLK2X rises before CLK has moved.  The ALU
  has driven the data since the edge (setup 45 ns against 8), the
  address has been on the bus since an earlier edge, and it changes
  109 ns after the pulse ends.  The UART gets the same pulse: 54 ns
  against its 40.
- **PCDRIVE** is the second half of a fetch or literal clock.  The flash
  gets its address 108 ns before the capturing edge (tACC 70).  The chip
  strobed in the previous execute clock keeps its address for the first
  half of the fetch clock, which covers the UART's 20 ns address hold
  and is why ADS# can be grounded.

Reads: the SRAM's data is on the bus about 30 ns into an execute clock,
the UART's about 50, a literal's at 178, captured at 217 with 3.5 ns of
setup.  The 16-bit ripple settles in about 120 ns, always at least a
clock before its result is used.

**Bus turnaround.**  When the driver of D changes at an edge the
outgoing chip takes up to 25 ns (flash), 7 ns (SRAM), 20 ns (UART) or
about 10 ns (a GAL) to let go, and the incoming one can start within a
few nanoseconds.  The overlap is tolerated by policy: it moves no data,
it costs a current spike, and the model is to bound and report it
rather than forbid it.  A delay-line-timed write and second-half-gated
drivers would remove it at the cost of one part, if the scope ever
says so.

**Clock rate.**  4.608 MHz because a quarter clock must be 40 ns for the
UART's write strobe.  Every other margin on the board is tens of
nanoseconds.

## 5. Parts

| Part | Count | Role |
|---|---|---|
| ATF22V10C-7 (DIP-24, socketed) | 12 | section 3 |
| SST39SF040 (DIP-32, socketed) | 2 | code, constants, literals |
| AS7C164A 8K x 8 (DIP-28) | 2 | registers and data |
| TL16C550 (LQFP-48) | 1 | serial |
| SP3232 (SSOP-16) + 5 x 100 nF | 1 | RS-232, two pairs |
| 9.216 MHz oscillator | 1 | CLK2X and the UART's XIN |
| MAX811L + button | 1 | reset |
| 5-pin serial header, 3 x 3-pin bank jumpers, decoupling | | |

20 chips.

## 6. Choices made along the way

1. **Registers in SRAM.**  Saves about eight GALs or the dual-port
   chips; a bad program can overwrite its registers.
2. **Microcode instead of MIPS I.**  Removes the state machine, the
   decoder and the instruction register's field latches, six GALs; loses
   gcc's assembler.  The macro assembler keeps MIPS syntax.
3. **The A latch is the address register.**  Its outputs are one net to
   the ALU and the address bus; a register access is a literal load of
   its address.  Saves the MAR.  The one-word register-address field
   was designed, then left out for simplicity (section 2).
4. **Horizontal microword.**  One bit per control line; the encoding
   into DRV/LD fields was saving bits the word did not need.
5. **Chip selects from address lines.**  The flash's CE#, the SRAM's two
   enables and the UART's three select pins decode A15 and A14 with
   wires; control has one read strobe per region and one write strobe.
6. **Two read strobes, not one.**  The SRAM and UART must not be strobed
   during fetch clocks (a UART read pops its FIFO), so the flash's OE#
   and the SRAM/UART OE# are separate flops.
7. **The 2x clock.**  A glitch-free write strobe that ends before the
   edge needs a transition that is not the clock's own; quarter two is
   it.  The 9.216 MHz oscillator serves the UART too.
8. **Turnaround overlap accepted** (section 4).
9. **A twelfth GAL for glue** rather than scattering four macrocells
   into spare corners of the PC, ALU and B chips.
10. **What is cheap to add later:** the register-address field, `or`,
    `sltu`, byte access for the UART, in-socket flash programming
    (WE_n from a spare word bit and the 39SF040 command sequence).
