# grit: the first board (proposal)

The smallest machine that runs real code out of the parts chosen for
crag: ATF22V10C GALs, SST39SF040 flash, AS7C164A 8K x 8 SRAM, the
TL16C550 and SP3232 serial pair, the MAX811L.  Its purpose is to learn
the physical side: programming GALs from the manifest, executing out of
a socketed flash, SRAM strobes from GAL logic, the serial link, a board
through JLCPCB.  Speed is not a goal.

Status: proposal for review.  Nothing is modelled yet.

## 1. The idea in one paragraph

A 16-bit MIPS I subset with MIPS I's own 32-bit instruction encodings,
executed one instruction at a time over several clocks by a small state
machine.  The 32 registers live in the SRAM (its first 64 bytes), so the
register file costs no chips.  Code runs straight out of the flash, so
there is no boot copier.  All data accesses are slow by construction (two
clocks), so the UART sits on the data bus like any memory and needs no
bridge.  17 GALs and 8 other chips.

## 2. Programmer's view

**Word.**  16 bits.  Registers are 16 bits, memory is 16-bit words at
even byte addresses, addresses are 16 bits.  There are no byte
operations.

**Instructions.**  MIPS I encodings, 32 bits, opcode and function fields
exactly MIPS I's, so the existing assembler emits them.  The differences
from MIPS I follow from the 16-bit word:

- The 16-bit immediate is the whole word.  `addiu rt, rs, imm` adds a
  full 16-bit value, so `li rt, imm` is one instruction and there is no
  LUI; sign extension does not arise.
- Branch and jump targets are absolute byte addresses: BEQ, BNE and J
  put the target address in the immediate field (the assembler encodes
  labels that way; the fields are the same bits).  PC-relative would
  need the PC on the ALU, which is a mux the board does not have.
- No delay slots.  Nothing is pipelined, so the instruction after a
  taken branch is not executed.
- Register `$0` is an ordinary SRAM word: writable, and nothing special
  at reset.  `andi $0, $0, 0` zeroes it (or any register); a program
  that wants `not` through `nor rd, rs, $0` does that first.
- `lw` and `sw` take a zero offset only (`lw rt, (rs)`, `sw rt, (rs)`);
  the assembler rejects others.  Addresses are computed with `addiu`
  first.  Section 3 says why.

| Instruction | Encoding | Does | Why it is there |
|---|---|---|---|
| `addu rd, rs, rt` | SPECIAL funct 0x21 | rd = rs + rt | add |
| `and rd, rs, rt` | SPECIAL 0x24 | rd = rs & rt | and |
| `nor rd, rs, rt` | SPECIAL 0x27 | rd = ~(rs \| rt) | `not rd, rs` = `nor rd, rs, $0` with `$0` zeroed |
| `addiu rt, rs, imm` | op 0x09 | rt = rs + imm | `li`, address arithmetic, counters |
| `andi rt, rs, imm` | op 0x0C | rt = rs & imm | masks, zeroing; same datapath as addiu, free |
| `lw rt, (rs)` | op 0x23, offset 0 | rt = mem16[rs] | load |
| `sw rt, (rs)` | op 0x2B, offset 0 | mem16[rs] = rt | store |
| `beq rs, rt, target` | op 0x04 | if rs == rt: PC = target | loops, polling |
| `j target` | op 0x02 | PC = target | same datapath as a taken branch, free |

Load, store, add, and, not and immediates were the request; `beq` and
`j` are there because polling a UART needs a loop, and `bne` is `beq`
to the other block.  Nothing else exists: no `jr` (no returns; a
subroutine is inlined or jumps back to a fixed place), no traps, no
multiply, no shifts, no set-on-less-than, no byte access.  Undefined
opcodes do something undefined.

**Memory map.**  One 64 KB space for code and data.  The flash is
readable with `lw`, so tables and strings live where the assembler put
them.

| Address | What |
|---|---|
| 0x0000 .. 0x7FFF | flash, 32 KB of the 512 KB; the chip's A16..A18 go to a 3-way jumper so one chip holds 8 programs |
| 0x8000 .. 0x803F | registers `$0` .. `$31`, `$n` at 0x8000 + 2n |
| 0x8040 .. 0xBFFF | SRAM, 16 KB (two 8K x 8 chips) |
| 0xC000 + 2n | TL16C550 register n on the low byte (n = 0..7); the high byte of a load is garbage, mask it |

Stores to the flash region do nothing in the first revision.  With one
more control line (the flash's WE#) they would be the SST39SF040's
byte-program command sequence, and a serial bootloader could burn code
in the socket; section 6.

**Reset.**  The MAX811L holds reset; the PC starts at code 0.  Programs
are written into the flash with the programmer, in the DIP-32 socket.

## 3. How it executes

Two shared buses.  The 16-bit data bus `D` joins everything: the
flash's outputs, the SRAMs, the UART's low byte, and the GAL latches.
The 15-bit address bus `A[15:1]` goes to the flash, the SRAMs and the
UART, and has two drivers: the PC while fetching and the A latch the
rest of the time.  The A latch is both the ALU's first operand and the
address register: its outputs feed the ALU and the address bus on the
same net, and a register access loads it with the register's address
(0x80 in the high byte, the index in bits 5:1) before the value.

The memories select themselves from the address: the flash's CE# is
A15, the SRAM's CE2 is A15 and CE1# is A14, the UART's CS0 and CS1 are
A14 and A15.  Control then needs one read strobe and one write strobe
for all three.

| Block | GALs | Notes |
|---|---|---|
| PC | 2 | 15 flops; counts by 2; loads from D on a taken branch or jump; drives A while fetching |
| rs, rt, rd | 3 | five flops each, loaded at F1 (rs, rt from D[9:0]) or F2 (rd from D[15:11]); each drives its index onto D[5:1] when asked |
| Decode | 1 | at F1 the instruction class from D[15:10], at F2 the ALU function from D[5:0]; five flops to control and the ALU |
| A latch | 2 | operand and address; loads a value or an index from D |
| B latch | 2 | operand; the immediate lands here at F2 |
| ALU | 4 | four 4-bit slices: add (ripple within the slice, carry between), and, nor, pass B; a not-equal output per slice for `beq`; drives D |
| Control | 3 | state counter, the strobes, latch enables, output enables, PC count and load; the divide-by-two for the clock |
| | **17** | plus or minus one once the pins are packed with `galpack` |

There is no instruction-register latch for the low half.  The
immediate goes straight into B, `rd` and `funct` have their own flops,
and a branch target is fetched again: the PC steps past the low half
only when the instruction ends, so at BR the flash is still presenting
it and the PC loads it from the bus.

Every instruction is a fixed sequence of states, one clock each unless
marked:

```
F1   D = flash[PC]  -> rs, rt, class;   PC += 2
F2   D = flash[PC]  -> rd, ALU function, B (the immediate);  PC += 2 unless beq or j
RA1  D[5:1] = rs    -> A as 0x8000 | index << 1
RA2  D = SRAM[A]    -> A
RB1  D[5:1] = rt    -> A                              (R-type, beq, sw)
RB2  D = SRAM[A]    -> B
X1   ALU = f(A, B)                                    (16-bit ripple settles)
X2   D = ALU        -> B                              (the result parks in B: A is about to become the index)
WI   D[5:1] = rd or rt -> A
WB   SRAM[A] = B (ALU passes B)                       (write pulse in the low half)
MR   D = mem[A]     -> B                              (two clocks; A holds rs's value, the address)
MW   mem[A] = B                                       (two clocks)
BR   D = flash[PC], the low half again; PC loads it if taken, else PC += 2
```

| Instruction | States | Clocks |
|---|---|---|
| addu, and, nor | F1 F2 RA1 RA2 RB1 RB2 X1 X2 WI WB | 10 |
| addiu, andi | F1 F2 RA1 RA2 X1 X2 WI WB | 8 |
| lw | F1 F2 RA1 RA2 MR MR WI WB | 8 |
| sw | F1 F2 RB1 RB2 RA1 RA2 MW MW | 8 (rt into B first, then rs into A, which is the address) |
| beq | F1 F2 RA1 RA2 RB1 RB2 X1 BR | 8 |
| j | F1 BR | 2 |

About 0.85 million instructions a second at 7.4 MHz.  Plenty.
`docs/grit-blocks.html` is the block diagram and the same table as
control lines per state.

**Why there are no offsets.**  A store with an offset needs three
values, the base, the offset and the data, and the machine has two
latches, one of which is also the address.  With a zero offset the
address is a register value that lands in A and stays there while B
holds the data.  A full `sw rt, off(rs)` or `lw rt, off(rs)` would
need the sum parked somewhere while the other register is read: a
third latch (two GALs) or a hidden register slot and four more states.

## 4. Timing, and why there are no delay lines

**Clock.**  One 14.7456 MHz oscillator drives the UART's XIN and a
divide-by-two flop in the control GAL: the CPU clock is 7.3728 MHz,
135.6 ns, 50 % by construction.  (8 MHz would need a second oscillator
for nothing.)

**Fetch.**  Flash tACC is 70 ns; the PC's outputs enable and are valid
within about 10 ns of the edge; the IR needs 3.5 ns before the next:
under 85 of 135 ns.  One clock per half.

**Register and SRAM access.**  The SRAM is 15 ns.  A read is one clock;
a write's WE# is the write state gated with the clock's low half, a
68 ns pulse from the clock edge itself, no tap.  The data (ALU outputs
on D) has been stable since the X states.  Every margin is tens of
nanoseconds; there is nothing to bin.

**Data access (lw, sw).**  Two clocks, always, whichever chip is
addressed.  The strobe (RD# or WR#) is the whole first clock, 135 ns
against the 16550's 40 ns; MAR has been valid for a clock before it
(setup 7 ns needed); data is valid 45 ns into a read; the strobe ends
and the state holds the address for another clock (hold 20 ns needed).
Two accesses are at least 10 clocks apart, more than the 87 ns cycle
time.  So "loads take long enough" is a property of the state machine,
not of an address decoder or a bridge: the UART is just a slow memory,
and the SRAM does not mind being read slowly.

**Serial.**  As on crag: TL16C550, SP3232 with RTS#/CTS# on its second
pair to the 5-pin header, auto-flow control, 115200 baud is divisor 8.
The registers are polled through LSR.

## 5. Parts

| Part | Count | Role |
|---|---|---|
| ATF22V10C-7 (DIP-24, socketed) | 17 | everything in section 3 |
| SST39SF040 (DIP-32, socketed) | 2 | code, 16 bits wide |
| AS7C164A 8K x 8 (DIP-28) | 2 | registers and data, 16 bits wide |
| TL16C550 (LQFP-48) | 1 | serial |
| SP3232 (SSOP-16) + 4 x 100 nF | 1 | RS-232 levels, two pairs |
| 14.7456 MHz oscillator | 1 | UART and CPU clock |
| MAX811L + button | 1 | reset |
| 5-pin serial header, 3-way bank jumper, decoupling | | |

About 26 placed parts, 21 of them DIP in sockets, and the two SMD parts
are the ones JLCPCB already had in stock.

## 6. Choices to argue about

1. **Registers in SRAM.**  Costs about four clocks per instruction, saves
   about eight GALs or the dual-port chips.  A bad program can overwrite
   its registers.  I would keep it: the whole point is a small board.
1b. **The A latch as the address register.**  Saved the two-GAL MAR.
   It forced the instruction register to be held as fields rather than
   halves (rs, rt, rd and a decode chip), because the index has to ride
   D[5:1] to fit the A latch's pins, and it costs ALU ops one clock to
   park the result in B.  Going further, `rd == rt` as an assembler rule
   would delete the rd chip: one more GAL, not taken for now.
2. **16-bit data bus (two flashes, two SRAMs)** against an 8-bit bus with
   one of each.  8 bits halves the memory chips and the data traces and
   makes the UART a natural byte device, at the cost of a byte-phase in
   every state (roughly twice the clocks) and a little more control
   logic.  I lean 16 because the control is simpler and it uses the
   chips; the 8-bit version is the fallback if the GAL count must drop.
3. **One address space.**  An earlier draft was Harvard, with the PC
   wired to the flash and MAR to the SRAM.  Sharing the address bus
   costs an output-enable pin on the PC and MAR chips and a three-way
   decode instead of two, no chips, and buys constants in flash and
   the same bus model as crag.  The only way it could have saved chips
   is by dropping the PC and keeping it in an SRAM slot, fetching
   through MAR: about six clocks per fetch half and an increment
   function in the ALU.  Not taken.  A later revision can give the
   flash its WE# and program it in the socket through the UART.
4. **Absolute branch targets, no delay slot, zero-offset `sw`.**  All
   assembler-level differences from MIPS I; the instruction bit layout
   is untouched.  A future pipelined board (pebble) can reintroduce
   PC-relative and the slot; the code for grit will not be reused
   anyway.
5. **What is cheap to add later:** `bne` (the same states, the branch
   condition inverted), `or`/`ori` (two product terms per ALU bit),
   `sltu` (a subtract is add with B inverted and carry in: one more ALU
   function and a carry-out bit), `jr` (four states, the ALU passing A
   to the PC), `jal` (PC onto D: costs pins on the PC GALs, probably one
   more chip), byte loads for the UART, offsets on `lw` (two states) and `sw` (above).
6. **Clock.**  7.37 MHz from the UART's oscillator, or a separate 8 MHz
   can and the same numbers.  Everything above has a factor of two of
   margin at either.
