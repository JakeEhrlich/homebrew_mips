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
machine.  The 32 registers live in the SRAM (the first 64 bytes), so the
register file costs no chips.  Code runs straight out of the flash, so
there is no boot copier.  All data accesses are slow by construction (two
clocks), so the UART sits on the data bus like any memory and needs no
bridge.  About 19 GALs and 8 other chips.

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
- `sw` takes a zero offset only (`sw rt, ($rs)`); the assembler rejects
  others.  Section 3 says why; `lw` keeps its offset.

| Instruction | Encoding | Does | Why it is there |
|---|---|---|---|
| `addu rd, rs, rt` | SPECIAL funct 0x21 | rd = rs + rt | add |
| `and rd, rs, rt` | SPECIAL 0x24 | rd = rs & rt | and |
| `nor rd, rs, rt` | SPECIAL 0x27 | rd = ~(rs \| rt) | `not rd, rs` = `nor rd, rs, $0` with `$0` zeroed |
| `addiu rt, rs, imm` | op 0x09 | rt = rs + imm | `li`, address arithmetic, counters |
| `andi rt, rs, imm` | op 0x0C | rt = rs & imm | masks, zeroing; same datapath as addiu, free |
| `lw rt, off(rs)` | op 0x23 | rt = mem16[rs + off] | load |
| `sw rt, (rs)` | op 0x2B, offset 0 | mem16[rs] = rt | store |
| `beq rs, rt, target` | op 0x04 | if rs == rt: PC = target | loops, polling |
| `j target` | op 0x02 | PC = target | same datapath as a taken branch, free |

Load, store, add, and, not and immediates were the request; `beq` and
`j` are there because polling a UART needs a loop, and `bne` is `beq`
to the other block.  Nothing else exists: no `jr` (no returns; a
subroutine is inlined or jumps back to a fixed place), no traps, no
multiply, no shifts, no set-on-less-than, no byte access.  Undefined
opcodes do something undefined.

**Memory map.**  Code and data are separate spaces (Harvard), each 64 KB,
because the PC then drives the flash's address pins directly and the
data address register drives the SRAM's, with no address mux between.

| Space | Address | What |
|---|---|---|
| code | 0x0000 .. 0xFFFF | flash, 64 KB of the 512 KB; the chip's A16..A18 go to a 3-way jumper so one chip holds 8 programs |
| data | 0x0000 .. 0x003F | registers `$0` .. `$31`, `$n` at 2n |
| data | 0x0040 .. 0x3FFF | SRAM, 16 KB (two 8K x 8 chips); mirrored to 0x7FFF |
| data | 0x8000 + 2n | TL16C550 register n on the low byte (n = 0..7); the high byte of a load is garbage, mask it |

The flash is not readable as data, so constant tables and strings are
built with `li` and `sw` at start-up.  Section 6 has the fix if that
turns out to hurt.

**Reset.**  The MAX811L holds reset; the PC starts at code 0.  Programs
are written into the flash with the programmer, in the DIP-32 socket.

## 3. How it executes

One 16-bit data bus `D` joins everything: the flash's outputs, the
SRAMs, the UART's low byte, and the GAL latches.  Two address buses,
each driven by one register: the PC drives the flash, the data address
register MAR drives the SRAM and UART.

| Block | Bits | GALs | Notes |
|---|---|---|---|
| PC | 16 | 2 | counts by 2 per half-fetch; loads from D on a taken branch, jump or jr |
| IR high half | 16 | 2 | op, rs, rt; rs or rt driven onto D[15:11] to address a register |
| IR low half | 16 | 2 | imm, or rd and funct; driven onto D whole (imm to B, target to PC) or as D[15:11] (rd to address a register) |
| A latch | 16 | 2 | ALU operand, loaded from D |
| B latch | 16 | 2 | ALU operand, loaded from D |
| ALU | 16 | 4 | four 4-bit slices: add (ripple within the slice, carry between), and, nor, pass A, pass B, plus a not-equal output per slice for BEQ/BNE; outputs drive D through their output enables |
| MAR | 16 | 2 | loaded from the ALU result or, for a register access, with {0, index, 0} taken from D[15:11]; every SRAM access, register or data, goes through it |
| Control | | 3 | state counter, decode of op and funct, strobes (flash OE, SRAM CE/OE/WE, UART CS/RD/WR), latch enables, output enables, PC count and load, ALU function; the divide-by-two for the clock |
| | | **19** | plus or minus two once the pins are packed with `galpack` |

Every instruction is a fixed sequence of states, one clock each unless
marked:

```
F1   D = flash[PC]        -> IR high;   PC += 2
F2   D = flash[PC]        -> IR low;    PC += 2
RA   MAR = &rs;  D = SRAM -> A
RB   MAR = &rt;  D = SRAM -> B                (R-type, beq)
RI   D = IR low (imm)     -> B                (I-type)
X    ALU = f(A, B)                            (two clocks: 16-bit ripple)
WB   MAR = &rd or &rt;  SRAM = ALU            (write pulse in the second half)
MA   MAR = ALU                                (lw: the address; sw: pass B)
MR   D = data[MAR]        -> B                (two clocks)
MW   data[MAR] = ALU (pass A)                 (two clocks)
BR   if taken: D = IR low -> PC               (beq, j: PC load)
```

| Instruction | States | Clocks |
|---|---|---|
| addu, and, nor | F1 F2 RA RB X X WB | 7 |
| addiu, andi | F1 F2 RA RI X X WB | 7 |
| lw | F1 F2 RA RI X X MA MR MR X WB | 11 (the ALU passes B, the loaded word, to the register) |
| sw | F1 F2 RA' RB' X MA MW MW | 8 (RA' loads rt into A, RB' loads rs into B; MAR = B; the ALU passes A) |
| beq | F1 F2 RA RB X X BR | 7 |
| j | F1 F2 BR | 3 |

About 0.9 million instructions a second at 7.4 MHz.  Plenty.

**Why `sw` has no offset.**  A store needs three values, the base, the
offset and the data, and the machine has two latches.  Reading the data
register after the address is in MAR is not possible, because register
reads go through MAR too.  With a zero offset the address is a
register, so both reads happen first and MAR is loaded last.  A full
`sw rt, off(rs)` costs either two more GALs (a separate register-index
driver on the SRAM's address pins, so MAR survives a register read) or
four more states that park the computed address in a hidden 33rd
register and read it back.  `lw` does not have the problem: its data
arrives after the address is used up.

## 4. Timing, and why there are no delay lines

**Clock.**  One 14.7456 MHz oscillator drives the UART's XIN and a
divide-by-two flop in the control GAL: the CPU clock is 7.3728 MHz,
135.6 ns, 50 % by construction.  (8 MHz would need a second oscillator
for nothing.)

**Fetch.**  Flash tACC is 70 ns; the PC is valid 5.5 ns after the edge;
the IR needs 3.5 ns before the next: 79 of 135 ns.  One clock per half.

**Register and SRAM access.**  The SRAM is 15 ns.  A read is one clock;
a write's WE# is the write state gated with the clock's low half, a
68 ns pulse from the clock edge itself, no tap.  The data (ALU outputs
on D) has been stable since the X states.  Every margin is tens of
nanoseconds; there is nothing to bin.

**Data-space access (lw, sw).**  Two clocks, always, whichever chip is
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
| ATF22V10C-7 (DIP-24, socketed) | 19 | everything in section 3 |
| SST39SF040 (DIP-32, socketed) | 2 | code, 16 bits wide |
| AS7C164A 8K x 8 (DIP-28) | 2 | registers and data, 16 bits wide |
| TL16C550 (LQFP-48) | 1 | serial |
| SP3232 (SSOP-16) + 4 x 100 nF | 1 | RS-232 levels, two pairs |
| 14.7456 MHz oscillator | 1 | UART and CPU clock |
| MAX811L + button | 1 | reset |
| 5-pin serial header, 3-way bank jumper, decoupling | | |

About 28 placed parts, 23 of them DIP in sockets, and the two SMD parts
are the ones JLCPCB already had in stock.

## 6. Choices to argue about

1. **Registers in SRAM.**  Costs about four clocks per instruction, saves
   about eight GALs or the dual-port chips.  A bad program can overwrite
   its registers.  I would keep it: the whole point is a small board.
2. **16-bit data bus (two flashes, two SRAMs)** against an 8-bit bus with
   one of each.  8 bits halves the memory chips and the data traces and
   makes the UART a natural byte device, at the cost of a byte-phase in
   every state (roughly twice the clocks) and a little more control
   logic.  I lean 16 because the control is simpler and it uses the
   chips; the 8-bit version is the fallback if the GAL count must drop.
3. **Harvard.**  No address mux, but no constants in flash.  Making the
   flash readable as data costs the mux (about two GALs, MAR onto the
   flash address bus through output enables) and one more state in lw.
   I would start Harvard and add it if strings in `li`/`sw` get old.
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
   more chip), byte loads for the UART, `sw` with an offset (above).
6. **Clock.**  7.37 MHz from the UART's oscillator, or a separate 8 MHz
   can and the same numbers.  Everything above has a factor of two of
   margin at either.
