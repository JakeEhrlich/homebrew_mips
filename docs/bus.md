# The crag bus

The memory-mapped I/O bus: how a load or store reaches a device and what
the device must do.  Every peripheral is a device on this bus; the
instruction and data SRAMs are on their own paths (docs/memory-timing.md)
and stay on the board.

The rule is short: **a device is a memory.**  It answers within the cycle,
exactly as the data SRAMs do.  There is no ready line, no wait state, no
handshake, and the pipeline never stalls for I/O.  Anything that cannot
answer within the cycle presents registers that can: a command register
the program writes, a status or data register it reads back some known
number of instructions later.  "Block until finished" is a two-instruction
poll loop, which is what the software does anyway.

Simulated in `tests/uart.rs` (the serial port, a device built from GALs,
end to end over the wire) and `tests/bus.rs` (an empty slot).

## 1. The contract

Times from the rising clock edge that starts the access's MEM cycle.

| | CPU | Device |
|---|---|---|
| Address BA[26:2], byte enables BBE[3:0]#, direction | valid by 5.5 ns, held to the next edge | decode the slot combinationally; do not latch |
| Read strobe BRD# | low from 13 ns to 13 ns after the next edge | drive BD from BRD# low and the slot match, so no earlier than 15 ns; data valid by 30.5 ns; release within 20 ns of BRD# rising |
| Read data BD[31:0] | captured at 30.5 ns (3.5 ns before the next edge) | |
| Write strobe BWR#, write data BD | data valid by 8 ns, held to the next edge + 2 ns; strobe low 13 ns to 13 ns after the next edge | take the data at the edge that ends the strobe cycle, or at any edge while the strobe is low |
| BB8 | sampled at 23 ns | high while selected if the device drives BD[7:0] only (the CPU zero-extends); tri-state otherwise |

A device therefore gets 25 ns from address to data on a read, and one
full cycle of stable data on a write.  A register bank in a GAL meets
both with one gate level to spare.

## 2. Signals

| Signal | Direction | Width | Source |
|---|---|---|---|
| BA[26:2] | CPU to device | 25 | MR[26:2], the EX/MEM result register |
| BBE[3:0]# | CPU to device | 4 | MBE0..3_n (chip macc0): lanes written by a store, all low for a load |
| BD[31:0] | both | 32 | DQ: the EX/MEM store-data drivers for a write, the device for a read |
| BRD#, BWR# | CPU to device | 2 | chip stl0: MIO and the direction |
| BB8 | device to CPU | 1 | the selected device, tri-state, pulled down |
| CLK, RESET | CPU to device | 2 | |
| IRQ | device to CPU | 1 | reserved: one line into a status bit the CPU can poll |
| 5 V, GND | | | |

## 3. Address space

An access is I/O when the **base register** (rs) has bit 31 set, not the
effective address: the CPU decides from the forwarded operand in EX,
before the sum exists, so the SRAM write gate and chip select are held
off in time.  The effective address selects the device:

| Bits | Use |
|---|---|
| 31 | I/O (from the base register; the sum's bit 31 is not decoded) |
| 30:27 | ignored |
| 26:23 | **slot**: 16 devices, 8 MB each |
| 22:2 | within the device |
| 1:0 | byte lane |

Slot 0 is the serial port (docs/uart.md).  A slot with nothing in it
reads whatever the floating bus holds and ignores writes; nothing hangs.
Software base addresses: 0x8000_0000 + slot * 0x0080_0000.

## 4. What the CPU does around an access

- MIO (EX/MEM control): the access in MEM is I/O.  It deselects the data
  SRAMs (CE#) and suppresses the write pulse, and makes the strobes.
- OEN (chip stl0, registered): the data SRAMs' outputs are turned off for
  an I/O load's MEM cycle and the one after, the way they are for a
  store, so the device and the SRAM never drive DQ together: the SRAM is
  off by 10.5 ns, a device may drive from 15 ns; the device is off by
  20.5 ns into the next cycle, the SRAM back on the cycle after.
- HOLD: a memory load in ID right behind an I/O load in EX is held one
  cycle, as a load behind a store is, because the SRAM is still off in
  the cycle after the I/O access.  A store behind any load already holds.
- Narrow loads pick their lane and sign in MEM/WB as for memory; a byte
  device's upper bytes read as zero (BB8).

Nothing else changes.  A load from a device is one cycle, a store to a
device is one cycle, and no pipeline register has a hold input for I/O.

## 5. Devices that are not memories

- **Slow chip behind registers.**  The program writes a command (an
  address, a byte) into a register; the device works at its own pace and
  the program reads a status bit, or simply reads the result after a
  known number of instructions, in the spirit of the load delay slot.
  A flash chip: write the address, read the word three instructions
  later.  The serial port: write a byte, poll busy.
- **Display.**  A framebuffer the program writes into a cycle at a time,
  a swap register; the card scans its own memory out.
- **Input.**  A latch the program reads; the card blocks its own updates
  while BRD# is low so a read is never torn.
- **Audio.**  A sample buffer the program fills and a counter that clocks
  samples into a DAC, with a half-empty flag.

The WAD does not need any of this if it fits in RAM: the boot ROMs are
copied into data memory before the CPU starts (docs/boot.md).

## 6. Off the board (not built)

- Connector: DIN 41612, 96 pins, rows for the 65 signals with the rest
  ground and 5 V.  A 2 x 40 header is the cheap first backplane.
- Data transceivers: BD must not be the CPU's DQ once a backplane hangs
  on it; two 74ABT16245 (or eight GALs as buffers) between DQ and the
  connector, direction from BRD#, enabled by MIO.  BA, BBE# and the
  strobes are registered or single-level GAL outputs and can drive a
  short backplane directly.
- A memory card is the experiment that says whether the SRAM write
  timing (zero margin against the pulse, hold margins of 2 to 4 ns
  minus skew) survives a connector.  The protocol is identical; only
  the skew budget changes.

## 7. History

The bus was first built with a device-driven ready line and a
turnaround cycle (a minimum of two clocks plus one per access, because
the register-file write copies, clocked by an 18 ns delay-line tap,
need the hold decision by 11.5 ns and a device's ready cannot be known
that early), then as fixed wait states.  Both were removed: the machine
polls its peripherals and its only slow chips are memories with fixed
access times, so every device can be a memory-shaped register file, and
the CPU is simpler without any I/O stall at all (154 GALs instead of
162, before the serial port's own ten).
