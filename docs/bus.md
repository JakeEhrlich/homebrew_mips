# The crag bus

The memory-mapped I/O bus: how a load or store reaches a device, what the
device sees, and what it must do.  Every peripheral (the UART today; a
timer, a display, a gamepad port, WAD flash, audio later) is a device on
this bus.  Nothing else is: the instruction and data SRAMs are on their
own paths with their own timing (docs/memory-timing.md), and stay on the
board.

The bus is synchronous to the CPU clock.  Devices are clocked from it
and meet ordinary setup and hold at its rising edges; there is no
asynchronous handshake and no synchroniser.  A device that needs one
puts it on its own card and pays the latency itself.

Simulated in `tests/uart.rs` (the UART through its adapter) and
`tests/bus.rs` (an empty slot).  Everything below is on the board today;
the connector and the transceivers that would take it off the board are
in section 7.

## 1. Signals

| Signal | Direction | Width | Source | Meaning |
|---|---|---|---|---|
| BA[26:2] | CPU to device | 25 | MR[26:2] (EX/MEM result register) | word address; bits 30:27 of the effective address are ignored, bit 31 makes it I/O |
| BBE[3:0]# | CPU to device | 4 | MBE0..3_n (chip macc0) | byte lanes written by a store; all low for a load |
| BD[31:0] | both | 32 | DQ | data: driven by the EX/MEM store-data drivers for a write, by the device for a read |
| BRD# | CPU to device | 1 | bus0 | read strobe: an access is a read and is in progress |
| BWR# | CPU to device | 1 | bus0 | write strobe |
| BACK | CPU to device | 1 | bus0 | the transfer cycle: the access completes at the end of this cycle |
| BRDY | device to CPU | 1 | the selected device (tri-state, pulled down) | the device is ready: sample me at the next edge |
| BB8 | device to CPU | 1 | the selected device (tri-state, pulled down) | byte device: only BD[7:0] is driven on a read, the CPU zero-extends |
| CLK | CPU to device | 1 | | the CPU clock |
| RESET | CPU to device | 1 | | active high, synchronous release |
| IRQ | device to CPU | 1 | reserved | one line into a status bit the CPU can poll; no interrupts exist |

Power: 5 V and ground.

## 2. Address space

An access is I/O when the **base register** (rs) has bit 31 set, not the
effective address: the CPU decides from the forwarded operand in EX,
before the sum exists, so the SRAM write gate and chip select can be
held off in time.  The effective address then selects the device:

| Bits | Use |
|---|---|
| 31 | I/O (from the base register; the sum's bit 31 is not decoded) |
| 30:27 | ignored |
| 26:23 | **slot**: 16 devices, 8 MB each |
| 22:2 | within the device |
| 1:0 | byte lane (byte enables on a store, lane select on a load) |

Slot 0 is the UART.  A slot with nothing in it reads as garbage after the
timeout (section 4) and ignores writes.  Software base addresses:
0x8000_0000 + slot * 0x0080_0000.

## 3. An access, cycle by cycle

The pipeline holds (`WAIT`) for the whole access: PC, IF/ID, ID/EX,
EX/MEM (including the result register, so the address stays on BA),
MEM/WB, and the register-file write copies all freeze.  Times are from
the rising edge that starts each cycle.

**Cycle 1** (the access reaches MEM): BA, BBE#, and for a write BD are
valid 5.5 ns after the edge (registered).  BRD# or BWR# falls at 13 ns
(combinational from the registered MIO, direction and turnaround flags:
glitch-free).  The device decodes its slot and starts.

**Cycles 1 .. n**: the strobe stays low.  When the device can complete
the access it raises BRDY, with read data valid on BD, in time for the
edge (setup 3.5 ns for a GAL sampling it; the CPU samples BRDY into
RDYS at the edge).  A device that is always ready ties BRDY to its own
select: BRDY at 13 + 7.5 = 20.5 ns, in time.

**Cycle n+1**, the transfer cycle: RDYS is high, WAIT falls at 13 ns,
BACK rises at 13 ns.  The device holds its read data.  At the end of
the cycle the CPU captures BD into MEM/WB and the pipeline moves.  A
device that must know exactly when a write is taken latches BD at the
edge where it sees BACK high.

**Cycle n+2**, turnaround (T): the strobes are high for one cycle, BACK
low.  The device releases BD and drops BRDY and BB8.  A back-to-back
access starts its cycle 1 after this one.

So the minimum access is two clocks plus one of turnaround: cycle 1
with BRDY answered immediately, the transfer cycle, T.  A fast device
adds nothing; a slow one adds cycles before raising BRDY.

Why two and not one: the register-file write copies are clocked by a
delay-line tap 18 ns into the cycle and must know by then whether the
pipeline will move at the end of it; BRDY cannot be known that early, so
the decision is made from the *registered* copy of it, one cycle later.

## 4. Timeout

The controller counts cycles while the strobe is out and no ready has
been registered; at 31 it completes the access itself (RDYS set from the
count).  An empty slot therefore costs 33 clocks and returns whatever
the floating bus holds.  The count is 5 bits; a device that needs longer
than 31 cycles for one access does not fit this bus and should buffer.

## 5. What a device must do

- Decode its slot from BA[26:23] combinationally; qualify everything
  with BRD# or BWR#.
- Drive BRDY high only while selected and ready, and BB8 high while
  selected if it is a byte device; tri-state both otherwise (the bus
  pulls them down).  Never drive them while the strobes are high.
- On a read, drive BD from when it is ready until the strobes rise
  (the T cycle), and tri-state otherwise.  A byte device drives BD[7:0]
  only and asserts BB8.
- On a write, take BD at the edge where BACK is high, or any earlier
  edge after it saw the strobe if its data path is registered.
- Keep every input to the CPU registered or a single GAL level from
  registered signals: BRDY and BB8 must be valid 23 ns after the edge at
  the latest (the CPU's WAIT and load selects are one level behind them
  with 3.5 ns of setup).
- Reset: on RESET, release everything.

## 6. The controller (chip bus0)

One ATF22V10C on CLK, synchronous reset:

| Output | Kind | Equation |
|---|---|---|
| RDYS | reg | (BRDY or CNT = 31) and MIO and not T |
| T | reg | RDYS and MIO and not T |
| CNT[4:0] | reg | counts while MIO and not T and not RDYS, saturates at 31, else 0 |
| BRD# | comb | not (MIO and MMR and not T) |
| BWR# | comb | not (MIO and MMW and not T) |
| BACK | comb | MIO and RDYS and not T |

and in the stall chips: WAIT = MIO and (T or not RDYS); HOLDW = HOLD or
WAIT.  The write copies rebuild the same condition from MIO, T and RDYS
(all registered) because WAIT itself is too late for their 18 ns tap.

## 7. Off the board (not built)

- Connector: DIN 41612, 96 pins (3 x 32), rows for the 66 signals with
  the rest ground and 5 V.  A 2 x 40 header is the cheap first
  backplane.
- Data transceivers: BD must not be the CPU's DQ once a backplane hangs
  on it; two 74ABT16245 (or eight GALs as buffers) between DQ and the
  connector, direction from BRD#, enabled by MIO.  BA, BBE# and the
  strobes are registered GAL outputs and can drive the connector
  directly on a short backplane.
- IRQ: reserve the pin now.

## 8. The UART as a device (chips uadp0, uadp1)

The first device, and the template.  Two GALs on CLK turn a bus access
into what the TL16C550 needs (docs/uart.md section 4):

- SEL: registered, set by a strobe with slot 0 decoded, cleared when the
  count runs out.  SEL is the 16550's CS0 and drives BB8.
- RDW: the direction, captured with SEL.
- K[3:0]: cycle count within the access.
- URD#: low for K = 1 to 13 (through the transfer cycle: the 16550 holds
  its data while RD is low).  BRDY at K = 12.  Fourteen cycles per read,
  which also satisfies the 425 ns FIFO read spacing.
- UWR#: low for K = 1 to 3; BRDY at K = 4, a cycle after the strobe
  ended (the 16550's data hold); six cycles per write.
- UA[2:0]: BA[4:2] captured when SEL rises and held until it falls, for
  the 16550's 20 ns address hold after RD.

A device with a simpler bus (a register file of GAL flops, say) needs
only the slot decode and BRDY = select: one chip, or part of one.
