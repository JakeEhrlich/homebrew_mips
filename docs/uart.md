# The serial port: a 16550 behind the bridge

A TL16C550 UART (16-byte FIFOs each way, hardware RTS/CTS flow control)
on the crag bus (docs/bus.md) as device 0, behind a six-GAL **bridge**
that makes it look like a memory: the CPU writes a command word in one
cycle and reads the result in one cycle, and the bridge runs the chip's
own slow strobes in between.  The same bridge, with a wider address
latch, turns any slow chip (a flash for the WAD, say) into a bus device.

Simulated end to end in `tests/uart.rs`: the netlist's bridge against
the chip model, which checks every bus-timing requirement of the
datasheet on every strobe; the program against the reference simulator.

## 1. Why a bridge and not the chip on the bus

The 16550's own bus wants 40 ns strobes, 87 ns cycles, 425 ns between
reads of its FIFO or status, a 20 ns address hold after RD and a 5 ns
data hold after WR.  The crag bus gives a device 25 ns and moves on.  So
the chip gets a private data bus and its own address, chip select and
strobes, all from the bridge's registers, and the CPU never waits.

Why the 16550 and not a UART built from GALs: buffering plus flow
control is what makes a serial link reliable when the other end streams.
The chip holds sixteen bytes each way and, in auto-flow mode, drops RTS#
when its receive FIFO is nearly full and stops transmitting while CTS# is
high, all without the CPU.  A GAL UART with one byte of buffer was built
first and worked; it is in the history, not on the board.

## 2. Registers (slot 0, base 0x8000_0000)

A byte device: reads return the byte on lane 0, zero above (bus BB8).

| Offset | Read | Write |
|---|---|---|
| 0 | RDATA: the result of the last read command | CMD: bits 7:0 data, bits 10:8 the 16550 register (A2..A0), bit 15 = read. Ignored while BUSY |
| 4 | STATUS: bit 0 BUSY | |

A read command takes 14 cycles (476 ns, which also covers the 425 ns
FIFO read spacing); a write command takes 5.  Software either polls BUSY
or, knowing the latency, spaces its instructions.  The 16550 register
map is the chip's (RBR/THR/DLL at 0, IER/DLM at 1, IIR/FCR at 2, LCR 3,
MCR 4, LSR 5, MSR 6, SCR 7).  Set-up for 115200 baud, 8N1, FIFOs and
auto-flow: LCR = 0x83, DLL = 8, DLM = 0, LCR = 0x03, FCR = 0x07, MCR =
0x22.  Send: read LSR until bit 5 (THRE), write THR.  Receive: read LSR
until bit 0 (DR), read RBR.  The test program in `tests/uart.rs` does
exactly this with a five-instruction command-and-wait sequence.

## 3. The bridge (chips sbr0 .. sbr5)

| Signal | Kind | What |
|---|---|---|
| SLOT0, SBCMD, SBRDD, SBRDS | comb | slot decode and the three accesses |
| START | comb | a command while not busy |
| SBA[2:0], UD[7:0], SBRW | reg | address, data and direction, latched by START. UD's pins are the chip's private data bus: driven by the latch during a write (UDOE), by the chip during a read |
| BUSY | reg | from START until K reaches 13 (read) or 4 (write); also the chip's CS0 |
| K[3:0] | reg | cycle count within the command |
| URD#, UWR# | reg | RD# low for K = 1..3, WR# low for K = 1..2 |
| LATCH | comb | K = 3 of a read: RDATA takes UD at the end of it |
| RDATA[7:0] | reg | the result |
| DQ[7:0] drivers | comb, tri-state | RDATA or STATUS onto lane 0, enabled from BRD# and SLOT0 |
| BB8 | comb, tri-state | byte device while selected |

Against the datasheet: CS valid a cycle before the strobe (7 ns needed);
RD 102 ns and WR 68 ns wide (40); data latched 85 ns after RD fell (45
max to valid); address held until the next command (20 after RD); data
held two cycles past WR (5); consecutive commands 14 cycles apart (87
cycle time, 425 FIFO spacing).  The chip model asserts all of these.

## 4. Board

- UD[7:0] to D0-7, SBA to A0-2, BUSY to CS0, CS1 high, CS2# low, ADS# low,
  URD# to RD1# with RD2 low, UWR# to WR1# with WR2 low, MR = RESET.
- Crystal 14.7456 MHz (3225, 12 pF) on XIN / XOUT with two 18 pF load
  capacitors; BAUDOUT to RCLK.
- SOUT, SIN, RTS#, CTS# through the SP3232's two driver / receiver pairs
  to a 5-pin header (GND, TX, RX, RTS, CTS) or a DE-9.  DTR# to DSR# and
  DCD#, RI# high.
- Parts: TL16C550DPTR (LCSC C544406; the C in LQFP-48 or PLCC-44 is the
  same pinout), SP3232EEY-L/TR (C13482, JLCPCB basic), crystal C2885591.
