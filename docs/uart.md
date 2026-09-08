# The serial port

A UART built from ten ATF22V10C, sitting on the crag bus (docs/bus.md) as
device 0.  8N1, LSB first, idle high, any rate from a programmable
divisor.  It replaced a TL16C550 and its adapter: the 16550's own bus
wants 40 ns strobes, 425 ns between reads and a 20 ns address hold,
which is a slow bus of its own, while the whole point of the crag bus is
that a device answers within the cycle.  The GALs do, the 16550 was thin
in stock, and the crystal went with it.

Simulated end to end in `tests/uart.rs`: the netlist's UART on the wire
against a bit-level terminal model, the program against the reference
simulator.

## 1. Registers

Base 0x8000_0000 (slot 0).  A byte device: reads return the byte on
lane 0, zero above (bus BB8); byte loads at offset 0 or 4 work as
expected.

| Offset | Read | Write |
|---|---|---|
| 0 | RXD: the last received byte; reading clears RXVALID | TXD: send the byte (when TXBUSY is clear; a write while busy is ignored) |
| 4 | STATUS: bit 0 TXBUSY, bit 1 RXVALID | |
| 8 | | DIV: bit period = DIV + 1 clocks |

115200 baud at 34 ns is DIV = 255: 8.70 us per bit, 114.9 kbaud, 0.3 %
off, well inside the 2 % a UART tolerates.  The receiver needs DIV >= 7
(section 3).

Software: write DIV once.  Send: poll STATUS until TXBUSY is clear, write
TXD.  Receive: poll STATUS until RXVALID is set, read RXD.  A character
takes 10 bit periods either way; the receiver holds one character while
the next arrives, so a loop that reads within one character time never
loses one.

## 2. Transmitter

- DIV register: 8 bits, written from DQ[7:0] on a store to offset 8.
- Bit-period counter TXB: an 8-bit down-counter reloaded with DIV on each
  tick and while idle; TXTICK when it reaches zero and the transmitter is
  busy.
- Shift register TXS[9:0], stored complemented so that a cleared register
  is an idle line: loaded with {stop, data, start} by a write to TXD
  when not busy, shifted right on every tick with idle filling in.  TXS0
  is the SOUT pin (an active-low output).
- Bit counter TXC: loaded with 10 by the start, decremented on each tick;
  TXBUSY from the start until the tick that takes it to zero.

## 3. Receiver

- SINS1, SINS2: two synchroniser flops on SIN (the first is the
  metastability stage, `sync` in the model), stored complemented so that
  reset reads as an idle line.
- A start bit (SINS2 low while idle) starts the bit-period counter RXB
  from DIV / 2, so the first tick lands in the middle of the start bit;
  after that it reloads with DIV, so every later tick is mid-bit.
  Detection latency is three clocks (two synchroniser flops and the tick
  register), so the samples sit 3 / (DIV + 1) of a bit late: DIV >= 7
  keeps them in the middle half.
- RXS[7:0]: the samples shift in on every tick; after nine ticks they
  hold d7..d0.
- RXC: 10 per character; on the tenth tick (the stop bit) RXDONE loads
  the holding register RXD from RXS and sets RXVALID.  A read of RXD
  clears RXVALID.  The receiver goes idle and the next start bit can
  follow at once.
- No parity, no framing error, no overrun flag: a character arriving
  while RXD is unread overwrites it on completion.

## 4. Chips

| Chip | Holds |
|---|---|
| uart0 | slot decode, the four access decodes, START, BB8, DIV bit 0 |
| uart1 | DIV bits 1..7 |
| uart2 | TXB, TXTICK |
| uart3 | SOUT and TXS1..7 |
| uart4 | TXS8..9, TXC, TXBUSY, SINS1..2 |
| uart5 | RXB, RXTICK, RXS0 |
| uart6 | RXS1..7, RXACT, RXC0..1 |
| uart7 | RXC2..3, RXDONE, VALID, RXD0..4 |
| uart8 | RXD5..7, DQ0..2 drivers |
| uart9 | DQ3..7 drivers |

The packing is the tool's; the equations are in `cpu::uart_block`.

## 5. Bus timing

The port meets the bus contract (bus.md section 1) with one gate level
in hand:

- Decode: MR valid 5.5 ns, SLOT0 13 ns, the access decodes 20.5 ns; a
  register written from DQ sees its enable with 10 ns of setup.
- Read: the DQ drivers enable from BRD# low and SLOT0, 15 ns at the
  earliest, data valid by 20.5 ns; the SRAM outputs are off by 10.5 ns
  (OEN) and the drivers are off by 20.5 ns into the following cycle,
  during which the SRAM stays off.
- BB8 valid by 20.5 ns, sampled at 23.

## 6. Board

- SOUT and SIN to the SP3232 (T1IN, R1OUT), its RS-232 side to a DE-9 or
  a 3-pin header (TX, RX, GND).  A USB-serial adapter with TTL levels
  can bypass the transceiver.
- No crystal: the bit clock is the CPU clock divided.  If the CPU period
  changes, DIV changes in software.
