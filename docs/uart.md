# Bus wait and the UART

A serial console on the data bus: one 16550-class UART behind a wait
state that freezes the pipeline for a fixed number of clocks around every
I/O access.  Simulated end to end in `tests/uart.rs` (line set-up, three
characters received and echoed, transmitter drained, register read-back),
against the reference simulator, with the UART's datasheet bus timing
checked on every access by the chip model.

## 1. Parts (all SMD, JLCPCB-assemblable, LCSC stock checked 2026-09-07)

| Part | LCSC | Role |
|---|---|---|
| TL16C550DPTR (TI, LQFP-48) or TL16C550CPFBR / TL16C550CFNR | C544406 / C882798 / C2653186 | the UART. C and D have the same pinout (LQFP-48) and the same bus timing; the D runs 4.5 to 5.5 V and clocks to 24 MHz, the C 4.75 to 5.25 V and 16 MHz. Stock is a handful of pieces of each; MaxLinear ST16C550 (C2653205 PLCC-44, C517966 TQFP-48) is a functional substitute with a different pinout |
| SP3232EEY-L/TR (MaxLinear, TSSOP-16) | C13482 | RS-232 driver / receiver, 3 to 5.5 V, JLCPCB *basic* part. Four 0.1 uF charge-pump capacitors |
| 14.7456 MHz crystal, 3225, 12 pF (YXC X3225147456MOB4SI) | C2885591 | UART clock on XIN / XOUT with two load capacitors. 1.8432 MHz is through-hole only; 14.7456 / 16 = 921600 baud at divisor 1, divisor 8 for 115200 |
| ATF22V10C | 2 (uadp0, uadp1) | the bus adapter: slot decode, strobes, register select, ready |

The bus controller is one GAL (bus0), the adapter two; the hold inputs
the wait puts on the pipeline registers cost six more chips when the wait
was first added (forwarding control by two, operand B, branch target,
stall, MEM/WB).

## 2. Address space

An access is I/O when the **base register** has bit 31 set:

```
lui   $s0, 0xFFFF
ori   $s0, $s0, 0x8000     # or: addiu $s0, $zero, -32768
lw    $t0, 20($s0)         # LSR
sw    $t1, 0($s0)          # THR
```

The base register, not the effective address, because the decision has to
be known before the address is: the store write gate (`MMWB`, docs/memory-
timing.md section 6) is registered on the EX/MEM edge from the *forwarded*
operand A's bit 31, which is stable mid-way through EX, while the sum is
the ALU's critical path.  The data memory's CE# also follows it (`MIO`),
so nothing in data memory is touched by an I/O access whatever the offset.
In the reference simulator (`iss::is_io`, `iss::io_slot`) the rule is the
same.  The bus itself is defined in docs/bus.md.

The UART is bus slot 0 (address bits 26:23 zero); register select is
address bits 4:2, i.e. the 16550's eight registers at word offsets:

| Offset | Read | Write |
|---|---|---|
| 0 | RBR (DLL when LCR.7) | THR (DLL when LCR.7) |
| 4 | IER (DLM when LCR.7) | IER (DLM) |
| 8 | IIR | FCR |
| 12 | LCR | LCR |
| 16 | MCR | MCR |
| 20 | LSR | - |
| 24 | MSR | - |
| 28 | SCR | SCR |

The UART is a byte device (bus BB8): a word load returns its byte
zero-extended, a byte load at offset 0 the byte with the usual sign or
zero extension; stores of any size write bits 7:0 (a byte store's
replication puts the byte on lane 0 anyway).  A second device would take a second
chip-select term on the same wait.

## 3. On the bus

The UART is device 0 of the crag bus (docs/bus.md): base address
0x8000_0000, registers at word offsets 0 to 28.  Its adapter (two GALs,
uadp0 and uadp1) turns a bus access into the 16550's strobes and answers
ready when they have been long enough:

- Read: RD# low for 13 cycles (through the transfer cycle, since the
  16550 holds its data while RD is low), ready at cycle 12, so a read
  takes 14 clocks plus the turnaround; that also satisfies the 425 ns
  spacing the FIFO wants between reads.
- Write: WR# low for 3 cycles, ready one cycle after it rises (the
  16550's 5 ns data hold), so a write takes 6 clocks plus turnaround.
- A2..A0 are captured when the adapter selects and held until it
  releases, for the 16550's 20 ns address hold after RD.
- The adapter's select is the 16550's CS0 and the bus's byte-device flag
  BB8: the CPU zero-extends the byte the UART drives on lane 0.

What the pipeline does during the access is the bus's business (bus.md
section 3): everything holds, including the EX/MEM result register that
carries the address.

## 4. Timing (TL16C550C/D system timing requirements, ADS# low)

| Requirement | Datasheet | Design |
|---|---|---|
| CS / address valid before strobe start (td4, td5, td7, td8) | 7 ns | SEL and UA change 2 to 5.5 ns after the edge that selects; the strobe starts 2 ns after the next edge: 30.5 ns |
| Strobe width (tw6, tw7) | 40 ns | 3 or 13 cycles |
| RD to data valid (td10) | 45 ns max | strobe starts by cycle 1 + 5.5 ns; MEM/WB captures at the end of cycle 13: 400 ns later |
| Data hold after RD end | (float 0 to 20 ns after RD rises) | RD rises 2 to 5.5 ns after the capturing edge |
| Address hold after RD end (th7) / WR end (th4) | 20 / 10 ns | UA changes when SEL drops, a cycle or more later |
| CS hold after strobe end (th3, th6) | 10 ns | SEL drops a cycle after the last strobe |
| Data setup before WR end (tsu3) | 15 ns | store data drives DQ from the access's first cycle |
| Data hold after WR end (th5) | 5 ns | WR ends a cycle before the transfer; the drivers let go after it |
| Cycle time (tcR, tcW) | 87 ns | 6 or 14 cycles between strobes |
| FIFO mode read spacing | 425 ns | 14 cycles = 476 ns at 34 ns |
| Master reset (tw8) | 1 us | RESET lasts the boot copy, or 32 clocks plus the supervisor's timeout without one |

The chip model treats a strobe's 2 to 5.5 ns transition window as "the
edge is somewhere in here" and checks every requirement against the
pessimistic end.  The UART drives DQ[7:0] only; the model drives them
unknown between the strobe start and data valid, and again for up to
20 ns after the strobe ends.

The data memory during an I/O access: CE# (`DMEN_n = MIO | boot code
phase`) rises 3 to 7.5 ns after MIO, i.e. by 13 ns into the first cycle,
and the UART does not drive before its strobe.  The store gate does not
fire for an I/O store (`MMWB = XMW & !FA31`), so a WE# pulse never meets
a low CE#.

## 5. Board

- UART: CS0 = BB8 (the adapter's select, pulled down), CS1 high; CS2#
  low; ADS# low; RD1# = URD_n, RD2 low; WR1# = UWR_n, WR2 low; MR = RESET; D0-7 = DQ0-7; A0-2 = UA0-2; XIN /
  XOUT crystal; RTS# to CTS#, DTR# to DSR# and DCD#, RI# high (the MSR
  reads back the MCR, which the model reproduces); INTRPT, DDIS, TXRDY,
  RXRDY, BAUDOUT, RCLK: BAUDOUT to RCLK, the rest unconnected.
- SP3232: T1IN = SOUT, R1OUT = SIN, to a DE-9 or a 3-pin header (TX, RX,
  GND).  A USB-serial adapter with TTL levels can bypass it.
- The UART's XIN wants CMOS levels (0.7 VCC): a crystal, or a 5 V
  oscillator, not a 3.3 V one.

## 6. Software

Polling only (no interrupts, no CP0).  Set-up: LCR = 0x83, DLL / DLM =
divisor, LCR = 0x03 (8N1), FCR = 0x07 (FIFOs on).  Transmit: wait for
LSR.5 (THRE), write THR.  Receive: wait for LSR.0 (DR), read RBR.  The
test program in `tests/uart.rs` is the reference.

The reference simulator's UART (`uart16550::Core`) is timeless:
transmission is instant and the terminal's characters arrive as soon as
the program has started polling the line status.  The chip model spaces
both by the character time at the programmed divisor and crystal, so a
program that polls behaves identically on both; one that does not wait
for THRE can lose characters on the chip (a warning) and not in the
reference.
