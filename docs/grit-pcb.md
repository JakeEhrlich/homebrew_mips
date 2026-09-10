# Grit: the board

What sits around the machine of `docs/grit.md` on the PCB, and the bill
of materials.  The single source of truth is `boards/grit/netlist.json`
(regenerate with `cargo run --release -- export grit`): every part below
is a chip record there with its pins, its package and, where it is
settled, its LCSC part number.  Nothing in this layer is modelled; the
simulator drives CLK at the multiplexer's output exactly as the
oscillator would through it, and sees every other part as passive.

## 1. Ground rules

- **Assembly.**  JLCPCB places everything SMD.  Their through-hole
  service places the DIP sockets, the 2.54 mm headers and the DB9; if it
  will not take the DB9, that one is a hand job.  The GALs, flash and
  SRAM come to the board only as sockets: Jake supplies and programs the
  chips.
- **Sockets.**  GALs in DIP-24 300 mil sockets; flash in DIP-32 600 mil;
  SRAM in DIP-28 600 mil.  The ATF22V10C is the narrow DIP, so the
  socket must be the 7.62 mm one, not the 15.24 mm one that shares the
  catalogue page.
- **Power.**  5 V in on a 5.5/2.1 mm barrel jack, centre positive.  A
  1.1 A polyfuse, then a P-MOSFET against reverse polarity (drain on the
  input, source on the rail, gate to ground), then the rail: 100 uF
  electrolytic at the jack, four 10 uF spread across the board, 100 nF
  at every chip.  A power LED.  Nothing regulates: the supply is a 5 V
  wall adapter.  The MAX811L's threshold is 4.63 V, so a supply that
  sags below that resets the machine rather than letting it wander.
- **One name per net**, as everywhere: a pin is `chip.function`, a net
  has one name, active-low nets end in `_n`.

## 2. The clock, and single step

```
osc0 OUT ──── OSC ─────────────────┐
                                   │ I0
   VCC ─┐                          ▼
  swm0  ├── STEPMODE ─────────── S  umux0 (74LVC1G157) ── Y ── CLK ── every GAL
   GND ─┘                          ▲
                                   │ I1
  VCC ── 10k ──┬── STEPBTN_n ── uinv0 (74LVC1G14) ── STEPCLK
               │
  swstep0 ─────┤  (to GND while pressed)
               │
  100 nF ──────┘
```

- **Run.**  Slide switch to RUN: STEPMODE low, CLK is OSC, 4 MHz.
- **Step.**  Slide switch to STEP: CLK is the button.  The button pulls a
  10k / 100 nF node low; the Schmitt inverter squares the bounce
  (1 ms time constant, hysteresis on the way back); a press is one
  rising edge of CLK, so one clock of the machine, and the release is
  the falling edge.  Every LED and every header then shows a machine
  standing still between two edges.
- **Switching.**  The multiplexer is not glitch-free at the instant the
  select changes.  Hold the reset button, flip the switch, release.
- **Header.**  `jclk0`: CLK, OSC, GND, so an analyzer can see the clock
  the machine is on and the oscillator it came from.
- **The oscillator** is the one part not yet picked from stock: any
  4.000 MHz, 5 V CMOS, 4-pin SMD can (7050 or 5032; pin 1 enable, 2 GND,
  3 out, 4 VCC).  It must be a 5 V part: the multiplexer runs at 5 V and
  wants 3.5 V for a high, which a 3.3 V oscillator does not promise.

## 3. Reset

MAX811L: RST_n to the sequencer's flags chip, MR# to the reset button
(pulled up inside the part, debounced inside the part).  The button and
the power-on timeout are the same event to the machine: RST_n low, then
released at some instant, synchronised by RS1 and RESET.

## 4. Bank switches

Two 4-position DIP switches with 10k pull-ups.  A position ON pulls its
line low.

| Switch | Positions | Lines | What |
|---|---|---|---|
| swb0 | 1..4 | PBANK0..3 = program flash A15..A18 | 16 banks of 32 K words; all ON = bank 0 |
| swb1 | 1..3 (4 unused) | UBANK0..2 = microcode ROM A10..A12 | 8 microcode images; all ON = image 0 |

The simulation ties the bank nets low, that is, every switch ON.

Two 10k pull-downs (`rd0`, `rd1`) hold Addr14 and Addr15 low while
nothing drives the address bus, so the selects say "flash" in the idle
word at every handoff.

## 5. Debug LEDs

Thirty LEDs, 0603 red, 1k, behind four 74HC541 buffers so that no logic
net carries more than one CMOS input of load.  An LED shows the level of
its net's pin.  The three active-low nets (MEMRD_n, WE_n, RST_n) have
their LEDs hung from VCC instead, so they light when the strobe or the
reset is *active*; silk marks them.

| Buffer | Signals |
|---|---|
| ubuf0 | CLK, RESET, PCDRV, ADRV, MEMRD_n, WE_n, ALUOE, TDRV |
| ubuf1 | ALD, BLD, TLD, PCLD, PCINC, IRLD, NEL, AUX0 |
| ubuf2 | IR0..IR4, STEP0..STEP2 |
| ubuf3 | STEP3, F0, F1, AUX1, RST_n (three inputs spare, grounded) |

Plus the power LED.  At 4 MHz the CLK LED glows at half; in step mode it
shows the level, which is the point.  The IR and STEP LEDs read the
opcode and the step in binary; the control-line LEDs read the microword
as the pipeline register holds it.

## 6. Analyzer headers

Four 2 x 10 headers, 0.1", plus the clock header.  Every one carries
CLK, so any pod can trigger on the machine's edge.

| Header | Pins 1..20 |
|---|---|
| ja0 (address) | ADDR0..ADDR15, CLK, RESET, GND, GND |
| jd0 (data) | D0..D15, CLK, MEMRD_n, WE_n, GND |
| jc0 (control) | PCDRV, MEMRD_n, ALUOE, WE_n, ALD, BLD, PCLD, PCINC, IRLD, F0, F1, ADRV, TLD, TDRV, CLK, RESET, NEL, AUX0, AUX1, GND |
| js0 (sequencer) | IR0..IR4, STEP0..STEP3, NEL, CLK, RESET, RST_n, STEPMODE, OSC, AUX0, AUX1, GND, GND, GND |
| jclk0 | CLK, OSC, GND |

## 7. Serial

SP3232 with its four 100 nF charge-pump capacitors (`c1`..`c4`) and a
DB9 **female**, wired as a DCE, so a straight cable reaches a PC or a
USB-serial adapter: pin 2 is data out of grit, 3 data in, 7 the far
end's RTS (grit's CTS#), 8 grit's RTS# (the far end's CTS), 5 ground;
1, 4, 6, 9 open.  The UART runs on its own 14.7456 MHz crystal (the same
part as crag, in stock as a 3225) with two 18 pF loads; divisor 8 is
115200 baud.

## 8. Layout rules

1. **Clock skew under 2 ns between any two GALs.**  This is the one
   timing rule the model found (docs/grit.md section 8): the GAL's
   minimum clock-to-output is 2 ns, so a clock arriving at one chip
   more than 2 ns after another is a hold violation on the PC's
   self-addressed load.  2 ns is about 300 mm of FR4 trace, so it is
   met by routing CLK from the multiplexer as one net with no chip
   more than about 150 mm further from `umux0` than any other.  Do not
   daisy-chain CLK across the board and back.
2. **Everything else has 100 ns or more of margin** at 250 ns a clock;
   trace length does not matter for data, address or control.
3. **Decoupling** 100 nF within a few millimetres of each chip's VCC
   pin, on the same side where possible; the 10 uF parts one per
   region (GAL rows, ROM/SRAM row, UART corner, power entry).
4. **Sockets and headers on top**; the SMD is a single side too if the
   board allows, since one-sided assembly is cheaper and the debug
   side is the side you look at.
5. **Silk**: the LED names, the active-low marks on the three
   VCC-hung LEDs, the header pin 1s and the pin names, ON = 0 on the
   bank switches, RUN / STEP on the slide switch, centre-positive at
   the jack.

## 9. The PCB project (flatland)

`boards/grit/pcb/` is a flatland project generated from the board file,
built up in stages so that each step can be looked at before and after
routing:

```
cargo run --release -- export grit > boards/grit/netlist.json
python3 boards/grit/pcb/build.py --stage=N            # library, netlist, placement, hand wiring, check
python3 boards/grit/pcb/build.py --stage=N --route    # + freerouting, check, renders
python3 boards/grit/pcb/build.py --stage=7 --route --outputs   # + gerbers, BOM, pick-and-place
```

| Stage | Adds |
|---|---|
| 1 | program flash, SRAM, UART, on explicit bus wires |
| 2 | a0, a1 (the address drivers) |
| 3 | pc0, pc1, b0, b1, t0, t1 |
| 4 | the ALU |
| 5 | the sequencer at the left end of the bus; flags chip, microcode ROM and pipeline register above the data bus |
| 6 | oscillator, multiplexer, inverter, supervisor, transceiver |
| 7 | everything else: passives, LEDs and buffers, switches, headers, jack, DB9 |

**The buses are literal wires.**  Sixteen data lines and sixteen address
lines run the length of the board on the top layer at 1.0 mm pitch, the
data bus above the chip row and the address bus below it.  The DIPs sit
rotated so their data pins face the data bus and their address pins the
address bus; every bus pin gets a bottom-layer stub beside its column
(0.55 mm pitch, three slots outside the column and five inside, in an
order that no jog crosses another stub) to a 0.2/0.5 mm via on its
line.  The UART's 0.5 mm pads escape straight out to a via row and the
bus pads continue the same way.  Header pins on bus nets and the
select pull-downs get stubs too.  Every SMD pad on 5 V or ground gets a
via to the planes.  All of that is drawn by the generator before the
router runs.

**The router sees only what is left.**  Freerouting cannot leave the
LQFP's pads, cannot add plane vias, and treats protected wiring it
cannot "complete" as work, thrashing on it.  So the DSN it is given is
rewritten: nets that flatland already reports as one island are dropped
and their copper becomes keepouts, the escape vias of unfinished nets
become one-pin parts, and the inner layers are marked as power layers.
The residual nets (control lines, clock, reset, serial, LEDs) route in
seconds.  `--keep-redundant` on the import, because flatland's
redundant-segment pass is quadratic and takes hours on this board.

**Floorplan**, 300 x 175 mm, four layers, y up:

| Where | What |
|---|---|
| the chip row, y = 60 | seq0, rom0, rom1, ram0, ram1, uart0, a0, a1, pc0, pc1, b0, b1, t0, t1, alu0..alu3, left to right (GALs at 13 mm pitch, memories at 20) |
| above the data bus | seq1, uc0, uc1, mir0, mir1; the data-bus header; the bulk capacitors |
| top edge | 30 LEDs and their buffers; the DB9 at the top-right corner with the transceiver under it |
| left strip | the jack (plug from the left edge), fuse, MOSFET, 100 uF, power LED; the clock corner above them; the clock header top-left |
| bottom edge | bank DIP switches and pull-ups, the control and sequencer headers, the address header under B and T where no stubs run |

Decoupling sits over the socket column whose stubs run the other way.
Design check: 0 errors with everything placed and the buses drawn,
before routing.  The unverified footprints (SOT-143, the switches, the
oscillator, the DB9's pin order) are the things to hold the datasheets
against before ordering.

## 10. Bill of materials

From the board file.  "Verify" marks a number believed to be a JLCPCB
basic part but not confirmed this session.

| Qty | Part | Package | LCSC | Refs |
|---|---|---|---|---|
| 16 | ATF22V10C-7PX (Jake supplies) in DS1009-24AT1NX-0A2 socket | DIP-24 300 mil | socket to source | seq0..1, mir0..1, pc0..1, a0..1, b0..1, t0..1, alu0..3 |
| 4 | SST39SF040-70-4C-PHE (Jake supplies) in DS1009-32AT1WX-0A2 socket | DIP-32 600 mil | C72122 (socket) | rom0..1, uc0..1 |
| 2 | AS7C164A-15PCN (Jake supplies) in DS1009-28AT1WX-0A2 socket | DIP-28 600 mil | C72121 (socket) | ram0..1 |
| 1 | TL16C550DPTR | LQFP-48 | to confirm (stock thin) | uart0 |
| 1 | SP3232EEY-L/TR | TSSOP-16 | C13482 | xcvr0 |
| 1 | Crystal 14.7456 MHz 12 pF | 3225 | C2885591 | x1 |
| 1 | MAX811LEUS+T | SOT-143 | to confirm | rst0 |
| 1 | Oscillator 4.000 MHz 5 V CMOS | 7050 | to pick | osc0 |
| 1 | 74LVC1G157GW,125 multiplexer | SC-88 | C135822 | umux0 |
| 1 | SN74LVC1G14DBVR Schmitt inverter | SOT-23-5 | C7835 | uinv0 |
| 4 | 74HC541D,653 octal buffer | SOIC-20 | C126008 | ubuf0..3 |
| 1 | AO3401A P-MOSFET | SOT-23 | C15127 | q0 |
| 1 | Polyfuse 1.1 A SMD1206P110TF/16 | 1206 | C523825 | f0 |
| 1 | CUI PJ-002AH-SMT barrel jack 5.5/2.0 mm | SMT | C22434687 | jpwr0 |
| 1 | DB9 female right angle | THT | C9900026339 (JLC assembly library) | j1 |
| 2 | TS-1187A-B-A-B tactile switch | SMD | C318884 | swstep0, swr0 |
| 1 | MSK12C02 slide switch | SMD | C431540 | swm0 |
| 2 | EM-04-Q DIP switch 4 positions | SMD | C501635 | swb0, swb1 |
| 4 | Header 2 x 10, 2.54 mm | THT | to pick | ja0, jd0, jc0, js0 |
| 1 | Header 1 x 3, 2.54 mm | THT | to pick | jclk0 |
| 1 | 100 uF 16 V electrolytic RVT1C101M0605 | SMD 6.3 x 5.4 | C970684 | cb0 |
| 4 | 10 uF 0805 X5R 25 V | 0805 | C15850 | cb1..4 |
| 37 | 100 nF 0603 X7R | 0603 | C14663 | c1..4, cd_*, cst0 |
| 2 | 18 pF 0603 C0G | 0603 | C1653 (verify) | xc1, xc2 |
| 30 | LED red 0603 | 0603 | C2286 (verify) | ledp0, led_* |
| 30 | 1k 0603 | 0603 | C21190 | rlp0, rl_* |
| 10 | 10k 0603 | 0603 | C25804 | rstep0, rp_*, rd0, rd1 |

161 parts.  Still to source: the 300 mil DIP-24 socket's LCSC number,
the 5 V oscillator, the headers, and confirmation of the UART and the
supervisor in stock.
