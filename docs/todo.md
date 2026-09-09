# What is left

The crag model is the north star: it says which parts and which timing
the machine needs.  Physical work comes incrementally through the
stepping-stone boards (grit and its successors, roadmap to follow) and
is not listed here.

## Verification of the current design

- [x] Soak test: random programs over the whole instruction subset
      against the reference simulator (`tests/soak.rs`; `SOAK_PROGRAMS`
      and `SOAK_SEED` scale it).
- [x] Realistic programs: sort, byte strings, recursion with a stack,
      a table-driven checksum (`tests/programs.rs`).
- [x] Physical fuzz (`tests/fuzz.rs`, docs/fuzz.md): per-pin trace
      delays, clock duty and jitter, random power-up contents.  Found
      that the clock must be a divided 50 % clock and that the delay
      lines must be binned to +-2 ns.
- [x] Back-edge slack (`mips32 slack`, docs/slack.md): the longest
      backward traces are on the branch-resolution path, 2.5 ns of slack
      shared by two back edges at 34 ns.
- [ ] Per-net loading in the model: the wide back buses carry 7 to 14
      inputs and the GAL timings are at the test load.
- [ ] Clock buffer model: the fuzz gives every pin its own delay, but
      the buffer tree's structure (which chips share a branch) is not
      modelled.
- [ ] Measured grades for the delay lines and the gate, once binned.

## grit (the first board)

- [x] Design, model, netlist, board file, soak and fuzz (docs/grit.md).
- [ ] Bank jumper headers and the Addr14/Addr15 pull-downs as parts in
      the board file (they are nets and pulls today).
- [ ] The PCB (Jake, flatland).
- [ ] The macro assembler (MIPS-looking syntax over the grit ISA).
- [ ] Why the fuzz fails at 6 ns per pin: a PC bit's flop blinks unknown
      at an edge with PCLD low (an unknown on D reaching a minimised
      term?), and the flash's address goes unknown through a fetch.
- [ ] JEDEC manifest for the 13 GALs and the two ROM images.

## Upgrades needed for Doom

- [ ] Program size: 13-bit PC, 32 KB instruction memory.  Wider PC through
      the incrementer, branch adder and boot copier; larger instruction
      SRAMs; bigger code region in the ROM layout.
- [ ] Data memory: 1 MB.  Whether the WAD fits is the rewriter's call; if
      not, a flash behind a bridge like the serial one.

## Cards (each its own board file)

- [ ] Serial: move the 16550, its adapter, crystal and transceiver off
      the main board.
- [ ] Timer: a free-running counter, polled.
- [ ] Display: framebuffer with its own scan-out, a swap register.
- [ ] Input: a gamepad latch, updates blocked while read.
- [ ] Audio: a sample buffer and a DAC clock, half-empty flag.

## Board file and tooling

- [ ] Main board as CPU + memories + boot + reset + clock + connector; a
      machine is a main board plus cards, joined at the connector nets.
- [ ] Oscillator as a part in the file (CLK is still stimulus).
- [ ] Decoupling capacitors as a generator rule.
- [ ] KiCad export and the JEDEC manifest, both from the board file.

## Parked

- Multiply and divide (Jake's design).

## Settled

- GALs in DIP-24 sockets: programmable, and it looks the part.
- Boot ROMs are DIP-32 (as bought).
- The bus is fast-only; slow chips go behind a bridge (docs/bus.md).
- The clock is a 2x oscillator divided by two; the delay lines are
  binned (docs/fuzz.md).
