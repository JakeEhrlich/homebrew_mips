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
- [ ] Clock buffer and trace-delay model: every hold margin is still
      "N minus skew" with skew unmodelled.
- [ ] Measured grades for the delay lines and the gate, once binned.

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
