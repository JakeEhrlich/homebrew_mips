# Boot: how the SRAMs get their contents

The instruction and data memories are volatile. At power-on a small copier
fills both from four parallel flash chips while the CPU is held in reset,
then hands the machine to the reset synchroniser. No CPU logic runs during
the copy; the CPU's own address counter and write gate are borrowed for it.

Simulated end to end in `tests/boot.rs` (instruction memory and data memory
left unknown, filled from ROM images, program then runs against the
reference), from three different supervisor release phases.

## 1. Parts

| Part | Count | Role |
|---|---|---|
| SST39SF040 (512K x 8 flash, 5 V, 70 ns, DIP-32 / PLCC-32) or any JEDEC 512K x 8 EEPROM / flash | 4 | one per byte lane of the instruction bus; each holds its lane's byte of every code word and every data word |
| ATF22V10C | 3 | sequencer (chips bseq0-2) |
| ATF22V10C | 2 | boot address buffers (badr0, badr1): PC and region number onto the data-memory address nets |
| ATF22V10C | 4 | boot data buffers (bdat0-3): instruction bus onto the data bus during the data phases |

Nine GALs, four ROMs. Nothing else is added: the address counter is the
PC, the write pulses are the existing ones. (Net +8 GALs: the forwarding
control lost its async reset and repacked one chip smaller.)

For development, a microcontroller with 19 address inputs and 8 data outputs
can sit in one ROM socket and emulate it, so the image changes without
reprogramming flash. That is a header, not a design change.

## 2. ROM image layout

Lane `l` (byte `l` of each word) goes in ROM `l`. Within a ROM:

| Address | Contents |
|---|---|
| A18 = 1, A12..0 = word index | code: instruction word `w`, byte `l` |
| A18 = 0, A17..13 = region `p`, A12..0 = word index | data region `p`: data word `p * 8192 + w`, byte `l` |

A region is 8K words (32 KB). Region 0 is data addresses 0..32K, region 1
the next 32 KB, and so on: 32 regions cover the 1 MB data memory. How many
regions are copied is a wired constant (NPH, five pins: the number of the
last region), so a small program need not wait for 1 MB.

The simulator's `Boot::Copy { code_words_log2, data_regions, data }` uses
smaller regions to keep tests short; the logic is identical, only the wiring
of WRAP and of the region bits moves.

## 3. Sequence

1. Power-on. The supervisor holds RST_n low; the synchroniser's registers
   power up with RESET asserted; the sequencer's registers power up in
   the "boot pending" state (BOOT = 1, CODE = 1, DONE = 0). The ROMs' OE#
   is low (ROMOE_n = DONE), so they already drive the instruction bus;
   instruction memory's OE# is high (BOOT) and the EX/MEM chips have let
   go of the data-memory address nets.
2. The supervisor releases; RS1 (its synchronised copy) falls. The
   sequencer starts counting (BOOTCNT). RESET_PC = RESET & !BOOTCNT
   drops, so the PC register leaves reset while everything else stays in
   it. One cycle later the address buffers take the data-memory address
   nets (BOOTCNTD).
3. Code phase, one word per five clocks:
   - steps 0-2: the PC has just changed; the ROM's 70 ns access time
     passes (3 cycles = 102 ns);
   - step 3: instruction memory's CE2 (IMEN) goes high and its WE#
     (BOOTWI_n) low for the whole cycle: a 30 ns write of the ROM byte on
     each lane at address PC;
   - step 4: both go back; the address is still held (the PC advances at
     the end of this step, HOLD is low only here).
   The PC counts 0..8191; PC15, the incrementer's carry, is WRAP.
4. WRAP: the sequencer clears the PC (PCCLR, through RESET_PC), sets
   CODE = 0, loads PHASE = NPH, restarts the step counter.
5. Data phases: the same five steps, but the ROM data goes through the
   data buffers onto the data bus (BOOTD) and the write is a data-memory
   store: MMWB high during step 3 makes the store gate produce its 7 ns
   pulse, the same pulse the CPU's stores use. The data-memory address is
   the PC plus the region number through the address buffers; instruction
   memory is deselected (IMEN low). At each WRAP the PC clears and PHASE
   counts down; when PHASE = 0 wraps, DONE.
6. DONE: BOOTCNT stops, the ROMs turn off, the address buffers release, a
   cycle later BOOT drops (instruction memory and the EX/MEM chips drive
   again), the synchroniser sees RS1 low and DONE high and releases RESET
   two edges later. RESET_PC follows RESET, so the PC starts from 0.

Time on the board: 8K words of code plus 32 regions of data at 5 clocks
per word, 34 ns: about 46 ms, comfortably inside a supervisor's reset
timeout only if the supervisor is *not* what ends the boot: it is not.
RESET is released by DONE, however long the copy takes.

A manual reset (the button on the MAX811L's MR#, RST_n low again)
restarts the whole sequence and recopies. The part debounces the button,
and a short low that only lands between clock edges is simply missed by
the synchroniser, never captured as unknown. Tested in
`reset_button_recopies_and_reruns` (three press phases, mid-program).

## 4. Timing points the simulator checks

- Instruction-memory write: CE2 and WE# change 2 to 5.5 ns after the edge
  starting step 3 and again after the edge starting step 4; the write is
  their overlap, at least 28.5 ns, ending before the PC changes 34 ns
  later (tHA 0, 28 ns of margin).
- Data-memory write: identical to a store (see memory-timing.md section
  6.1): the address (PC through the buffers) and data (ROM through the
  data buffers) are stable from step 0.
- Bus hand-overs: ROM off (DONE + 7.5 + 25 ns) before instruction memory
  on (BOOT falls a cycle later, + 5 ns): 6.5 ns. Address buffers off
  (BOOTCNT + 7.5) a full cycle before the EX/MEM chips on (BOOTCNTD).
  Data buffers off (BOOTCNT) a full cycle before OEN lets data memory
  drive.
- The forwarding-control registers have no async reset any more: under
  reset they settle to "no hit" from their inputs, so an undriven EX/MEM
  result bus during the copy never reaches the ALU.

## 5. PCB notes

- ROM sockets on the instruction bus; ROM WE# tied high; ROM address
  A0-12 from PC[14:2], A13-17 from PHASE, A18 from CODE.
- NPH0-4 and SKIP as jumpers or solder bridges (SKIP high boots without
  copying, for a board with a debugger filling the SRAMs).
- The boot buffers add one load per instruction-bus line and one driver
  per data-bus line; keep them near the SRAMs.
