# Pipeline plan (rough, phase 1)

Target: MIPS-I as the R2000 defined it, minus coprocessors, multiply/divide
and traps. Classic five stages, one branch delay slot, one load delay slot.
Parts: ATF22V10C-7 for logic and registers, CY7C131 for the register file,
AS7C164A for memory. Cycle budget 34 ns, all numbers worst-case datasheet.

## Instruction subset

Phase 1, enough to run compiled loops without shifts:

| type | instructions |
|---|---|
| R | ADDU SUBU AND OR XOR NOR SLT SLTU JR |
| I | ADDIU ANDI ORI XORI SLTI SLTIU LUI LW SW BEQ BNE |
| J | J JAL |

ADD, SUB and ADDI decode as their unsigned forms (no overflow trap).
NOP is `SLL r0, r0, 0`, all-zero, which matters below.

Phase 2: SLL SRL SRA SLLV SRLV SRAV (a barrel shifter, sized separately),
LB LBU LH LHU SB SH (byte lanes on the data SRAM), BLEZ BGTZ BLTZ BGEZ,
JALR.

## Memory

Harvard split, since IF and MEM both access memory every cycle and the
AS7C164A is single ported:

- Instruction memory: 4 x AS7C164A, 32 bits wide, 8K words (32 KB), read
  only from the pipeline's point of view. Preloaded in simulation; the
  bootloader writes it later.
- Data memory: 4 x AS7C164A, 32 KB, one WE# per chip so byte stores fall
  out in phase 2.
- Addresses: the chips see A[14:2]. No decoding, no protection. Code cannot
  read itself.

## Stages

Every pipeline register is a GAL registered output, and every register
absorbs the mux in front of it: a D input is a sum of products, so a 2:1 or
3:1 select costs product terms and input pins, not a chip. That is the
single biggest lever in this design, and it is why the mux-heavy places
below (PC, ID/EX operands, MEM/WB) are listed as "register with mux".

### IF

- PC register with a 4:1 mux folded in: PC+4, branch target, jump target,
  JR target. 30 bits (word aligned), 3 bits per GAL.
- PC+4 incrementer, 30 bits, from the PC register outputs, in parallel with
  the fetch.
- Instruction SRAM: address from PC, data captured into IF/ID at the next
  edge. PC lands at 5.5, data at 20.5, setup 3.5: 24 of 34.
- IF/ID register: instruction (32) and PC+4 (30). The instruction bits carry
  a `kill` term: `D = instr & !kill`, which turns the captured word into NOP
  for free when a taken branch squashes it.

### ID

- Decode GAL(s): opcode and funct to about ten control bits.
- Register file: the steer design from `tests/netlist.rs`. rs/rt from IF/ID
  at 5.5, steer GAL at 13, data at 28.5. Write port CE falls at 14 to 22 and
  rises by 34.
- Immediate: sign or zero extend by wiring the registered bit 15.
- Branch target adder: PC+4 plus sign-extended offset times 4, from IF/ID,
  in parallel with the register read. Result into ID/EX.
- Jump target: PC+4[31:28] joined with instr[25:0] times 4, pure wiring,
  selected straight into the PC register at the end of ID. Jumps cost only
  their delay slot.
- JR: rs data at 28.5 into the PC register's JR input, setup 3.5: 32 of 34.
- ID/EX register: operand A (32), operand B (32), immediate (32, but only 17
  distinct bits), PC+4 or branch target (30), destination and control (about
  12). The operand registers absorb a 2:1 mux: register file data, or MEM/WB
  data when the steer fired (the distance-3 forward).

### EX

- Forwarding muxes on both ALU inputs: ID/EX, EX/MEM, MEM/WB (distances 1
  and 2, including loads at distance 2). Select lines come from register
  number compares done in ID and registered.
- ALU: add, subtract, and, or, xor, nor, slt, sltu. Branch equality from
  the same subtract, or a separate comparator if that is faster.
- Budget: operands land at 5.5, capture setup 3.5, leaving 25 ns for the
  forwarding mux plus the ALU. That is three GAL levels including the mux,
  so either the ALU's first level absorbs the mux or the ALU is two levels
  deep. This is the critical path of the machine.
- Branch decision leaves EX; the PC register selects the branch target at
  the end of EX. The instruction fetched during EX is squashed via `kill`.
  Taken branches cost the delay slot plus one.
- EX/MEM register: result (32), store data (32), destination and control.

### MEM

- Data SRAM: address from EX/MEM at 5.5, data at 20.5, captured at 30.5.
- Stores: WE# strobe from the same clock-generator phase as the register
  file write (falls 14 to 22, rises by 34). Address must be stable before
  WE# falls and after it rises, which that window gives.
- MEM/WB register with a 2:1 mux folded in: ALU result or load data, plus
  destination and write enable.

### WB

- Register file write from MEM/WB during the CE_W window. The distance-3
  reader in ID is steered off and takes MEM/WB directly.

## Hazards

| case | handled by |
|---|---|
| ALU result, distance 1 and 2 | forwarding mux at ALU input |
| ALU result, distance 3 | register file steer plus ID/EX operand mux from MEM/WB |
| load result, distance 1 | load delay slot, software |
| load result, distance 2 | forwarding mux at ALU input from MEM/WB |
| load result, distance 3 | same as ALU distance 3 |
| branch | resolved in EX, delay slot runs, next instruction killed if taken |
| jump, JAL, JR | resolved in ID, no penalty beyond the delay slot |
| register 0 | write enable gated on rd != 0; preloaded zero |

No interlocks anywhere. Reset: PC forced to 0 through the GAL asynchronous
reset term, pipeline registers to NOP.

## Rough chip count, phase 1

| block | GALs |
|---|---|
| PC register with mux | 10 |
| PC+4 incrementer | 5 |
| IF/ID | 7 |
| decode, steer, forwarding control | 4 |
| branch target adder | 8 |
| ID/EX (operands with mux, immediate, PC, control) | 15 |
| ALU input forwarding muxes | 13 |
| ALU | 12 to 16 |
| EX/MEM | 9 |
| MEM/WB with mux, control | 8 |
| total | about 95 |

Plus 8 CY7C131 and 8 AS7C164A. Pipeline registers are roughly half of
the GALs. Phase 2 adds about 15 for the shifter and a handful for byte
lanes and the extra branches.

## Build order

Each step is a netlist of real chip models driven by a testbench, checked
against a reference, with no chip warnings, before the next one starts.

1. Reference: a tiny instruction-set simulator and assembler for the subset
   in Rust. This is the oracle for everything after.
2. GAL library: config generators for register-with-mux, incrementer slice,
   adder slice, equality comparator, decoder. Each with its own tests.
3. IF: PC, incrementer, instruction SRAM, IF/ID. Straight-line code.
4. ID: decode, register file, immediates, jump and JR into the PC.
5. EX: ALU and forwarding. ALU programs against the reference.
6. MEM and WB: data SRAM, loads and stores.
7. Branches and the squash.
8. Phase 2 instructions.

Then JEDEC export so the burned chips are the modelled chips, and the
logic-analyser comparison.
