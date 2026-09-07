# Boards

One directory per PCB.  Each holds a generated `netlist.json`, the single
description of that board, and a `chipmap.html` rendered from it.

| Board | What it is |
|---|---|
| `crag` | the full machine: MIPS-I pipeline, boot ROMs, reset, bus wait, UART. Plays Doom. |
| `grit`, `pebble`, `cobble`, `boulder`, `tor` | smaller boards on the way up, each proving one thing (the netlist flow, speed, the SRAM timing, ...). Not all will exist; more may. |

## The board file

`boards/<name>/netlist.json` (`src/board.rs`) carries:

- **chips**: name, orderable part number, package, diagram block and
  stage, an optional role (`rf:00`, `imem:2`, `dmem:1`, `rom:3`, `uart`,
  `supervisor`, `gate`, `dl:0`), the simulation model (`kind` = `gal`,
  `sram8k`, `sram16`, `dual_port1k`, `ds1100`, `gate`, `supervisor`,
  `rom`, `uart`, `passive`) with its parameters, and every connected
  pin: number, name, net, kind (`in`, `out`, `bidir`, `power`,
  `passive`).  For a GAL the model holds the equations and which signal
  each pin carries; the pin list holds the board net, which differs
  where nets were merged after packing.
- **nets**: name, tie to a rail, pull resistor, role (`clk`, `reset`,
  `reset_button`).
- **params**: the design parameters the wiring depends on (boot region
  size, data regions, SKIP, the write-gate tap).
- **layout**: the chip map's columns.

Power pins are on `VCC` / `GND`.  Passive parts (transceiver, crystal,
capacitors, connectors) have no simulation model and appear only for the
board.  Decoupling capacitors are not yet listed.

What is *not* in the file: programs, memory images, and test-bench
choices (delay-line tolerance grade, gate delay range, supervisor release
phase, terminal input).  Those are `board::Load` options.

## Flow

```
src/cpu.rs            the crag design: GAL blocks and wiring
  build_netlist()  ->  netlist with part metadata
  board()          ->  Board (export)
mips32 export crag                          > boards/crag/netlist.json
mips32 chipmap crag docs/chipmap_template.html > boards/crag/chipmap.html
Cpu::build()      = board() -> instantiate -> load images -> reset -> run
```

Every test goes through the board file: `Cpu::build` exports the design
and instantiates the export, so a test passing proves the file is
complete.  `tests/board.rs` checks that the committed file matches the
builder (regenerate after any design change), that every block is in the
layout, every GAL signal has a pin, every net is declared, and that a
test-sized boot variant differs from the board only in the boot wiring.

Coming: KiCad export from the same file; the JEDEC manifest keyed by a
hash of each GAL's equations and pins.
