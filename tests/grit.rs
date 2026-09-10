//! grit (docs/grit.md): the GALs fit, the microcode obeys its rules, and
//! programs run on the netlist as the reference interpreter says.
use mips32::grit::{assemble, microcode, gal_specs, Build, Grit, Iss, Op};

fn run(src: &str, max_clocks: u64, opt: Build) -> (Grit, Iss) {
    let p = assemble(src).unwrap();
    let mut iss = Iss::new(&p.words);
    iss.uart.send(&opt.uart_rx);
    iss.run(100_000).unwrap();
    let mut g = Grit::build(&p.words, &opt);
    assert!(g.run_until_halt(max_clocks), "netlist did not halt in {max_clocks} clocks; ir {:?} step {:?} addr {:?}", g.ir(), g.step_count(), g.addr());
    let rw = g.reset_warnings();
    assert!(rw.is_empty(), "during reset: {:?}", rw.iter().take(6).collect::<Vec<_>>());
    let w = g.warnings();
    assert!(w.is_empty(), "{} warnings, e.g. {:?}", w.len(), w.iter().take(10).collect::<Vec<_>>());
    assert_eq!(g.a(), Some(iss.a), "A");
    assert_eq!(g.b(), Some(iss.b), "B");
    for r in 0..32 {
        assert_eq!(g.reg(r), Some(iss.reg(r)), "r{r}");
    }
    eprintln!("{} instructions, {} clocks", iss.retired, g.clocks);
    (g, iss)
}

#[test]
fn every_gal_fits() {
    for s in gal_specs() {
        s.fits().unwrap();
        let ins = s.external_inputs().len();
        eprintln!("{}: {} inputs, {} outputs", s.name, ins, s.eqs.len());
    }
}

#[test]
fn microcode_obeys_its_rules() {
    let rom = microcode();
    assert_eq!(rom.len(), 1024);
    assert_eq!(rom[mips32::grit::ucode_addr(Op::Halt as u8, false, 0)], 0);
}

#[test]
fn add_and_store() {
    let (g, _) = run(
        "
        LDA 5
        LDB 7
        ADDB          ; B = 12
        LDA &r1
        STB (A)       ; r1 = 12
        LDA 0x1234
        LDB 0x00FF
        ANDB          ; B = 0x34
        LDA &r2
        STB (A)
        HALT
        ",
        400,
        Build::default(),
    );
    assert_eq!(g.reg(1), Some(12));
    assert_eq!(g.reg(2), Some(0x34));
}

#[test]
fn loads_from_ram_and_flash() {
    let (g, _) = run(
        "
        LDA &r1
        LDB 0x8040     ; a data address
        STB (A)        ; r1 = 0x8040
        LDA &r1
        LDA (A)        ; A = 0x8040
        LDB 0xBEEF
        STB (A)        ; mem[0x8040] = 0xBEEF
        LDA table
        LDB (A)        ; B = 0x0042 from the flash
        LDA &r2
        STB (A)
        LDA 0x8040
        LDB (A)        ; B = 0xBEEF
        LDA &r3
        STB (A)
        HALT
    table:
        .word 0x0042
        ",
        600,
        Build::default(),
    );
    assert_eq!(g.reg(2), Some(0x42));
    assert_eq!(g.reg(3), Some(0xBEEF));
    assert_eq!(g.ram_word(0x8040), Some(0xBEEF));
}

#[test]
fn countdown_loop() {
    // r1 = 5; while r1 != 0: r2 += 3, r1 -= 1.
    let (g, _) = run(
        "
        LDA &r1
        LDB 5
        STB (A)
        LDA &r2
        LDB 0
        STB (A)
    loop:
        LDA &r1
        LDB (A)         ; B = r1
        LDA 0
        JEQ done        ; A == B: r1 is zero
        LDA &r2
        LDA (A)         ; A = r2
        LDB 3
        ADDB            ; B = r2 + 3
        LDA &r2
        STB (A)
        LDA &r1
        LDB (A)
        LDA -1
        ADDB            ; B = r1 - 1
        LDA &r1
        STB (A)
        JMP loop
    done:
        HALT
        ",
        4000,
        Build::default(),
    );
    assert_eq!(g.reg(1), Some(0));
    assert_eq!(g.reg(2), Some(15));
}

#[test]
fn not_via_nor() {
    let (g, _) = run(
        "
        LDA 0x00F0
        LDB 0
        NORB           ; B = ~0x00F0
        LDA &r1
        STB (A)
        LDA 0x0F0F
        MOVAB          ; A = B = 0xFF0F
        LDB 0x0001
        ADDA           ; A = 0xFF10
        LDB 0
        JEQ never
        NOP
        HALT
    never:
        LDA 0x1111
        HALT
        ",
        600,
        Build::default(),
    );
    assert_eq!(g.reg(1), Some(0xFF0F));
    assert_eq!(g.a(), Some(0xFF10));
}

/// Serial: set the line up, send "hi", read one character back.
#[test]
fn uart_hello() {
    let src = "
        LDA 0xC006      ; LCR
        LDB 0x80        ; DLAB
        STB (A)
        LDA 0xC000      ; DLL
        LDB 1
        STB (A)
        LDA 0xC002      ; DLM
        LDB 0
        STB (A)
        LDA 0xC006      ; LCR: 8N1
        LDB 0x03
        STB (A)
        LDA 0xC004      ; FCR: FIFOs on
        LDB 0x07
        STB (A)
        LDA 0xC008      ; MCR: DTR RTS auto-flow
        LDB 0x22
        STB (A)
        LDB 0x68        ; 'h'
        JMP send
    back1:
        LDB 0x69        ; 'i'
        JMP send2
    back2:
        ; wait for a received character, store it in r1
    rxwait:
        LDA 0xC00A      ; LSR
        LDB (A)
        LDA 0x00FF
        ANDB
        LDA 0x0001
        ANDB            ; B = DR bit
        LDA 0
        JEQ rxwait
        LDA 0xC000
        LDB (A)
        LDA 0x00FF
        ANDB
        LDA &r1
        STB (A)
        HALT
        ; send: B holds the character; wait for THRE, write it, return
    send:
        LDA &r2
        STB (A)         ; r2 = the character
    txwait:
        LDA 0xC00A
        LDB (A)
        LDA 0x00FF
        ANDB
        LDA 0x0020
        ANDB
        LDA 0
        JEQ txwait
        LDA &r2
        LDB (A)
        LDA 0xC000
        STB (A)
        JMP back1
    send2:
        LDA &r2
        STB (A)
    txwait2:
        LDA 0xC00A
        LDB (A)
        LDA 0x00FF
        ANDB
        LDA 0x0020
        ANDB
        LDA 0
        JEQ txwait2
        LDA &r2
        LDB (A)
        LDA 0xC000
        STB (A)
        JMP back2
        ";
    // A fast UART clock so a character is tens of clocks, not thousands.
    let opt = Build { uart_xin_hz: 16.0 * 115_200.0 * 8.0, uart_rx: b"Z".to_vec(), ..Build::default() };
    let (g, _) = run(src, 20_000, opt);
    assert_eq!(g.uart_tx(), b"hi");
    assert_eq!(g.reg(1), Some(b'Z' as u16));
}

/// The supervisor releases reset at any phase of the clock.
#[test]
fn reset_release_phase_sweep() {
    let src = "LDA 0x1234\n LDB 0x0001\n ADDB\n LDA &r1\n STB (A)\n HALT\n";
    for phase in [0.0, 5.0, 20.0, 54.0, 100.0, 108.0, 150.0, 200.0, 216.0] {
        let p = assemble(src).unwrap();
        let mut g = Grit::build(&p.words, &Build { reset_phase_ns: phase, ..Build::default() });
        assert!(g.run_until_halt(300), "phase {phase}: did not halt");
        let w = g.warnings();
        assert!(w.is_empty(), "phase {phase}: {:?}", w.iter().take(5).collect::<Vec<_>>());
        assert_eq!(g.reg(1), Some(0x1235), "phase {phase}");
    }
}

#[test]
fn reset_button_restarts() {
    let src = "LDA &r1\n LDB (A)\n LDA 1\n ADDB\n LDA &r1\n STB (A)\n HALT\n";
    let p = assemble(src).unwrap();
    let mut g = Grit::build(&p.words, &Build::default());
    assert!(g.run_until_halt(300));
    assert_eq!(g.reg(1), Some(1));
    g.press_reset(17.0, 3000.0);
    assert!(g.run_until_halt(300));
    assert_eq!(g.reg(1), Some(2));
    assert!(g.warnings().is_empty(), "{:?}", g.warnings().iter().take(5).collect::<Vec<_>>());
}

/// The board file: round trip through JSON, no drift from the committed
/// file, and a program runs from the loaded file.
#[test]
fn board_file() {
    use mips32::board::Board;
    let b = mips32::grit::board();
    let json = b.to_json();
    let back = Board::from_json(&json).unwrap();
    assert_eq!(back, b);
    let committed = std::fs::read_to_string("boards/grit/netlist.json").expect("boards/grit/netlist.json");
    let cb = Board::from_json(&committed).unwrap();
    assert_eq!(cb, b, "boards/grit/netlist.json is stale: run `mips32 export grit > boards/grit/netlist.json`");
    let p = assemble("LDA 3\n LDB 4\n ADDB\n LDA &r1\n STB (A)\n HALT\n").unwrap();
    let mut g = Grit::from_board(&cb, &p.words, &Build::default());
    assert!(g.run_until_halt(300));
    assert!(g.warnings().is_empty());
    assert_eq!(g.reg(1), Some(7));
    let gals = cb.chips.iter().filter(|c| matches!(c.model, mips32::board::Model::Gal { .. })).count();
    assert_eq!(gals, 15);
}
