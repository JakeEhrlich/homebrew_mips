//! `mips32 export <board>`  prints the board file (netlist.json).
//! `mips32 chipmap <board> <template.html>`  renders the chip map page.
use mips32::board::Board;

fn board(name: &str) -> Board {
    match name {
        "crag" => mips32::cpu::board(&mips32::cpu::Params::board()),
        _ => {
            eprintln!("unknown board {name}");
            std::process::exit(2);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(String::as_str) {
        Some("export") if args.len() == 3 => println!("{}", board(&args[2]).to_json()),
        Some("chipmap") if args.len() == 4 => {
            let b = board(&args[2]);
            let template = std::fs::read_to_string(&args[3]).expect("template");
            let data = serde_json::to_string(&b).expect("json");
            print!("{}", template.replace("__BOARD_JSON__", &data));
        }
        _ => eprintln!("usage: mips32 export <board> | mips32 chipmap <board> <template.html>"),
    }
}
