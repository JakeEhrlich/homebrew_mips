//! `mips32 graph` prints the CPU structure as JSON (for the diagram page).
fn main() {
    let arg = std::env::args().nth(1).unwrap_or_default();
    match arg.as_str() {
        "graph" => println!("{}", mips32::cpu::structure_json()),
        _ => eprintln!("usage: mips32 graph"),
    }
}
