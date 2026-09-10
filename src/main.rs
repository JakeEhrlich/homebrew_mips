//! `mips32 export <board>`  prints the board file (netlist.json).
//! `mips32 chipmap <board> <template.html>`  renders the chip map page.
use mips32::board::Board;

fn board(name: &str) -> Board {
    match name {
        "crag" => mips32::cpu::board(&mips32::cpu::Params::board()),
        "grit" => mips32::grit::board(),
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
            let title = format!("{}{} Chip Map", b.name[..1].to_uppercase(), &b.name[1..]);
            print!("{}", template.replace("__BOARD_JSON__", &data).replace("__TITLE__", &title));
        }
        Some("slack") if args.len() >= 3 => slack(&board(&args[2]), args.get(3).and_then(|v| v.parse().ok()).unwrap_or(8.0), args.get(4).and_then(|v| v.parse().ok()).unwrap_or(2)),
        _ => eprintln!("usage: mips32 export <board> | mips32 chipmap <board> <template.html> | mips32 slack <board> [max_ns] [programs]"),
    }
}

/// Slack of every backward bus, as a markdown table on stdout; progress
/// on stderr.  Bisected against the first `programs` soak programs, in
/// parallel across buses.
fn slack(b: &Board, hi_ns: f64, programs: u64) {
    use mips32::slack::{all_edges, backward_edges, max_delay, Program};
    use std::sync::Mutex;
    let progs: Vec<Program> = (1..=programs).map(|s| Program::new(&format!("soak {s}"), &mips32::soak::program(s, 160))).collect();
    let mut edges = backward_edges(b);
    edges.push(all_edges(&edges));
    if let Ok(only) = std::env::var("SLACK_BUS") {
        edges.retain(|e| only.split(',').any(|o| o == e.bus));
    }
    let next = Mutex::new(0usize);
    let results = Mutex::new(Vec::new());
    let workers = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4).min(edges.len());
    std::thread::scope(|sc| {
        for _ in 0..workers {
            sc.spawn(|| loop {
                let i = { let mut n = next.lock().unwrap(); let i = *n; *n += 1; i };
                let Some(e) = edges.get(i) else { break };
                let r = max_delay(&progs, e, hi_ns, 0.25);
                eprintln!("{:8} {:>5.2} ns  {}", r.bus, r.pass_ns, r.fail.as_ref().map(|(d, m)| format!("fails at {d:.2}: {m}")).unwrap_or_else(|| "passes at the maximum".into()));
                results.lock().unwrap().push((i, r));
            });
        }
    });
    let mut results = results.into_inner().unwrap();
    results.sort_by_key(|(i, _)| *i);
    println!("| Bus | Nets | Back pins | From | To | Max delay | FR4 (6.5 ps/mm) | First failure |");
    println!("|---|---|---|---|---|---|---|---|");
    for (i, r) in &results {
        let e = &edges[*i];
        let (d, fail) = match &r.fail {
            None => (format!(">= {:.2} ns", r.pass_ns), String::new()),
            Some((at, m)) => (format!("{:.2} ns", r.pass_ns), format!("at {at:.2} ns: {}", m.replace('|', "\\|"))),
        };
        println!("| {} | {} | {} | {} | {} | {} | {:.0} mm | {} |", e.bus, e.nets.len(), e.pins.len(), e.from.join(", "), e.to.join(", "), d, r.pass_ns / 0.0065, fail);
    }
}
