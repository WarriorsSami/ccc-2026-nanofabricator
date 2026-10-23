//! Per-pattern merge: for each pattern pick the shortest program among the
//! candidate files that the exact simulator accepts (same CLI/stderr contract as
//! notebook/tools/merge.cpp).
//! usage: nano-validate <input.txt> <out.txt> <cand1.txt> [cand2.txt ...]
use nano::{parse_input, read_programs, simulate, write_program, Geometry, Prog, Target};
use std::process::exit;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 4 {
        exit(1);
    }
    let text = std::fs::read_to_string(&args[1]).unwrap_or_default();
    let cases = match parse_input(&text) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("input error: {e}");
            exit(1);
        }
    };
    let cands: Vec<Vec<Prog>> = args[3..]
        .iter()
        .map(|p| read_programs(&String::from_utf8_lossy(&std::fs::read(p).unwrap_or_default())))
        .collect();
    let mut out = String::new();
    let mut total: i64 = 0;
    let mut wins = vec![0usize; cands.len()];
    for (k, case) in cases.iter().enumerate() {
        let g = Geometry::new(case.radius);
        let t = Target::new(&g, case);
        let mut best: Option<(usize, &[nano::Line])> = None;
        let mut who = 0usize;
        for (c, cand) in cands.iter().enumerate() {
            let Some(p) = cand.get(k) else { continue };
            let used = simulate(&g, &t, p);
            if used < 0 {
                continue;
            }
            let used = used as usize;
            if best.map_or(true, |(len, _)| used < len) {
                best = Some((used, &p[..used]));
                who = c;
            }
        }
        let Some((len, prog)) = best else {
            let _ = std::fs::write(&args[2], &out); // the C++ tool streams boards, so a partial file remains
            eprintln!("pattern {k}: no valid candidate");
            exit(2);
        };
        wins[who] += 1;
        total += len as i64;
        if k > 0 {
            out.push('\n');
        }
        write_program(&mut out, prog);
    }
    std::fs::write(&args[2], out).unwrap_or_else(|e| {
        eprintln!("cannot write {}: {e}", args[2]);
        exit(1);
    });
    eprint!("merged total {total} | wins per candidate:");
    for w in wins {
        eprint!(" {w}");
    }
    eprintln!();
}
