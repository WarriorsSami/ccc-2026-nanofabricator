mod core;
mod opt;
mod search;

use crate::core::*;
use crate::opt::*;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const INPUTS: [&str; 4] = ["1-small", "2-medium", "3-large", "4-enormous"];

fn root() -> PathBuf {
    PathBuf::from(
        std::env::var("MICRO_ROOT")
            .unwrap_or_else(|_| "/Volumes/MacOS-Ext/Development/ccc-hackathon/ccc-2026-nanofabricator".to_string()),
    )
}

fn board_dir(inp: &str) -> PathBuf {
    root().join("notebook/boards").join(inp)
}

fn read_index(inp: &str) -> Vec<(usize, usize)> {
    let mut v = Vec::new();
    let text = fs::read_to_string(board_dir(inp).join("index.tsv")).unwrap_or_default();
    for line in text.lines() {
        if line.starts_with('#') || line.trim().is_empty() {
            continue;
        }
        let mut it = line.split('\t');
        let idx: usize = it.next().unwrap().parse().unwrap();
        let lines: usize = it.next().unwrap().parse().unwrap();
        v.push((idx, lines));
    }
    v
}

fn load(inp: &str, idx: usize) -> Option<(Pattern, Prog)> {
    let d = board_dir(inp);
    let ins = fs::read_to_string(d.join(format!("{:03}.in", idx))).ok()?;
    let prog = fs::read_to_string(d.join(format!("{:03}.prog", idx))).ok()?;
    let mut pats = parse_input(&ins);
    Some((pats.remove(0), parse_prog(&prog)))
}

fn w_for(nrel: usize) -> usize {
    ((nrel + 63) / 64).max(1)
}

macro_rules! dispatch {
    ($w:expr, $f:ident, $($arg:expr),*) => {
        match $w {
            1 => $f::<1>($($arg),*),
            2 => $f::<2>($($arg),*),
            3 => $f::<3>($($arg),*),
            4 => $f::<4>($($arg),*),
            5 => $f::<5>($($arg),*),
            6 => $f::<6>($($arg),*),
            _ => $f::<7>($($arg),*),
        }
    };
}

fn optimize_w<const W: usize>(geo: &Geo, pat: &Pattern, rel: &Rel, prog: Prog, cfg: Cfg) -> (Prog, u64, u64, String) {
    let b = Board::<W>::new(geo, pat, rel);
    let mut o = Opt::new(&b, cfg);
    let r = o.optimize(prog);
    (r, o.windows, o.nodes, o.found.join(","))
}

fn check_w<const W: usize>(geo: &Geo, pat: &Pattern, rel: &Rel, prog: &Prog) -> Option<usize> {
    let b = Board::<W>::new(geo, pat, rel);
    b.run(prog)
}

fn optimize_one(pat: &Pattern, prog: Prog, cfg: Cfg) -> (Prog, u64, u64, String) {
    let geo = Geo::new(pat.radius);
    let rel = relevant(&geo, pat);
    let w = w_for(rel.nrel);
    dispatch!(w, optimize_w, &geo, pat, &rel, prog, cfg)
}

fn cmd_check() {
    let t0 = Instant::now();
    let mut nb = 0;
    let mut bad = 0;
    for inp in INPUTS {
        for (idx, lines) in read_index(inp) {
            let Some((pat, prog)) = load(inp, idx) else {
                println!("{} {} missing", inp, idx);
                bad += 1;
                continue;
            };
            let geo = Geo::new(pat.radius);
            let rel = relevant(&geo, &pat);
            let w = w_for(rel.nrel);
            let r = dispatch!(w, check_w, &geo, &pat, &rel, &prog);
            nb += 1;
            if r != Some(lines) || prog.len() != lines {
                println!("MISMATCH {} {} index={} sim={:?} proglen={}", inp, idx, lines, r, prog.len());
                bad += 1;
            }
        }
    }
    println!("checked {} boards, {} bad, {:.3}s", nb, bad, t0.elapsed().as_secs_f64());
}

fn cmd_one(args: &[String]) {
    let inp = &args[0];
    let idx: usize = args[1].parse().unwrap();
    let kmax: usize = args.get(2).map(|s| s.parse().unwrap()).unwrap_or(5);
    let secs: f64 = args.get(3).map(|s| s.parse().unwrap()).unwrap_or(10.0);
    let cap: u64 = args.get(4).map(|s| s.parse().unwrap()).unwrap_or(200000);
    let relaxed = args.get(5).map(|s| s != "0").unwrap_or(true);
    let (pat, prog) = load(inp, idx).unwrap();
    let n0 = prog.len();
    let t0 = Instant::now();
    let cfg = Cfg { kmax, secs, cap, relaxed, sims_cap: 30 };
    let (r, wins, nodes, found) = optimize_one(&pat, prog, cfg);
    println!(
        "{} {} {} -> {} windows={} nodes={} {:.3}s [{}]",
        inp,
        idx,
        n0,
        r.len(),
        wins,
        nodes,
        t0.elapsed().as_secs_f64(),
        found
    );
    let out = root().join(format!("work/opt3/one_{}_{}.prog", inp, idx));
    fs::write(&out, prog_text(&r)).unwrap();
}

fn now_epoch() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs()
}

/// run <shard> <nshards> <until_epoch> <secs_per_board> <kmax> <cap> [inputs csv] [minlines]
fn cmd_run(args: &[String]) {
    let shard: usize = args[0].parse().unwrap();
    let nshards: usize = args[1].parse().unwrap();
    let until: u64 = args[2].parse().unwrap();
    let secs: f64 = args[3].parse().unwrap();
    let kmax: usize = args[4].parse().unwrap();
    let cap: u64 = args[5].parse().unwrap();
    let only: Vec<String> = args
        .get(6)
        .map(|s| s.split(',').map(|x| x.to_string()).collect())
        .unwrap_or_else(|| INPUTS.iter().map(|s| s.to_string()).collect());
    let minlines: usize = args.get(7).map(|s| s.parse().unwrap()).unwrap_or(0);
    let cfg = Cfg { kmax, secs, cap, relaxed: true, sims_cap: 30 };
    let candd = root().join("work/rustmicro/cand");
    let _ = fs::create_dir_all(&candd);
    // (input, idx) -> last program hash optimized with these settings
    let mut done: HashMap<(String, usize), u64> = HashMap::new();
    let mut saved_total = 0i64;
    let mut boards = 0u64;
    let t0 = Instant::now();
    let mut handles = Vec::new();
    let mut round = 0;
    loop {
        round += 1;
        let mut list: Vec<(usize, String, usize)> = Vec::new();
        for inp in INPUTS {
            if !only.iter().any(|s| s == inp) {
                continue;
            }
            for (idx, lines) in read_index(inp) {
                if lines >= minlines {
                    list.push((lines, inp.to_string(), idx));
                }
            }
        }
        list.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
        let mut round_saved = 0i64;
        let mut round_boards = 0;
        for (pos, (_, inp, idx)) in list.iter().enumerate() {
            if pos % nshards != shard {
                continue;
            }
            if until > 0 && now_epoch() >= until {
                break;
            }
            let Some((pat, prog)) = load(inp, *idx) else { continue };
            let mut hsh = 1469598103934665603u64;
            for l in &prog {
                for e in 0..6 {
                    hsh = (hsh ^ l[e] as u64).wrapping_mul(1099511628211);
                }
            }
            if done.get(&(inp.clone(), *idx)) == Some(&hsh) {
                continue;
            }
            let n0 = prog.len();
            let tb = Instant::now();
            let (r, wins, nodes, found) = optimize_one(&pat, prog, cfg);
            boards += 1;
            round_boards += 1;
            if r.len() < n0 {
                let f = candd.join(format!("{}_{:03}.prog", inp, idx));
                fs::write(&f, prog_text(&r)).unwrap();
                let inp2 = inp.clone();
                let idx2 = *idx;
                let rt = root();
                handles.push(std::thread::spawn(move || {
                    let _ = Command::new("python3")
                        .arg(rt.join("notebook/tools/nb.py"))
                        .arg("import-board")
                        .arg(&inp2)
                        .arg(idx2.to_string())
                        .arg(&f)
                        .arg("rustmicro")
                        .output();
                }));
                saved_total += (n0 - r.len()) as i64;
                round_saved += (n0 - r.len()) as i64;
                eprintln!(
                    "[r{} s{}] {} {:03}: {} -> {} (windows {} nodes {}, {:.2}s) [{}]",
                    round,
                    shard,
                    inp,
                    idx,
                    n0,
                    r.len(),
                    wins,
                    nodes,
                    tb.elapsed().as_secs_f64(),
                    found
                );
                let mut hh = 1469598103934665603u64;
                for l in &r {
                    for e in 0..6 {
                        hh = (hh ^ l[e] as u64).wrapping_mul(1099511628211);
                    }
                }
                done.insert((inp.clone(), *idx), hh);
            } else {
                done.insert((inp.clone(), *idx), hsh);
            }
        }
        eprintln!(
            "[r{} s{}] round done: boards {} saved {} total_saved {} total_boards {} {:.1}s ({:.2} boards/s)",
            round,
            shard,
            round_boards,
            round_saved,
            saved_total,
            boards,
            t0.elapsed().as_secs_f64(),
            boards as f64 / t0.elapsed().as_secs_f64()
        );
        if until == 0 || now_epoch() >= until {
            break;
        }
        if round_boards == 0 {
            std::thread::sleep(Duration::from_secs(20));
        }
    }
    for h in handles {
        let _ = h.join();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    match args.get(1).map(|s| s.as_str()) {
        Some("check") => cmd_check(),
        Some("one") => cmd_one(&args[2..]),
        Some("run") => cmd_run(&args[2..]),
        _ => eprintln!("usage: micro-opt check | one <input> <idx> [kmax secs cap relaxed] | run <shard> <nshards> <until> <secs> <kmax> <cap> [inputs] [minlines]"),
    }
}
