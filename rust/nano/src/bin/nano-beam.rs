//! Beam search generator (port of level-6/strategies/beam-search).
//! usage: nano-beam <input> <out> [--width W] [--boards i,j,k] [--seed S] [--noise X]
//!                  [--spec "K D mu lwScale Z noise seed gamma,..."] [--threads N<=2]
//!                  [--bound LEN] [--index index.tsv]
//! Output: one program per selected board (in --boards order, else all boards), blank-line separated.
//! A board with no program gets a single noop line (invalid placeholder keeps the alignment).
use nano::beam::{beam_program, default_spec, parse_spec, BeamParams};
use nano::{parse_input, simulate, write_program, Geometry, Prog, Target, NOOP};
use std::collections::HashMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: nano-beam <input> <out> [--width W] [--boards i,j,k] [--seed S] [--noise X] [--spec S] [--threads N] [--bound LEN] [--index tsv]");
        std::process::exit(1);
    }
    let (mut width, mut seed, mut noise, mut threads, mut bound) = (0usize, None::<u64>, None::<f64>, 2usize, 0usize);
    let (mut boards, mut spec, mut index_path) = (None::<Vec<usize>>, None::<String>, None::<String>);
    let mut i = 3;
    while i < args.len() {
        let v = args.get(i + 1).cloned().unwrap_or_default();
        match args[i].as_str() {
            "--width" => width = v.parse().expect("--width"),
            "--seed" => seed = Some(v.parse().expect("--seed")),
            "--noise" => noise = Some(v.parse().expect("--noise")),
            "--threads" => threads = v.parse::<usize>().expect("--threads").clamp(1, 2),
            "--bound" => bound = v.parse().expect("--bound"),
            "--boards" => boards = Some(v.split(',').filter(|s| !s.is_empty()).map(|s| s.parse().expect("--boards")).collect()),
            "--spec" => spec = Some(v),
            "--index" => index_path = Some(v),
            o => {
                eprintln!("unknown option {o}");
                std::process::exit(1);
            }
        }
        i += 2;
    }
    let text = std::fs::read_to_string(&args[1]).expect("read input");
    let cases = parse_input(&text).expect("parse input");
    let ids: Vec<usize> = boards.unwrap_or_else(|| (0..cases.len()).collect());
    let mut bounds: HashMap<usize, usize> = HashMap::new();
    if let Some(p) = &index_path {
        for line in std::fs::read_to_string(p).unwrap_or_default().lines() {
            let f: Vec<&str> = line.split('\t').collect();
            if f.len() >= 2 && !line.starts_with('#') {
                if let (Ok(a), Ok(b)) = (f[0].parse::<usize>(), f[1].parse::<usize>()) {
                    bounds.insert(a, b);
                }
            }
        }
    }
    struct Task {
        pos: usize,
        bp: BeamParams,
    }
    let geoms: Vec<(Geometry, Target)> = ids
        .iter()
        .map(|&b| {
            let g = Geometry::new(cases[b].radius);
            let t = Target::new(&g, &cases[b]);
            (g, t)
        })
        .collect();
    let mut tasks: Vec<Task> = Vec::new();
    for (pos, (g, _)) in geoms.iter().enumerate() {
        let sp = spec.clone().unwrap_or_else(|| default_spec(g.words).to_string());
        for mut bp in parse_spec(&sp) {
            if width > 0 {
                bp.k = width;
            }
            if let Some(n) = noise {
                if bp.noise == 0.0 {
                    bp.noise = n;
                }
            }
            if let Some(s) = seed {
                bp.seed = s;
            }
            tasks.push(Task { pos, bp });
        }
    }
    let results: Mutex<Vec<Option<Prog>>> = Mutex::new(vec![None; ids.len()]);
    let next = AtomicUsize::new(0);
    let t0 = Instant::now();
    std::thread::scope(|sc| {
        for _ in 0..threads {
            sc.spawn(|| loop {
                let ti = next.fetch_add(1, Ordering::Relaxed);
                if ti >= tasks.len() {
                    break;
                }
                let task = &tasks[ti];
                let (g, t) = &geoms[task.pos];
                let board = ids[task.pos];
                let mut mx = bound;
                if let Some(&b) = bounds.get(&board) {
                    mx = if mx == 0 { b } else { mx.min(b) };
                }
                {
                    // never search past the best already found for this board
                    let r = results.lock().unwrap();
                    if let Some(p) = &r[task.pos] {
                        mx = if mx == 0 { p.len() } else { mx.min(p.len()) };
                    }
                }
                let ts = Instant::now();
                let res = beam_program(g, t, &task.bp, mx);
                let mut used = -1;
                if let Some(mut p) = res {
                    used = simulate(g, t, &p);
                    if used >= 0 {
                        p.truncate(used as usize);
                        let mut r = results.lock().unwrap();
                        if r[task.pos].as_ref().map_or(true, |q| p.len() < q.len()) {
                            r[task.pos] = Some(p);
                        }
                    }
                }
                eprintln!(
                    "board {board} R{} K{} D{} -> {used} ({:.2}s)",
                    g.r,
                    task.bp.k,
                    task.bp.d,
                    ts.elapsed().as_secs_f64()
                );
            });
        }
    });
    let results = results.into_inner().unwrap();
    let mut out = String::new();
    let mut total = 0usize;
    for (pos, r) in results.iter().enumerate() {
        if pos > 0 {
            out.push('\n');
        }
        match r {
            Some(p) => {
                total += p.len();
                write_program(&mut out, p);
            }
            None => write_program(&mut out, &[NOOP]),
        }
    }
    std::fs::write(&args[2], out).expect("write output");
    eprintln!("nano-beam total lines {total} in {:.2}s", t0.elapsed().as_secs_f64());
}
