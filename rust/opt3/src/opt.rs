// Program-level optimiser: cheap edits and window replacement (exact / relaxed).
use crate::core::*;
use crate::search::search_window;
use std::collections::HashSet;
use std::time::Instant;

#[derive(Clone, Copy)]
pub struct Cfg {
    pub kmax: usize,
    pub secs: f64,
    pub cap: u64,
    pub relaxed: bool,
    pub sims_cap: u32,
}

pub struct Opt<'a, const W: usize> {
    pub b: &'a Board<W>,
    pub cfg: Cfg,
    pub memo: HashSet<u64>,
    pub deadline: Instant,
    pub windows: u64,
    pub nodes: u64,
    pub found: Vec<String>,
}

fn mix(h: u64, v: u64) -> u64 {
    let mut h = h ^ v;
    h = h.wrapping_mul(0x100000001b3);
    h ^ (h >> 29)
}

fn merge_ops(a: u8, b: u8) -> Option<u8> {
    match (a, b) {
        (b'-', x) => Some(x),
        (x, b'-') => Some(x),
        (b'F', b'B') | (b'B', b'F') | (b'T', b'T') => Some(b'-'),
        _ => None,
    }
}

impl<'a, const W: usize> Opt<'a, W> {
    pub fn new(b: &'a Board<W>, cfg: Cfg) -> Self {
        Opt {
            b,
            cfg,
            memo: HashSet::new(),
            deadline: Instant::now() + std::time::Duration::from_secs_f64(cfg.secs),
            windows: 0,
            nodes: 0,
            found: Vec::new(),
        }
    }

    fn timeout(&self) -> bool {
        Instant::now() > self.deadline
    }

    fn try_cand(&self, mut cand: Prog, n: usize) -> Option<Prog> {
        if let Some(len) = self.b.run(&cand) {
            if len < n {
                cand.truncate(len);
                return Some(cand);
            }
        }
        None
    }

    pub fn optimize(&mut self, mut prog: Prog) -> Prog {
        if let Some(n) = self.b.run(&prog) {
            prog.truncate(n);
        } else {
            return prog;
        }
        loop {
            if self.timeout() {
                break;
            }
            if self.cheap(&mut prog) {
                self.found.push("cheap".to_string());
                continue;
            }
            if self.windows(&mut prog) {
                continue;
            }
            break;
        }
        prog
    }

    fn cheap(&self, prog: &mut Prog) -> bool {
        let n = prog.len();
        let b = self.b;
        // delete a line
        for i in (0..n).rev() {
            let mut c = prog.clone();
            c.remove(i);
            if let Some(c) = self.try_cand(c, n) {
                *prog = c;
                return true;
            }
        }
        // merge adjacent lines
        for i in (0..n.saturating_sub(1)).rev() {
            let mut m = [b'-'; 6];
            let mut ok = true;
            for e in 0..6 {
                match merge_ops(prog[i][e], prog[i + 1][e]) {
                    Some(x) => m[e] = x,
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok {
                continue;
            }
            let mut c: Prog = prog[..i].to_vec();
            c.push(m);
            c.extend_from_slice(&prog[i + 2..]);
            if let Some(c) = self.try_cand(c, n) {
                *prog = c;
                return true;
            }
        }
        // shift a single instruction by +-1..3 lines (into a free slot, or merging)
        for i in 0..n {
            for e in 0..6 {
                let op = prog[i][e];
                if op == b'-' {
                    continue;
                }
                for d in [-1i32, 1, -2, 2, -3, 3] {
                    let j = i as i32 + d;
                    if j < 0 || j >= n as i32 {
                        continue;
                    }
                    let j = j as usize;
                    let Some(nj) = merge_ops(prog[j][e], op) else { continue };
                    let mut c = prog.clone();
                    c[j][e] = nj;
                    c[i][e] = b'-';
                    if c[i].iter().all(|&x| x == b'-') {
                        c.remove(i);
                    }
                    if let Some(c) = self.try_cand(c, n) {
                        *prog = c;
                        return true;
                    }
                }
            }
        }
        let _ = b;
        false
    }

    fn care_at(&self, prog: &[Line], states: &[St<W>], j: usize) -> [u64; W] {
        let n = prog.len();
        let b = self.b;
        let mut care = [0u64; W];
        for c in 0..b.nrel {
            let mut s2 = states[j];
            s2.m[c >> 6] ^= 1u64 << (c & 63);
            let mut differs = false;
            for t in j..n {
                let ev = b.step(&mut s2, &prog[t]);
                let ev0 = states[t + 1].acc != states[t].acc;
                if ev != ev0 {
                    differs = true;
                    break;
                }
                if s2.m == states[t + 1].m {
                    break;
                }
            }
            if differs {
                care[c >> 6] |= 1u64 << (c & 63);
            }
        }
        care
    }

    fn windows(&mut self, prog: &mut Prog) -> bool {
        let n = prog.len();
        let b = self.b;
        let states = b.states(prog);
        let mut care_cache: Vec<Option<[u64; W]>> = vec![None; n + 1];
        let mut suffix_hash = vec![0u64; n + 1];
        for j in (0..n).rev() {
            let mut h = suffix_hash[j + 1];
            for e in 0..6 {
                h = mix(h, prog[j][e] as u64);
            }
            suffix_hash[j] = h;
        }
        let kmax = self.cfg.kmax.min(n);
        for k in 2..=kmax {
            for i in (0..=(n - k)).rev() {
                if self.timeout() {
                    return false;
                }
                let j = i + k;
                let tail = j == n;
                let s = &states[i];
                let t = &states[j];
                let need = if tail { b.count - s.acc } else { t.acc - s.acc };
                if need < 0 || need > 3 || (tail && need < 1) {
                    continue;
                }
                let lo = if k >= 3 { k - 2 } else { 1 };
                for l in lo..k {
                    if need as usize > l {
                        continue;
                    }
                    for relaxed in [false, true] {
                        let care = if relaxed {
                            if !self.cfg.relaxed || tail {
                                continue;
                            }
                            let c = match care_cache[j] {
                                Some(c) => c,
                                None => {
                                    let c = self.care_at(prog, &states, j);
                                    care_cache[j] = Some(c);
                                    c
                                }
                            };
                            if c == b.valid {
                                continue;
                            }
                            c
                        } else {
                            b.valid
                        };
                        let mut key = mix(0x9e3779b97f4a7c15, s.hash());
                        key = mix(key, if tail { 0xdeadbeef } else { t.hash() });
                        key = mix(key, l as u64 + 100 * relaxed as u64);
                        if relaxed {
                            for w in 0..W {
                                key = mix(key, care[w]);
                            }
                            key = mix(key, suffix_hash[j]);
                        }
                        if !self.memo.insert(key) {
                            continue;
                        }
                        self.windows += 1;
                        let mut found: Option<Prog> = None;
                        let mut sims = 0u32;
                        let sims_cap = self.cfg.sims_cap;
                        let mut nodes = 0u64;
                        let prog_ref: &Prog = prog;
                        let r = search_window(
                            b,
                            s,
                            if tail { None } else { Some(t) },
                            need,
                            l,
                            care,
                            self.cfg.cap,
                            &mut nodes,
                            &mut |lines: &[Line]| -> u8 {
                                let mut cand: Prog = prog_ref[..i].to_vec();
                                cand.extend_from_slice(lines);
                                if !tail {
                                    cand.extend_from_slice(&prog_ref[j..]);
                                }
                                sims += 1;
                                if let Some(len) = b.run(&cand) {
                                    if len < n {
                                        cand.truncate(len);
                                        found = Some(cand);
                                        return 1;
                                    }
                                }
                                if sims >= sims_cap {
                                    2
                                } else {
                                    0
                                }
                            },
                        );
                        self.nodes += nodes;
                        let _ = r;
                        if let Some(c) = found {
                            *prog = c;
                            self.found.push(format!("k{}l{}{}{}", k, l, if relaxed { "R" } else { "E" }, if tail { "T" } else { "" }));
                            return true;
                        }
                    }
                }
            }
        }
        false
    }
}
