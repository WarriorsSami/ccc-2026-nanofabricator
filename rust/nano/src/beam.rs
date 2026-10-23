//! Beam search over ticks (port of level-6/strategies/beam-search `beamProgramW`).
//!
//! State = (offsets, on bits, pattern matter, copies). Every state expands into
//! all 4^6 lines; the value of a child is copies, stamped cells and a pair-aware
//! potential: for d = 1..D, the missing cells that at least two emitters can
//! reach within d ticks. Hot loops are const-generic over the bitset width `W`
//! and use unchecked indexing.

use crate::{axis_of, Geometry, Line, Prog, Target, NOOP};
use std::collections::HashSet;

#[derive(Clone, Debug)]
pub struct BeamParams {
    pub k: usize,
    pub d: usize,
    pub lw: [f64; 8],
    pub mu: f64,
    pub gamma: f64,
    pub stuck: i32,
    pub z: f64,
    pub noise: f64,
    pub seed: u64,
}

impl Default for BeamParams {
    fn default() -> Self {
        BeamParams {
            k: 20,
            d: 3,
            lw: [0.3, 0.15, 0.08, 0.04, 0.02, 0.01, 0.005, 0.0025],
            mu: 0.02,
            gamma: 0.5,
            stuck: 0,
            z: 1e9,
            noise: 0.0,
            seed: 1,
        }
    }
}

/// Parses the `L6_BEAMS` item format: "K D [mu [lwScale [Z [noise [seed [gamma]]]]]]".
pub fn parse_spec_item(item: &str) -> Option<BeamParams> {
    let mut bp = BeamParams::default();
    let mut it = item.split_ascii_whitespace();
    bp.k = it.next()?.parse().ok()?;
    bp.d = it.next()?.parse().ok()?;
    if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
        bp.mu = x;
        if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
            for d in 0..8 {
                bp.lw[d] = x * 0.5f64.powi(d as i32);
            }
            if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
                bp.z = x;
                if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
                    bp.noise = x;
                    if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
                        bp.seed = x as u64;
                        if let Some(x) = it.next().and_then(|s| s.parse::<f64>().ok()) {
                            bp.gamma = x;
                        }
                    }
                }
            }
        }
    }
    Some(bp)
}

pub fn parse_spec(spec: &str) -> Vec<BeamParams> {
    spec.split(',').filter_map(parse_spec_item).collect()
}

/// Default configurations by board size (words of 64 cells), as in the C++ solver.
pub fn default_spec(words: usize) -> &'static str {
    match words {
        1 => "400 2 0.9 0.3 10,150 3",
        2 | 3 => "100 2 0.9 0.3 10,30 0 0.9",
        _ => "50 0 0.9,30 2 0.9 0.3 10",
    }
}

#[derive(Clone, Copy)]
struct State<const W: usize> {
    off: [i8; 6],
    on: u8,
    acc: u16,
    m: [u64; W],
}

#[derive(Clone, Copy)]
struct Cand {
    score: f64,
    parent: u32,
    code: u16,
}

#[derive(Clone, Copy)]
struct Opt {
    off: i8,
    on: u8,
    code: u8,
    phi: f64,
}

const NO_OPT: Opt = Opt { off: 0, on: 0, code: 0, phi: 0.0 };

struct Ctx<'a, const W: usize> {
    lm: &'a [[u64; W]],
    reach: &'a [[u64; W]],
    span: usize,
    r: i32,
    d: usize,
    dstride: usize,
    p: [u64; W],
    halo: [u64; W],
    pn: usize,
    lw: [f64; 8],
    z: f64,
    noise: f64,
    // per-state scratch
    s: State<W>,
    si: u32,
    acc_base: f64,
    opts: [[Opt; 4]; 6],
    nopts: [usize; 6],
    pick: [usize; 6],
    l1: [[u64; W]; 7],
    l2: [[u64; W]; 7],
    l3: [[u64; W]; 7],
    a1: [[[u64; W]; 8]; 7],
    a2: [[[u64; W]; 8]; 7],
    cands: Vec<Cand>,
    thresh: f64,
    m: usize,
    rng: u64,
}

impl<'a, const W: usize> Ctx<'a, W> {
    #[inline(always)]
    fn upd(&mut self, e: usize, k: usize) -> f64 {
        // SAFETY: e < 6, k < nopts[e] <= 4; |op.off| <= R so all table indices are in range.
        unsafe {
            let op = *self.opts.get_unchecked(e).get_unchecked(k);
            *self.pick.get_unchecked_mut(e) = k;
            let base = e * self.span + (op.off as i32 + self.r) as usize;
            if op.on != 0 {
                let l = self.lm.get_unchecked(base);
                let (l1, l2, l3) = (self.l1[e], self.l2[e], self.l3[e]);
                for i in 0..W {
                    self.l3[e + 1][i] = l3[i] | (l2[i] & l[i]);
                    self.l2[e + 1][i] = l2[i] | (l1[i] & l[i]);
                    self.l1[e + 1][i] = l1[i] | l[i];
                }
            } else {
                self.l1[e + 1] = self.l1[e];
                self.l2[e + 1] = self.l2[e];
                self.l3[e + 1] = self.l3[e];
            }
            let rbase = (base * 2 + op.on as usize) * self.dstride;
            for d in 0..self.d {
                let q = self.reach.get_unchecked(rbase + d);
                let (a1, a2) = (self.a1[e][d], self.a2[e][d]);
                for i in 0..W {
                    self.a2[e + 1][d][i] = a2[i] | (a1[i] & q[i]);
                    self.a1[e + 1][d][i] = a1[i] | q[i];
                }
            }
            op.phi
        }
    }

    #[inline(always)]
    fn leaf(&mut self, phi: f64) {
        let mut nm = [0u64; W];
        let mut full = true;
        for i in 0..W {
            let e2 = self.l2[6][i] & !self.l3[6][i];
            if e2 & self.halo[i] != 0 {
                return;
            }
            nm[i] = ((self.s.m[i] & !self.l3[6][i]) | e2) & self.p[i];
            if nm[i] != self.p[i] {
                full = false;
            }
        }
        let mut score;
        let mut need = [0u64; W];
        if full {
            score = self.acc_base + (self.pn + 1) as f64;
            need = self.p;
        } else {
            score = self.acc_base;
            let mut stamped = 0u32;
            for i in 0..W {
                stamped += nm[i].count_ones();
                need[i] = self.p[i] & !nm[i];
            }
            score += stamped as f64;
        }
        let mut pot = 0.0f64;
        for d in 0..self.d {
            let mut c = 0u32;
            for i in 0..W {
                c += (self.a2[6][d][i] & need[i]).count_ones();
            }
            pot += self.lw[d] * c as f64;
        }
        {
            let mut need_n = 0u32;
            for i in 0..W {
                need_n += need[i].count_ones();
            }
            if need_n as f64 > self.z {
                pot *= self.z / need_n as f64;
            }
        }
        score += pot + phi;
        if self.noise > 0.0 {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 7;
            self.rng ^= self.rng << 17;
            score += self.noise * ((self.rng >> 11) as f64) * (1.0 / 9007199254740992.0);
        }
        if score <= self.thresh {
            return;
        }
        let mut code = 0u16;
        for e in 0..6 {
            // SAFETY: pick[e] < nopts[e] <= 4.
            let c = unsafe { self.opts.get_unchecked(e).get_unchecked(self.pick[e]).code };
            code |= (c as u16) << (2 * e);
        }
        self.cands.push(Cand { score, parent: self.si, code });
        if self.cands.len() >= 4 * self.m {
            self.prune();
        }
    }

    #[cold]
    fn prune(&mut self) {
        let m = self.m;
        self.cands.select_nth_unstable_by(m - 1, |a, b| b.score.total_cmp(&a.score));
        self.cands.truncate(m);
        self.thresh = self.cands[m - 1].score;
    }

    fn expand(&mut self) {
        for k0 in 0..self.nopts[0] {
            let p0 = self.upd(0, k0);
            for k1 in 0..self.nopts[1] {
                let p1 = p0 + self.upd(1, k1);
                for k2 in 0..self.nopts[2] {
                    let p2 = p1 + self.upd(2, k2);
                    for k3 in 0..self.nopts[3] {
                        let p3 = p2 + self.upd(3, k3);
                        for k4 in 0..self.nopts[4] {
                            let p4 = p3 + self.upd(4, k4);
                            for k5 in 0..self.nopts[5] {
                                let p5 = p4 + self.upd(5, k5);
                                self.leaf(p5);
                            }
                        }
                    }
                }
            }
        }
    }
}

struct Sim<'a, const W: usize> {
    lm: &'a [[u64; W]],
    span: usize,
    r: i32,
    p: [u64; W],
    halo: [u64; W],
}

impl<'a, const W: usize> Sim<'a, W> {
    /// Applies a line (codes 0 noop 1 F 2 B 3 T). None if an emitter leaves +-R or the halo gets matter.
    fn apply(&self, s: &State<W>, code: &[u8; 6]) -> Option<State<W>> {
        let mut out = *s;
        let mut g1 = [0u64; W];
        let mut g2 = [0u64; W];
        let mut g3 = [0u64; W];
        for e in 0..6 {
            match code[e] {
                1 => out.off[e] += 1,
                2 => out.off[e] -= 1,
                3 => out.on ^= 1 << e,
                _ => {}
            }
            if (out.off[e] as i32).abs() > self.r {
                return None;
            }
            if out.on >> e & 1 != 0 {
                let l = &self.lm[e * self.span + (out.off[e] as i32 + self.r) as usize];
                for i in 0..W {
                    g3[i] |= g2[i] & l[i];
                    g2[i] |= g1[i] & l[i];
                    g1[i] |= l[i];
                }
            }
        }
        let mut full = true;
        for i in 0..W {
            let e2 = g2[i] & !g3[i];
            if e2 & self.halo[i] != 0 {
                return None;
            }
            out.m[i] = ((out.m[i] & !g3[i]) | e2) & self.p[i];
            if out.m[i] != self.p[i] {
                full = false;
            }
        }
        if full {
            out.acc += 1;
            out.m = [0u64; W];
        }
        Some(out)
    }
}

fn hash_state<const W: usize>(s: &State<W>) -> u64 {
    let mut h: u64 = 0x9e3779b97f4a7c15;
    let mut mix = |v: u64| {
        h ^= v.wrapping_add(0x9e3779b97f4a7c15).wrapping_add(h << 6).wrapping_add(h >> 2);
        h = h.wrapping_mul(0xff51afd7ed558ccd);
    };
    let mut a: u64 = 0;
    for e in 0..6 {
        a = a.wrapping_mul(64).wrapping_add((s.off[e] as i64 + 32) as u64);
    }
    mix(a);
    mix(s.on as u64 | ((s.acc as u64) << 8));
    for i in 0..W {
        mix(s.m[i]);
    }
    h
}

fn reconstruct(history: &[Vec<(u32, u16)>], depth_len: usize, mut idx: usize) -> Prog {
    const CODES: [u8; 4] = [b'-', b'F', b'B', b'T'];
    let mut prog = vec![NOOP; depth_len];
    for dd in (0..depth_len).rev() {
        let (par, cd) = history[dd][idx];
        let mut line: Line = NOOP;
        for e in 0..6 {
            line[e] = CODES[((cd >> (2 * e)) & 3) as usize];
        }
        prog[dd] = line;
        idx = par as usize;
    }
    prog
}

/// Beam search for one board. `max_len`: give up once a program can no longer be
/// shorter than this many lines (0 = unlimited). Returns None on failure.
pub fn beam_program_w<const W: usize>(g: &Geometry, t: &Target, bp: &BeamParams, max_len: usize) -> Option<Prog> {
    let r = g.r;
    let span = g.span();
    let dd = bp.d;
    let dstride = dd.max(1);
    let to_w = |b: &[u64; crate::KW]| -> [u64; W] {
        let mut o = [0u64; W];
        o.copy_from_slice(&b[..W]);
        o
    };
    let lm: Vec<[u64; W]> = g.masks.iter().map(|m| to_w(m)).collect();
    // reach[((e*span + o+R)*2 + on)*dstride + (d-1)]
    let mut reach = vec![[0u64; W]; 6 * span * 2 * dstride];
    for e in 0..6 {
        for o in -r..=r {
            for on in 0..2usize {
                for d in 1..=dd {
                    let rad = d as i32 - if on == 1 { 0 } else { 1 };
                    let mut m = [0u64; W];
                    for o2 in (o - rad).max(-r)..=(o + rad).min(r) {
                        let l = &lm[e * span + (o2 + r) as usize];
                        for i in 0..W {
                            m[i] |= l[i];
                        }
                    }
                    reach[((e * span + (o + r) as usize) * 2 + on) * dstride + d - 1] = m;
                }
            }
        }
    }
    let p = to_w(&t.p_mask);
    let halo = to_w(&t.halo_mask);
    let pn = t.cells.len();
    let decay: Vec<f64> = (0..(4 * r + 4) as usize).map(|d| bp.gamma.powi(d as i32)).collect();
    let sim = Sim::<W> { lm: &lm, span, r, p, halo };

    let count = t.count;
    let limit = 40 * pn * count + 200;
    let k = bp.k;
    let m = k * 4;

    let mut rng: u64 = bp
        .seed
        .wrapping_mul(0x9e3779b97f4a7c15)
        .wrapping_add((pn as u64).wrapping_mul(1315423911))
        .wrapping_add(88172645463325252);
    for &id in &t.cells {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(id as u64);
    }
    if rng == 0 {
        rng = 1;
    }

    let mut ctx = Ctx::<W> {
        lm: &lm,
        reach: &reach,
        span,
        r,
        d: dd,
        dstride,
        p,
        halo,
        pn,
        lw: bp.lw,
        z: bp.z,
        noise: bp.noise,
        s: State { off: [0; 6], on: 0, acc: 0, m: [0; W] },
        si: 0,
        acc_base: 0.0,
        opts: [[NO_OPT; 4]; 6],
        nopts: [0; 6],
        pick: [0; 6],
        l1: [[0; W]; 7],
        l2: [[0; W]; 7],
        l3: [[0; W]; 7],
        a1: [[[0; W]; 8]; 7],
        a2: [[[0; W]; 8]; 7],
        cands: Vec::with_capacity(4 * m + 16),
        thresh: -1e30,
        m,
        rng,
    };

    let mut beam: Vec<State<W>> = vec![State { off: [0; 6], on: 0, acc: 0, m: [0; W] }];
    let mut history: Vec<Vec<(u32, u16)>> = Vec::new();
    let mut need_cells: Vec<usize> = Vec::new();
    let mut last_prog: i64 = -1;
    let mut last_improve: usize = 0;
    let stuck_ticks: i64 = if bp.stuck > 0 { bp.stuck as i64 } else { 2 * r as i64 };
    let mut depth: usize = 0;
    let dbg = std::env::var_os("NANO_BDBG").is_some();
    loop {
        if depth > limit {
            return None;
        }
        if max_len > 0 && depth + 1 >= max_len {
            return None;
        }
        ctx.cands.clear();
        ctx.thresh = -1e30;
        for si in 0..beam.len() {
            let s = beam[si];
            need_cells.clear();
            for &id in &t.cells {
                if (s.m[id >> 6] >> (id & 63)) & 1 == 0 {
                    need_cells.push(id);
                }
            }
            let phi_norm = bp.mu / (6.0 * need_cells.len().max(1) as f64);
            for e in 0..6usize {
                ctx.nopts[e] = 0;
                for kk in 0..4u8 {
                    let mut op = Opt { off: s.off[e], on: (s.on >> e) & 1, code: kk, phi: 0.0 };
                    if kk == 1 {
                        op.off += 1;
                    }
                    if kk == 2 {
                        op.off -= 1;
                    }
                    if kk == 3 {
                        op.on ^= 1;
                    }
                    if (op.off as i32).abs() > r {
                        continue;
                    }
                    let mut phi = 0.0f64;
                    let extra = if op.on != 0 { 0 } else { 1 };
                    for &id in &need_cells {
                        phi += decay[((op.off as i32 - g.off_for[e][id]).abs() + extra) as usize];
                    }
                    op.phi = phi * phi_norm;
                    ctx.opts[e][ctx.nopts[e]] = op;
                    ctx.nopts[e] += 1;
                }
            }
            ctx.s = s;
            ctx.si = si as u32;
            ctx.acc_base = (s.acc as usize * (pn + 1)) as f64;
            ctx.l1[0] = [0; W];
            ctx.l2[0] = [0; W];
            ctx.l3[0] = [0; W];
            for d in 0..dd {
                ctx.a1[0][d] = [0; W];
                ctx.a2[0][d] = [0; W];
            }
            ctx.expand();
        }
        if ctx.cands.is_empty() {
            return None;
        }
        ctx.cands.sort_unstable_by(|a, b| {
            b.score.total_cmp(&a.score).then(a.parent.cmp(&b.parent)).then(a.code.cmp(&b.code))
        });
        {
            // progress = copies and stamped cells of the best state in the beam
            let mut best_prog: i64 = -1;
            for s in &beam {
                let mut c = s.acc as i64 * (pn as i64 + 1);
                for i in 0..W {
                    c += s.m[i].count_ones() as i64;
                }
                best_prog = best_prog.max(c);
            }
            if dbg && depth % 20 == 0 {
                eprintln!("d{} prog {} best {}", depth, best_prog, ctx.cands[0].score);
            }
            if best_prog > last_prog {
                last_prog = best_prog;
                last_improve = depth;
            }
            if (depth - last_improve) as i64 > stuck_ticks {
                // take the best-progress state; walk the cheapest pair to the nearest missing cell
                let mut si = 0usize;
                let mut sc: i64 = -1;
                for (i, s) in beam.iter().enumerate() {
                    let mut c = s.acc as i64 * (pn as i64 + 1);
                    for w in 0..W {
                        c += s.m[w].count_ones() as i64;
                    }
                    if c > sc {
                        sc = c;
                        si = i;
                    }
                }
                let mut cur = beam[si];
                let (mut best_cell, mut ba, mut bb, mut best_cost) = (usize::MAX, 0usize, 0usize, i32::MAX);
                for &id in &t.cells {
                    if (cur.m[id >> 6] >> (id & 63)) & 1 != 0 {
                        continue;
                    }
                    for x in 0..6usize {
                        for y in x + 1..6 {
                            if axis_of(x) == axis_of(y) {
                                continue;
                            }
                            let cost = (cur.off[x] as i32 - g.off_for[x][id])
                                .abs()
                                .max((cur.off[y] as i32 - g.off_for[y][id]).abs());
                            if cost < best_cost {
                                best_cost = cost;
                                best_cell = id;
                                ba = x;
                                bb = y;
                            }
                        }
                    }
                }
                if best_cell == usize::MAX {
                    return None;
                }
                let mut delta = [0i32; 6];
                let mut start_move = [0i32; 6];
                let mut off_first = [false; 6];
                let mut on_last = [false; 6];
                let mut ticks = 0i32;
                for e in 0..6 {
                    delta[e] = if e == ba || e == bb { g.off_for[e][best_cell] - cur.off[e] as i32 } else { 0 };
                    off_first[e] = (cur.on >> e) & 1 != 0;
                    on_last[e] = e == ba || e == bb;
                    start_move[e] = if off_first[e] { 1 } else { 0 };
                    ticks = ticks.max(start_move[e] + delta[e].abs() + if on_last[e] { 1 } else { 0 });
                }
                let mut parent = si as u32;
                for kk in 0..ticks {
                    let mut code = [0u8; 6];
                    let mut cd = 0u16;
                    for e in 0..6 {
                        if off_first[e] && kk == 0 {
                            code[e] = 3;
                        } else if on_last[e] && kk == ticks - 1 {
                            code[e] = 3;
                        } else if kk >= start_move[e] && kk < start_move[e] + delta[e].abs() {
                            code[e] = if delta[e] > 0 { 1 } else { 2 };
                        }
                        cd |= (code[e] as u16) << (2 * e);
                    }
                    let ns = sim.apply(&cur, &code)?;
                    history.push(vec![(parent, cd)]);
                    parent = 0;
                    cur = ns;
                    depth += 1;
                    if cur.acc as usize >= count {
                        return Some(reconstruct(&history, depth, 0));
                    }
                }
                beam = vec![cur];
                last_improve = depth;
                continue;
            }
        }
        let mut next: Vec<State<W>> = Vec::with_capacity(k);
        let mut hist: Vec<(u32, u16)> = Vec::with_capacity(k);
        let mut seen: HashSet<u64> = HashSet::with_capacity(2 * k);
        for c in &ctx.cands {
            if next.len() >= k {
                break;
            }
            let mut code = [0u8; 6];
            for e in 0..6 {
                code[e] = ((c.code >> (2 * e)) & 3) as u8;
            }
            let Some(ns) = sim.apply(&beam[c.parent as usize], &code) else { continue };
            if !seen.insert(hash_state(&ns)) {
                continue;
            }
            hist.push((c.parent, c.code));
            next.push(ns);
            if ns.acc as usize >= count {
                history.push(hist);
                let idx = next.len() - 1;
                return Some(reconstruct(&history, depth + 1, idx));
            }
        }
        history.push(hist);
        beam = next;
        depth += 1;
    }
}

pub fn beam_program(g: &Geometry, t: &Target, bp: &BeamParams, max_len: usize) -> Option<Prog> {
    match g.words {
        0 | 1 => beam_program_w::<1>(g, t, bp, max_len),
        2 => beam_program_w::<2>(g, t, bp, max_len),
        3 => beam_program_w::<3>(g, t, bp, max_len),
        4 => beam_program_w::<4>(g, t, bp, max_len),
        5 => beam_program_w::<5>(g, t, bp, max_len),
        6 => beam_program_w::<6>(g, t, bp, max_len),
        7 => beam_program_w::<7>(g, t, bp, max_len),
        _ => None,
    }
}
