// Geometry, pattern, board bitsets and the exact simulator (mirrors level-6/main.cpp).
use std::collections::HashSet;

pub type Line = [u8; 6];
pub type Prog = Vec<Line>;

pub const NONE: u8 = 255;
pub const MAXL: usize = 7;

pub struct Pattern {
    pub radius: i32,
    pub rows: i32,
    pub cols: i32,
    pub count: i32,
    pub grid: Vec<Vec<u8>>,
}

pub fn parse_input(text: &str) -> Vec<Pattern> {
    let mut it = text.split_whitespace();
    let n: usize = it.next().unwrap().parse().unwrap();
    let mut res = Vec::new();
    for _ in 0..n {
        let radius: i32 = it.next().unwrap().parse().unwrap();
        let rows: i32 = it.next().unwrap().parse().unwrap();
        let cols: i32 = it.next().unwrap().parse().unwrap();
        let count: i32 = it.next().unwrap().parse().unwrap();
        let mut grid = Vec::new();
        for _ in 0..rows {
            grid.push(it.next().unwrap().as_bytes().to_vec());
        }
        res.push(Pattern { radius, rows, cols, count, grid });
    }
    res
}

pub fn parse_prog(text: &str) -> Prog {
    let mut p = Vec::new();
    for line in text.lines() {
        let toks: Vec<&str> = line.split_whitespace().collect();
        if toks.is_empty() {
            continue;
        }
        let mut l = [b'-'; 6];
        for e in 0..6.min(toks.len()) {
            l[e] = toks[e].as_bytes()[0];
        }
        p.push(l);
    }
    p
}

pub fn prog_text(p: &[Line]) -> String {
    let mut s = String::new();
    for l in p {
        for e in 0..6 {
            s.push(l[e] as char);
            s.push(if e < 5 { ' ' } else { '\n' });
        }
    }
    s
}

const STEPS: [(i32, i32); 6] = [(0, 2), (1, 1), (1, -1), (0, -2), (-1, -1), (-1, 1)];
const NEIGH: [(i32, i32); 6] = [(0, 2), (0, -2), (1, 1), (1, -1), (-1, 1), (-1, -1)];

pub struct Geo {
    pub r: i32,
    pub rows: i32,
    pub cols: i32,
    pub ring: i32,
    pub cell_of: Vec<i32>,
    pub cell_pos: Vec<(i32, i32)>,
    pub nbrs: Vec<Vec<usize>>,
    pub beam: Vec<Vec<Vec<usize>>>, // [e][o+R] -> cells
}

impl Geo {
    pub fn at(&self, r: i32, c: i32) -> i32 {
        if r < 0 || r >= self.rows || c < 0 || c >= self.cols {
            -1
        } else {
            self.cell_of[(r * self.cols + c) as usize]
        }
    }
    pub fn new(r: i32) -> Geo {
        let rows = 2 * r + 1;
        let cols = 4 * r + 1;
        let ring = 6 * r;
        let mut cell_of = vec![-1i32; (rows * cols) as usize];
        let mut cell_pos = Vec::new();
        let (cr, cc) = (r, 2 * r);
        for rr in 0..rows {
            for c in 0..cols {
                let dr = (rr - cr).abs();
                let dc = (c - cc).abs();
                if (dr + dc) % 2 != 0 || dc > 2 * r - dr {
                    continue;
                }
                if dr == r || dc == 2 * r - dr {
                    continue;
                }
                cell_of[(rr * cols + c) as usize] = cell_pos.len() as i32;
                cell_pos.push((rr, c));
            }
        }
        let mut g = Geo { r, rows, cols, ring, cell_of, cell_pos, nbrs: Vec::new(), beam: Vec::new() };
        let mut nbrs = vec![Vec::new(); g.cell_pos.len()];
        for id in 0..g.cell_pos.len() {
            for (dr, dc) in NEIGH {
                let n = g.at(g.cell_pos[id].0 + dr, g.cell_pos[id].1 + dc);
                if n >= 0 {
                    nbrs[id].push(n as usize);
                }
            }
        }
        g.nbrs = nbrs;
        let mut rail: Vec<(i32, i32)> = Vec::new();
        for k in 0..r {
            rail.push((r - k, k));
        }
        for k in 0..r {
            rail.push((0, r + 2 * k));
        }
        for k in 0..r {
            rail.push((k, 3 * r + k));
        }
        for k in 0..r {
            rail.push((r + k, 4 * r - k));
        }
        for k in 0..r {
            rail.push((2 * r, 3 * r - 2 * k));
        }
        for k in 0..r {
            rail.push((2 * r - k, r - k));
        }
        let mut beam = vec![vec![Vec::new(); (2 * r + 1) as usize]; 6];
        for e in 0..6usize {
            for o in -r..=r {
                let idx = (((e as i32 * r + o) % ring) + ring) % ring;
                let (r0, c0) = rail[idx as usize];
                let (dr, dc) = STEPS[e];
                let (mut rr, mut c) = (r0 + dr, c0 + dc);
                loop {
                    let id = g.at(rr, c);
                    if id < 0 {
                        break;
                    }
                    beam[e][(o + r) as usize].push(id as usize);
                    rr += dr;
                    c += dc;
                }
            }
        }
        g.beam = beam;
        g
    }
}

pub struct Rel {
    pub is_p: Vec<bool>,
    pub is_h: Vec<bool>,
    pub rel_idx: Vec<i32>,
    pub nrel: usize,
}

pub fn relevant(geo: &Geo, pat: &Pattern) -> Rel {
    let n = geo.cell_pos.len();
    let mut is_p = vec![false; n];
    let mut is_h = vec![false; n];
    for id in 0..n {
        let (r, c) = geo.cell_pos[id];
        if pat.grid[r as usize][c as usize] == b'M' {
            is_p[id] = true;
        }
    }
    for id in 0..n {
        if is_p[id] {
            for &nb in &geo.nbrs[id] {
                if !is_p[nb] {
                    is_h[nb] = true;
                }
            }
        }
    }
    let mut rel_idx = vec![-1i32; n];
    let mut m = 0;
    for id in 0..n {
        if is_p[id] || is_h[id] {
            rel_idx[id] = m;
            m += 1;
        }
    }
    Rel { is_p, is_h, rel_idx, nrel: m as usize }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct St<const W: usize> {
    pub off: [i32; 6],
    pub on: u8,
    pub acc: i32,
    pub m: [u64; W],
}

impl<const W: usize> St<W> {
    pub fn new() -> Self {
        St { off: [0; 6], on: 0, acc: 0, m: [0; W] }
    }
    pub fn hash(&self) -> u64 {
        let mut h: u64 = 0xcbf29ce484222325;
        let mut mix = |v: u64| {
            h ^= v;
            h = h.wrapping_mul(0x100000001b3);
            h ^= h >> 29;
        };
        for e in 0..6 {
            mix((self.off[e] + 1000) as u64);
        }
        mix(self.on as u64 | ((self.acc as u64) << 8));
        for w in 0..W {
            mix(self.m[w]);
        }
        h
    }
}

pub struct Board<const W: usize> {
    pub r: i32,
    pub ring: i32,
    pub count: i32,
    pub span: usize,
    pub nrel: usize,
    pub p: [u64; W],
    pub h: [u64; W],
    pub valid: [u64; W],
    pub line: Vec<[u64; W]>, // e*span + o + r
    pub lrel: Vec<bool>,
}

#[derive(Clone, Copy)]
pub struct Walk {
    pub acts: [u8; MAXL],
    pub beam: [u8; MAXL],
}

impl<const W: usize> Board<W> {
    pub fn new(geo: &Geo, pat: &Pattern, rel: &Rel) -> Self {
        let r = geo.r;
        let span = (2 * r + 1) as usize;
        let mut p = [0u64; W];
        let mut h = [0u64; W];
        let mut valid = [0u64; W];
        for id in 0..geo.cell_pos.len() {
            let ri = rel.rel_idx[id];
            if ri < 0 {
                continue;
            }
            let ri = ri as usize;
            valid[ri >> 6] |= 1 << (ri & 63);
            if rel.is_p[id] {
                p[ri >> 6] |= 1 << (ri & 63);
            } else {
                h[ri >> 6] |= 1 << (ri & 63);
            }
        }
        let mut line = vec![[0u64; W]; 6 * span];
        let mut lrel = vec![false; 6 * span];
        for e in 0..6 {
            for o in 0..span {
                for &id in &geo.beam[e][o] {
                    let ri = rel.rel_idx[id];
                    if ri >= 0 {
                        let ri = ri as usize;
                        line[e * span + o][ri >> 6] |= 1 << (ri & 63);
                        lrel[e * span + o] = true;
                    }
                }
            }
        }
        Board { r, ring: geo.ring, count: pat.count, span, nrel: rel.nrel, p, h, valid, line, lrel }
    }

    #[inline(always)]
    pub fn step(&self, s: &mut St<W>, l: &Line) -> bool {
        let r = self.r;
        let r3 = 3 * r;
        let mut c1 = [0u64; W];
        let mut c2 = [0u64; W];
        let mut c3 = [0u64; W];
        for e in 0..6 {
            let mut o = s.off[e];
            match l[e] {
                b'F' => {
                    o += 1;
                    if o > r3 {
                        o -= self.ring;
                    }
                }
                b'B' => {
                    o -= 1;
                    if o <= -r3 {
                        o += self.ring;
                    }
                }
                b'T' => s.on ^= 1 << e,
                _ => {}
            }
            s.off[e] = o;
            if (s.on >> e) & 1 != 0 && o.abs() <= r {
                let m = unsafe { self.line.get_unchecked(e * self.span + (o + r) as usize) };
                for w in 0..W {
                    c3[w] |= c2[w] & m[w];
                    c2[w] |= c1[w] & m[w];
                    c1[w] |= m[w];
                }
            }
        }
        let mut ok = true;
        for w in 0..W {
            s.m[w] = (s.m[w] | c2[w]) & !c3[w];
        }
        for w in 0..W {
            ok &= (self.p[w] & !s.m[w]) == 0 && (self.h[w] & s.m[w]) == 0;
        }
        if ok {
            for w in 0..W {
                s.m[w] &= !self.p[w];
            }
            s.acc += 1;
        }
        ok
    }

    /// Number of lines until `count` submissions, or None.
    pub fn run(&self, prog: &[Line]) -> Option<usize> {
        let mut s = St::<W>::new();
        for (t, l) in prog.iter().enumerate() {
            self.step(&mut s, l);
            if s.acc >= self.count {
                return Some(t + 1);
            }
        }
        None
    }

    pub fn states(&self, prog: &[Line]) -> Vec<St<W>> {
        let mut v = Vec::with_capacity(prog.len() + 1);
        let mut s = St::<W>::new();
        v.push(s);
        for l in prog {
            self.step(&mut s, l);
            v.push(s);
        }
        v
    }

    #[inline]
    fn circ(&self, a: i32, b: i32) -> i32 {
        let d = (a - b).rem_euclid(self.ring);
        d.min(self.ring - d)
    }

    pub fn gen_walks(&self, e: usize, off0: i32, on0: u8, tgt: Option<(i32, u8)>, l: usize) -> Vec<Walk> {
        let mut out = Vec::new();
        let mut seen: HashSet<[u8; MAXL]> = HashSet::new();
        let mut acts = [b'-'; MAXL];
        let mut beam = [NONE; MAXL];
        self.walk_rec(e, 0, off0, on0, tgt, l, &mut acts, &mut beam, &mut seen, &mut out);
        out
    }

    #[allow(clippy::too_many_arguments)]
    fn walk_rec(
        &self,
        e: usize,
        q: usize,
        o: i32,
        a: u8,
        tgt: Option<(i32, u8)>,
        l: usize,
        acts: &mut [u8; MAXL],
        beam: &mut [u8; MAXL],
        seen: &mut HashSet<[u8; MAXL]>,
        out: &mut Vec<Walk>,
    ) {
        if q == l {
            if let Some((to, ta)) = tgt {
                if o != to || a != ta {
                    return;
                }
            }
            if seen.insert(*beam) {
                out.push(Walk { acts: *acts, beam: *beam });
            }
            return;
        }
        for &op in &[b'-', b'F', b'B', b'T'] {
            let mut o2 = o;
            let mut a2 = a;
            match op {
                b'F' => {
                    o2 += 1;
                    if o2 > 3 * self.r {
                        o2 -= self.ring;
                    }
                }
                b'B' => {
                    o2 -= 1;
                    if o2 <= -3 * self.r {
                        o2 += self.ring;
                    }
                }
                b'T' => a2 ^= 1,
                _ => {}
            }
            if let Some((to, ta)) = tgt {
                let need = self.circ(o2, to) + (a2 != ta) as i32;
                if need > (l - q - 1) as i32 {
                    continue;
                }
            }
            acts[q] = op;
            let b = if a2 == 1 && o2.abs() <= self.r && self.lrel[e * self.span + (o2 + self.r) as usize] {
                (o2 + self.r) as u8
            } else {
                NONE
            };
            beam[q] = b;
            self.walk_rec(e, q + 1, o2, a2, tgt, l, acts, beam, seen, out);
        }
        acts[q] = b'-';
        beam[q] = NONE;
    }
}
