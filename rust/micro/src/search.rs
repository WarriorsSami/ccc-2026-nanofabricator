// Bitset DFS over per-emitter walks with possibility-set pruning.
use crate::core::*;

type Cnt<const W: usize> = [[[u64; W]; 3]; MAXL];

pub struct Search<'a, const W: usize> {
    b: &'a Board<W>,
    l: usize,
    s: St<W>,
    has_t: bool,
    tm: [u64; W],
    t_acc: i32,
    care: [u64; W],
    walks: [Vec<Walk>; 6],
    ord: [usize; 6],
    rem: Vec<Cnt<W>>,
    cur: Vec<Cnt<W>>,
    pick: [usize; 6],
    accpat: u32,
    pub nodes: u64,
    cap: u64,
}

impl<'a, const W: usize> Search<'a, W> {
    fn feasible(&self, d: usize) -> bool {
        let b = self.b;
        let mut zz = [0u64; W];
        let mut oo = self.s.m;
        for w in 0..W {
            zz[w] = !self.s.m[w] & b.valid[w];
        }
        let cur = &self.cur[d];
        let rem = &self.rem[d];
        for q in 0..self.l {
            let n = &cur[q];
            let r = &rem[q];
            for w in 0..W {
                let (n1, n2, n3) = (n[0][w], n[1][w], n[2][w]);
                let (m1, m2, m3) = (r[0][w], r[1][w], r[2][w]);
                let less = !n2;
                let ge2 = n2 | m2 | (n1 & m1);
                let ge3 = n3 | m3 | (n2 & m1) | (n1 & m2);
                let can2 = !n3 & ge2;
                oo[w] = can2 | (oo[w] & less);
                zz[w] = ge3 | (zz[w] & less);
            }
            if (self.accpat >> q) & 1 != 0 {
                for w in 0..W {
                    if ((b.p[w] & !oo[w]) | (b.h[w] & !zz[w])) != 0 {
                        return false;
                    }
                }
                for w in 0..W {
                    oo[w] = 0;
                    zz[w] = b.valid[w];
                }
            }
        }
        if self.has_t {
            for w in 0..W {
                let c = self.care[w];
                if ((self.tm[w] & c & !oo[w]) | (!self.tm[w] & c & !zz[w])) != 0 {
                    return false;
                }
            }
        }
        true
    }

    fn leaf(&mut self, sink: &mut dyn FnMut(&[Line]) -> u8) -> u8 {
        let b = self.b;
        let mut st = self.s;
        for q in 0..self.l {
            let c = &self.cur[6][q];
            for w in 0..W {
                st.m[w] = (st.m[w] | c[1][w]) & !c[2][w];
            }
            let mut ok = true;
            for w in 0..W {
                ok &= (b.p[w] & !st.m[w]) == 0 && (b.h[w] & st.m[w]) == 0;
            }
            if ok {
                for w in 0..W {
                    st.m[w] &= !b.p[w];
                }
                st.acc += 1;
            }
        }
        if self.has_t {
            if st.acc != self.t_acc {
                return 0;
            }
            for w in 0..W {
                if (st.m[w] ^ self.tm[w]) & self.care[w] != 0 {
                    return 0;
                }
            }
        } else if st.acc < b.count {
            return 0;
        }
        let mut lines = [[b'-'; 6]; MAXL];
        for e in 0..6 {
            let wk = &self.walks[e][self.pick[e]];
            for q in 0..self.l {
                lines[q][e] = wk.acts[q];
            }
        }
        sink(&lines[..self.l])
    }

    fn dfs(&mut self, d: usize, sink: &mut dyn FnMut(&[Line]) -> u8) -> u8 {
        if d == 6 {
            return self.leaf(sink);
        }
        let b = self.b;
        let e = self.ord[d];
        let nw = self.walks[e].len();
        for wi in 0..nw {
            self.nodes += 1;
            if self.nodes > self.cap {
                return 2;
            }
            let walk = self.walks[e][wi];
            for q in 0..self.l {
                if walk.beam[q] == NONE {
                    self.cur[d + 1][q] = self.cur[d][q];
                } else {
                    let m = &b.line[e * b.span + walk.beam[q] as usize];
                    let c = &self.cur[d][q];
                    let mut n = [[0u64; W]; 3];
                    for w in 0..W {
                        n[2][w] = c[2][w] | (c[1][w] & m[w]);
                        n[1][w] = c[1][w] | (c[0][w] & m[w]);
                        n[0][w] = c[0][w] | m[w];
                    }
                    self.cur[d + 1][q] = n;
                }
            }
            if !self.feasible(d + 1) {
                continue;
            }
            self.pick[e] = wi;
            let r = self.dfs(d + 1, sink);
            if r != 0 {
                return r;
            }
        }
        0
    }
}

/// Search for `l` lines leading from `s` to `tgt` (matter equal on `care`, same acc), or, when
/// `tgt` is None, to reaching `count` accepted patterns. Returns 0 none, 1 accepted by sink, 2 aborted.
pub fn search_window<const W: usize>(
    b: &Board<W>,
    s: &St<W>,
    tgt: Option<&St<W>>,
    need: i32,
    l: usize,
    care: [u64; W],
    cap: u64,
    nodes_out: &mut u64,
    sink: &mut dyn FnMut(&[Line]) -> u8,
) -> u8 {
    if need < 0 || need as usize > l || l == 0 || l > MAXL - 1 {
        return 0;
    }
    let mut walks: [Vec<Walk>; 6] = Default::default();
    for e in 0..6 {
        let t = tgt.map(|t| (t.off[e], (t.on >> e) & 1));
        walks[e] = b.gen_walks(e, s.off[e], (s.on >> e) & 1, t, l);
        if walks[e].is_empty() {
            return 0;
        }
    }
    let mut ord = [0, 1, 2, 3, 4, 5];
    ord.sort_by_key(|&e| walks[e].len());
    // union masks
    let mut um = vec![[[0u64; W]; MAXL]; 6];
    for e in 0..6 {
        for wk in &walks[e] {
            for q in 0..l {
                if wk.beam[q] != NONE {
                    let m = &b.line[e * b.span + wk.beam[q] as usize];
                    for w in 0..W {
                        um[e][q][w] |= m[w];
                    }
                }
            }
        }
    }
    let zero: Cnt<W> = [[[0u64; W]; 3]; MAXL];
    let mut rem = vec![zero; 7];
    for d in (0..6).rev() {
        let mut r = rem[d + 1];
        let e = ord[d];
        for q in 0..l {
            for w in 0..W {
                let u = um[e][q][w];
                r[q][2][w] |= r[q][1][w] & u;
                r[q][1][w] |= r[q][0][w] & u;
                r[q][0][w] |= u;
            }
        }
        rem[d] = r;
    }
    let mut sr = Search {
        b,
        l,
        s: *s,
        has_t: tgt.is_some(),
        tm: tgt.map(|t| t.m).unwrap_or([0; W]),
        t_acc: tgt.map(|t| t.acc).unwrap_or(0),
        care,
        walks,
        ord,
        rem,
        cur: vec![zero; 7],
        pick: [0; 6],
        accpat: 0,
        nodes: 0,
        cap,
    };
    let mut res = 0u8;
    for pat in 0u32..(1u32 << l) {
        if pat.count_ones() as i32 != need {
            continue;
        }
        if tgt.is_none() && (pat >> (l - 1)) & 1 == 0 {
            continue;
        }
        sr.accpat = pat;
        if !sr.feasible(0) {
            continue;
        }
        let r = sr.dfs(0, sink);
        if r != 0 {
            res = r;
            break;
        }
    }
    *nodes_out += sr.nodes;
    res
}
