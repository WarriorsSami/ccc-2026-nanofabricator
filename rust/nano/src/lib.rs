//! Nanofabricator level 6 core: parsing, geometry, exact simulator (identical
//! to `simulate()` in level-6/main.cpp) and the beam search (`beam`).
//!
//! Bitsets are `[u64; KW]` (up to 448 interior cells; radius 12 has 397).

pub mod beam;

pub const KW: usize = 7;
pub type Bits = [u64; KW];
pub type Line = [u8; 6];
pub type Prog = Vec<Line>;
pub const NOOP: Line = *b"------";

// Laser digit -> step in the char grid.
//   1 = right  2 = down-right  3 = down-left  4 = left  5 = up-left  6 = up-right
const LASER_STEPS: [(i32, i32); 6] = [(0, 2), (1, 1), (1, -1), (0, -2), (-1, -1), (-1, 1)];
const NEIGHBOURS: [(i32, i32); 6] = [(0, 2), (0, -2), (1, 1), (1, -1), (-1, 1), (-1, -1)];

/// Emitters e and e+3 fire along the same axis.
#[inline]
pub fn axis_of(e: usize) -> usize {
    e % 3
}

#[derive(Clone, Debug)]
pub struct Pattern {
    pub radius: usize,
    pub rows: usize,
    pub cols: usize,
    pub count: usize,
    pub grid: Vec<Vec<u8>>,
}

/// Input format: N, then per board "R rows cols count" followed by `rows` map lines.
pub fn parse_input(text: &str) -> Result<Vec<Pattern>, String> {
    let mut it = text.split_ascii_whitespace();
    let mut next = |what: &str| it.next().ok_or_else(|| format!("unexpected end of input reading {what}"));
    let n: usize = next("N")?.parse().map_err(|e| format!("bad N: {e}"))?;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let mut num = |what: &str| -> Result<usize, String> {
            next(what)?.parse::<usize>().map_err(|e| format!("bad {what}: {e}"))
        };
        let radius = num("radius")?;
        let rows = num("rows")?;
        let cols = num("cols")?;
        let count = num("count")?;
        let mut grid = Vec::with_capacity(rows);
        for _ in 0..rows {
            grid.push(next("map row")?.as_bytes().to_vec());
        }
        out.push(Pattern { radius, rows, cols, count, grid });
    }
    Ok(out)
}

/// Program files: lines of 6 tokens (first char of each token), blank line between programs.
pub fn read_programs(text: &str) -> Vec<Prog> {
    let mut progs: Vec<Prog> = vec![Vec::new()];
    for line in text.split('\n') {
        if line.bytes().all(|b| b == b' ' || b == b'\t' || b == b'\r') {
            if !progs.last().unwrap().is_empty() {
                progs.push(Vec::new());
            }
            continue;
        }
        let mut l = NOOP;
        for (e, tok) in line.split_ascii_whitespace().take(6).enumerate() {
            l[e] = tok.as_bytes()[0];
        }
        progs.last_mut().unwrap().push(l);
    }
    if progs.last().unwrap().is_empty() {
        progs.pop();
    }
    progs
}

pub fn write_program(out: &mut String, prog: &[Line]) {
    for l in prog {
        for e in 0..6 {
            out.push(l[e] as char);
            out.push(if e < 5 { ' ' } else { '\n' });
        }
    }
}

/// Board geometry for one radius.
pub struct Geometry {
    pub r: i32,
    pub rows: i32,
    pub cols: i32,
    pub ring_size: i32,
    pub n: usize,
    pub words: usize,
    pub cell_of: Vec<i32>,
    pub cell_pos: Vec<(i32, i32)>,
    pub neighbours: Vec<Vec<usize>>,
    /// beam[e][o + R]: cells hit by emitter e at offset o from its home corner.
    pub beam: [Vec<Vec<usize>>; 6],
    /// off_for[e][cell]: the offset at which emitter e's beam crosses the cell.
    pub off_for: [Vec<i32>; 6],
    /// masks[e * span + o + R]: bitset of beam[e][o + R].
    pub masks: Vec<Bits>,
}

impl Geometry {
    pub fn new(radius: usize) -> Geometry {
        let r = radius as i32;
        let rows = 2 * r + 1;
        let cols = 4 * r + 1;
        let ring_size = 6 * r;
        let mut cell_of = vec![-1i32; (rows * cols) as usize];
        let mut cell_pos: Vec<(i32, i32)> = Vec::new();
        let (cr, cc) = (r, 2 * r);
        for row in 0..rows {
            for col in 0..cols {
                let dr = (row - cr).abs();
                let dc = (col - cc).abs();
                if (dr + dc) % 2 != 0 || dc > 2 * r - dr {
                    continue;
                }
                if dr == r || dc == 2 * r - dr {
                    continue; // border rail
                }
                cell_of[(row * cols + col) as usize] = cell_pos.len() as i32;
                cell_pos.push((row, col));
            }
        }
        let n = cell_pos.len();
        let at = |row: i32, col: i32| -> i32 {
            if row < 0 || row >= rows || col < 0 || col >= cols {
                -1
            } else {
                cell_of[(row * cols + col) as usize]
            }
        };
        let mut neighbours = vec![Vec::new(); n];
        for id in 0..n {
            for &(dr, dc) in &NEIGHBOURS {
                let nb = at(cell_pos[id].0 + dr, cell_pos[id].1 + dc);
                if nb >= 0 {
                    neighbours[id].push(nb as usize);
                }
            }
        }
        // Rail cells in clockwise order from the left corner (emitter 1's home).
        let mut ring: Vec<(i32, i32)> = Vec::new();
        for k in 0..r {
            ring.push((r - k, k));
        }
        for k in 0..r {
            ring.push((0, r + 2 * k));
        }
        for k in 0..r {
            ring.push((k, 3 * r + k));
        }
        for k in 0..r {
            ring.push((r + k, 4 * r - k));
        }
        for k in 0..r {
            ring.push((2 * r, 3 * r - 2 * k));
        }
        for k in 0..r {
            ring.push((2 * r - k, r - k));
        }
        let ring_index = |e: i32, o: i32| -> usize { ((((e * r + o) % ring_size) + ring_size) % ring_size) as usize };
        let span = (2 * r + 1) as usize;
        let mut beam: [Vec<Vec<usize>>; 6] = Default::default();
        let mut masks = vec![[0u64; KW]; 6 * span];
        for e in 0..6usize {
            beam[e] = vec![Vec::new(); span];
            for o in -r..=r {
                let (r0, c0) = ring[ring_index(e as i32, o)];
                let (dr, dc) = LASER_STEPS[e];
                let (mut rr, mut cc2) = (r0 + dr, c0 + dc);
                loop {
                    let id = at(rr, cc2);
                    if id < 0 {
                        break;
                    }
                    beam[e][(o + r) as usize].push(id as usize);
                    masks[e * span + (o + r) as usize][(id as usize) >> 6] |= 1u64 << (id & 63);
                    rr += dr;
                    cc2 += dc;
                }
            }
        }
        let mut off_for: [Vec<i32>; 6] = Default::default();
        for e in 0..6 {
            off_for[e] = vec![i32::MIN; n];
            for o in -r..=r {
                for &id in &beam[e][(o + r) as usize] {
                    off_for[e][id] = o;
                }
            }
        }
        Geometry { r, rows, cols, ring_size, n, words: (n + 63) / 64, cell_of, cell_pos, neighbours, beam, off_for, masks }
    }

    #[inline]
    pub fn num_cells(&self) -> usize {
        self.n
    }
    #[inline]
    pub fn span(&self) -> usize {
        (2 * self.r + 1) as usize
    }
    #[inline]
    pub fn beam_at(&self, e: usize, o: i32) -> &[usize] {
        &self.beam[e][(o + self.r) as usize]
    }
    #[inline]
    pub fn mask(&self, e: usize, o: i32) -> &Bits {
        debug_assert!(o.abs() <= self.r);
        // SAFETY: callers guarantee |o| <= R, so the index is < 6 * span.
        unsafe { self.masks.get_unchecked(e * self.span() + (o + self.r) as usize) }
    }
}

/// The pattern in cell ids: pattern cells and the halo of non-pattern neighbours.
pub struct Target {
    pub cells: Vec<usize>,
    pub is_p: Vec<bool>,
    pub is_halo: Vec<bool>,
    pub count: usize,
    pub p_mask: Bits,
    pub halo_mask: Bits,
}

impl Target {
    pub fn new(g: &Geometry, p: &Pattern) -> Target {
        let n = g.num_cells();
        let mut is_p = vec![false; n];
        let mut is_halo = vec![false; n];
        let mut cells = Vec::new();
        for id in 0..n {
            let (row, col) = g.cell_pos[id];
            if p.grid[row as usize][col as usize] == b'M' {
                is_p[id] = true;
                cells.push(id);
            }
        }
        for &id in &cells {
            for &nb in &g.neighbours[id] {
                if !is_p[nb] {
                    is_halo[nb] = true;
                }
            }
        }
        let mut p_mask = [0u64; KW];
        let mut halo_mask = [0u64; KW];
        for id in 0..n {
            if is_p[id] {
                p_mask[id >> 6] |= 1 << (id & 63);
            }
            if is_halo[id] {
                halo_mask[id >> 6] |= 1 << (id & 63);
            }
        }
        Target { cells, is_p, is_halo, count: p.count, p_mask, halo_mask }
    }
}

/// Exact simulation: the line count after which `count` patterns were submitted,
/// or -1 if the program never gets there. Identical semantics to the C++ `simulate()`.
pub fn simulate(g: &Geometry, t: &Target, prog: &[Line]) -> i32 {
    let r = g.r;
    let w = g.words.max(1);
    let mut off = [0i32; 6];
    let mut active = [false; 6];
    let mut matter = [0u64; KW];
    let mut accepted = 0usize;
    for (tick, line) in prog.iter().enumerate() {
        for e in 0..6 {
            match line[e] {
                b'F' => off[e] += 1,
                b'B' => off[e] -= 1,
                b'T' => active[e] = !active[e],
                _ => {}
            }
            // Offsets wrap around the rail; keep them in (-3R, 3R].
            if off[e] > 3 * r {
                off[e] -= g.ring_size;
            }
            if off[e] <= -3 * r {
                off[e] += g.ring_size;
            }
        }
        let mut g1 = [0u64; KW];
        let mut g2 = [0u64; KW];
        let mut g3 = [0u64; KW];
        for e in 0..6 {
            if !active[e] || off[e].abs() > r {
                continue; // outside +-R the beam runs along the rail
            }
            let m = g.mask(e, off[e]);
            for i in 0..w {
                g3[i] |= g2[i] & m[i];
                g2[i] |= g1[i] & m[i];
                g1[i] |= m[i];
            }
        }
        let mut present = true;
        for i in 0..w {
            matter[i] = (matter[i] & !g3[i]) | (g2[i] & !g3[i]);
            if matter[i] & t.p_mask[i] != t.p_mask[i] || matter[i] & t.halo_mask[i] != 0 {
                present = false;
            }
        }
        if present {
            for i in 0..w {
                matter[i] &= !t.p_mask[i];
            }
            accepted += 1;
            if accepted >= t.count {
                return tick as i32 + 1;
            }
        }
    }
    -1
}
