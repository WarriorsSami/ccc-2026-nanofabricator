// Nanofabricator — level 6
//
// `ccc run`    builds this file, runs every input, diffs the example.
// `ccc submit` does the same, then uploads the outputs.
//
// Optimisation level: for every pattern, emit a short program that submits it
// at least `count` times. Every candidate program is checked with an exact
// simulator, and the shortest valid one wins.
#include "ccc.hpp"
#include <atomic>
#include <thread>

constexpr int LEVEL = 6;

struct Pattern {
    int radius = 0;
    int rows = 0;
    int cols = 0;
    int count = 0;
    std::vector<std::string> grid;
};

struct Input {
    // N, then N patterns: "radius rows cols count" followed by `rows` map lines.
    std::vector<Pattern> cases;
};

Input parse(Reader& r) {
    Input in;
    const auto n = r.next<i64>();
    for (i64 i = 0; i < n; ++i) {
        auto& p = in.cases.emplace_back();
        r >> p.radius >> p.rows >> p.cols >> p.count;
        for (int k = 0; k < p.rows; ++k) p.grid.push_back(r.next<std::string>());
    }
    return in;
}

using Line = std::array<char, 6>;
using Prog = std::vector<Line>;

constexpr Line kNoop = {'-', '-', '-', '-', '-', '-'};

// Laser digit -> step in the char grid.
//   1 = →   2 = ↘   3 = ↙   4 = ←   5 = ↖   6 = ↗
constexpr std::array<std::pair<int, int>, 6> kLaserSteps = {{
    {0, 2}, {1, 1}, {1, -1}, {0, -2}, {-1, -1}, {-1, 1},
}};

constexpr std::array<std::pair<int, int>, 6> kNeighbours = {{
    {0, 2}, {0, -2}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
}};

// Emitters e and e+3 fire along the same axis (in opposite directions).
int axisOf(int e) { return e % 3; }

// Board geometry for one radius: interior cells, the border rail, and which
// interior cells each emitter's beam covers from each rail position.
struct Geometry {
    int R = 0, rows = 0, cols = 0, ringSize = 0;
    std::vector<int> cellOf;                    // grid index -> interior cell id, or -1
    std::vector<std::pair<int, int>> cellPos;   // cell id -> (row, col)
    std::vector<std::vector<int>> neighbours;   // cell id -> interior neighbour ids
    // beam[e][o + R]: cells hit by emitter e at offset o from its home corner.
    std::array<std::vector<std::vector<int>>, 6> beam;
    // offFor[e][cell]: the offset at which emitter e's beam crosses the cell.
    std::array<std::vector<int>, 6> offFor;

    explicit Geometry(int radius) : R(radius), rows(2 * radius + 1), cols(4 * radius + 1), ringSize(6 * radius) {
        cellOf.assign(rows * cols, -1);
        const int centerRow = R, centerCol = 2 * R;
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < cols; ++c) {
                const int dr = std::abs(r - centerRow), dc = std::abs(c - centerCol);
                if ((dr + dc) % 2 != 0 || dc > 2 * R - dr) continue;
                if (dr == R || dc == 2 * R - dr) continue;  // border rail
                cellOf[r * cols + c] = static_cast<int>(cellPos.size());
                cellPos.push_back({r, c});
            }
        neighbours.resize(cellPos.size());
        for (int id = 0; id < static_cast<int>(cellPos.size()); ++id)
            for (const auto& [dr, dc] : kNeighbours) {
                const int n = at(cellPos[id].first + dr, cellPos[id].second + dc);
                if (n >= 0) neighbours[id].push_back(n);
            }

        // Rail cells in clockwise order from the left corner (emitter 1's home).
        std::vector<std::pair<int, int>> ring;
        for (int k = 0; k < R; ++k) ring.push_back({R - k, k});
        for (int k = 0; k < R; ++k) ring.push_back({0, R + 2 * k});
        for (int k = 0; k < R; ++k) ring.push_back({k, 3 * R + k});
        for (int k = 0; k < R; ++k) ring.push_back({R + k, 4 * R - k});
        for (int k = 0; k < R; ++k) ring.push_back({2 * R, 3 * R - 2 * k});
        for (int k = 0; k < R; ++k) ring.push_back({2 * R - k, R - k});

        for (int e = 0; e < 6; ++e) {
            beam[e].resize(2 * R + 1);
            for (int o = -R; o <= R; ++o) {
                const auto [r0, c0] = ring[ringIndex(e, o)];
                const auto [dr, dc] = kLaserSteps[e];
                for (int r = r0 + dr, c = c0 + dc;; r += dr, c += dc) {
                    const int id = at(r, c);
                    if (id < 0) break;
                    beam[e][o + R].push_back(id);
                }
            }
        }
        for (int e = 0; e < 6; ++e) {
            offFor[e].assign(numCells(), INT_MIN);
            for (int o = -R; o <= R; ++o)
                for (int id : beamAt(e, o)) offFor[e][id] = o;
        }
    }

    int at(int r, int c) const {
        if (r < 0 || r >= rows || c < 0 || c >= cols) return -1;
        return cellOf[r * cols + c];
    }
    int ringIndex(int e, int o) const { return ((e * R + o) % ringSize + ringSize) % ringSize; }
    int numCells() const { return static_cast<int>(cellPos.size()); }
    const std::vector<int>& beamAt(int e, int o) const { return beam[e][o + R]; }
};

// The pattern in cell ids: which cells must hold matter, and the "halo" of
// cells next to it that must stay empty when it is picked up.
struct Target {
    std::vector<int> cells;
    std::vector<char> isP, isHalo;
    int count = 0;

    Target(const Geometry& g, const Pattern& p) : count(p.count) {
        isP.assign(g.numCells(), 0);
        isHalo.assign(g.numCells(), 0);
        for (int id = 0; id < g.numCells(); ++id) {
            const auto [r, c] = g.cellPos[id];
            if (p.grid[r][c] == 'M') {
                isP[id] = 1;
                cells.push_back(id);
            }
        }
        for (int id : cells)
            for (int n : g.neighbours[id])
                if (!isP[n]) isHalo[n] = 1;
    }
};

// Exact simulation: returns the line count after which `count` patterns were
// submitted, or -1 if the program never gets there.
int simulate(const Geometry& g, const Target& t, const Prog& prog) {
    std::array<int, 6> off{};
    std::array<bool, 6> active{};
    std::vector<char> matter(g.numCells(), 0);
    std::vector<int> hits(g.numCells(), 0);
    int accepted = 0;
    for (int tick = 0; tick < static_cast<int>(prog.size()); ++tick) {
        for (int e = 0; e < 6; ++e) {
            switch (prog[tick][e]) {
                case 'F': ++off[e]; break;
                case 'B': --off[e]; break;
                case 'T': active[e] = !active[e]; break;
                default: break;
            }
            // Offsets wrap around the rail; keep them in (-3R, 3R].
            if (off[e] > 3 * g.R) off[e] -= g.ringSize;
            if (off[e] <= -3 * g.R) off[e] += g.ringSize;
        }
        std::vector<int> touched;
        for (int e = 0; e < 6; ++e) {
            if (!active[e] || std::abs(off[e]) > g.R) continue;  // outside ±R the beam runs along the rail
            for (int id : g.beamAt(e, off[e])) {
                if (hits[id]++ == 0) touched.push_back(id);
            }
        }
        for (int id : touched) {
            if (hits[id] == 2) matter[id] = 1;
            else if (hits[id] >= 3) matter[id] = 0;
            hits[id] = 0;
        }
        bool present = true;
        for (int id : t.cells) present = present && matter[id];
        if (present)
            for (int id = 0; id < g.numCells() && present; ++id) present = !(t.isHalo[id] && matter[id]);
        if (present) {
            for (int id : t.cells) matter[id] = 0;
            if (++accepted >= t.count) return tick + 1;
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Strategy 1: a static configuration whose exactly-two-beam cells contain the
// pattern and avoid its halo. It re-creates the pattern every tick, so after
// setup every further copy costs one line.

struct StaticSearch {
    const Geometry& g;
    const Target& t;
    std::vector<std::vector<int>> options;  // per emitter: candidate offsets, or INT_MIN = inactive
    std::vector<int> hits;
    std::array<int, 6> choice{};
    std::array<int, 6> best{};
    int bestCost = INT_MAX;
    i64 nodes = 0;
    static constexpr i64 kNodeLimit = 20'000'000;
    static constexpr int kOff = INT_MIN;

    StaticSearch(const Geometry& geo, const Target& tgt) : g(geo), t(tgt) {
        hits.assign(g.numCells(), 0);
        for (int e = 0; e < 6; ++e) {
            auto& opts = options.emplace_back();
            opts.push_back(kOff);
            std::vector<int> useful;
            for (int o = -(g.R - 1); o <= g.R - 1; ++o) {
                bool touches = false;
                for (int id : g.beamAt(e, o)) touches = touches || t.isP[id] || t.isHalo[id];
                if (touches) useful.push_back(o);
            }
            std::sort(useful.begin(), useful.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
            opts.insert(opts.end(), useful.begin(), useful.end());
        }
    }

    void run() { dfs(0, 0); }

    void dfs(int e, int cost) {
        if (++nodes > kNodeLimit || cost >= bestCost) return;
        if (e == 6) {
            for (int id : t.cells)
                if (hits[id] != 2) return;
            for (int id = 0; id < g.numCells(); ++id)
                if (t.isHalo[id] && hits[id] == 2) return;
            bestCost = cost;
            best = choice;
            return;
        }
        for (int o : options[e]) {
            choice[e] = o;
            if (o == kOff) {
                dfs(e + 1, cost);
                continue;
            }
            bool overfull = false;
            for (int id : g.beamAt(e, o))
                if (++hits[id] > 2 && t.isP[id]) overfull = true;
            if (!overfull) dfs(e + 1, std::max(cost, std::abs(o) + 1));
            for (int id : g.beamAt(e, o)) --hits[id];
        }
    }
};

// Moves every used emitter to its offset while off, then switches them all on
// in one tick; every later noop tick re-creates the pattern.
Prog staticProgram(const std::array<int, 6>& cfg, int count) {
    int far = 0;
    for (int o : cfg)
        if (o != StaticSearch::kOff) far = std::max(far, std::abs(o));
    Prog prog(far + 1, kNoop);
    for (int e = 0; e < 6; ++e) {
        const int o = cfg[e];
        if (o == StaticSearch::kOff) continue;
        for (int k = 0; k < std::abs(o); ++k) prog[k][e] = o > 0 ? 'F' : 'B';
        prog[far][e] = 'T';
    }
    for (int k = 1; k < count; ++k) prog.push_back(kNoop);
    return prog;
}

// ---------------------------------------------------------------------------
// Strategy 2: a pen made of two emitters on different axes. While both are on,
// the cell where their beams cross gets matter. A BFS over (offsets, on/off)
// walks the pen to the nearest missing pattern cell without ever touching the
// halo; stray matter elsewhere is harmless.

Prog penProgram(const Geometry& g, const Target& t, int e1, int e2) {
    const int R = g.R, span = 2 * R + 1;
    std::vector<int> cross(span * span, -1);
    {
        std::vector<int> mark(g.numCells(), INT_MIN);
        for (int o1 = -R; o1 <= R; ++o1) {
            for (int id : g.beamAt(e1, o1)) mark[id] = o1;
            for (int o2 = -R; o2 <= R; ++o2)
                for (int id : g.beamAt(e2, o2))
                    if (mark[id] == o1) cross[(o1 + R) * span + (o2 + R)] = id;
        }
    }
    auto encode = [&](int o1, int o2, int a1, int a2) { return (((o1 + R) * span + (o2 + R)) * 2 + a1) * 2 + a2; };
    const int numStates = span * span * 4;
    auto stampOf = [&](int s) {
        if ((s & 3) != 3) return -1;
        return cross[s >> 2];
    };

    std::vector<char> matter(g.numCells(), 0);
    int missing = static_cast<int>(t.cells.size());
    int accepted = 0;
    int state = encode(0, 0, 0, 0);
    Prog prog;

    std::vector<int> dist(numStates), parent(numStates), via(numStates);
    while (accepted < t.count) {
        std::fill(dist.begin(), dist.end(), -1);
        std::vector<int> frontier = {state};
        dist[state] = 0;
        std::vector<int> goals;
        for (int depth = 0; goals.empty() && !frontier.empty(); ++depth) {
            std::vector<int> next;
            for (int s : frontier) {
                const int a2 = s & 1, a1 = (s >> 1) & 1;
                const int o2 = (s >> 2) % span - R, o1 = (s >> 2) / span - R;
                for (int m = 0; m < 16; ++m) {
                    int n1 = o1, n2 = o2, b1 = a1, b2 = a2;
                    const int i1 = m & 3, i2 = m >> 2;  // 0 noop, 1 F, 2 B, 3 T
                    if (i1 == 1) ++n1; else if (i1 == 2) --n1; else if (i1 == 3) b1 ^= 1;
                    if (i2 == 1) ++n2; else if (i2 == 2) --n2; else if (i2 == 3) b2 ^= 1;
                    if (std::abs(n1) > R || std::abs(n2) > R) continue;
                    const int ns = encode(n1, n2, b1, b2);
                    if (dist[ns] >= 0) continue;
                    const int stamp = stampOf(ns);
                    if (stamp >= 0 && t.isHalo[stamp]) continue;
                    dist[ns] = depth + 1;
                    parent[ns] = s;
                    via[ns] = m;
                    if (stamp >= 0 && t.isP[stamp] && !matter[stamp]) goals.push_back(ns);
                    else next.push_back(ns);
                }
            }
            frontier = std::move(next);
        }

        // Prefer pattern cells with few missing neighbours, so the pen does
        // not strand isolated cells behind it.
        int goal = goals.front(), bestScore = INT_MAX;
        for (int s : goals) {
            int score = 0;
            for (int n : g.neighbours[stampOf(s)]) score += t.isP[n] && !matter[n];
            if (score < bestScore) bestScore = score, goal = s;
        }

        std::vector<int> path;
        for (int s = goal; s != state; s = parent[s]) path.push_back(s);
        std::reverse(path.begin(), path.end());
        for (int s : path) {
            Line line = kNoop;
            const int m = via[s];
            const char codes[4] = {'-', 'F', 'B', 'T'};
            line[e1] = codes[m & 3];
            line[e2] = codes[m >> 2];
            prog.push_back(line);
            const int stamp = stampOf(s);
            if (stamp >= 0 && !matter[stamp]) {
                matter[stamp] = 1;
                if (t.isP[stamp]) --missing;
            }
            if (missing == 0) {
                ++accepted;
                for (int id : t.cells) matter[id] = 0;
                missing = static_cast<int>(t.cells.size());
            }
        }
        state = goal;
    }
    return prog;
}

// ---------------------------------------------------------------------------
// Strategy 3: roll + erase. Two emitters of one axis on the same line paint
// that whole line each tick; sweeping them across every line that holds
// pattern cells paints the pattern (and plenty of stray matter). Then an
// "eraser" — three emitters, one per axis, crossing in one cell — walks the
// non-pattern cells and destroys every painted halo cell.

// Incremental simulator that records the program as it is built.
struct Builder {
    const Geometry& g;
    const Target& t;
    std::array<int, 6> off{};
    std::array<bool, 6> on{};
    std::vector<char> matter;
    std::vector<int> hits;
    int accepted = 0;
    Prog prog;

    Builder(const Geometry& geo, const Target& tgt) : g(geo), t(tgt), matter(geo.numCells(), 0), hits(geo.numCells(), 0) {}

    bool done() const { return accepted >= t.count; }

    void step(const Line& line) {
        prog.push_back(line);
        for (int e = 0; e < 6; ++e) {
            if (line[e] == 'F') ++off[e];
            else if (line[e] == 'B') --off[e];
            else if (line[e] == 'T') on[e] = !on[e];
        }
        std::vector<int> touched;
        for (int e = 0; e < 6; ++e) {
            if (!on[e] || std::abs(off[e]) > g.R) continue;
            for (int id : g.beamAt(e, off[e]))
                if (hits[id]++ == 0) touched.push_back(id);
        }
        for (int id : touched) {
            if (hits[id] == 2) matter[id] = 1;
            else if (hits[id] >= 3) matter[id] = 0;
            hits[id] = 0;
        }
        bool present = true;
        for (int id : t.cells) present = present && matter[id];
        for (int id = 0; id < g.numCells() && present; ++id) present = !(t.isHalo[id] && matter[id]);
        if (present) {
            for (int id : t.cells) matter[id] = 0;
            ++accepted;
        }
    }
};

constexpr int kKeep = INT_MIN;

// Moves emitters to `target` offsets (kKeep = stay) and leaves exactly the
// `wantOn` ones active. Every active emitter except `carrier` is switched off
// in the first tick and everything that must end active is switched on
// together in the last tick, so in between at most the carrier is active and
// nothing gets stamped.
void transition(Builder& b, const std::array<int, 6>& target, const std::array<bool, 6>& wantOn, int carrier) {
    if (carrier >= 0 && (!b.on[carrier] || !wantOn[carrier])) carrier = -1;
    std::array<int, 6> delta{}, startMove{};
    std::array<bool, 6> offFirst{}, onLast{};
    int ticks = 0;
    for (int e = 0; e < 6; ++e) {
        delta[e] = target[e] == kKeep ? 0 : target[e] - b.off[e];
        offFirst[e] = b.on[e] && e != carrier;
        onLast[e] = wantOn[e] && e != carrier;
        startMove[e] = offFirst[e] ? 1 : 0;
        ticks = std::max(ticks, startMove[e] + std::abs(delta[e]) + (onLast[e] ? 1 : 0));
    }
    // An emitter that is already in place and on can simply stay on when it
    // is the only one active afterwards — handled by the carrier rule above.
    for (int k = 0; k < ticks; ++k) {
        Line line = kNoop;
        for (int e = 0; e < 6; ++e) {
            if (offFirst[e] && k == 0) line[e] = 'T';
            else if (onLast[e] && k == ticks - 1) line[e] = 'T';
            else if (k >= startMove[e] && k < startMove[e] + std::abs(delta[e])) line[e] = delta[e] > 0 ? 'F' : 'B';
        }
        b.step(line);
    }
}

Prog rollEraseProgram(const Geometry& g, const Target& t, int x, int u, int v) {
    const int f = x + 3;
    int lo = INT_MAX, hi = INT_MIN;
    for (int id : t.cells) {
        lo = std::min(lo, g.offFor[x][id]);
        hi = std::max(hi, g.offFor[x][id]);
    }
    auto partner = [&](int o) { return g.offFor[f][g.beamAt(x, o).front()]; };
    const int eraser[3] = {x, u, v};
    const int n = g.numCells();

    Builder b(g, t);
    std::vector<int> dist(2 * n), parent(2 * n);
    while (!b.done()) {
        // Roll across every line holding pattern cells.
        const int start = std::abs(b.off[x] - lo) <= std::abs(b.off[x] - hi) ? lo : hi;
        const int end = start == lo ? hi : lo;
        std::array<int, 6> target;
        target.fill(kKeep);
        target[x] = start;
        target[f] = partner(start);
        std::array<bool, 6> wantOn{};
        wantOn[x] = wantOn[f] = true;
        transition(b, target, wantOn, x);
        for (int o = start; o != end && !b.done();) {
            const int next = o + (end > o ? 1 : -1);
            Line line = kNoop;
            line[x] = next > o ? 'F' : 'B';
            line[f] = partner(next) > partner(o) ? 'F' : 'B';
            b.step(line);
            o = next;
        }
        if (b.done()) break;

        // Erase the painted halo.
        auto dirty = [&](int id) { return t.isHalo[id] && b.matter[id]; };
        int first = -1, firstCost = INT_MAX;
        for (int id = 0; id < n; ++id) {
            if (!dirty(id)) continue;
            int cost = std::abs(g.offFor[x][id] - b.off[x]);
            for (int e : {u, v}) cost = std::max(cost, std::abs(g.offFor[e][id] - b.off[e]) + 1);
            if (cost < firstCost) firstCost = cost, first = id;
        }
        if (first < 0) return {};  // nothing to erase, yet not accepted: give up
        target.fill(kKeep);
        wantOn.fill(false);
        for (int e : eraser) target[e] = g.offFor[e][first], wantOn[e] = true;
        transition(b, target, wantOn, x);

        // BFS over (cell, mode): mode 1 = eraser on (destroys where it walks,
        // may not enter pattern cells), mode 0 = two of its emitters off.
        int cur = first * 2 + 1;
        const int copiesBefore = b.accepted;
        while (b.accepted == copiesBefore) {
            std::fill(dist.begin(), dist.end(), -1);
            std::deque<int> queue = {cur};
            dist[cur] = 0;
            int goal = -1;
            while (!queue.empty() && goal < 0) {
                const int s = queue.front();
                queue.pop_front();
                const int cell = s / 2, mode = s % 2;
                auto push = [&](int ns) {
                    if (dist[ns] >= 0) return;
                    dist[ns] = dist[s] + 1;
                    parent[ns] = s;
                    if (ns % 2 == 1 && dirty(ns / 2) && goal < 0) goal = ns;
                    queue.push_back(ns);
                };
                if (mode == 1) push(cell * 2);                  // switch two emitters off
                else if (!t.isP[cell]) push(cell * 2 + 1);       // switch them back on here
                for (int nb : g.neighbours[cell]) {
                    if (mode == 1 && t.isP[nb]) continue;
                    push(nb * 2 + mode);
                }
            }
            if (goal < 0) return {};
            std::vector<int> path;
            for (int s = goal; s != cur; s = parent[s]) path.push_back(s);
            std::reverse(path.begin(), path.end());
            int prev = cur;
            for (int s : path) {
                Line line = kNoop;
                if (s / 2 == prev / 2) {
                    line[u] = line[v] = 'T';
                } else {
                    for (int e : eraser) {
                        const int d = g.offFor[e][s / 2] - g.offFor[e][prev / 2];
                        if (d != 0) line[e] = d > 0 ? 'F' : 'B';
                    }
                }
                b.step(line);
                prev = s;
                if (b.accepted != copiesBefore) break;
            }
            cur = goal;
        }
    }
    return b.prog;
}

// ---------------------------------------------------------------------------
// Strategy 4: greedy over all six emitters. Every tick tries all 4^6
// instruction lines, rejects those that put matter on the halo or destroy
// finished pattern cells, and takes the one stamping the most missing pattern
// cells; ties go to the line whose beams sit closest to missing cells.

constexpr int kWords = 7;  // up to 448 interior cells (radius 12 has 397)
struct Bits {
    std::array<u64, kWords> w{};
    void set(int i) { w[i >> 6] |= u64{1} << (i & 63); }
};

bool greedyRun(const Geometry& g, const Target& t, Builder& b, double gamma, double lambda, int stopAt, std::size_t limit) {
    const int R = g.R, n = g.numCells();
    if (n > kWords * 64) return false;
    std::array<std::vector<Bits>, 6> lineMask;
    for (int e = 0; e < 6; ++e) {
        lineMask[e].resize(2 * R + 1);
        for (int o = -R; o <= R; ++o)
            for (int id : g.beamAt(e, o)) lineMask[e][o + R].set(id);
    }
    Bits halo;
    for (int id = 0; id < n; ++id)
        if (t.isHalo[id]) halo.set(id);
    std::vector<double> decay(4 * R + 4);
    for (std::size_t d = 0; d < decay.size(); ++d) decay[d] = std::pow(gamma, static_cast<double>(d));

    int idle = 0;

    struct Option { char code; int off; bool on; double phi; };
    std::array<std::array<Option, 4>, 6> opts;
    std::array<int, 6> numOpts{};
    std::array<int, 6> pick{}, bestPick{};
    std::array<Bits, 7> ge1, ge2, ge3;

    while (b.accepted < stopAt) {
        if (b.prog.size() > limit) return false;
        Bits need, done;
        std::vector<int> needCells;
        for (int id : t.cells) {
            if (b.matter[id]) done.set(id);
            else need.set(id), needCells.push_back(id);
        }
        for (int e = 0; e < 6; ++e) {
            numOpts[e] = 0;
            const char codes[4] = {'-', 'F', 'B', 'T'};
            for (int k = 0; k < 4; ++k) {
                Option op{codes[k], b.off[e], b.on[e], 0.0};
                if (k == 1) ++op.off;
                if (k == 2) --op.off;
                if (k == 3) op.on = !op.on;
                if (std::abs(op.off) > R) continue;
                for (int id : needCells) {
                    const int d = std::abs(op.off - g.offFor[e][id]) + (op.on ? 0 : 1);
                    op.phi += decay[d];
                }
                opts[e][numOpts[e]++] = op;
            }
        }

        int bestGain = -1;
        double bestPhi = -1, bestVal = -1e18;
        const double phiScale = lambda / (6.0 * std::max<std::size_t>(1, needCells.size()));
        std::function<void(int, double)> dfs = [&](int e, double phi) {
            if (e == 6) {
                bool bad = false;
                int gain = 0;
                for (int w = 0; w < kWords; ++w) {
                    const u64 e2 = ge2[6].w[w] & ~ge3[6].w[w];
                    if ((e2 & halo.w[w]) || (ge3[6].w[w] & done.w[w])) bad = true;
                    gain += std::popcount(e2 & need.w[w]);
                }
                if (bad) return;
                const double val = gain + phi * phiScale;
                if (val > bestVal) bestVal = val, bestGain = gain, bestPhi = phi, bestPick = pick;
                return;
            }
            for (int k = 0; k < numOpts[e]; ++k) {
                const Option& op = opts[e][k];
                pick[e] = k;
                ge1[e + 1] = ge1[e], ge2[e + 1] = ge2[e], ge3[e + 1] = ge3[e];
                if (op.on) {
                    const Bits& L = lineMask[e][op.off + R];
                    for (int w = 0; w < kWords; ++w) {
                        ge3[e + 1].w[w] |= ge2[e].w[w] & L.w[w];
                        ge2[e + 1].w[w] |= ge1[e].w[w] & L.w[w];
                        ge1[e + 1].w[w] |= L.w[w];
                    }
                }
                dfs(e + 1, phi + op.phi);
            }
        };
        dfs(0, 0.0);

        if (bestGain <= 0) ++idle; else idle = 0;
        if (bestGain < 0 || idle > 2 * R) {
            // Stuck: switch everything off, then stamp the nearest missing
            // cell with the cheapest pair of emitters.
            int bestCell = -1, bestA = -1, bestB = -1, bestCost = INT_MAX;
            for (int id : needCells)
                for (int a = 0; a < 6; ++a)
                    for (int c = a + 1; c < 6; ++c) {
                        if (axisOf(a) == axisOf(c)) continue;
                        const int cost = std::max(std::abs(b.off[a] - g.offFor[a][id]), std::abs(b.off[c] - g.offFor[c][id]));
                        if (cost < bestCost) bestCost = cost, bestCell = id, bestA = a, bestB = c;
                    }
            std::array<int, 6> target;
            target.fill(kKeep);
            target[bestA] = g.offFor[bestA][bestCell];
            target[bestB] = g.offFor[bestB][bestCell];
            std::array<bool, 6> wantOn{};
            wantOn[bestA] = wantOn[bestB] = true;
            transition(b, target, wantOn, -1);
            idle = 0;
            continue;
        }
        Line line = kNoop;
        for (int e = 0; e < 6; ++e) line[e] = opts[e][bestPick[e]].code;
        b.step(line);
    }
    return true;
}

Prog greedyProgram(const Geometry& g, const Target& t, double gamma, double lambda) {
    Builder b(g, t);
    const std::size_t limit = 40 * t.cells.size() * static_cast<std::size_t>(t.count) + 200;
    if (!greedyRun(g, t, b, gamma, lambda, t.count, limit)) return {};
    return b.prog;
}

// ---------------------------------------------------------------------------
// Strategy 5: anchor sweep. The two emitters of one axis are put on the same
// line ("anchor"): that whole line is painted every tick. The anchor walks
// across every line holding pattern cells, one line per tick, while the four
// other emitters ("cutters") are steered by a beam search: they cut the halo
// cells on the anchor line (a third beam empties a cell) and stamp more
// pattern cells where they cross each other. Halo cells may never get matter.
// Whatever the sweep leaves missing is finished by the greedy.

struct AState {
    std::array<int8_t, 6> off;
    uint8_t on;
    int8_t aligned;   // anchor pair in place on one line
    int8_t row;       // anchor (emitter x) offset when aligned
    Bits matter;
    int parent;
    Line line;
    int done;
    double score;
    int acc = 0;
};

inline u64 hashState(const AState& s) {
    u64 h = 1469598103934665603ULL;
    auto mix = [&](u64 v) { h ^= v; h *= 1099511628211ULL; h ^= h >> 29; };
    for (int e = 0; e < 6; ++e) mix(static_cast<u64>(s.off[e] + 64));
    mix(s.on);
    mix(static_cast<u64>(s.acc) + 7777);
    mix(static_cast<u64>(s.aligned) * 131 + static_cast<u64>(s.row + 64));
    for (int w = 0; w < kWords; ++w) mix(s.matter.w[w]);
    return h;
}

struct AnchorCtx {
    const Geometry& g;
    const Target& t;
    int R, n;
    std::array<std::vector<Bits>, 6> lineMask;
    Bits halo, pat;
    AnchorCtx(const Geometry& geo, const Target& tgt) : g(geo), t(tgt), R(geo.R), n(geo.numCells()) {
        for (int e = 0; e < 6; ++e) {
            lineMask[e].resize(2 * R + 1);
            for (int o = -R; o <= R; ++o)
                for (int id : g.beamAt(e, o)) lineMask[e][o + R].set(id);
        }
        for (int id = 0; id < n; ++id) {
            if (t.isHalo[id]) halo.set(id);
            if (t.isP[id]) pat.set(id);
        }
    }
};

// Runs one sweep from the builder's state, appending its lines to `b`.
// Returns false if no valid sweep was found (b unchanged then).
bool anchorSweep(const AnchorCtx& C, Builder& b, int x, int dirPref, int K, int lookRows) {
    const Geometry& g = C.g;
    const Target& t = C.t;
    const int R = C.R, f = x + 3;
    int lo = INT_MAX, hi = INT_MIN;
    for (int id : t.cells) lo = std::min(lo, g.offFor[x][id]), hi = std::max(hi, g.offFor[x][id]);
    auto partner = [&](int o) { return g.offFor[f][g.beamAt(x, o).front()]; };
    int dir = dirPref;
    if (dir == 0) dir = std::abs(b.off[x] - lo) <= std::abs(b.off[x] - hi) ? 1 : -1;
    const int start = dir > 0 ? lo : hi, end = dir > 0 ? hi : lo;
    const int tgtX = start, tgtF = partner(start);
    std::array<int, 4> cut;
    {
        int k = 0;
        for (int e = 0; e < 6; ++e)
            if (e != x && e != f) cut[k++] = e;
    }
    // Halo cells per anchor row (for the cutter potential).
    std::vector<std::vector<int>> haloRow(2 * R + 1);
    for (int id = 0; id < C.n; ++id)
        if (t.isHalo[id]) haloRow[g.offFor[x][id] + R].push_back(id);
    const int P = static_cast<int>(t.cells.size());

    AState s0{};
    for (int e = 0; e < 6; ++e) s0.off[e] = static_cast<int8_t>(b.off[e]), s0.on |= b.on[e] ? (1 << e) : 0;
    for (int id = 0; id < C.n; ++id)
        if (b.matter[id]) s0.matter.set(id);
    for (int e = 0; e < 6; ++e)
        if (std::abs(b.off[e]) > R) return false;
    s0.aligned = (!(s0.on >> x & 1) && !(s0.on >> f & 1) && s0.off[x] == tgtX && s0.off[f] == tgtF);
    s0.row = s0.aligned ? static_cast<int8_t>(tgtX) : 0;
    s0.parent = -1;
    s0.line = kNoop;
    s0.done = 0;
    for (int w = 0; w < kWords; ++w) s0.done += std::popcount(s0.matter.w[w] & C.pat.w[w]);

    const int approach = std::max(std::abs(b.off[x] - tgtX) + (b.on[x] ? 1 : 0), std::abs(b.off[f] - tgtF) + (b.on[f] ? 1 : 0));
    const int rows = std::abs(end - start) + 1;
    const int cap = approach + rows + rows / 2 + 6;

    std::vector<std::vector<AState>> gens;
    gens.push_back({s0});
    std::vector<AState> kids;
    std::unordered_set<u64> seen;
    int finalIdx = -1;
    const char codes[4] = {'-', 'F', 'B', 'T'};

    for (int tick = 0; tick < cap && finalIdx < 0; ++tick) {
        const auto& cur = gens.back();
        bool allFinished = true;
        for (const auto& s : cur)
            if (!(s.aligned && s.row == end)) allFinished = false;
        if (allFinished && tick > 0) break;
        kids.clear();
        for (int si = 0; si < static_cast<int>(cur.size()) && finalIdx < 0; ++si) {
            const AState& s = cur[si];
            // Anchor options: (line chars for x and f, new offsets, new on bits, aligned, row)
            struct AOpt { char cx, cf; int ox, of; bool onx, onf; int8_t al, row; };
            std::array<AOpt, 3> aopts;
            int na = 0;
            const bool onx = s.on >> x & 1, onf = s.on >> f & 1;
            if (!s.aligned) {
                AOpt a{'-', '-', s.off[x], s.off[f], onx, onf, 0, 0};
                if (onx) a.cx = 'T', a.onx = false;
                else if (s.off[x] != tgtX) a.cx = s.off[x] < tgtX ? 'F' : 'B', a.ox += s.off[x] < tgtX ? 1 : -1;
                if (onf) a.cf = 'T', a.onf = false;
                else if (s.off[f] != tgtF) a.cf = s.off[f] < tgtF ? 'F' : 'B', a.of += s.off[f] < tgtF ? 1 : -1;
                if (!a.onx && !a.onf && a.ox == tgtX && a.of == tgtF) a.al = 1, a.row = static_cast<int8_t>(tgtX);
                aopts[na++] = a;
            } else {
                aopts[na++] = {'-', '-', s.off[x], s.off[f], onx, onf, 1, s.row};
                aopts[na++] = {'T', 'T', s.off[x], s.off[f], !onx, !onf, 1, s.row};
                if (s.row != end) {
                    const int nr = s.row + dir, np = partner(nr);
                    const int dpf = np - s.off[f];
                    aopts[na++] = {dir > 0 ? 'F' : 'B', dpf > 0 ? 'F' : 'B', nr, np, onx, onf, 1, static_cast<int8_t>(nr)};
                }
            }
            for (int ai = 0; ai < na && finalIdx < 0; ++ai) {
                const AOpt& a = aopts[ai];
                Bits g1, g2, g3;
                if (a.onx) g1 = C.lineMask[x][a.ox + R];
                if (a.onf) {
                    const Bits& L = C.lineMask[f][a.of + R];
                    for (int w = 0; w < kWords; ++w) g2.w[w] = g1.w[w] & L.w[w], g1.w[w] |= L.w[w];
                }
                // Enumerate cutter options.
                struct COpt { int off; bool on; int code; const Bits* L; };
                std::array<std::array<COpt, 4>, 4> co;
                std::array<int, 4> nco{};
                for (int q = 0; q < 4; ++q) {
                    const int e = cut[q];
                    for (int m = 0; m < 4; ++m) {
                        int o = s.off[e];
                        bool on = s.on >> e & 1;
                        if (m == 1) ++o;
                        else if (m == 2) --o;
                        else if (m == 3) on = !on;
                        if (std::abs(o) > R) continue;
                        co[q][nco[q]++] = {o, on, m, on ? &C.lineMask[e][o + R] : nullptr};
                    }
                }
                std::array<Bits, 5> G1, G2, G3;
                G1[0] = g1, G2[0] = g2, G3[0] = g3;
                auto add = [&](int k, const COpt& op) {
                    if (!op.L) { G1[k + 1] = G1[k], G2[k + 1] = G2[k], G3[k + 1] = G3[k]; return; }
                    const Bits& L = *op.L;
                    for (int w = 0; w < kWords; ++w) {
                        G3[k + 1].w[w] = G3[k].w[w] | (G2[k].w[w] & L.w[w]);
                        G2[k + 1].w[w] = G2[k].w[w] | (G1[k].w[w] & L.w[w]);
                        G1[k + 1].w[w] = G1[k].w[w] | L.w[w];
                    }
                };
                std::array<int, 4> pk{};
                for (pk[0] = 0; pk[0] < nco[0] && finalIdx < 0; ++pk[0]) {
                    add(0, co[0][pk[0]]);
                    for (pk[1] = 0; pk[1] < nco[1] && finalIdx < 0; ++pk[1]) {
                        add(1, co[1][pk[1]]);
                        for (pk[2] = 0; pk[2] < nco[2] && finalIdx < 0; ++pk[2]) {
                            add(2, co[2][pk[2]]);
                            for (pk[3] = 0; pk[3] < nco[3] && finalIdx < 0; ++pk[3]) {
                                add(3, co[3][pk[3]]);
                                AState c;
                                bool bad = false;
                                int done = 0;
                                for (int w = 0; w < kWords; ++w) {
                                    const u64 e2 = G2[4].w[w] & ~G3[4].w[w];
                                    if (e2 & C.halo.w[w]) { bad = true; break; }
                                    c.matter.w[w] = (s.matter.w[w] & ~G3[4].w[w]) | e2;
                                    done += std::popcount(c.matter.w[w] & C.pat.w[w]);
                                }
                                if (bad) continue;
                                c.off = s.off;
                                c.off[x] = static_cast<int8_t>(a.ox), c.off[f] = static_cast<int8_t>(a.of);
                                c.on = static_cast<uint8_t>((s.on & ~((1 << x) | (1 << f))) | (a.onx ? 1 << x : 0) | (a.onf ? 1 << f : 0));
                                c.line = kNoop;
                                c.line[x] = a.cx, c.line[f] = a.cf;
                                for (int q = 0; q < 4; ++q) {
                                    const int e = cut[q];
                                    const COpt& op = co[q][pk[q]];
                                    c.off[e] = static_cast<int8_t>(op.off);
                                    c.on = static_cast<uint8_t>((c.on & ~(1 << e)) | (op.on ? 1 << e : 0));
                                    c.line[e] = codes[op.code];
                                }
                                c.aligned = a.al, c.row = a.row;
                                c.parent = si;
                                c.done = done;
                                c.score = done * 1000.0;
                                if (done == P) {
                                    kids.clear();
                                    kids.push_back(c);
                                    finalIdx = 0;
                                    break;
                                }
                                kids.push_back(c);
                            }
                        }
                    }
                }
            }
        }
        if (kids.empty()) return false;
        if (finalIdx >= 0) {
            gens.push_back(kids);
            break;
        }
        {
            // Potential only for the kids that can make the cut.
            std::vector<int> cnt(P + 2, 0);
            for (const auto& c : kids) ++cnt[c.done];
            int thr = P + 1, acc = 0;
            while (thr > 0 && acc + cnt[thr] < K) acc += cnt[thr], --thr;
            for (auto& c : kids) {
                if (c.done < thr) continue;
                double pot = 0;
                if (c.aligned) {
                    for (int r = 1; r <= lookRows; ++r) {
                        const int rr = c.row + dir * r;
                        if ((rr - end) * dir > 0) break;
                        for (int h : haloRow[rr + R]) {
                            int bestD = 99;
                            for (int q = 0; q < 4; ++q) {
                                const int e = cut[q];
                                const int d = std::abs(c.off[e] - g.offFor[e][h]) + ((c.on >> e & 1) ? 0 : 1);
                                bestD = std::min(bestD, std::max(0, d - r + 1));
                            }
                            pot += std::min(bestD, 6);
                        }
                    }
                }
                c.score -= pot;
            }
        }
        std::vector<int> order;
        {
            int thr2 = P + 1, acc2 = 0;
            std::vector<int> cnt(P + 2, 0);
            for (const auto& c : kids) ++cnt[c.done];
            while (thr2 > 0 && acc2 < 4 * K) acc2 += cnt[thr2], --thr2;
            for (int i = 0; i < static_cast<int>(kids.size()); ++i)
                if (kids[i].done >= thr2) order.push_back(i);
        }
        std::sort(order.begin(), order.end(), [&](int a, int c) { return kids[a].score > kids[c].score; });
        seen.clear();
        std::vector<AState> next;
        for (int i : order) {
            const auto& c = kids[i];
            if (static_cast<int>(next.size()) >= K) break;
            if (!seen.insert(hashState(c)).second) continue;
            next.push_back(c);
        }
        gens.push_back(std::move(next));
    }
    // Pick the best final state.
    const auto& last = gens.back();
    int bi = 0;
    if (finalIdx >= 0) bi = finalIdx;
    else
        for (int i = 1; i < static_cast<int>(last.size()); ++i)
            if (last[i].score > last[bi].score) bi = i;
    std::vector<Line> path;
    for (int gi = static_cast<int>(gens.size()) - 1, idx = bi; gi > 0; --gi) {
        path.push_back(gens[gi][idx].line);
        idx = gens[gi][idx].parent;
    }
    std::reverse(path.begin(), path.end());
    if (path.empty()) return false;
    const int acc = b.accepted;
    for (const auto& l : path) {
        b.step(l);
        if (b.accepted != acc) break;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Beam version of the greedy: keeps the K best states per tick (pattern cells
// held + proximity potential), each expanded with all 4^6 lines.

struct GCand { double score; int parent; uint16_t code; };

// Runs until one more copy is accepted (returns true) or it stalls (returns
// false after applying the best partial path).
bool beamGreedyRun(const AnchorCtx& C, Builder& b, double gamma, double lambda, int K, int cap, int copies = 1) {
    const Geometry& g = C.g;
    const Target& t = C.t;
    const int R = C.R;
    const int P = static_cast<int>(t.cells.size());
    for (int e = 0; e < 6; ++e)
        if (std::abs(b.off[e]) > R) return false;
    std::vector<double> decay(4 * R + 4);
    for (std::size_t d = 0; d < decay.size(); ++d) decay[d] = std::pow(gamma, static_cast<double>(d));

    AState s0{};
    for (int e = 0; e < 6; ++e) s0.off[e] = static_cast<int8_t>(b.off[e]), s0.on |= b.on[e] ? (1 << e) : 0;
    for (int id = 0; id < C.n; ++id)
        if (b.matter[id]) s0.matter.set(id);
    s0.parent = -1;
    s0.line = kNoop;
    for (int w = 0; w < kWords; ++w) s0.done += std::popcount(s0.matter.w[w] & C.pat.w[w]);

    std::vector<std::vector<AState>> gens;
    gens.push_back({s0});
    std::vector<GCand> cands;
    std::unordered_set<u64> seen;
    const char codes[4] = {'-', 'F', 'B', 'T'};
    int finalParent = -1, finalCode = -1;
    double finalScore = -1;
    // Potential of each emitter option w.r.t. the whole pattern (for the next copy).
    std::array<std::vector<double>, 6> fullPhi;
    for (int e = 0; e < 6; ++e) {
        fullPhi[e].assign((2 * R + 1) * 2, 0.0);
        for (int o = -R; o <= R; ++o)
            for (int on = 0; on < 2; ++on)
                for (int id : t.cells) fullPhi[e][(o + R) * 2 + on] += decay[std::abs(o - g.offFor[e][id]) + (on ? 0 : 1)];
    }
    int bestDone = s0.done, sinceBest = 0;
    const int stride = P + 1;

    struct Option { int code; int off; bool on; double phi; const Bits* L; };
    for (int tick = 0; tick < cap && finalParent < 0; ++tick) {
        const auto& cur = gens.back();
        cands.clear();
        for (int si = 0; si < static_cast<int>(cur.size()); ++si) {
            const AState& s = cur[si];
            std::vector<int> needCells;
            for (int id : t.cells)
                if (!(s.matter.w[id >> 6] >> (id & 63) & 1)) needCells.push_back(id);
            std::array<std::array<Option, 4>, 6> opts;
            std::array<int, 6> numOpts{};
            for (int e = 0; e < 6; ++e) {
                for (int k = 0; k < 4; ++k) {
                    Option op{k, s.off[e], static_cast<bool>(s.on >> e & 1), 0.0, nullptr};
                    if (k == 1) ++op.off;
                    if (k == 2) --op.off;
                    if (k == 3) op.on = !op.on;
                    if (std::abs(op.off) > R) continue;
                    for (int id : needCells) {
                        const int d = std::abs(op.off - g.offFor[e][id]) + (op.on ? 0 : 1);
                        op.phi += decay[d];
                    }
                    if (op.on) op.L = &C.lineMask[e][op.off + R];
                    opts[e][numOpts[e]++] = op;
                }
            }
            const double phiScale = lambda / (6.0 * std::max<std::size_t>(1, needCells.size()));
            std::array<Bits, 7> G1, G2, G3;
            std::array<int, 6> pk{};
            std::array<double, 7> phi{};
            // iterative DFS over 6 emitters
            int e = 0;
            pk[0] = -1;
            while (e >= 0) {
                if (++pk[e] >= numOpts[e]) { --e; continue; }
                const Option& op = opts[e][pk[e]];
                phi[e + 1] = phi[e] + op.phi;
                if (!op.L) G1[e + 1] = G1[e], G2[e + 1] = G2[e], G3[e + 1] = G3[e];
                else {
                    const Bits& L = *op.L;
                    for (int w = 0; w < kWords; ++w) {
                        G3[e + 1].w[w] = G3[e].w[w] | (G2[e].w[w] & L.w[w]);
                        G2[e + 1].w[w] = G2[e].w[w] | (G1[e].w[w] & L.w[w]);
                        G1[e + 1].w[w] = G1[e].w[w] | L.w[w];
                    }
                }
                if (e < 5) { ++e; pk[e] = -1; continue; }
                bool bad = false;
                int done = 0;
                for (int w = 0; w < kWords; ++w) {
                    const u64 e2 = G2[6].w[w] & ~G3[6].w[w];
                    if (e2 & C.halo.w[w]) { bad = true; break; }
                    done += std::popcount(((s.matter.w[w] & ~G3[6].w[w]) | e2) & C.pat.w[w]);
                }
                if (bad) continue;
                uint16_t code = 0;
                for (int q = 0; q < 6; ++q) code |= static_cast<uint16_t>(opts[q][pk[q]].code << (2 * q));
                if (done == P && s.acc + 1 >= copies) {
                    double fs = 0;
                    for (int q = 0; q < 6; ++q) {
                        const Option& o2 = opts[q][pk[q]];
                        fs += fullPhi[q][(o2.off + R) * 2 + (o2.on ? 1 : 0)];
                    }
                    if (fs > finalScore) finalScore = fs, finalParent = si, finalCode = code;
                    continue;
                }
                const int prog = done == P ? (s.acc + 1) * stride : s.acc * stride + done;
                cands.push_back({prog + phi[6] * phiScale, si, code});
            }
        }
        auto materialize = [&](int parent, int code) {
            const AState& s = cur[parent];
            AState c{};
            c.off = s.off;
            c.on = s.on;
            c.parent = parent;
            c.line = kNoop;
            Bits g1, g2, g3;
            for (int e2 = 0; e2 < 6; ++e2) {
                const int k = code >> (2 * e2) & 3;
                c.line[e2] = codes[k];
                if (k == 1) ++c.off[e2];
                if (k == 2) --c.off[e2];
                if (k == 3) c.on ^= static_cast<uint8_t>(1 << e2);
                if (c.on >> e2 & 1) {
                    const Bits& L = C.lineMask[e2][c.off[e2] + R];
                    for (int w = 0; w < kWords; ++w) g3.w[w] |= g2.w[w] & L.w[w], g2.w[w] |= g1.w[w] & L.w[w], g1.w[w] |= L.w[w];
                }
            }
            c.done = 0;
            for (int w = 0; w < kWords; ++w) {
                c.matter.w[w] = ((s.matter.w[w] & ~g3.w[w]) | (g2.w[w] & ~g3.w[w])) & C.pat.w[w];
                c.done += std::popcount(c.matter.w[w] & C.pat.w[w]);
            }
            c.acc = s.acc;
            if (c.done == P) {
                ++c.acc;
                c.done = 0;
                c.matter = Bits{};
            }
            return c;
        };
        if (finalParent >= 0) {
            gens.push_back({materialize(finalParent, finalCode)});
            break;
        }
        if (cands.empty()) break;
        static const int takeF = std::getenv("L6_TAKE") ? std::atoi(std::getenv("L6_TAKE")) : 8;
        const std::size_t take = std::min<std::size_t>(cands.size(), static_cast<std::size_t>(takeF * K));
        std::nth_element(cands.begin(), cands.begin() + (take - 1), cands.end(), [](const GCand& a, const GCand& c) { return a.score > c.score; });
        std::sort(cands.begin(), cands.begin() + take, [](const GCand& a, const GCand& c) { return a.score > c.score; });
        seen.clear();
        std::vector<AState> next;
        static const double pairW = std::getenv("L6_PAIR") ? std::atof(std::getenv("L6_PAIR")) : 2.0;
        if (pairW > 0) {
            std::vector<AState> pool;
            pool.reserve(take);
            for (std::size_t i = 0; i < take; ++i) {
                AState c = materialize(cands[i].parent, cands[i].code);
                c.score = cands[i].score;
                // Pair potential: for every missing cell, how soon two beams of
                // different axes can cross there.
                double pp = 0;
                int nNeed = 0;
                for (int id : t.cells) {
                    if (c.matter.w[id >> 6] >> (id & 63) & 1) continue;
                    ++nNeed;
                    int m[3];
                    for (int a = 0; a < 3; ++a) {
                        const int d1 = std::abs(c.off[a] - g.offFor[a][id]) + ((c.on >> a & 1) ? 0 : 1);
                        const int d2 = std::abs(c.off[a + 3] - g.offFor[a + 3][id]) + ((c.on >> (a + 3) & 1) ? 0 : 1);
                        m[a] = std::min(d1, d2);
                    }
                    std::sort(m, m + 3);
                    pp += decay[m[1]];
                }
                c.score += pairW * pp / std::max(1, nNeed);
                pool.push_back(c);
            }
            std::stable_sort(pool.begin(), pool.end(), [](const AState& a, const AState& c) { return a.score > c.score; });
            static const int perParent = std::getenv("L6_PP") ? std::atoi(std::getenv("L6_PP")) : 1000;
            std::vector<int> used(cur.size(), 0);
            for (auto& c : pool) {
                if (static_cast<int>(next.size()) >= K) break;
                if (used[c.parent] >= perParent) continue;
                if (!seen.insert(hashState(c)).second) continue;
                ++used[c.parent];
                next.push_back(c);
            }
        } else
        for (std::size_t i = 0; i < take && static_cast<int>(next.size()) < K; ++i) {
            AState c = materialize(cands[i].parent, cands[i].code);
            c.score = cands[i].score;
            if (!seen.insert(hashState(c)).second) continue;
            next.push_back(c);
        }
        if (next[0].acc * stride + next[0].done > bestDone) bestDone = next[0].acc * stride + next[0].done, sinceBest = 0;
        else if (++sinceBest > 2 * R + 2) { gens.push_back(std::move(next)); break; }
        gens.push_back(std::move(next));
    }
    std::vector<Line> path;
    for (int gi = static_cast<int>(gens.size()) - 1, idx = 0; gi > 0; --gi) {
        path.push_back(gens[gi][idx].line);
        idx = gens[gi][idx].parent;
    }
    std::reverse(path.begin(), path.end());
    const int acc = b.accepted;
    for (const auto& l : path) {
        b.step(l);
        if (b.accepted >= acc + copies) return true;
    }
    return false;
}

Prog beamGreedyProgram(const Geometry& g, const Target& t, double gamma, double lambda, int K, bool multi = false) {
    if (g.numCells() > kWords * 64) return {};
    AnchorCtx C(g, t);
    Builder b(g, t);
    const std::size_t limit = 40 * t.cells.size() * static_cast<std::size_t>(t.count) + 200;
    while (!b.done()) {
        if (b.prog.size() > limit) return {};
        const int acc = b.accepted;
        if (!beamGreedyRun(C, b, gamma, lambda, K, 100000, multi ? t.count - b.accepted : 1) && b.accepted == acc)
            if (!greedyRun(g, t, b, gamma, lambda, acc + 1, limit)) return {};
    }
    return b.prog;
}

Prog anchorProgram(const Geometry& g, const Target& t, int x, int dir0, int K, int lookRows, double gamma, double lambda, int finishK) {
    const int n = g.numCells();
    if (n > kWords * 64) return {};
    AnchorCtx C(g, t);
    Builder b(g, t);
    const std::size_t limit = 40 * t.cells.size() * static_cast<std::size_t>(t.count) + 200;
    bool first = true;
    while (!b.done()) {
        if (b.prog.size() > limit) return {};
        const int acc = b.accepted;
        const std::size_t t0 = b.prog.size();
        anchorSweep(C, b, x, first ? dir0 : 0, K, lookRows);
        const std::size_t t1 = b.prog.size();
        int have = 0;
        for (int id : t.cells) have += b.matter[id];
        first = false;
        if (b.accepted == acc && finishK > 0) beamGreedyRun(C, b, gamma, lambda, finishK, 400);
        if (b.accepted == acc)
            if (!greedyRun(g, t, b, gamma, lambda, acc + 1, limit)) return {};
        static const bool adbg = std::getenv("L6_ADEBUG") != nullptr;
        if (adbg) std::cerr << "[x" << x << " sweep " << (t1 - t0) << " have " << (b.accepted != acc && t1 == b.prog.size() ? (int)t.cells.size() : have) << "/" << t.cells.size() << " greedy " << (b.prog.size() - t1) << "] ";
    }
    return b.prog;
}

// ---------------------------------------------------------------------------
// Strategy 6: a period-2 cycle. Every emitter alternates between two states
// (A, B) that differ by one instruction (move or toggle). After the setup each
// copy costs two lines.

struct Period2Search {
    const Geometry& g;
    const Target& t;
    struct Opt { int oA; bool onA; int oB; bool onB; int cost; };
    std::vector<std::vector<Opt>> options;
    std::vector<int> hA, hB;
    std::array<Opt, 6> choice{}, best{};
    int bestCost = INT_MAX;
    i64 nodes = 0;
    i64 nodeLimit;

    Period2Search(const Geometry& geo, const Target& tgt, i64 limit) : g(geo), t(tgt), nodeLimit(limit) {
        hA.assign(g.numCells(), 0);
        hB.assign(g.numCells(), 0);
        for (int e = 0; e < 6; ++e) {
            auto& opts = options.emplace_back();
            opts.push_back({0, false, 0, false, 0});
            std::vector<int> useful;
            for (int o = -(g.R - 1); o <= g.R - 1; ++o) {
                bool touches = false;
                for (int id : g.beamAt(e, o)) touches = touches || t.isP[id] || t.isHalo[id];
                if (touches) useful.push_back(o);
            }
            std::sort(useful.begin(), useful.end(), [](int a, int b) { return std::abs(a) < std::abs(b); });
            for (int o : useful) {
                opts.push_back({o, true, o, true, std::abs(o) + 1});
                opts.push_back({o, true, o, false, std::abs(o) + 1});
                opts.push_back({o, false, o, true, std::abs(o) + 1});
                for (int d : {1, -1})
                    if (std::abs(o + d) <= g.R) opts.push_back({o, true, o + d, true, std::abs(o) + 1});
            }
        }
    }

    void run() { dfs(0, 0); }

    void apply(int e, const Opt& op, int s) {
        if (op.onA) for (int id : g.beamAt(e, op.oA)) hA[id] += s;
        if (op.onB) for (int id : g.beamAt(e, op.oB)) hB[id] += s;
    }

    void dfs(int e, int cost) {
        if (++nodes > nodeLimit || cost >= bestCost) return;
        if (e == 6) {
            for (int id = 0; id < g.numCells(); ++id)
                if (t.isHalo[id] && (hA[id] == 2 || hB[id] == 2)) return;
            bool okB = true, okA = true;
            for (int id : t.cells) {
                okB = okB && ((hA[id] == 2 && hB[id] < 3) || hB[id] == 2);
                okA = okA && ((hB[id] == 2 && hA[id] < 3) || hA[id] == 2);
            }
            if (!okA && !okB) return;
            bestCost = cost;
            best = choice;
            return;
        }
        for (const Opt& op : options[e]) {
            choice[e] = op;
            apply(e, op, 1);
            bool dead = false;
            for (int id : t.cells)
                if (hA[id] >= 3 && hB[id] >= 3) { dead = true; break; }
            if (!dead) dfs(e + 1, std::max(cost, op.cost));
            apply(e, op, -1);
        }
    }
};

Prog period2Program(const Geometry& g, const Target& t, const std::array<Period2Search::Opt, 6>& cfg) {
    int far = 0;
    for (const auto& op : cfg)
        if (op.onA || op.onB) far = std::max(far, std::abs(op.oA));
    Prog prog(far + 1, kNoop);
    for (int e = 0; e < 6; ++e) {
        const auto& op = cfg[e];
        if (!op.onA && !op.onB) continue;
        for (int k = 0; k < std::abs(op.oA); ++k) prog[k][e] = op.oA > 0 ? 'F' : 'B';
        if (op.onA) prog[far][e] = 'T';
    }
    // Now in state A. Alternate B, A, B, ...
    for (int k = 0; k < 2 * t.count + 2; ++k) {
        Line line = kNoop;
        const bool toB = (k % 2 == 0);
        for (int e = 0; e < 6; ++e) {
            const auto& op = cfg[e];
            if (!op.onA && !op.onB) continue;
            if (op.onA != op.onB) line[e] = 'T';
            else if (op.oA != op.oB) line[e] = ((op.oB > op.oA) == toB) ? 'F' : 'B';
        }
        prog.push_back(line);
    }
    return prog;
}

Prog solvePattern(const Pattern& p) {
    const Geometry g(p.radius);
    const Target t(g, p);
    Prog best;
    static const bool debug = std::getenv("L6_DEBUG") != nullptr;
    std::string label;
    auto consider = [&](Prog prog) {
        const int used = simulate(g, t, prog);
        if (debug) std::cerr << label << "=" << used << " ";
        if (used < 0) return;
        prog.resize(used);
        if (best.empty() || prog.size() < best.size()) best = std::move(prog);
    };

    static const bool greedyOnly = std::getenv("L6_GREEDY_ONLY") != nullptr || std::getenv("L6_PENROLL") == nullptr;
    StaticSearch search(g, t);
    search.run();
    if (debug) std::cerr << "R" << g.R << " P" << t.cells.size() << " cnt" << t.count << " | ";
    label = "static";
    if (search.bestCost != INT_MAX) consider(staticProgram(search.best, t.count));
    static const i64 p2Limit = std::getenv("L6_P2") ? std::atoll(std::getenv("L6_P2")) : 5'000'000;
    if (t.count >= 3 && p2Limit > 0) {
        Period2Search p2(g, t, p2Limit);
        p2.run();
        label = "period2";
        if (p2.bestCost != INT_MAX) consider(period2Program(g, t, p2.best));
    }

    for (int e1 = 0; e1 < 6 && !greedyOnly; ++e1)
        for (int e2 = e1 + 1; e2 < 6; ++e2)
            if (axisOf(e1) != axisOf(e2)) label = "pen", consider(penProgram(g, t, e1, e2));

    static const std::vector<std::pair<double, double>> params = [] {
        std::vector<std::pair<double, double>> v;
        if (const char* env = std::getenv("L6_PARAMS")) {
            std::istringstream ss(env);
            double a, b2;
            while (ss >> a >> b2) v.push_back({a, b2});
        } else {
            v = {{0.5, 0.9}, {0.25, 0.9}, {0.7, 0.9}, {0.5, 0.5}};
        }
        return v;
    }();
    for (const auto& [gamma, lambda] : params) {
        label = "greedy";
        Prog prog = greedyProgram(g, t, gamma, lambda);
        if (!prog.empty()) consider(std::move(prog));
    }

    static const int beamK = std::getenv("L6_BK") ? std::atoi(std::getenv("L6_BK")) : 16;
    static const bool noBeam = std::getenv("L6_NO_BEAM") != nullptr;
    for (const auto& [gamma, lambda] : params) {
        if (noBeam) break;
        label = "beam";
        Prog prog = beamGreedyProgram(g, t, gamma, lambda, beamK);
        if (!prog.empty()) consider(std::move(prog));
    }
    static const int anchorK = std::getenv("L6_K") ? std::atoi(std::getenv("L6_K")) : 24;
    static const bool noAnchor = std::getenv("L6_NO_ANCHOR") != nullptr;
    for (int x = 0; x < 3 && !noAnchor; ++x)
        for (int d : {1, -1}) {
            label = "anchor";
            Prog prog = anchorProgram(g, t, x, d, anchorK, 2, 0.5, 0.9, beamK);
            if (!prog.empty()) consider(std::move(prog));
        }

    for (int x = 0; x < 3 && !greedyOnly; ++x)
        for (int ui : {0, 3})
            for (int vi : {0, 3}) {
                const int u = (x + 1) % 3 + ui, v = (x + 2) % 3 + vi;
                Prog prog = rollEraseProgram(g, t, x, u, v);
                label = "roll";
                if (!prog.empty()) consider(std::move(prog));
            }

    if (debug) std::cerr << "-> " << best.size() << "\n";
    if (best.empty()) throw std::runtime_error("no valid program found");
    return best;
}

void solve(const Input& in, std::ostream& out) {
    i64 total = 0;
    if (const char* chk = std::getenv("L6_CHECK")) {
        // Validate an existing output file against this input.
        std::ifstream f(chk);
        std::vector<Prog> progs(1);
        std::string s;
        while (std::getline(f, s)) {
            if (s.empty()) { if (!progs.back().empty()) progs.emplace_back(); continue; }
            Line l = kNoop;
            int k = 0;
            for (char ch : s)
                if (ch != ' ' && k < 6) l[k++] = ch;
            progs.back().push_back(l);
        }
        if (progs.back().empty()) progs.pop_back();
        int bad = 0;
        if (progs.size() != in.cases.size()) std::cerr << "COUNT MISMATCH " << progs.size() << " vs " << in.cases.size() << "\n";
        for (std::size_t i = 0; i < std::min(progs.size(), in.cases.size()); ++i) {
            const Geometry g(in.cases[i].radius);
            const Target t(g, in.cases[i]);
            const int used = simulate(g, t, progs[i]);
            if (used < 0 || used != static_cast<int>(progs[i].size())) ++bad, std::cerr << "case " << i << " used " << used << " len " << progs[i].size() << "\n";
            total += static_cast<i64>(progs[i].size());
        }
        std::cerr << "CHECK total " << total << " bad " << bad << "\n";
        return;
    }
    std::vector<Prog> progs(in.cases.size());
    {
        std::atomic<std::size_t> nextCase{0};
        const int threads = std::getenv("L6_THREADS") ? std::atoi(std::getenv("L6_THREADS")) : 8;
        std::vector<std::thread> pool;
        for (int k = 0; k < threads; ++k)
            pool.emplace_back([&] {
                for (std::size_t i; (i = nextCase++) < in.cases.size();) progs[i] = solvePattern(in.cases[i]);
            });
        for (auto& th : pool) th.join();
    }
    for (std::size_t i = 0; i < in.cases.size(); ++i) {
        const Prog& prog = progs[i];
        total += static_cast<i64>(prog.size());
        if (i > 0) out << '\n';
        for (const auto& line : prog) {
            for (int e = 0; e < 6; ++e) out << line[e] << (e < 5 ? ' ' : '\n');
        }
    }
    std::cerr << "score (total lines): " << total << '\n';
}

int main(int argc, char** argv) {
    return ccc::run_level<Input>(LEVEL, argc, argv, parse, solve);
}
