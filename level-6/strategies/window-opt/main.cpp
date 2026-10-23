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
#include <mutex>

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

Prog greedyProgram(const Geometry& g, const Target& t, double gamma, double lambda) {
    const int R = g.R, n = g.numCells();
    if (n > kWords * 64) return {};
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

    Builder b(g, t);
    const std::size_t limit = 40 * t.cells.size() * static_cast<std::size_t>(t.count) + 200;
    int idle = 0;

    struct Option { char code; int off; bool on; double phi; };
    std::array<std::array<Option, 4>, 6> opts;
    std::array<int, 6> numOpts{};
    std::array<int, 6> pick{}, bestPick{};
    std::array<Bits, 7> ge1, ge2, ge3;

    while (!b.done()) {
        if (b.prog.size() > limit) return {};
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
    return b.prog;
}

// ---------------------------------------------------------------------------
// Strategy 5: beam search over ticks. Each state is (offsets, on bits, pattern
// matter, copies). Every state expands into all 4^6 lines; the value of a child
// is copies, stamped cells, and a pair-aware potential: for d = 1..D, the
// missing cells that at least two emitters can reach within d ticks.

template <int W>
struct BW {
    u64 w[W];
};

struct BeamParams {
    int K = 20;          // beam width
    int D = 3;           // potential levels
    double lw[8] = {0.3, 0.15, 0.08, 0.04, 0.02, 0.01, 0.005, 0.0025};
    double mu = 0.02;    // separable proximity tie-breaker
    double gamma = 0.5;
    int stuck = 0;
    double Z = 1e9;      // potential is scaled by min(1, Z / |need|)
    double noise = 0;    // random tie-breaking amplitude
    u64 seed = 1;
    double hp = -1;      // halo penalty; <0 = halo matter forbidden
    bool hasInit = false;
    std::array<int, 6> initOff{};
    int initOn = 0, initAcc = 0;
    std::vector<int> initM;
};

template <int W>
Prog beamProgramW(const Geometry& g, const Target& t, const BeamParams& bp) {
    const int R = g.R, n = g.numCells(), span = 2 * R + 1;
    const int D = bp.D;
    using B = BW<W>;
    auto zero = [] { B b; for (int i = 0; i < W; ++i) b.w[i] = 0; return b; };
    auto setb = [](B& b, int i) { b.w[i >> 6] |= u64{1} << (i & 63); };
    // lineMask[e][o+R]
    std::vector<B> lineMask(6 * span, zero());
    for (int e = 0; e < 6; ++e)
        for (int o = -R; o <= R; ++o)
            for (int id : g.beamAt(e, o)) setb(lineMask[e * span + o + R], id);
    // reach[((e*span + o+R)*2 + on)*D + (d-1)]
    std::vector<B> reach(6 * span * 2 * std::max(D, 1), zero());
    for (int e = 0; e < 6; ++e)
        for (int o = -R; o <= R; ++o)
            for (int on = 0; on < 2; ++on)
                for (int d = 1; d <= D; ++d) {
                    B& m = reach[((e * span + o + R) * 2 + on) * D + d - 1];
                    const int rad = d - (on ? 0 : 1);
                    for (int o2 = std::max(-R, o - rad); o2 <= std::min(R, o + rad); ++o2)
                        for (int i = 0; i < W; ++i) m.w[i] |= lineMask[e * span + o2 + R].w[i];
                }
    B P = zero(), halo = zero();
    for (int id : t.cells) setb(P, id);
    for (int id = 0; id < n; ++id)
        if (t.isHalo[id]) setb(halo, id);
    const int Pn = static_cast<int>(t.cells.size());
    std::vector<double> decay(4 * R + 4);
    for (std::size_t d = 0; d < decay.size(); ++d) decay[d] = std::pow(bp.gamma, static_cast<double>(d));
    const bool allowHalo = bp.hp >= 0;
    B PH = zero();
    for (int i = 0; i < W; ++i) PH.w[i] = P.w[i] | halo.w[i];
    auto progressOf = [&](const auto& s) {
        int c = s.acc * (Pn + 1);
        for (int i = 0; i < W; ++i) c += std::popcount(s.m.w[i] & P.w[i]) - std::popcount(s.m.w[i] & halo.w[i]);
        return c;
    };

    struct State {
        std::array<int8_t, 6> off;
        uint8_t on;
        uint8_t acc;
        B m;
    };
    auto hashState = [&](const State& s) {
        u64 h = 0x9e3779b97f4a7c15ULL;
        auto mix = [&](u64 v) { h ^= v + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2); h *= 0xff51afd7ed558ccdULL; };
        u64 a = 0;
        for (int e = 0; e < 6; ++e) a = a * 64 + (s.off[e] + 32);
        mix(a);
        mix(s.on | (u64(s.acc) << 8));
        for (int i = 0; i < W; ++i) mix(s.m.w[i]);
        return h;
    };
    // Apply a line (codes 0 noop 1 F 2 B 3 T) to a state.
    auto apply = [&](const State& s, const std::array<uint8_t, 6>& code, State& out) -> bool {
        out = s;
        B g1 = zero(), g2 = zero(), g3 = zero();
        for (int e = 0; e < 6; ++e) {
            if (code[e] == 1) ++out.off[e];
            else if (code[e] == 2) --out.off[e];
            else if (code[e] == 3) out.on ^= 1 << e;
            if (std::abs(out.off[e]) > R) return false;
            if (out.on >> e & 1) {
                const B& L = lineMask[e * span + out.off[e] + R];
                for (int i = 0; i < W; ++i) {
                    g3.w[i] |= g2.w[i] & L.w[i];
                    g2.w[i] |= g1.w[i] & L.w[i];
                    g1.w[i] |= L.w[i];
                }
            }
        }
        bool full = true;
        for (int i = 0; i < W; ++i) {
            const u64 e2 = g2.w[i] & ~g3.w[i];
            if (!allowHalo && (e2 & halo.w[i])) return false;
            out.m.w[i] = ((out.m.w[i] & ~g3.w[i]) | e2) & PH.w[i];
            if ((out.m.w[i] & PH.w[i]) != P.w[i]) full = false;
        }
        if (full) {
            ++out.acc;
            out.m = zero();
        }
        return true;
    };

    struct Cand {
        double score;
        int parent;
        uint16_t code;
    };
    std::vector<State> beam = {State{{0, 0, 0, 0, 0, 0}, 0, 0, zero()}};
    if (bp.hasInit) {
        State& s0 = beam[0];
        for (int e = 0; e < 6; ++e) s0.off[e] = static_cast<int8_t>(bp.initOff[e]);
        s0.on = static_cast<uint8_t>(bp.initOn);
        s0.acc = static_cast<uint8_t>(bp.initAcc);
        for (int id : bp.initM) setb(s0.m, id);
    }
    std::vector<std::vector<std::pair<int, uint16_t>>> history;  // per depth: (parent, code) for each beam state
    const std::size_t limit = 40 * t.cells.size() * static_cast<std::size_t>(t.count) + 200;
    const int K = bp.K;
    std::vector<Cand> cands;
    cands.reserve(1 << 16);

    // DFS scratch
    struct Opt { int8_t off; uint8_t on; uint8_t code; double phi; };
    std::array<std::array<Opt, 4>, 6> opts;
    std::array<int, 6> numOpts{};
    std::array<B, 7> l1, l2, l3;
    std::vector<std::array<B, 7>> a1(std::max(D, 1)), a2(std::max(D, 1));
    std::vector<int> needCells;

    u64 rng = bp.seed * 0x9e3779b97f4a7c15ULL + static_cast<u64>(t.cells.size()) * 1315423911ULL + 88172645463325252ULL;
    for (int id : t.cells) rng = rng * 6364136223846793005ULL + id;
    if (!rng) rng = 1;
    int lastProg = -1;
    std::size_t lastImprove = 0;
    const int stuckTicks = bp.stuck > 0 ? bp.stuck : 2 * R;
    for (std::size_t depth = 0;; ++depth) {
        if (depth > limit) { if (std::getenv("L6_BDBG")) std::cerr << "beamfail0 depth=" << depth << "\n"; return {}; }
        cands.clear();
        double thresh = -1e30;
        const std::size_t M = static_cast<std::size_t>(K) * 4;
        auto prune = [&] {
            std::nth_element(cands.begin(), cands.begin() + M - 1, cands.end(),
                             [](const Cand& a, const Cand& b) { return a.score > b.score; });
            cands.resize(M);
            thresh = cands[M - 1].score;
        };
        for (int si = 0; si < static_cast<int>(beam.size()); ++si) {
            const State& s = beam[si];
            needCells.clear();
            for (int id : t.cells)
                if (!(s.m.w[id >> 6] >> (id & 63) & 1)) needCells.push_back(id);
            const double phiNorm = bp.mu / (6.0 * std::max<std::size_t>(1, needCells.size()));
            for (int e = 0; e < 6; ++e) {
                numOpts[e] = 0;
                for (int k = 0; k < 4; ++k) {
                    Opt op{s.off[e], static_cast<uint8_t>(s.on >> e & 1), static_cast<uint8_t>(k), 0.0};
                    if (k == 1) ++op.off;
                    if (k == 2) --op.off;
                    if (k == 3) op.on ^= 1;
                    if (std::abs(op.off) > R) continue;
                    double phi = 0;
                    for (int id : needCells) phi += decay[std::abs(op.off - g.offFor[e][id]) + (op.on ? 0 : 1)];
                    op.phi = static_cast<double>(phi * phiNorm);
                    opts[e][numOpts[e]++] = op;
                }
            }
            l1[0] = l2[0] = l3[0] = zero();
            for (int d = 0; d < D; ++d) a1[d][0] = a2[d][0] = zero();
            std::array<int, 6> pick{};
            const double accBase = static_cast<double>((s.acc) * (Pn + 1));
            auto leaf = [&](double phi) {
                int stamped = 0;
                B nm, need;
                bool full = true;
                int hcnt = 0;
                for (int i = 0; i < W; ++i) {
                    const u64 e2 = l2[6].w[i] & ~l3[6].w[i];
                    if (!allowHalo && (e2 & halo.w[i])) return;
                    nm.w[i] = ((s.m.w[i] & ~l3[6].w[i]) | e2) & PH.w[i];
                    if (nm.w[i] != P.w[i]) full = false;
                    hcnt += std::popcount(nm.w[i] & halo.w[i]);
                }
                double score;
                if (full) {
                    score = accBase + (Pn + 1);
                    need = P;
                } else {
                    score = accBase;
                    for (int i = 0; i < W; ++i) {
                        stamped += std::popcount(nm.w[i] & P.w[i]);
                        need.w[i] = P.w[i] & ~nm.w[i];
                    }
                    score += stamped - bp.hp * hcnt * (allowHalo ? 1 : 0);
                }
                double pot = 0;
                for (int d = 0; d < D; ++d) {
                    int c = 0;
                    for (int i = 0; i < W; ++i) c += std::popcount(a2[d][6].w[i] & need.w[i]);
                    pot += bp.lw[d] * c;
                }
                {
                    int needN = 0;
                    for (int i = 0; i < W; ++i) needN += std::popcount(need.w[i]);
                    if (needN > bp.Z) pot *= bp.Z / needN;
                }
                score += static_cast<double>(pot) + phi;
                if (bp.noise > 0) {
                    rng ^= rng << 13, rng ^= rng >> 7, rng ^= rng << 17;
                    score += bp.noise * static_cast<double>(rng >> 11) * 0x1.0p-53;
                }
                if (score <= thresh) return;
                uint16_t code = 0;
                for (int e = 0; e < 6; ++e) code |= opts[e][pick[e]].code << (2 * e);
                cands.push_back({score, si, code});
                if (cands.size() >= 4 * M) prune();
            };
            // iterative nested loops via recursion lambda
            std::function<void(int, double)> dfs = [&](int e, double phi) {
                if (e == 6) { leaf(phi); return; }
                for (int k = 0; k < numOpts[e]; ++k) {
                    const Opt& op = opts[e][k];
                    pick[e] = k;
                    if (op.on) {
                        const B& L = lineMask[e * span + op.off + R];
                        for (int i = 0; i < W; ++i) {
                            l3[e + 1].w[i] = l3[e].w[i] | (l2[e].w[i] & L.w[i]);
                            l2[e + 1].w[i] = l2[e].w[i] | (l1[e].w[i] & L.w[i]);
                            l1[e + 1].w[i] = l1[e].w[i] | L.w[i];
                        }
                    } else {
                        l1[e + 1] = l1[e], l2[e + 1] = l2[e], l3[e + 1] = l3[e];
                    }
                    for (int d = 0; d < D; ++d) {
                        const B& Q = reach[((e * span + op.off + R) * 2 + op.on) * D + d];
                        for (int i = 0; i < W; ++i) {
                            a2[d][e + 1].w[i] = a2[d][e].w[i] | (a1[d][e].w[i] & Q.w[i]);
                            a1[d][e + 1].w[i] = a1[d][e].w[i] | Q.w[i];
                        }
                    }
                    dfs(e + 1, phi + op.phi);
                }
            };
            dfs(0, 0.0);
        }
        if (cands.empty()) { if (std::getenv("L6_BDBG")) std::cerr << "beamfail1 depth=" << depth << "\n"; return {}; }
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.score > b.score; });
        {
            // progress = copies and stamped cells of the best state in the beam
            int bestProg = -1000000;
            for (const State& s : beam) bestProg = std::max(bestProg, progressOf(s));
            if (bestProg > lastProg) lastProg = bestProg, lastImprove = depth;
            if (std::getenv("L6_BDBG") && depth % 20 == 0) std::cerr << "d" << depth << " prog " << bestProg << " best " << cands[0].score << "\n";
            if (static_cast<int>(depth - lastImprove) > stuckTicks) {
                // take the best-progress state; walk the cheapest pair to the nearest missing cell
                int si = 0, sc = -1000000;
                for (int i = 0; i < static_cast<int>(beam.size()); ++i) {
                    int c = progressOf(beam[i]);
                    if (c > sc) sc = c, si = i;
                }
                State cur = beam[si];
                int bestCell = -1, bA = -1, bB = -1, bC = -1, bestCost = INT_MAX;
                for (int id : t.cells) {
                    if (cur.m.w[id >> 6] >> (id & 63) & 1) continue;
                    for (int x = 0; x < 6; ++x)
                        for (int y = x + 1; y < 6; ++y) {
                            if (axisOf(x) == axisOf(y)) continue;
                            const int cost = std::max(std::abs(cur.off[x] - g.offFor[x][id]), std::abs(cur.off[y] - g.offFor[y][id]));
                            if (cost < bestCost) bestCost = cost, bestCell = id, bA = x, bB = y;
                        }
                }
                if (bestCell < 0) {
                    // pattern complete but halo dirty: erase one halo cell with three beams (one per axis)
                    for (int id = 0; id < n; ++id) {
                        if (!t.isHalo[id] || !(cur.m.w[id >> 6] >> (id & 63) & 1)) continue;
                        for (int x = 0; x < 2; ++x) for (int y = 0; y < 2; ++y) for (int z = 0; z < 2; ++z) {
                            const int ea = 0 + 3 * x, eb = 1 + 3 * y, ec = 2 + 3 * z;
                            const int cost = std::max({std::abs(cur.off[ea] - g.offFor[ea][id]), std::abs(cur.off[eb] - g.offFor[eb][id]), std::abs(cur.off[ec] - g.offFor[ec][id])});
                            if (cost < bestCost) bestCost = cost, bestCell = id, bA = ea, bB = eb, bC = ec;
                        }
                    }
                }
                if (std::getenv("L6_BDBG")) std::cerr << "stuck at " << depth << " prog " << sc << " cell " << bestCell << " pair " << bA << bB << " cost " << bestCost << "\n";
                if (bestCell < 0) { if (std::getenv("L6_BDBG")) std::cerr << "beamfail2 depth=" << depth << "\n"; return {}; }
                std::array<int, 6> delta{}, startMove{};
                std::array<bool, 6> offFirst{}, onLast{};
                int ticks = 0;
                for (int e = 0; e < 6; ++e) {
                    delta[e] = (e == bA || e == bB || e == bC) ? g.offFor[e][bestCell] - cur.off[e] : 0;
                    offFirst[e] = cur.on >> e & 1;
                    onLast[e] = e == bA || e == bB || e == bC;
                    startMove[e] = offFirst[e] ? 1 : 0;
                    ticks = std::max(ticks, startMove[e] + std::abs(delta[e]) + (onLast[e] ? 1 : 0));
                }
                int parent = si;
                for (int k = 0; k < ticks; ++k) {
                    std::array<uint8_t, 6> code{};
                    uint16_t cd = 0;
                    for (int e = 0; e < 6; ++e) {
                        if (offFirst[e] && k == 0) code[e] = 3;
                        else if (onLast[e] && k == ticks - 1) code[e] = 3;
                        else if (k >= startMove[e] && k < startMove[e] + std::abs(delta[e])) code[e] = delta[e] > 0 ? 1 : 2;
                        cd |= code[e] << (2 * e);
                    }
                    State ns;
                    if (!apply(cur, code, ns)) { if (std::getenv("L6_BDBG")) std::cerr << "beamfail3 depth=" << depth << "\n"; return {}; }
                    history.push_back({{parent, cd}});
                    parent = 0;
                    cur = ns;
                    ++depth;
                    if (cur.acc >= t.count) {
                        // reconstruct
                        Prog prog(depth);
                        int idx = 0;
                        for (int dd = static_cast<int>(depth) - 1; dd >= 0; --dd) {
                            const auto [par, c2] = history[dd][idx];
                            Line line = kNoop;
                            const char codes[4] = {'-', 'F', 'B', 'T'};
                            for (int e = 0; e < 6; ++e) line[e] = codes[(c2 >> (2 * e)) & 3];
                            prog[dd] = line;
                            idx = par;
                        }
                        return prog;
                    }
                }
                --depth;  // loop increments
                beam = {cur};
                lastImprove = depth + 1;
                continue;
            }
        }
        std::vector<State> next;
        std::vector<std::pair<int, uint16_t>> hist;
        std::unordered_set<u64> seen;
        for (const Cand& c : cands) {
            if (static_cast<int>(next.size()) >= K) break;
            std::array<uint8_t, 6> code;
            for (int e = 0; e < 6; ++e) code[e] = (c.code >> (2 * e)) & 3;
            State ns;
            if (!apply(beam[c.parent], code, ns)) continue;
            if (!seen.insert(hashState(ns)).second) continue;
            hist.push_back({c.parent, c.code});
            next.push_back(ns);
            if (ns.acc >= t.count) {
                // reconstruct
                history.push_back(hist);
                Prog prog(depth + 1);
                int idx = static_cast<int>(next.size()) - 1;
                for (int dd = static_cast<int>(depth); dd >= 0; --dd) {
                    const auto [par, cd] = history[dd][idx];
                    Line line = kNoop;
                    const char codes[4] = {'-', 'F', 'B', 'T'};
                    for (int e = 0; e < 6; ++e) line[e] = codes[(cd >> (2 * e)) & 3];
                    prog[dd] = line;
                    idx = par;
                }
                return prog;
            }
        }
        history.push_back(std::move(hist));
        beam = std::move(next);
    }
}

Prog beamProgram(const Geometry& g, const Target& t, const BeamParams& bp) {
    const int words = (g.numCells() + 63) / 64;
    switch (words) {
        case 1: return beamProgramW<1>(g, t, bp);
        case 2: return beamProgramW<2>(g, t, bp);
        case 3: return beamProgramW<3>(g, t, bp);
        case 4: return beamProgramW<4>(g, t, bp);
        case 5: return beamProgramW<5>(g, t, bp);
        case 6: return beamProgramW<6>(g, t, bp);
        case 7: return beamProgramW<7>(g, t, bp);
        default: return {};
    }
}

Prog solvePattern(const Pattern& p) {
    const Geometry g(p.radius);
    const Target t(g, p);
    Prog best;
    static const bool debug = std::getenv("L6_DEBUG") != nullptr;
    std::string label;
    std::ostringstream dbg;
    auto consider = [&](Prog prog) {
        const int used = simulate(g, t, prog);
        if (debug) dbg << label << "=" << used << " ";
        if (used < 0) return;
        prog.resize(used);
        if (best.empty() || prog.size() < best.size()) best = std::move(prog);
    };

    static const bool greedyOnly = std::getenv("L6_GREEDY_ONLY") != nullptr;
    StaticSearch search(g, t);
    search.run();
    if (debug) dbg << "R" << g.R << " P" << t.cells.size() << " cnt" << t.count << " | ";
    label = "static";
    if (search.bestCost != INT_MAX) consider(staticProgram(search.best, t.count));

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
            v = {{0.5, 0.9}, {0.25, 0.9}};
        }
        return v;
    }();
    for (const auto& [gamma, lambda] : params) {
        label = "greedy";
        Prog prog = greedyProgram(g, t, gamma, lambda);
        if (!prog.empty()) consider(std::move(prog));
    }

    static const std::vector<BeamParams> beams = [] {
        std::vector<BeamParams> v;
        const char* env = std::getenv("L6_BEAMS");
        std::string spec = env ? env : "20 3";
        std::istringstream ss(spec);
        std::string item;
        while (std::getline(ss, item, ',')) {
            std::istringstream is(item);
            BeamParams bp;
            is >> bp.K >> bp.D;
            double x;
            if (is >> x) bp.mu = x;
            if (is >> x) for (int d = 0; d < 8; ++d) bp.lw[d] = x * std::pow(0.5, d);
            if (is >> x) bp.Z = x;
            if (is >> x) bp.noise = x;
            if (is >> x) bp.seed = static_cast<u64>(x);
            if (is >> x) bp.hp = x;
            v.push_back(bp);
        }
        return v;
    }();
    if (const char* pf = std::getenv("L6_PREFIX")) {
        std::ifstream f(pf);
        Prog pre;
        std::string ln;
        while (std::getline(f, ln)) {
            if (ln.find_first_not_of(" \r\t") == std::string::npos) continue;
            std::istringstream is(ln);
            Line l = kNoop;
            for (int e = 0; e < 6; ++e) { std::string tok; is >> tok; l[e] = tok[0]; }
            pre.push_back(l);
        }
        label = "pre";
        consider(pre);
        std::vector<int> ks;
        if (const char* kk = std::getenv("L6_PREFIXK")) { std::istringstream is(kk); int k; while (is >> k) ks.push_back(k); }
        const bool fromEnd = std::getenv("L6_FROMEND") != nullptr;
        for (int k0 : ks) {
            const int k = fromEnd ? static_cast<int>(pre.size()) - k0 : k0;
            if (k < 0 || k >= static_cast<int>(pre.size())) continue;
            Builder bd(g, t);
            for (int i = 0; i < k; ++i) bd.step(pre[i]);
            if (bd.done()) continue;
            bool ok = true;
            for (int id = 0; id < g.numCells(); ++id) if (t.isHalo[id] && bd.matter[id]) ok = false;
            for (int e = 0; e < 6; ++e) if (std::abs(bd.off[e]) > g.R) ok = false;
            if (!ok) continue;
            for (const auto& bp0 : beams) {
                BeamParams bp = bp0;
                bp.hasInit = true;
                for (int e = 0; e < 6; ++e) bp.initOff[e] = bd.off[e];
                bp.initOn = 0;
                for (int e = 0; e < 6; ++e) if (bd.on[e]) bp.initOn |= 1 << e;
                bp.initAcc = bd.accepted;
                for (int id : t.cells) if (bd.matter[id]) bp.initM.push_back(id);
                Prog suf = beamProgram(g, t, bp);
                if (suf.empty()) continue;
                Prog full(pre.begin(), pre.begin() + k);
                full.insert(full.end(), suf.begin(), suf.end());
                const int before = best.empty() ? INT_MAX : static_cast<int>(best.size());
                consider(full);
                if (static_cast<int>(best.size()) < before && std::getenv("L6_PDBG")) std::cerr << "k=" << k << " K=" << bp.K << " -> " << best.size() << "\n";
            }
        }
    }
    for (const auto& bp : beams) {
        if (std::getenv("L6_PREFIX")) break;
        label = "beam";
        Prog prog = beamProgram(g, t, bp);
        if (std::getenv("L6_BDBG")) std::cerr << "beam returned " << prog.size() << "\n";
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

    if (debug) {
        dbg << "-> " << best.size() << "\n";
        static std::mutex mu;
        std::lock_guard lk(mu);
        std::cerr << dbg.str();
    }
    if (best.empty()) throw std::runtime_error("no valid program found");
    return best;
}

void solve(const Input& in, std::ostream& out) {
    i64 total = 0;
    if (const char* vf = std::getenv("L6_VERIFY")) {
        // Verify an output file: every program must submit its pattern `count` times.
        std::ifstream f(vf);
        std::vector<Prog> progs(1);
        std::string ln;
        while (std::getline(f, ln)) {
            if (ln.find_first_not_of(" \r\t") == std::string::npos) { if (!progs.back().empty()) progs.emplace_back(); continue; }
            std::istringstream is(ln);
            Line l;
            for (int e = 0; e < 6; ++e) { std::string tok; is >> tok; l[e] = tok[0]; }
            progs.back().push_back(l);
        }
        if (progs.back().empty()) progs.pop_back();
        int bad = 0;
        if (progs.size() != in.cases.size()) std::cerr << "VERIFY count mismatch " << progs.size() << " vs " << in.cases.size() << "\n", ++bad;
        for (std::size_t i = 0; i < std::min(progs.size(), in.cases.size()); ++i) {
            const Geometry g(in.cases[i].radius);
            const Target t(g, in.cases[i]);
            if (simulate(g, t, progs[i]) < 0) { ++bad; std::cerr << "VERIFY fail pattern " << i << "\n"; }
            total += progs[i].size();
        }
        std::cerr << "VERIFY " << (bad ? "FAILED " : "OK ") << "total " << total << "\n";
        return;
    }
    std::vector<Prog> progs(in.cases.size());
    {
        std::atomic<std::size_t> nextIdx{0};
        std::vector<std::thread> pool;
        const unsigned nt = std::max(1u, std::thread::hardware_concurrency());
        for (unsigned k = 0; k < nt; ++k)
            pool.emplace_back([&] {
                for (std::size_t i; (i = nextIdx++) < in.cases.size();) progs[i] = solvePattern(in.cases[i]);
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
