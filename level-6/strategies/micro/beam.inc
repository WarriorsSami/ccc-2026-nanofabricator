// ---------------------------------------------------------------------------
// Strategy 7: beam search per copy. Like the greedy (no halo stamps, never
// destroy finished pattern cells) but keeps the best `width` states per tick,
// scored by finished cells + proximity potential.

struct BeamSearch {
    const Geometry& g;
    const Target& t;
    int W = 1;
    std::vector<int> relIdx;
    using B = std::array<u64, kWords>;
    std::array<std::vector<B>, 6> lineBits;
    B P{}, H{};
    std::vector<int> pcells;  // pattern cell ids
    std::mt19937 rng{777};
    std::vector<double> decay;
    double lambda = 0.9;
    int width = 50, keepPer = 12;

    BeamSearch(const Geometry& geo, const Target& tgt, double gamma) : g(geo), t(tgt) {
        const int n = g.numCells();
        relIdx.assign(n, -1);
        int m = 0;
        for (int id = 0; id < n; ++id)
            if (t.isP[id] || t.isHalo[id]) relIdx[id] = m++;
        W = (m + 63) / 64;
        for (int id = 0; id < n; ++id) {
            if (relIdx[id] < 0) continue;
            const int r = relIdx[id];
            if (t.isP[id]) P[r >> 6] |= u64{1} << (r & 63), pcells.push_back(id);
            else H[r >> 6] |= u64{1} << (r & 63);
        }
        const int R = g.R;
        for (int e = 0; e < 6; ++e) {
            lineBits[e].assign(2 * R + 1, B{});
            for (int o = -R; o <= R; ++o)
                for (int id : g.beamAt(e, o))
                    if (relIdx[id] >= 0) lineBits[e][o + R][relIdx[id] >> 6] |= u64{1} << (relIdx[id] & 63);
        }
        decay.resize(4 * R + 8);
        for (std::size_t d = 0; d < decay.size(); ++d) decay[d] = std::pow(gamma, static_cast<double>(d));
    }

    bool has(const B& b, int id) const { const int r = relIdx[id]; return (b[r >> 6] >> (r & 63)) & 1; }
    int popc(const B& b) const {
        int s = 0;
        for (int w = 0; w < W; ++w) s += std::popcount(b[w]);
        return s;
    }

    struct State {
        std::array<int8_t, 6> off, on;
        B M;
        int parent;
        Line line;
        double score;
    };

    std::array<int, 6> startOff{}, startOn{};
    int startAcc = 0;
    Prog run() {
        const int R = g.R;
        const int nP = static_cast<int>(pcells.size());
        State cur{};
        for (int e = 0; e < 6; ++e) cur.off[e] = startOff[e], cur.on[e] = startOn[e];
        cur.parent = -1;
        Prog prog;
        int accepted = startAcc;
        const char codes[4] = {'-', 'F', 'B', 'T'};
        const std::size_t limit = 40 * pcells.size() * static_cast<std::size_t>(t.count) + 200;
        while (accepted < t.count) {
            std::vector<std::vector<State>> layers;
            layers.push_back({cur});
            int goalIdx = -1;
            for (int depth = 1; goalIdx < 0; ++depth) {
                if (depth > 6 * R + 2 * nP || prog.size() + depth > limit) return {};
                std::vector<State> cand;
                const auto& prev = layers.back();
                for (int pi = 0; pi < static_cast<int>(prev.size()); ++pi) {
                    const State& ps = prev[pi];
                    std::vector<int> need;
                    for (int id : pcells)
                        if (!has(ps.M, id)) need.push_back(id);
                    struct Op { char code; int off, on; double phi; };
                    std::array<std::array<Op, 4>, 6> ops;
                    std::array<int, 6> nops{};
                    for (int e = 0; e < 6; ++e)
                        for (int c = 0; c < 4; ++c) {
                            Op op{codes[c], ps.off[e], ps.on[e], 0.0};
                            if (c == 1) ++op.off;
                            if (c == 2) --op.off;
                            if (c == 3) op.on ^= 1;
                            if (std::abs(op.off) > R && std::abs(op.off) > std::abs(ps.off[e])) continue;
                            for (int id : need) op.phi += decay[std::abs(op.off - g.offFor[e][id]) + (op.on ? 0 : 1)];
                            ops[e][nops[e]++] = op;
                        }
                    const double phiScale = lambda / (6.0 * std::max<std::size_t>(1, need.size()));
                    B done{};
                    for (int w = 0; w < W; ++w) done[w] = ps.M[w] & P[w];
                    const int doneCnt = popc(done);
                    std::array<int, 6> pick{};
                    std::array<B, 7> g1, g2, g3;
                    g1[0] = g2[0] = g3[0] = B{};
                    std::vector<State> local;
                    auto rec = [&](auto&& self, int e, double phi) -> void {
                        if (e == 6) {
                            int gain = 0;
                            for (int w = 0; w < W; ++w) {
                                const u64 e2 = g2[6][w] & ~g3[6][w];
                                if ((e2 & H[w]) || (g3[6][w] & done[w])) return;
                                gain += std::popcount(e2 & P[w] & ~done[w]);
                            }
                            State ns;
                            for (int q = 0; q < 6; ++q) ns.off[q] = ops[q][pick[q]].off, ns.on[q] = ops[q][pick[q]].on, ns.line[q] = ops[q][pick[q]].code;
                            ns.M.fill(0);
                            for (int w = 0; w < W; ++w) ns.M[w] = (ps.M[w] | (g2[6][w] & ~g3[6][w])) & ~g3[6][w] & P[w];
                            ns.parent = pi;
                            ns.score = doneCnt + gain + phi * phiScale + (rng() & 1023) * 1e-7;
                            local.push_back(ns);
                            return;
                        }
                        for (int c = 0; c < nops[e]; ++c) {
                            pick[e] = c;
                            const auto& op = ops[e][c];
                            if (op.on && std::abs(op.off) <= R) {
                                const B& L = lineBits[e][op.off + R];
                                for (int w = 0; w < W; ++w) {
                                    g3[e + 1][w] = g3[e][w] | (g2[e][w] & L[w]);
                                    g2[e + 1][w] = g2[e][w] | (g1[e][w] & L[w]);
                                    g1[e + 1][w] = g1[e][w] | L[w];
                                }
                            } else {
                                g1[e + 1] = g1[e], g2[e + 1] = g2[e], g3[e + 1] = g3[e];
                            }
                            self(self, e + 1, phi + op.phi);
                        }
                    };
                    rec(rec, 0, 0.0);
                    if (static_cast<int>(local.size()) > keepPer) {
                        std::nth_element(local.begin(), local.begin() + keepPer, local.end(), [](const State& a, const State& b) { return a.score > b.score; });
                        local.resize(keepPer);
                    }
                    for (auto& s : local) cand.push_back(s);
                }
                if (cand.empty()) return {};
                std::sort(cand.begin(), cand.end(), [](const State& a, const State& b) { return a.score > b.score; });
                std::vector<State> next;
                std::set<std::vector<u64>> seen;
                for (auto& s : cand) {
                    std::vector<u64> key(W + 1);
                    u64 cfg = 0;
                    for (int e = 0; e < 6; ++e) cfg = cfg * 64 + (s.off[e] + 20) * 2 + s.on[e];
                    key[0] = cfg;
                    for (int w = 0; w < W; ++w) key[w + 1] = s.M[w];
                    if (!seen.insert(key).second) continue;
                    if (popc(s.M) == nP && goalIdx < 0) goalIdx = static_cast<int>(next.size());
                    next.push_back(s);
                    if (static_cast<int>(next.size()) >= width) break;
                }
                layers.push_back(std::move(next));
            }
            // reconstruct
            Prog seg;
            int idx = goalIdx;
            for (int step = static_cast<int>(layers.size()) - 1; step >= 1; --step) {
                const State& s = layers[step][idx];
                seg.push_back(s.line);
                idx = s.parent;
            }
            std::reverse(seg.begin(), seg.end());
            for (auto& l : seg) prog.push_back(l);
            cur = layers.back()[goalIdx];
            cur.M.fill(0);
            cur.parent = -1;
            ++accepted;
        }
        return prog;
    }
};
