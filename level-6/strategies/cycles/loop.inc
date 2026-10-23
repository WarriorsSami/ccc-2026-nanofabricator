// ---------------------------------------------------------------------------
// Strategy 6: closed loop through a fixed configuration C. Beam search over
// ticks for k lines that leave C, return to C, and re-create the pattern with
// a clean halo on the last tick. Repeating the loop gives one copy per k lines.

struct LoopSearch {
    const Geometry& g;
    const Target& t;
    int W = 1;
    std::vector<int> relIdx;
    using B = std::array<u64, kWords>;
    std::array<std::vector<B>, 6> lineBits;
    B P{}, H{};
    std::mt19937 rng{12345};
    double hpen = 0.7;
    int keepPer = 24;

    LoopSearch(const Geometry& geo, const Target& tgt) : g(geo), t(tgt) {
        const int n = g.numCells();
        relIdx.assign(n, -1);
        int m = 0;
        for (int id = 0; id < n; ++id)
            if (t.isP[id] || t.isHalo[id]) relIdx[id] = m++;
        W = (m + 63) / 64;
        for (int id = 0; id < n; ++id) {
            if (relIdx[id] < 0) continue;
            const int r = relIdx[id];
            if (t.isP[id]) P[r >> 6] |= u64{1} << (r & 63);
            else H[r >> 6] |= u64{1} << (r & 63);
        }
        const int R = g.R;
        for (int e = 0; e < 6; ++e) {
            lineBits[e].assign(2 * R + 1, B{});
            for (int o = -R; o <= R; ++o)
                for (int id : g.beamAt(e, o))
                    if (relIdx[id] >= 0) lineBits[e][o + R][relIdx[id] >> 6] |= u64{1} << (relIdx[id] & 63);
        }
    }

    struct State {
        std::array<int8_t, 6> off;
        std::array<int8_t, 6> on;
        B M;
        int parent;
        Line line;
        double score;
    };

    int popc(const B& b) const {
        int s = 0;
        for (int w = 0; w < W; ++w) s += std::popcount(b[w]);
        return s;
    }

    // Returns k lines or empty.
    Prog search(const std::array<int, 6>& cOff, const std::array<int, 6>& cOn, int k, int beamWidth) {
        const int R = g.R;
        std::vector<std::vector<State>> layers;
        State s0{};
        for (int e = 0; e < 6; ++e) s0.off[e] = cOff[e], s0.on[e] = cOn[e];
        s0.parent = -1;
        layers.push_back({s0});
        const char codes[4] = {'-', 'F', 'B', 'T'};
        const int nP = popc(P);
        for (int step = 1; step <= k; ++step) {
            const int remain = k - step;
            std::vector<State> cand;
            const auto& prev = layers.back();
            for (int pi = 0; pi < static_cast<int>(prev.size()); ++pi) {
                const State& ps = prev[pi];
                // options per emitter
                struct Op { char code; int off, on; };
                std::array<std::array<Op, 4>, 6> ops;
                std::array<int, 6> nops{};
                for (int e = 0; e < 6; ++e)
                    for (int c = 0; c < 4; ++c) {
                        Op op{codes[c], ps.off[e], ps.on[e]};
                        if (c == 1) ++op.off;
                        if (c == 2) --op.off;
                        if (c == 3) op.on ^= 1;
                        if (std::abs(op.off) > R + 1) continue;
                        if (std::abs(op.off - cOff[e]) + (op.on != cOn[e] ? 1 : 0) > remain) continue;
                        ops[e][nops[e]++] = op;
                    }
                std::array<int, 6> pick{};
                std::array<B, 7> g1, g2, g3;
                g1[0] = g2[0] = g3[0] = B{};
                std::vector<State> local;
                std::function<void(int)> rec = [&](int e) {
                    if (e == 6) {
                        State ns;
                        for (int q = 0; q < 6; ++q) ns.off[q] = ops[q][pick[q]].off, ns.on[q] = ops[q][pick[q]].on, ns.line[q] = ops[q][pick[q]].code;
                        for (int w = 0; w < W; ++w) ns.M[w] = (ps.M[w] | (g2[6][w] & ~g3[6][w])) & ~g3[6][w];
                        ns.M.fill(0), void();
                        for (int w = 0; w < W; ++w) ns.M[w] = (ps.M[w] | (g2[6][w] & ~g3[6][w])) & ~g3[6][w];
                        ns.parent = pi;
                        B pm{}, hm{};
                        for (int w = 0; w < W; ++w) pm[w] = ns.M[w] & P[w], hm[w] = ns.M[w] & H[w];
                        const int pc = popc(pm), hc = popc(hm);
                        if (remain == 0 && (pc != nP || hc != 0)) return;
                        ns.score = pc - hpen * hc + (rng() & 1023) * 1e-5;
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
                        rec(e + 1);
                    }
                };
                rec(0);
                const int keep = keepPer;
                if (static_cast<int>(local.size()) > keep) {
                    std::nth_element(local.begin(), local.begin() + keep, local.end(), [](const State& a, const State& b) { return a.score > b.score; });
                    local.resize(keep);
                }
                for (auto& s : local) cand.push_back(s);
            }
            if (cand.empty()) return {};
            std::sort(cand.begin(), cand.end(), [](const State& a, const State& b) { return a.score > b.score; });
            // dedupe
            std::vector<State> next;
            std::set<std::vector<u64>> seen;
            for (auto& s : cand) {
                std::vector<u64> key(W + 2);
                u64 cfg = 0;
                for (int e = 0; e < 6; ++e) cfg = cfg * 64 + (s.off[e] + 20) * 2 + s.on[e];
                key[0] = cfg;
                for (int w = 0; w < W; ++w) key[w + 1] = s.M[w];
                if (!seen.insert(key).second) continue;
                next.push_back(s);
                if (static_cast<int>(next.size()) >= beamWidth) break;
            }
            layers.push_back(std::move(next));
        }
        // reconstruct from first final state
        Prog lines;
        int idx = 0;
        for (int step = k; step >= 1; --step) {
            const State& s = layers[step][idx];
            lines.push_back(s.line);
            idx = s.parent;
        }
        std::reverse(lines.begin(), lines.end());
        return lines;
    }
};

// Setup to C (moves while off, toggles on in the last tick), then the loop.
Prog loopProgram(const std::array<int, 6>& cOff, const std::array<int, 6>& cOn, const Prog& loop, int count) {
    int T0 = 0;
    for (int e = 0; e < 6; ++e) T0 = std::max(T0, std::abs(cOff[e]) + cOn[e]);
    Prog prog(T0, kNoop);
    for (int e = 0; e < 6; ++e) {
        for (int s = 0; s < std::abs(cOff[e]); ++s) prog[s][e] = cOff[e] > 0 ? 'F' : 'B';
        if (cOn[e]) prog[T0 - 1][e] = 'T';
    }
    for (int rep = 0; rep < count + 2; ++rep)
        for (const auto& l : loop) prog.push_back(l);
    return prog;
}

// Configurations at each submission tick of a program.
std::vector<std::pair<std::array<int, 6>, std::array<int, 6>>> submissionConfigs(const Geometry& g, const Target& t, const Prog& prog) {
    Builder b(g, t);
    std::vector<std::pair<std::array<int, 6>, std::array<int, 6>>> res;
    Target big = t;
    big.count = INT_MAX;
    Builder bb(g, big);
    for (const auto& l : prog) {
        const int before = bb.accepted;
        bb.step(l);
        if (bb.accepted != before) {
            std::array<int, 6> o{}, a{};
            for (int e = 0; e < 6; ++e) o[e] = bb.off[e], a[e] = bb.on[e];
            res.push_back({o, a});
        }
    }
    return res;
}
