// ---------------------------------------------------------------------------
// Strategy 5: periodic cycle. Every emitter follows a closed walk of length k
// (net zero moves, even toggles) from a start state. A DFS over emitters picks
// one walk each so that, in steady state, the pattern is re-created once per
// period with a clean halo. Cost ~ setup + k * count.

struct CycleSearch {
    static constexpr int kMaxK = 6;
    static constexpr int NONE = -128;
    const Geometry& g;
    const Target& t;
    int k = 2;
    int W = 1;
    std::vector<int> relIdx;
    using B = std::array<u64, kWords>;
    std::array<std::vector<B>, 6> lineBits;  // compact relevant-cell bits
    B P{}, H{};
    struct Opt {
        std::array<int, kMaxK> beam;  // offset or NONE per tick (state after line j)
        int cost;
        int o0, a0;
        std::array<char, kMaxK> walk;
    };
    std::array<std::vector<Opt>, 6> opts;
    std::array<int, 6> choice{};
    // counters per depth, per tick
    std::array<std::array<std::array<B, kMaxK>, 3>, 7> cnt{};
    i64 nodes = 0, nodeLimit = 3'000'000;
    int bestCost = INT_MAX;
    std::vector<std::array<int, 6>> found;  // candidate choices
    int maxFound = 40;

    CycleSearch(const Geometry& geo, const Target& tgt, int period) : g(geo), t(tgt), k(period) {
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
        // closed walks
        std::vector<std::array<char, kMaxK>> walks;
        int total = 1;
        for (int i = 0; i < k; ++i) total *= 4;
        const char codes[4] = {'-', 'F', 'B', 'T'};
        for (int w = 0; w < total; ++w) {
            std::array<char, kMaxK> s{};
            int x = w, f = 0, b = 0, tg = 0;
            for (int i = 0; i < k; ++i) {
                s[i] = codes[x & 3];
                if ((x & 3) == 1) ++f;
                if ((x & 3) == 2) ++b;
                if ((x & 3) == 3) ++tg;
                x >>= 2;
            }
            if (f == b && tg % 2 == 0) walks.push_back(s);
        }
        for (int e = 0; e < 6; ++e) {
            std::map<std::array<int, kMaxK>, Opt> seen;
            for (int a0 = 0; a0 < 2; ++a0)
                for (int o0 = -R; o0 <= R; ++o0)
                    for (const auto& wk : walks) {
                        Opt op{};
                        op.beam.fill(NONE);
                        int o = o0, a = a0;
                        for (int i = 0; i < k; ++i) {
                            if (wk[i] == 'F') ++o;
                            else if (wk[i] == 'B') --o;
                            else if (wk[i] == 'T') a ^= 1;
                            bool rel = false;
                            if (a && std::abs(o) <= R)
                                for (int w = 0; w < W; ++w) rel = rel || lineBits[e][o + R][w];
                            op.beam[i] = rel ? o : NONE;
                        }
                        op.cost = std::abs(o0) + a0;
                        op.o0 = o0, op.a0 = a0, op.walk = wk;
                        auto it = seen.find(op.beam);
                        if (it == seen.end() || it->second.cost > op.cost) seen[op.beam] = op;
                    }
            for (auto& [key, op] : seen) opts[e].push_back(op);
            std::sort(opts[e].begin(), opts[e].end(), [](const Opt& a, const Opt& b) { return a.cost < b.cost; });
        }
    }

    bool leafOk(const std::array<std::array<B, kMaxK>, 3>& c) const {
        for (int js = 0; js < k; ++js) {
            B M{};
            for (int s = 1; s <= k; ++s) {
                const int j = (js + s) % k;
                for (int w = 0; w < W; ++w) {
                    const u64 ex2 = c[1][j][w] & ~c[2][j][w];
                    M[w] = (M[w] | ex2) & ~c[2][j][w];
                }
            }
            bool ok = true;
            for (int w = 0; w < W && ok; ++w) ok = !(P[w] & ~M[w]) && !(H[w] & M[w]);
            if (ok) return true;
        }
        return false;
    }

    void run() { dfs(0, 0); }

    void dfs(int d, int cost) {
        if (++nodes > nodeLimit || cost >= bestCost) return;
        const auto& c = cnt[d];
        const int r = 6 - d;
        // pruning
        {
            B okP{}, ex2any{}, fix{};
            for (int j = 0; j < k; ++j)
                for (int w = 0; w < W; ++w) {
                    const u64 c1 = c[0][j][w], c2 = c[1][j][w], c3 = c[2][j][w];
                    u64 cond;
                    if (r == 0) cond = c2 & ~c3;
                    else if (r == 1) cond = c1 & ~c3;
                    else cond = ~c3;
                    okP[w] |= cond;
                    ex2any[w] |= c2 & ~c3;
                    u64 fx;
                    if (r == 0) fx = c3;
                    else if (r == 1) fx = c2;
                    else if (r == 2) fx = c1;
                    else fx = ~u64{0};
                    fix[w] |= fx;
                }
            for (int w = 0; w < W; ++w) {
                if (P[w] & ~okP[w]) return;
                if (H[w] & ex2any[w] & ~fix[w]) return;
            }
        }
        if (d == 6) {
            if (!leafOk(c)) return;
            if (static_cast<int>(found.size()) < maxFound) found.push_back(choice);
            else {
                found.erase(found.begin());
                found.push_back(choice);
            }
            bestCost = cost + 1;  // allow equal-cost alternatives? keep strictly better search
            bestCost = cost;
            return;
        }
        for (int i = 0; i < static_cast<int>(opts[d].size()); ++i) {
            const Opt& op = opts[d][i];
            if (std::max(cost, op.cost) >= bestCost) continue;
            choice[d] = i;
            auto& nc = cnt[d + 1];
            for (int j = 0; j < k; ++j) {
                if (op.beam[j] == NONE) {
                    for (int q = 0; q < 3; ++q) nc[q][j] = c[q][j];
                    continue;
                }
                const B& L = lineBits[d][op.beam[j] + g.R];
                for (int w = 0; w < W; ++w) {
                    nc[2][j][w] = c[2][j][w] | (c[1][j][w] & L[w]);
                    nc[1][j][w] = c[1][j][w] | (c[0][j][w] & L[w]);
                    nc[0][j][w] = c[0][j][w] | L[w];
                }
            }
            dfs(d + 1, std::max(cost, op.cost));
            if (nodes > nodeLimit) return;
        }
    }

    // Program: setup to the state after line `rot`, then the rotated cycle.
    Prog build(const std::array<int, 6>& ch, int rot, int count) const {
        std::array<int, 6> o{}, a{};
        std::array<std::array<char, kMaxK>, 6> wk{};
        for (int e = 0; e < 6; ++e) {
            const Opt& op = opts[e][ch[e]];
            o[e] = op.o0, a[e] = op.a0;
            for (int i = 0; i < rot; ++i) {
                const char cch = op.walk[i];
                if (cch == 'F') ++o[e];
                else if (cch == 'B') --o[e];
                else if (cch == 'T') a[e] ^= 1;
            }
            for (int i = 0; i < k; ++i) wk[e][i] = op.walk[(rot + i) % k];
        }
        int T0 = 0;
        for (int e = 0; e < 6; ++e) T0 = std::max(T0, std::abs(o[e]) + a[e]);
        Prog prog(T0, kNoop);
        for (int e = 0; e < 6; ++e) {
            for (int s = 0; s < std::abs(o[e]); ++s) prog[s][e] = o[e] > 0 ? 'F' : 'B';
            if (a[e]) prog[T0 - 1][e] = 'T';
        }
        for (int rep = 0; rep < count + 2; ++rep)
            for (int i = 0; i < k; ++i) {
                Line line;
                for (int e = 0; e < 6; ++e) line[e] = wk[e][i];
                prog.push_back(line);
            }
        return prog;
    }
};
