// Exact window optimizer: for windows [k, j) of a valid program, search for a
// path of < j-k lines from state S_k to exactly S_j (offsets, on, acc, matter on
// P∪halo). Also exact tail search to acceptance.
// usage: wopt <input> <programs> <out> <maxL> <seconds_per_prog>
#define main base_main
#include "main.cpp"
#undef main
#include <chrono>

std::vector<Prog> readPrograms(const std::string& path) {
    std::ifstream f(path);
    std::vector<Prog> progs(1);
    std::string line;
    while (std::getline(f, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) {
            if (!progs.back().empty()) progs.emplace_back();
            continue;
        }
        std::istringstream ss(line);
        Line l = kNoop;
        std::string tok;
        for (int e = 0; e < 6 && ss >> tok; ++e) l[e] = tok[0];
        progs.back().push_back(l);
    }
    if (progs.back().empty()) progs.pop_back();
    return progs;
}

struct St {
    std::array<int, 6> off{};
    int on = 0, acc = 0;
    Bits m;
    bool operator==(const St& o) const { return off == o.off && on == o.on && acc == o.acc && m.w == o.m.w; }
};

struct WOpt {
    const Geometry& g;
    const Target& t;
    int R, n;
    std::vector<Bits> lineMask;  // e*span + o+R
    Bits P, H, PH;
    int span;
    std::chrono::steady_clock::time_point deadline;
    long long nodes = 0;
    bool timedOut = false;

    WOpt(const Geometry& geo, const Target& tgt) : g(geo), t(tgt), R(geo.R), n(geo.numCells()) {
        span = 2 * R + 1;
        lineMask.resize(6 * span);
        for (int e = 0; e < 6; ++e)
            for (int o = -R; o <= R; ++o)
                for (int id : g.beamAt(e, o)) lineMask[e * span + o + R].set(id);
        for (int id : t.cells) P.set(id);
        for (int id = 0; id < n; ++id) if (t.isHalo[id]) H.set(id);
        for (int w = 0; w < kWords; ++w) PH.w[w] = P.w[w] | H.w[w];
    }

    static bool bit(const Bits& b, int i) { return b.w[i >> 6] >> (i & 63) & 1; }

    St stepState(const St& s, const Line& l) const {
        St o = s;
        for (int e = 0; e < 6; ++e) {
            if (l[e] == 'F') ++o.off[e];
            else if (l[e] == 'B') --o.off[e];
            else if (l[e] == 'T') o.on ^= 1 << e;
        }
        Bits g1, g2, g3;
        for (int e = 0; e < 6; ++e) {
            if (!(o.on >> e & 1) || std::abs(o.off[e]) > R) continue;
            const Bits& L = lineMask[e * span + o.off[e] + R];
            for (int w = 0; w < kWords; ++w) {
                g3.w[w] |= g2.w[w] & L.w[w];
                g2.w[w] |= g1.w[w] & L.w[w];
                g1.w[w] |= L.w[w];
            }
        }
        finish(o, g2, g3);
        return o;
    }
    void finish(St& o, const Bits& g2, const Bits& g3) const {
        bool full = true;
        for (int w = 0; w < kWords; ++w) {
            o.m.w[w] = ((o.m.w[w] & ~g3.w[w]) | (g2.w[w] & ~g3.w[w])) & PH.w[w];
            if (o.m.w[w] != P.w[w]) full = false;
        }
        if (full) {
            ++o.acc;
            o.m = Bits{};
        }
    }

    int dcov(const St& s, int e, int c) const {
        return std::abs(s.off[e] - g.offFor[e][c]) + ((s.on >> e & 1) ? 0 : 1);
    }
    // k-th smallest (k=2 or 3) coverage time of cell c
    int kth(const St& s, int c, int k) const {
        int d[6];
        for (int e = 0; e < 6; ++e) d[e] = dcov(s, e, c);
        std::sort(d, d + 6);
        return std::max(1, d[k - 1]);
    }

    // lower bound for reaching target state tg
    int hTarget(const St& s, const St& tg) const {
        int h = 0;
        for (int e = 0; e < 6; ++e) h = std::max(h, std::abs(s.off[e] - tg.off[e]) + (((s.on ^ tg.on) >> e) & 1));
        if (s.acc != tg.acc) return 1000;
        for (int w = 0; w < kWords; ++w) {
            u64 d = (s.m.w[w] ^ tg.m.w[w]) & PH.w[w];
            while (d) {
                const int b = std::countr_zero(d);
                d &= d - 1;
                const int c = w * 64 + b;
                h = std::max(h, kth(s, c, bit(tg.m, c) ? 2 : 3));
            }
        }
        return h;
    }
    // lower bound for finishing (acc >= count)
    int hTail(const St& s) const {
        if (s.acc >= t.count) return 0;
        int h = 1;
        bool anyMissing = false;
        for (int id : t.cells)
            if (!bit(s.m, id)) anyMissing = true, h = std::max(h, kth(s, id, 2));
        if (!anyMissing)
            for (int w = 0; w < kWords; ++w) {
                u64 d = s.m.w[w] & H.w[w];
                while (d) {
                    const int b = std::countr_zero(d);
                    d &= d - 1;
                    h = std::max(h, kth(s, w * 64 + b, 3));
                }
            }
        return h + (t.count - s.acc - 1);
    }

    // DFS: find path of length <= depthLeft from s reaching goal. mode 0: target tg, mode 1: tail.
    const St* tg = nullptr;
    int mode = 0;
    std::vector<Line> path;
    std::unordered_map<u64, int> failed;  // hash -> max depthLeft known to fail

    u64 hashSt(const St& s) const {
        u64 h = 1469598103934665603ULL;
        auto mix = [&](u64 v) { h ^= v; h *= 1099511628211ULL; h ^= h >> 29; };
        for (int e = 0; e < 6; ++e) mix(static_cast<u64>(s.off[e] + 1000));
        mix(static_cast<u64>(s.on) | (static_cast<u64>(s.acc) << 8));
        for (int w = 0; w < kWords; ++w) mix(s.m.w[w]);
        return h;
    }

    bool isGoal(const St& s) const { return mode == 0 ? s == *tg : s.acc >= t.count; }
    int hOf(const St& s) const { return mode == 0 ? hTarget(s, *tg) : hTail(s); }

    bool dfs(const St& s, int depthLeft) {
        if (isGoal(s)) return true;
        if (depthLeft == 0) return false;
        if (hOf(s) > depthLeft) return false;
        if ((++nodes & 1023) == 0 && std::chrono::steady_clock::now() > deadline) timedOut = true;
        if (timedOut) return false;
        const u64 hs = hashSt(s);
        auto it = failed.find(hs);
        if (it != failed.end() && it->second >= depthLeft) return false;
        // per-emitter options
        std::array<std::array<std::pair<int, int>, 4>, 6> opts;  // (off, on)
        std::array<std::array<char, 4>, 6> codes;
        std::array<int, 6> no{};
        const char cs[4] = {'-', 'F', 'B', 'T'};
        for (int e = 0; e < 6; ++e) {
            for (int k = 0; k < 4; ++k) {
                int o = s.off[e], on = s.on >> e & 1;
                if (k == 1) ++o;
                if (k == 2) --o;
                if (k == 3) on ^= 1;
                if (std::abs(o) > 3 * R) continue;
                if (mode == 0) {
                    const int c = std::abs(o - tg->off[e]) + (on != ((tg->on >> e) & 1));
                    if (c > depthLeft - 1) continue;
                } else {
                    if (std::abs(o) > R + 1) continue;
                }
                opts[e][no[e]] = {o, on};
                codes[e][no[e]] = cs[k];
                ++no[e];
            }
            if (no[e] == 0) { failed[hs] = std::max(failed[hs], depthLeft); return false; }
        }
        std::array<Bits, 7> l1, l2, l3;
        l1[0] = l2[0] = l3[0] = Bits{};
        std::array<int, 6> pick{};
        bool found = false;
        std::function<void(int)> rec = [&](int e) {
            if (found || timedOut) return;
            if (e == 6) {
                St c = s;
                for (int x = 0; x < 6; ++x) {
                    c.off[x] = opts[x][pick[x]].first;
                    c.on = (c.on & ~(1 << x)) | (opts[x][pick[x]].second << x);
                }
                finish(c, l2[6], l3[6]);
                path.push_back(Line{codes[0][pick[0]], codes[1][pick[1]], codes[2][pick[2]], codes[3][pick[3]], codes[4][pick[4]], codes[5][pick[5]]});
                if (dfs(c, depthLeft - 1)) { found = true; return; }
                path.pop_back();
                return;
            }
            for (int k = 0; k < no[e] && !found; ++k) {
                pick[e] = k;
                const auto [o, on] = opts[e][k];
                if (on && std::abs(o) <= R) {
                    const Bits& L = lineMask[e * span + o + R];
                    for (int w = 0; w < kWords; ++w) {
                        l3[e + 1].w[w] = l3[e].w[w] | (l2[e].w[w] & L.w[w]);
                        l2[e + 1].w[w] = l2[e].w[w] | (l1[e].w[w] & L.w[w]);
                        l1[e + 1].w[w] = l1[e].w[w] | L.w[w];
                    }
                } else {
                    l1[e + 1] = l1[e], l2[e + 1] = l2[e], l3[e + 1] = l3[e];
                }
                rec(e + 1);
            }
        };
        rec(0);
        if (!found) failed[hs] = std::max(failed[hs], depthLeft);
        return found;
    }
};

int main(int argc, char** argv) {
    if (argc < 6) return 1;
    const int maxL = std::stoi(argv[4]);
    const double secs = std::stod(argv[5]);
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    auto progs = readPrograms(argv[2]);
    std::ofstream out(argv[3]);
    i64 before = 0, after = 0;
    for (std::size_t pi = 0; pi < input.cases.size(); ++pi) {
        const Geometry g(input.cases[pi].radius);
        const Target t(g, input.cases[pi]);
        Prog best = progs.at(pi);
        int used = simulate(g, t, best);
        if (used < 0) { std::cerr << "invalid input prog " << pi << "\n"; return 2; }
        best.resize(used);
        before += best.size();
        WOpt W(g, t);
        const auto start = std::chrono::steady_clock::now();
        const auto endAll = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(secs));
        bool improved = true;
        while (improved && std::chrono::steady_clock::now() < endAll) {
            improved = false;
            const int N = best.size();
            std::vector<St> S(N + 1);
            for (int i = 0; i < N; ++i) S[i + 1] = W.stepState(S[i], best[i]);
            // tail search
            for (int k = N - 1; k >= std::max(0, N - maxL) && !improved; --k) {
                W.mode = 1;
                W.failed.clear();
                W.timedOut = false;
                W.deadline = std::min(endAll, std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(secs / 20)));
                for (int dl = 1; dl < N - k && !improved; ++dl) {
                    W.path.clear();
                    if (W.dfs(S[k], dl)) {
                        Prog cand(best.begin(), best.begin() + k);
                        cand.insert(cand.end(), W.path.begin(), W.path.end());
                        const int u = simulate(g, t, cand);
                        if (u >= 0 && u < N) {
                            cand.resize(u);
                            std::cerr << "p" << pi << " tail k=" << k << " " << N << " -> " << u << "\n";
                            best = cand;
                            improved = true;
                        }
                        break;
                    }
                    if (W.timedOut) break;
                }
            }
            if (improved) continue;
            for (int L = 2; L <= maxL && !improved; ++L)
                for (int k = 0; k + L <= N && !improved; ++k) {
                    if (std::chrono::steady_clock::now() > endAll) break;
                    const int j = k + L;
                    if (S[k].acc != S[j].acc) continue;
                    W.mode = 0;
                    W.tg = &S[j];
                    W.failed.clear();
                    W.timedOut = false;
                    W.deadline = std::min(endAll, std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(std::max(0.05, secs / 200))));
                    W.path.clear();
                    if (W.dfs(S[k], L - 1)) {
                        Prog cand(best.begin(), best.begin() + k);
                        cand.insert(cand.end(), W.path.begin(), W.path.end());
                        cand.insert(cand.end(), best.begin() + j, best.end());
                        const int u = simulate(g, t, cand);
                        if (u >= 0 && u < N) {
                            cand.resize(u);
                            std::cerr << "p" << pi << " window k=" << k << " L=" << L << " " << N << " -> " << u << "\n";
                            best = cand;
                            improved = true;
                        }
                    }
                }
        }
        if (simulate(g, t, best) < 0) { std::cerr << "BUG\n"; return 3; }
        after += best.size();
        if (pi > 0) out << '\n';
        for (const auto& l : best)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
        out.flush();
    }
    std::cerr << "wopt: " << before << " -> " << after << '\n';
}
