// Suffix re-solve: take the current program for one pattern, cut it at a
// random tick, and run a randomized beam search from that state; keep any
// strictly shorter valid program.
// usage: resolve <mini input> <programs> <idx> <seconds> <outfile>
#define main base_main
#include "base2.cpp"
#undef main
#include <random>

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

struct BP2 {
    int K = 20, D = 2;
    double lw[8];
    double mu = 0.9, gamma = 0.5, Z = 1e9, noise = 0;
    u64 seed = 1;
    int stuck = 0;
};

struct InitState {
    std::array<int, 6> off;
    int on;
    int acc;
    std::vector<int> matterP;  // pattern cell ids with matter
};

template <int W>
Prog beamFrom(const Geometry& g, const Target& t, const BP2& bp, const InitState& init, int maxLen) {
    const int R = g.R, n = g.numCells(), span = 2 * R + 1;
    const int D = bp.D;
    using B = BW<W>;
    auto zero = [] { B b; for (int i = 0; i < W; ++i) b.w[i] = 0; return b; };
    auto setb = [](B& b, int i) { b.w[i >> 6] |= u64{1} << (i & 63); };
    std::vector<B> lineMask(6 * span, zero());
    for (int e = 0; e < 6; ++e)
        for (int o = -R; o <= R; ++o)
            for (int id : g.beamAt(e, o)) setb(lineMask[e * span + o + R], id);
    const int DD = std::max(D, 1);
    std::vector<B> reach(6 * span * 2 * DD, zero());
    for (int e = 0; e < 6; ++e)
        for (int o = -R; o <= R; ++o)
            for (int on = 0; on < 2; ++on)
                for (int d = 1; d <= D; ++d) {
                    B& m = reach[((e * span + o + R) * 2 + on) * DD + d - 1];
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
            if (e2 & halo.w[i]) return false;
            out.m.w[i] = ((out.m.w[i] & ~g3.w[i]) | e2) & P.w[i];
            if (out.m.w[i] != P.w[i]) full = false;
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
    State s0;
    for (int e = 0; e < 6; ++e) s0.off[e] = static_cast<int8_t>(init.off[e]);
    s0.on = static_cast<uint8_t>(init.on);
    s0.acc = static_cast<uint8_t>(init.acc);
    s0.m = zero();
    for (int id : init.matterP) setb(s0.m, id);
    std::vector<State> beam = {s0};
    std::vector<std::vector<std::pair<int, uint16_t>>> history;
    const int K = bp.K;
    std::vector<Cand> cands;
    cands.reserve(1 << 16);

    struct Opt { int8_t off; uint8_t on; uint8_t code; double phi; };
    std::array<std::array<Opt, 4>, 6> opts;
    std::array<int, 6> numOpts{};
    std::array<B, 7> l1, l2, l3;
    std::vector<std::array<B, 7>> a1(DD), a2(DD);
    std::vector<int> needCells;

    u64 rng = bp.seed * 0x9e3779b97f4a7c15ULL + 88172645463325252ULL;
    if (!rng) rng = 1;
    int lastProg = -1;
    std::size_t lastImprove = 0;
    const int stuckTicks = bp.stuck > 0 ? bp.stuck : 2 * R;
    auto reconstruct = [&](int depthCount, int idx) {
        Prog prog(depthCount);
        for (int dd = depthCount - 1; dd >= 0; --dd) {
            const auto [par, cd] = history[dd][idx];
            Line line = kNoop;
            const char codes[4] = {'-', 'F', 'B', 'T'};
            for (int e = 0; e < 6; ++e) line[e] = codes[(cd >> (2 * e)) & 3];
            prog[dd] = line;
            idx = par;
        }
        return prog;
    };
    for (std::size_t depth = 0;; ++depth) {
        if (static_cast<int>(depth) >= maxLen) return {};
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
                    op.phi = phi * phiNorm;
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
                for (int i = 0; i < W; ++i) {
                    const u64 e2 = l2[6].w[i] & ~l3[6].w[i];
                    if (e2 & halo.w[i]) return;
                    nm.w[i] = ((s.m.w[i] & ~l3[6].w[i]) | e2) & P.w[i];
                    if (nm.w[i] != P.w[i]) full = false;
                }
                double score;
                if (full) {
                    score = accBase + (Pn + 1);
                    need = P;
                } else {
                    score = accBase;
                    for (int i = 0; i < W; ++i) {
                        stamped += std::popcount(nm.w[i]);
                        need.w[i] = P.w[i] & ~nm.w[i];
                    }
                    score += stamped;
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
                score += pot + phi;
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
                        const B& Q = reach[((e * span + op.off + R) * 2 + op.on) * DD + d];
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
        if (cands.empty()) return {};
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.score > b.score; });
        {
            int bestProg = -1;
            for (const State& s : beam) {
                int c = s.acc * (Pn + 1);
                for (int i = 0; i < W; ++i) c += std::popcount(s.m.w[i]);
                bestProg = std::max(bestProg, c);
            }
            if (bestProg > lastProg) lastProg = bestProg, lastImprove = depth;
            if (static_cast<int>(depth - lastImprove) > stuckTicks) return {};
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
                history.push_back(hist);
                return reconstruct(static_cast<int>(depth) + 1, static_cast<int>(next.size()) - 1);
            }
        }
        history.push_back(std::move(hist));
        beam = std::move(next);
    }
}

Prog beamFromAny(const Geometry& g, const Target& t, const BP2& bp, const InitState& init, int maxLen) {
    switch ((g.numCells() + 63) / 64) {
        case 1: return beamFrom<1>(g, t, bp, init, maxLen);
        case 2: return beamFrom<2>(g, t, bp, init, maxLen);
        case 3: return beamFrom<3>(g, t, bp, init, maxLen);
        case 4: return beamFrom<4>(g, t, bp, init, maxLen);
        case 5: return beamFrom<5>(g, t, bp, init, maxLen);
        case 6: return beamFrom<6>(g, t, bp, init, maxLen);
        case 7: return beamFrom<7>(g, t, bp, init, maxLen);
        default: return {};
    }
}

// Simulate the first k lines; return false if state is not representable.
bool stateAt(const Geometry& g, const Target& t, const Prog& prog, int k, InitState& st) {
    std::array<int, 6> off{};
    std::array<bool, 6> active{};
    std::vector<char> matter(g.numCells(), 0);
    std::vector<int> hits(g.numCells(), 0);
    int accepted = 0;
    for (int tick = 0; tick < k; ++tick) {
        for (int e = 0; e < 6; ++e) {
            switch (prog[tick][e]) {
                case 'F': ++off[e]; break;
                case 'B': --off[e]; break;
                case 'T': active[e] = !active[e]; break;
                default: break;
            }
            if (off[e] > 3 * g.R) off[e] -= g.ringSize;
            if (off[e] <= -3 * g.R) off[e] += g.ringSize;
        }
        std::vector<int> touched;
        for (int e = 0; e < 6; ++e) {
            if (!active[e] || std::abs(off[e]) > g.R) continue;
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
        if (present)
            for (int id = 0; id < g.numCells() && present; ++id) present = !(t.isHalo[id] && matter[id]);
        if (present) {
            for (int id : t.cells) matter[id] = 0;
            ++accepted;
        }
    }
    for (int e = 0; e < 6; ++e)
        if (std::abs(off[e]) > g.R) return false;
    for (int id = 0; id < g.numCells(); ++id)
        if (t.isHalo[id] && matter[id]) return false;
    st.off = off;
    st.on = 0;
    for (int e = 0; e < 6; ++e) st.on |= active[e] << e;
    st.acc = accepted;
    st.matterP.clear();
    for (int id : t.cells)
        if (matter[id]) st.matterP.push_back(id);
    return accepted < t.count;
}

void writeProg(const std::string& path, const Prog& p) {
    std::ofstream o(path + ".tmp");
    for (const auto& l : p)
        for (int e = 0; e < 6; ++e) o << l[e] << (e < 5 ? ' ' : '\n');
    o.close();
    std::rename((path + ".tmp").c_str(), path.c_str());
}

int main(int argc, char** argv) {
    if (argc < 6) return 1;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    auto progs = readPrograms(argv[2]);
    const int idx = std::atoi(argv[3]);
    const double secs = std::atof(argv[4]);
    const std::string outPath = argv[5];
    const u64 seed0 = argc > 6 ? std::strtoull(argv[6], nullptr, 10) : 1;
    const Pattern& p = input.cases[idx];
    const Geometry g(p.radius);
    const Target t(g, p);
    Prog best = progs[idx];
    int used = simulate(g, t, best);
    if (used < 0) { std::cerr << "invalid start\n"; return 2; }
    best.resize(used);
    std::cerr << "pattern " << idx << " start " << best.size() << "\n";
    writeProg(outPath, best);
    std::mt19937_64 rng(seed0 * 7919 + idx);
    auto t0 = std::chrono::steady_clock::now();
    int iters = 0;
    const char* modeEnv = std::getenv("RS_MODE");
    const int mode = modeEnv ? std::atoi(modeEnv) : 0;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < secs) {
        ++iters;
        const int L = static_cast<int>(best.size());
        int k;
        double u = std::uniform_real_distribution<double>(0, 1)(rng);
        if (mode == 1) k = 0;
        else k = static_cast<int>(u * u * 0 + u * (L - 2));
        if (k < 0) k = 0;
        InitState st;
        if (!stateAt(g, t, best, k, st)) continue;
        BP2 bp;
        const int Ks[] = {24, 32, 48, 64, 96, 32, 48};
        bp.K = Ks[rng() % 7];
        const int Ds[] = {0, 1, 2, 2, 3, 3};
        bp.D = Ds[rng() % 6];
        const double mus[] = {0.3, 0.6, 0.9, 1.2, 2.0};
        bp.mu = mus[rng() % 5];
        const double lws[] = {0.1, 0.2, 0.3, 0.5};
        double x = lws[rng() % 4];
        for (int d = 0; d < 8; ++d) bp.lw[d] = x * std::pow(0.5, d);
        const double Zs[] = {5, 10, 20, 1e9};
        bp.Z = Zs[rng() % 4];
        const double noises[] = {0, 0, 0.05, 0.2, 0.5, 1.0};
        bp.noise = noises[rng() % 6];
        const double gammas[] = {0.3, 0.5, 0.7};
        bp.gamma = gammas[rng() % 3];
        bp.seed = rng();
        // restrict expensive configs for early cuts
        Prog suf = beamFromAny(g, t, bp, st, L - k - 1);
        if (suf.empty()) continue;
        Prog cand(best.begin(), best.begin() + k);
        cand.insert(cand.end(), suf.begin(), suf.end());
        int u2 = simulate(g, t, cand);
        if (u2 < 0) { std::cerr << "BUG invalid cand\n"; continue; }
        cand.resize(u2);
        if (cand.size() < best.size()) {
            best = cand;
            writeProg(outPath, best);
            std::cerr << "pattern " << idx << " -> " << best.size() << " (cut " << k << " K" << bp.K << " D" << bp.D
                      << " mu" << bp.mu << " lw" << x << " Z" << bp.Z << " nz" << bp.noise << " g" << bp.gamma << ") t="
                      << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << "\n";
        }
    }
    std::cerr << "pattern " << idx << " final " << best.size() << " iters " << iters << "\n";
}
