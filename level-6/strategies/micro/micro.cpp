// Window re-planning: replace k consecutive lines by k-1 lines reaching an
// equivalent state (same config, same matter on pattern+halo, same accepted).
// usage: micro <input.txt> <programs.txt> <out.txt> [maxK=4] [nodeCap=200000]
#define main base_main
#include "base.cpp"
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

using B = std::array<u64, kWords>;
constexpr int NONE = -1000;

struct Micro {
    const Geometry& g;
    const Target& t;
    int W = 1;
    std::vector<int> relIdx;
    std::array<std::vector<B>, 6> lineBits;
    std::array<std::vector<char>, 6> lineRel;
    B P{}, H{};
    i64 nodeCap = 200000;
    int simCap = 0;

    struct St { std::array<int, 6> off{}, on{}; B M{}; int acc = 0; };

    Micro(const Geometry& geo, const Target& tgt) : g(geo), t(tgt) {
        const int n = g.numCells();
        relIdx.assign(n, -1);
        int m = 0;
        for (int id = 0; id < n; ++id)
            if (t.isP[id] || t.isHalo[id]) relIdx[id] = m++;
        W = std::max(1, (m + 63) / 64);
        for (int id = 0; id < n; ++id) {
            if (relIdx[id] < 0) continue;
            const int r = relIdx[id];
            if (t.isP[id]) P[r >> 6] |= u64{1} << (r & 63);
            else H[r >> 6] |= u64{1} << (r & 63);
        }
        const int R = g.R;
        for (int e = 0; e < 6; ++e) {
            lineBits[e].assign(2 * R + 1, B{});
            lineRel[e].assign(2 * R + 1, 0);
            for (int o = -R; o <= R; ++o) {
                for (int id : g.beamAt(e, o))
                    if (relIdx[id] >= 0) lineBits[e][o + R][relIdx[id] >> 6] |= u64{1} << (relIdx[id] & 63), lineRel[e][o + R] = 1;
            }
        }
    }

    int beamOf(int e, int o, int a) const {
        if (!a || std::abs(o) > g.R || !lineRel[e][o + g.R]) return NONE;
        return o;
    }

    // advance one tick given per-emitter beams
    void tick(St& s, const std::array<int, 6>& bm) const {
        B c1{}, c2{}, c3{};
        for (int e = 0; e < 6; ++e) {
            if (bm[e] == NONE) continue;
            const B& L = lineBits[e][bm[e] + g.R];
            for (int w = 0; w < W; ++w) {
                c3[w] |= c2[w] & L[w];
                c2[w] |= c1[w] & L[w];
                c1[w] |= L[w];
            }
        }
        bool complete = true;
        for (int w = 0; w < W; ++w) {
            s.M[w] = (s.M[w] | (c2[w] & ~c3[w])) & ~c3[w];
        }
        for (int w = 0; w < W; ++w) complete = complete && !(P[w] & ~s.M[w]) && !(H[w] & s.M[w]);
        if (complete) {
            for (int w = 0; w < W; ++w) s.M[w] &= ~P[w];
            ++s.acc;
        }
    }

    std::vector<St> states(const Prog& prog) const {
        std::vector<St> res;
        St s;
        res.push_back(s);
        for (const auto& l : prog) {
            for (int e = 0; e < 6; ++e) {
                if (l[e] == 'F') ++s.off[e];
                else if (l[e] == 'B') --s.off[e];
                else if (l[e] == 'T') s.on[e] ^= 1;
                if (s.off[e] > 3 * g.R) s.off[e] -= g.ringSize;
                if (s.off[e] <= -3 * g.R) s.off[e] += g.ringSize;
            }
            std::array<int, 6> bm;
            for (int e = 0; e < 6; ++e) bm[e] = beamOf(e, s.off[e], s.on[e]);
            tick(s, bm);
            res.push_back(s);
        }
        return res;
    }

    struct Walk { std::array<int, 6> beam; std::array<char, 6> acts; int prio; };

    // try to find L lines from S to equivalent of T (or acc>=count if final)
    bool solveWindow(const St& S, const St& T, int L, bool final, const Prog& orig, int i0, int k, Prog& outLines) {
        std::array<std::vector<Walk>, 6> walks;
        int total = 1;
        for (int q = 0; q < L; ++q) total *= 4;
        const char codes[4] = {'-', 'F', 'B', 'T'};
        double prod = 1;
        for (int e = 0; e < 6; ++e) {
            std::string origStr;
            for (int q = 0; q < k; ++q) origStr += orig[i0 + q][e];
            std::map<std::array<int, 6>, Walk> seen;
            for (int w = 0; w < total; ++w) {
                int o = S.off[e], a = S.on[e], x = w;
                Walk wk{};
                wk.beam.fill(NONE);
                std::string str;
                bool okRange = true;
                for (int q = 0; q < L; ++q) {
                    const char c = codes[x & 3];
                    x >>= 2;
                    wk.acts[q] = c;
                    str += c;
                    if (c == 'F') ++o;
                    else if (c == 'B') --o;
                    else if (c == 'T') a ^= 1;
                    if (std::abs(o) > 3 * g.R - 1) okRange = false;
                    wk.beam[q] = beamOf(e, o, a);
                }
                if (!okRange) continue;
                if (!final && (o != T.off[e] || a != T.on[e])) continue;
                // priority: obtainable by deleting one char of orig
                wk.prio = 2;
                for (int d = 0; d < k; ++d) {
                    std::string s2 = origStr.substr(0, d) + origStr.substr(d + 1);
                    if (s2 == str) { wk.prio = (origStr[d] == '-') ? 0 : 1; break; }
                }
                auto it = seen.find(wk.beam);
                if (it == seen.end() || it->second.prio > wk.prio) seen[wk.beam] = wk;
            }
            if (seen.empty()) return false;
            for (auto& [kk, v] : seen) walks[e].push_back(v);
            std::stable_sort(walks[e].begin(), walks[e].end(), [](const Walk& a, const Walk& b) { return a.prio < b.prio; });
            prod *= walks[e].size();
        }
        // order emitters by option count
        std::array<int, 6> ord;
        std::iota(ord.begin(), ord.end(), 0);
        std::sort(ord.begin(), ord.end(), [&](int a, int b) { return walks[a].size() < walks[b].size(); });
        std::array<int, 6> pick{};
        // counts per depth per tick
        std::vector<std::array<std::array<B, 3>, 6>> cnt(7);
        for (int q = 0; q < L; ++q) for (int c = 0; c < 3; ++c) cnt[0][q][c] = B{};
        i64 nodes = 0;
        int sims = 0;
        bool found = false;
        auto rec = [&](auto&& self, int d) -> void {
            if (found || ++nodes > nodeCap) return;
            if (d == 6) {
                St s = S;
                for (int q = 0; q < L; ++q) {
                    const auto& c = cnt[6][q];
                    for (int w = 0; w < W; ++w) s.M[w] = (s.M[w] | (c[1][w] & ~c[2][w])) & ~c[2][w];
                    bool complete = true;
                    for (int w = 0; w < W; ++w) complete = complete && !(P[w] & ~s.M[w]) && !(H[w] & s.M[w]);
                    if (complete) {
                        for (int w = 0; w < W; ++w) s.M[w] &= ~P[w];
                        ++s.acc;
                        if (final && s.acc >= t.count) { found = true; break; }
                    }
                }
                if (!final) {
                    bool same = s.acc == T.acc;
                    for (int w = 0; w < W && same; ++w) same = s.M[w] == T.M[w];
                    if (!same) {
                        if (sims >= simCap || s.acc < T.acc - 1) return;
                        ++sims;
                        Prog cand(orig.begin(), orig.begin() + i0);
                        for (int q = 0; q < L; ++q) {
                            Line l = kNoop;
                            for (int e = 0; e < 6; ++e) l[e] = walks[e][pick[e]].acts[q];
                            cand.push_back(l);
                        }
                        cand.insert(cand.end(), orig.begin() + i0 + k, orig.end());
                        const int used = simulate(g, t, cand);
                        if (used < 0) return;
                    }
                    found = true;
                }
                if (found) {
                    outLines.assign(L, kNoop);
                    for (int e = 0; e < 6; ++e)
                        for (int q = 0; q < L; ++q) outLines[q][e] = walks[e][pick[e]].acts[q];
                }
                return;
            }
            const int e = ord[d];
            for (int i = 0; i < static_cast<int>(walks[e].size()) && !found; ++i) {
                pick[e] = i;
                const Walk& wk = walks[e][i];
                for (int q = 0; q < L; ++q) {
                    if (wk.beam[q] == NONE) { cnt[d + 1][q] = cnt[d][q]; continue; }
                    const B& Lb = lineBits[e][wk.beam[q] + g.R];
                    for (int w = 0; w < W; ++w) {
                        cnt[d + 1][q][2][w] = cnt[d][q][2][w] | (cnt[d][q][1][w] & Lb[w]);
                        cnt[d + 1][q][1][w] = cnt[d][q][1][w] | (cnt[d][q][0][w] & Lb[w]);
                        cnt[d + 1][q][0][w] = cnt[d][q][0][w] | Lb[w];
                    }
                }
                self(self, d + 1);
                if (nodes > nodeCap) return;
            }
        };
        rec(rec, 0);
        return found;
    }

    Prog run(Prog prog, int maxK) {
        bool improved = true;
        while (improved) {
            improved = false;
            for (int k = 1; k <= maxK && !improved; ++k) {
                auto st = states(prog);
                for (int i = static_cast<int>(prog.size()) - k; i >= 0; --i) {
                    const bool final = (i + k == static_cast<int>(prog.size()));
                    Prog lines;
                    if (k == 1) {
                        // delete one line: check equivalence directly
                        Prog cand = prog;
                        cand.erase(cand.begin() + i);
                        const int used = simulate(g, t, cand);
                        if (used >= 0) { cand.resize(used); prog = cand; improved = true; break; }
                        continue;
                    }
                    if (!solveWindow(st[i], st[i + k], k - 1, final, prog, i, k, lines)) continue;
                    Prog cand(prog.begin(), prog.begin() + i);
                    for (auto& l : lines) cand.push_back(l);
                    if (!final) cand.insert(cand.end(), prog.begin() + i + k, prog.end());
                    const int used = simulate(g, t, cand);
                    if (used < 0 || used >= static_cast<int>(prog.size())) continue;
                    cand.resize(used);
                    prog = cand;
                    improved = true;
                    break;
                }
            }
        }
        return prog;
    }
};

int main(int argc, char** argv) {
    if (argc < 4) return 1;
    const int maxK = argc > 4 ? std::stoi(argv[4]) : 4;
    const i64 cap = argc > 5 ? std::stoll(argv[5]) : 200000;
    const int simCap = argc > 6 ? std::stoi(argv[6]) : 0;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    const auto progs = readPrograms(argv[2]);
    std::ofstream out(argv[3]);
    i64 before = 0, after = 0;
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        Prog p = progs.at(k);
        const int used = simulate(g, t, p);
        if (used < 0) { std::cerr << "pattern " << k << " invalid input program\n"; return 2; }
        p.resize(used);
        before += static_cast<i64>(p.size());
        Micro m(g, t);
        m.nodeCap = cap;
        m.simCap = simCap;
        Prog q = m.run(p, maxK);
        if (simulate(g, t, q) < 0 || q.size() > p.size()) q = p;
        after += static_cast<i64>(q.size());
        if (k > 0) out << '\n';
        for (const auto& l : q)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
    }
    std::cerr << "micro: " << before << " -> " << after << '\n';
}
