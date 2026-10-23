// Loop extraction: find state recurrences (same config + same matter on
// pattern/halo) inside candidate programs; a recurrence with m submissions in
// len lines is a loop. Combine the cheapest way to reach the loop state (any
// candidate's prefix) with repetitions of the loop.
// usage: loopx <input> <out> <cand1> [cand2 ...]
#define main base_main
#include "base.cpp"
#undef main

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

struct Snap { std::vector<int> key; int acc; };

// states before each tick (index 0 = initial) ... after tick i at index i+1
std::vector<Snap> trace(const Geometry& g, const Target& t, const Prog& prog) {
    std::array<int, 6> off{}, on{};
    std::vector<char> matter(g.numCells(), 0);
    std::vector<int> hits(g.numCells(), 0);
    int acc = 0;
    std::vector<Snap> res;
    auto snap = [&]() {
        Snap s;
        for (int e = 0; e < 6; ++e) s.key.push_back(off[e]), s.key.push_back(on[e]);
        int word = 0, bit = 0;
        for (int id = 0; id < g.numCells(); ++id) {
            if (!t.isP[id] && !t.isHalo[id]) continue;
            if (matter[id]) word |= 1 << bit;
            if (++bit == 30) s.key.push_back(word), word = 0, bit = 0;
        }
        s.key.push_back(word);
        s.acc = acc;
        res.push_back(s);
    };
    snap();
    for (const auto& l : prog) {
        for (int e = 0; e < 6; ++e) {
            if (l[e] == 'F') ++off[e];
            else if (l[e] == 'B') --off[e];
            else if (l[e] == 'T') on[e] ^= 1;
            if (off[e] > 3 * g.R) off[e] -= g.ringSize;
            if (off[e] <= -3 * g.R) off[e] += g.ringSize;
        }
        std::vector<int> touched;
        for (int e = 0; e < 6; ++e) {
            if (!on[e] || std::abs(off[e]) > g.R) continue;
            for (int id : g.beamAt(e, off[e])) if (hits[id]++ == 0) touched.push_back(id);
        }
        for (int id : touched) {
            if (hits[id] == 2) matter[id] = 1;
            else if (hits[id] >= 3) matter[id] = 0;
            hits[id] = 0;
        }
        bool present = true;
        for (int id : t.cells) present = present && matter[id];
        if (present) for (int id = 0; id < g.numCells() && present; ++id) present = !(t.isHalo[id] && matter[id]);
        if (present) { for (int id : t.cells) matter[id] = 0; ++acc; }
        snap();
    }
    return res;
}

int main(int argc, char** argv) {
    if (argc < 4) return 1;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    std::vector<std::vector<Prog>> cands;
    for (int i = 3; i < argc; ++i) cands.push_back(readPrograms(argv[i]));
    std::ofstream out(argv[2]);
    i64 before = 0, after = 0;
    int improvedCnt = 0;
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        std::vector<Prog> progs;
        Prog best;
        for (auto& c : cands) {
            if (k >= c.size()) continue;
            Prog p = c[k];
            // run without count limit to observe loops beyond the end? keep as is
            const int used = simulate(g, t, p);
            if (used < 0) continue;
            p.resize(used);
            if (best.empty() || p.size() < best.size()) best = p;
            progs.push_back(p);
        }
        if (best.empty()) { std::cerr << "no valid for " << k << "\n"; return 2; }
        before += static_cast<i64>(best.size());
        // reach map: key -> best (tick, acc) list
        std::map<std::vector<int>, std::vector<std::tuple<int, int, int>>> reach;  // (tick, acc, prog)
        std::vector<std::vector<Snap>> traces;
        for (std::size_t pi = 0; pi < progs.size(); ++pi) {
            traces.push_back(trace(g, t, progs[pi]));
            const auto& tr = traces.back();
            for (int i = 0; i < static_cast<int>(tr.size()); ++i) reach[tr[i].key].push_back({i, tr[i].acc, static_cast<int>(pi)});
        }
        // loops
        struct LoopC { int pi, a, b, m; };
        std::vector<LoopC> loops;
        for (std::size_t pi = 0; pi < progs.size(); ++pi) {
            const auto& tr = traces[pi];
            std::map<std::vector<int>, std::vector<int>> occ;
            for (int i = 0; i < static_cast<int>(tr.size()); ++i) occ[tr[i].key].push_back(i);
            for (auto& [key, v] : occ)
                for (std::size_t x = 0; x < v.size(); ++x)
                    for (std::size_t y = x + 1; y < v.size(); ++y) {
                        const int m = tr[v[y]].acc - tr[v[x]].acc;
                        if (m > 0) loops.push_back({static_cast<int>(pi), v[x], v[y], m});
                    }
        }
        struct Est { int total; int pi, a, b, m, rp, rt, ra; };
        std::vector<Est> ests;
        for (const auto& L : loops) {
            const auto& key = traces[L.pi][L.a].key;
            for (const auto& [tick, acc, rp] : reach[key]) {
                if (acc >= t.count) continue;
                const int len = L.b - L.a;
                const int reps = (t.count - acc + L.m - 1) / L.m;
                const int est = tick + reps * len;
                if (est - len < static_cast<int>(best.size())) ests.push_back({est, L.pi, L.a, L.b, L.m, rp, tick, acc});
            }
        }
        std::sort(ests.begin(), ests.end(), [](const Est& x, const Est& y) { return x.total < y.total; });
        int tries = 0;
        for (const auto& E : ests) {
            if (++tries > 200) break;
            if (E.total - (E.b - E.a) >= static_cast<int>(best.size())) break;
            Prog cand(progs[E.rp].begin(), progs[E.rp].begin() + E.rt);
            const int reps = (t.count - E.ra + E.m - 1) / E.m + 1;
            for (int q = 0; q < reps; ++q) cand.insert(cand.end(), progs[E.pi].begin() + E.a, progs[E.pi].begin() + E.b);
            const int used = simulate(g, t, cand);
            if (used < 0) continue;
            cand.resize(used);
            if (cand.size() < best.size()) best = cand;
        }
        if (static_cast<i64>(best.size()) < before - after - 0) {}
        after += static_cast<i64>(best.size());
        if (k > 0) out << '\n';
        for (const auto& l : best)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
    }
    std::cerr << "loopx: " << before << " -> " << after << '\n';
}
