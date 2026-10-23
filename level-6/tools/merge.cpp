// Per-pattern merge: for each pattern, pick the shortest program among the
// candidate output files that the exact simulator accepts.
// usage: merge <input.txt> <out.txt> <candidate1.txt> [candidate2.txt ...]
#define main base_main
#include "../main.cpp"
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

int main(int argc, char** argv) {
    if (argc < 4) return 1;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    std::vector<std::vector<Prog>> cands;
    for (int i = 3; i < argc; ++i) cands.push_back(readPrograms(argv[i]));
    std::ofstream out(argv[2]);
    i64 total = 0;
    std::vector<int> wins(cands.size(), 0);
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        Prog best;
        int who = -1;
        for (std::size_t c = 0; c < cands.size(); ++c) {
            if (k >= cands[c].size()) continue;
            Prog p = cands[c][k];
            const int used = simulate(g, t, p);
            if (used < 0) continue;
            p.resize(used);
            if (best.empty() || p.size() < best.size()) best = p, who = static_cast<int>(c);
        }
        if (best.empty()) { std::cerr << "pattern " << k << ": no valid candidate\n"; return 2; }
        ++wins[who];
        total += static_cast<i64>(best.size());
        if (k > 0) out << '\n';
        for (const auto& l : best)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
    }
    std::cerr << "merged total " << total << " | wins per candidate:";
    for (int w : wins) std::cerr << ' ' << w;
    std::cerr << '\n';
}
