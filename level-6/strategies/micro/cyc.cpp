// Deep periodic-cycle search against the current best programs.
// usage: cyc <input> <programs> <out> <k> <nodeCap> <part> <nparts>
#define main base_main
#include "base.cpp"
#undef main
#include "cycle.inc"

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
    if (argc < 8) return 1;
    const int K = std::stoi(argv[4]);
    const i64 cap = std::stoll(argv[5]);
    const int part = std::stoi(argv[6]), nparts = std::stoi(argv[7]);
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    const auto progs = readPrograms(argv[2]);
    std::ofstream out(argv[3]);
    i64 before = 0, after = 0;
    // order by potential gain
    std::vector<std::pair<int, std::size_t>> order;
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const int cur = static_cast<int>(progs.at(k).size());
        order.push_back({-(cur - K * input.cases[k].count), k});
    }
    std::sort(order.begin(), order.end());
    int pos = 0;
    for (auto [negGain, k] : order) {
        const bool mine = (pos++ % nparts) == part;
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        Prog best = progs.at(k);
        const int used = simulate(g, t, best);
        if (used < 0) return 2;
        best.resize(used);
        before += static_cast<i64>(best.size());
        static const int maxP = std::getenv("CYC_MAXP") ? std::atoi(std::getenv("CYC_MAXP")) : 1000;
        if (mine && static_cast<int>(t.cells.size()) <= maxP && t.count > 1 && K * (t.count - 1) + 1 < static_cast<int>(best.size())) {
            CycleSearch cs(g, t, K);
            cs.nodeLimit = cap;
            cs.maxFound = 60;
            cs.run();
            for (const auto& ch : cs.found)
                for (int rot = 0; rot < K; ++rot) {
                    Prog p = cs.build(ch, rot, t.count);
                    const int u = simulate(g, t, p);
                    if (u < 0) continue;
                    p.resize(u);
                    if (p.size() < best.size()) {
                        best = p;
                        std::cerr << "pattern " << k << " improved to " << best.size() << std::endl;
                        std::ofstream f(std::string(argv[3]) + "_" + std::to_string(k) + ".prog");
                        for (const auto& l : best)
                            for (int e = 0; e < 6; ++e) f << l[e] << (e < 5 ? ' ' : '\n');
                    }
                }
        }
        after += static_cast<i64>(best.size());
    }
    std::cerr << "cyc: " << before << " -> " << after << '\n';
}
