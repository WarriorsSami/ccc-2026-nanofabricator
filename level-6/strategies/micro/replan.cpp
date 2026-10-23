// Suffix re-planning: keep the program up to submission j, then beam-search
// the remaining copies from that configuration. usage:
// replan <input> <programs> <out> [width=30] [keep=12] [maxStarts=4]
#define main base_main
#include "base.cpp"
#undef main
#include <random>
#include "beam.inc"

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
    const int width = argc > 4 ? std::stoi(argv[4]) : 30;
    const int keep = argc > 5 ? std::stoi(argv[5]) : 12;
    const int maxStarts = argc > 6 ? std::stoi(argv[6]) : 4;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    const auto progs = readPrograms(argv[2]);
    std::ofstream out(argv[3]);
    i64 before = 0, after = 0;
    const std::vector<std::pair<double, double>> params = {{0.5, 0.9}, {0.25, 0.9}};
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        Prog best = progs.at(k);
        const int used = simulate(g, t, best);
        if (used < 0) { std::cerr << "pattern " << k << " invalid input program\n"; return 2; }
        best.resize(used);
        before += static_cast<i64>(best.size());
        // submission ticks and configs
        Target big = t;
        big.count = INT_MAX;
        Builder bb(g, big);
        std::vector<std::pair<int, std::pair<std::array<int, 6>, std::array<int, 6>>>> subs;  // (prefix length, config)
        subs.push_back({0, {{}, {}}});
        for (std::size_t i = 0; i < best.size(); ++i) {
            const int b4 = bb.accepted;
            bb.step(best[i]);
            if (bb.accepted != b4) {
                std::array<int, 6> o{}, a{};
                for (int e = 0; e < 6; ++e) o[e] = bb.off[e], a[e] = bb.on[e];
                subs.push_back({static_cast<int>(i + 1), {o, a}});
            }
        }
        subs.pop_back();  // last submission = end
        // pick starting points: last maxStarts submissions + start
        std::vector<int> js;
        for (int j = static_cast<int>(subs.size()) - 1; j >= 0 && static_cast<int>(js.size()) < maxStarts; --j) js.push_back(j);
        if (std::find(js.begin(), js.end(), 0) == js.end()) js.push_back(0);
        const Prog orig = best;
        for (int j : js) {
            const int pre = subs[j].first;
            if (pre + 1 >= static_cast<int>(best.size())) continue;
            for (const auto& [gamma, lambda] : params) {
                BeamSearch bs(g, t, gamma);
                bs.lambda = lambda, bs.width = width, bs.keepPer = keep;
                bs.startOff = subs[j].second.first, bs.startOn = subs[j].second.second;
                bs.startAcc = j;
                Prog suf = bs.run();
                if (suf.empty()) continue;
                Prog cand(orig.begin(), orig.begin() + pre);
                cand.insert(cand.end(), suf.begin(), suf.end());
                const int u = simulate(g, t, cand);
                if (u < 0) continue;
                cand.resize(u);
                if (cand.size() < best.size()) best = cand;
            }
        }
        after += static_cast<i64>(best.size());
        if (k > 0) out << '\n';
        for (const auto& l : best)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
    }
    std::cerr << "replan: " << before << " -> " << after << '\n';
}
