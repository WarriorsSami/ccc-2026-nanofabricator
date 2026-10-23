// DSL-level optimiser: takes finished programs (F/B/T/- lines) and shortens
// them with edits checked by the exact simulator — no regeneration needed.
// usage: polish <input.txt> <programs.txt> <out.txt> [seconds_per_program=0.5]
#define main base_main
#include "../main.cpp"
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

struct Polisher {
    const Geometry& g;
    const Target& t;
    Prog best;
    std::mt19937 rng{12345};

    bool accept(Prog cand) {
        const int used = simulate(g, t, cand);
        if (used < 0) return false;
        cand.resize(used);
        if (cand.size() > best.size()) return false;
        best = std::move(cand);
        return true;
    }

    static bool isNoop(const Line& l) { return l == kNoop; }

    // Remove line i, re-homing its instructions into free slots of nearby
    // lines (nearest first, in the given direction preference).
    bool dissolve(std::size_t i, int maxDist, bool preferEarlier) {
        Prog cand = best;
        const Line moved = cand[i];
        for (int e = 0; e < 6; ++e) {
            if (moved[e] == '-') continue;
            bool placed = false;
            for (int d = 1; d <= maxDist && !placed; ++d)
                for (int s : {preferEarlier ? -d : d, preferEarlier ? d : -d}) {
                    const long j = static_cast<long>(i) + s;
                    if (j < 0 || j >= static_cast<long>(cand.size()) || cand[j][e] != '-') continue;
                    cand[j][e] = moved[e];
                    placed = true;
                    break;
                }
            if (!placed) return false;
        }
        cand.erase(cand.begin() + static_cast<long>(i));
        return accept(std::move(cand));
    }

    bool deterministicPass() {
        bool any = false;
        for (std::size_t i = best.size(); i-- > 0;) {
            if (i >= best.size()) continue;
            Prog cand = best;
            cand.erase(cand.begin() + static_cast<long>(i));
            if (accept(std::move(cand))) { any = true; continue; }
            for (int dist : {1, 2, 4})
                if (dissolve(i, dist, true) || dissolve(i, dist, false)) { any = true; break; }
        }
        return any;
    }

    // Random neutral moves (shift one instruction to a free slot nearby) that
    // may unlock later deletions.
    void randomSearch(double seconds) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        while (std::chrono::steady_clock::now() < deadline && best.size() > 1) {
            Prog cand = best;
            const int moves = 1 + static_cast<int>(rng() % 3);
            for (int m = 0; m < moves; ++m) {
                const std::size_t i = rng() % cand.size();
                const int e = static_cast<int>(rng() % 6);
                if (cand[i][e] == '-') continue;
                const int s = static_cast<int>(rng() % 5) - 2;
                const long j = static_cast<long>(i) + (s >= 0 ? s + 1 : s);
                if (j < 0 || j >= static_cast<long>(cand.size()) || cand[j][e] != '-') continue;
                std::swap(cand[i][e], cand[j][e]);
            }
            const std::size_t before = best.size();
            Prog keep = best;
            if (!accept(cand)) continue;
            // Neutral move accepted; see if it unlocks a shortening.
            if (!deterministicPass() && best.size() == before && (rng() % 4 != 0)) best = keep;
        }
    }
};

int main(int argc, char** argv) {
    if (argc < 4) return 1;
    const double seconds = argc > 4 ? std::stod(argv[4]) : 0.5;
    std::ifstream in(argv[1]);
    ccc::Reader r(in);
    const Input input = parse(r);
    const auto progs = readPrograms(argv[2]);
    std::ofstream out(argv[3]);
    i64 before = 0, after = 0;
    for (std::size_t k = 0; k < input.cases.size(); ++k) {
        const Geometry g(input.cases[k].radius);
        const Target t(g, input.cases[k]);
        Polisher p{g, t, progs.at(k)};
        const int used = simulate(g, t, p.best);
        if (used < 0) { std::cerr << "pattern " << k << " invalid input program\n"; return 2; }
        p.best.resize(used);
        before += static_cast<i64>(p.best.size());
        while (p.deterministicPass()) {}
        if (seconds > 0) p.randomSearch(seconds);
        if (simulate(g, t, p.best) < 0) { std::cerr << "BUG: invalid result\n"; return 3; }
        after += static_cast<i64>(p.best.size());
        if (k > 0) out << '\n';
        for (const auto& l : p.best)
            for (int e = 0; e < 6; ++e) out << l[e] << (e < 5 ? ' ' : '\n');
    }
    std::cerr << "polish: " << before << " -> " << after << '\n';
}
