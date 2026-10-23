// Nanofabricator — level 1
//
// `ccc run`    builds this file, runs every input, diffs the example.
// `ccc submit` does the same, then uploads the outputs.
#include "ccc.hpp"

constexpr int LEVEL = 1;

// >>> example input
//   3
//   3
//   6
//   4
// <<< example input

struct Input {
    // Guessed from the example: N, then N lines (one case per line).
    std::vector<std::vector<i64>> cases;
};

// Reader: next<T>(), line_values<T>(), r >> x, fields(), record<S, Ts...>(),
// set_separators(" ,") for comma-separated input; see Reader in ccc.hpp.
Input parse(Reader& r) {
    Input in;
    const auto n = r.next<i64>();
    for (i64 i = 0; i < n; ++i) in.cases.push_back(r.line_values<i64>());
    return in;
}

std::vector<std::string> buildHexPattern(int radius) {
    const int rows = 2 * radius + 1;
    const int cols = 4 * radius + 1;
    const int centerRow = radius;
    const int centerCol = 2 * radius;

    std::vector<std::string> grid(rows, std::string(cols, '-'));
    std::vector<std::vector<int>> wave(rows, std::vector<int>(cols, -1));

    // A position belongs to the hexagon when it lies within the outline
    // narrowed by one column per row away from the center line.
    auto insideHex = [&](int r, int c) {
        const int dr = std::abs(r - centerRow);
        const int dc = std::abs(c - centerCol);
        return dr <= radius && dc <= 2 * radius - dr;
    };

    std::queue<std::pair<int, int>> frontier;
    frontier.push({centerRow, centerCol});
    wave[centerRow][centerCol] = 0;

    static const int kDr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    static const int kDc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    while (!frontier.empty()) {
        const auto [r, c] = frontier.front();
        frontier.pop();

        const int dr = std::abs(r - centerRow);
        const int dc = std::abs(c - centerCol);
        const bool isCell = (dr + dc) % 2 == 0;
        const bool onWall = dr == radius || dc == 2 * radius - dr;

        if (!isCell)
            grid[r][c] = '-';
        else
            grid[r][c] = onWall ? '*' : 'O';

        for (int k = 0; k < 8; ++k) {
            const int nr = r + kDr[k];
            const int nc = c + kDc[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
            if (wave[nr][nc] != -1 || !insideHex(nr, nc)) continue;
            wave[nr][nc] = wave[r][c] + 1;
            frontier.push({nr, nc});
        }
    }

    return grid;
}

void solve(const Input& in, std::ostream& out) {
    for (const auto& c : in.cases) {
        for (const std::string& row : buildHexPattern(c[0]))
            out << row << '\n';
        out << '\n';
    }
}

int main(int argc, char** argv) {
    return ccc::run_level<Input>(LEVEL, argc, argv, parse, solve);
}
