// Nanofabricator — level 2
//
// `ccc run`    builds this file, runs every input, diffs the example.
// `ccc submit` does the same, then uploads the outputs.
#include "ccc.hpp"

constexpr int LEVEL = 2;

// >>> example input
//   6
//   4 9 17
//   ----*-*-*-*-*----
//   ---*-O-O-O-O-*---
//   --*-O-O-O-O-O-*--
//   -*-O-O-O-O-O-O-*-
//   1-O-O-O-O-O-O-O-*
//   -*-O-O-O-O-O-O-*-
//   --*-O-O-O-O-O-*--
//   ---*-O-O-O-O-*---
//   ----*-*-*-*-*----
//   
//   4 9 17
//   ----*-*-*-*-*----
//   ---2-O-O-O-O-*---
//   … (52 more lines)
// <<< example input

struct Board {
    int radius = 0;
    int rows = 0;
    int cols = 0;
    std::vector<std::string> grid;
};

struct Input {
    // N, then N boards: "radius rows cols" followed by `rows` map lines.
    std::vector<Board> cases;
};

// Reader: next<T>(), line_values<T>(), r >> x, fields(), record<S, Ts...>(),
// set_separators(" ,") for comma-separated input; see Reader in ccc.hpp.
Input parse(Reader& r) {
    Input in;
    const auto n = r.next<i64>();
    for (i64 i = 0; i < n; ++i) {
        auto& board = in.cases.emplace_back();
        r >> board.radius >> board.rows >> board.cols;
        for (int k = 0; k < board.rows; ++k) board.grid.push_back(r.next<std::string>());
    }
    return in;
}

// Laser digit -> step in the char grid. Same-row neighbours sit two columns
// apart (a '-' link between them); diagonal neighbours are one row and one
// column away.
//   1 = →   2 = ↘   3 = ↙   4 = ←   5 = ↖   6 = ↗
constexpr std::array<std::pair<int, int>, 7> kLaserSteps = {{
    {0, 0}, {0, 2}, {1, 1}, {1, -1}, {0, -2}, {-1, -1}, {-1, 1},
}};

bool isLaser(char ch) { return ch >= '1' && ch <= '6'; }

// Marks every cell along the beam with 'M' until it hits a wall or the edge.
void fireLaser(std::vector<std::string>& grid, int row, int col, int laser) {
    const auto [dr, dc] = kLaserSteps[laser];
    for (int r = row + dr, c = col + dc;
         r >= 0 && r < static_cast<int>(grid.size()) && c >= 0 && c < static_cast<int>(grid[r].size());
         r += dr, c += dc) {
        char& ch = grid[r][c];
        if (ch == 'O' || ch == '0')
            ch = 'M';
        else if (ch != 'M')
            break;  // wall or another laser
    }
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
    for (const auto& board : in.cases) {
        std::vector<std::string> grid = board.grid;

        // Collect lasers before firing so beams never trigger each other.
        std::vector<std::tuple<int, int, int>> lasers;
        for (int r = 0; r < static_cast<int>(grid.size()); ++r)
            for (int c = 0; c < static_cast<int>(grid[r].size()); ++c)
                if (isLaser(grid[r][c])) lasers.emplace_back(r, c, grid[r][c] - '0');

        for (const auto& [r, c, laser] : lasers) fireLaser(grid, r, c, laser);

        for (const auto& row : grid) out << row << '\n';
        out << '\n';
    }
}

int main(int argc, char** argv) {
    return ccc::run_level<Input>(LEVEL, argc, argv, parse, solve);
}
