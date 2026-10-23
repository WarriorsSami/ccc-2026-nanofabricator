// Nanofabricator — level 4
//
// `ccc run`    builds this file, runs every input, diffs the example.
// `ccc submit` does the same, then uploads the outputs.
#include "ccc.hpp"

constexpr int LEVEL = 4;

// >>> example input
//   3
//   2
//   3
//   F T T F - -
//   T - - T - -
//   T B - B T -
//   
//   3
//   6
//   B B B B B B
//   - - - T T -
//   - - T - T -
//   - T - T - -
//   T - T - - -
//   - T - - - T
//   … (10 more lines)
// <<< example input

struct Board {
    int radius = 0;
    int rows = 0;
    int cols = 0;
    std::vector<std::string> grid;
};

struct Program {
    int radius = 0;
    // One instruction line per tick, one instruction per emitter.
    std::vector<std::array<char, 6>> lines;
};

struct Input {
    // N, then N programs: radius, line count, then that many instruction lines.
    std::vector<Program> cases;
};

// Reader: next<T>(), line_values<T>(), r >> x, fields(), record<S, Ts...>(),
// set_separators(" ,") for comma-separated input; see Reader in ccc.hpp.
Input parse(Reader& r) {
    Input in;
    const auto n = r.next<i64>();
    for (i64 i = 0; i < n; ++i) {
        auto& prog = in.cases.emplace_back();
        prog.radius = static_cast<int>(r.next<i64>());
        const auto lines = r.next<i64>();
        for (i64 k = 0; k < lines; ++k) {
            auto& line = prog.lines.emplace_back();
            for (char& ins : line) ins = r.next<std::string>()[0];
        }
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

bool isCell(char ch) { return ch == 'O' || ch == '0' || ch == 'M'; }

// Adds one hit to every cell along the beam until it hits a wall or the edge.
void fireLaser(const std::vector<std::string>& grid, std::vector<std::vector<int>>& hits,
               int row, int col, int laser) {
    const auto [dr, dc] = kLaserSteps[laser];
    for (int r = row + dr, c = col + dc;
         r >= 0 && r < static_cast<int>(grid.size()) && c >= 0 && c < static_cast<int>(grid[r].size());
         r += dr, c += dc) {
        if (!isCell(grid[r][c])) break;  // wall or another emitter
        ++hits[r][c];
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

// Border cells in clockwise order, starting at the left corner (emitter 1's
// home). Emitter k starts at index (k - 1) * radius.
std::vector<std::pair<int, int>> borderRing(int radius) {
    const int R = radius;
    std::vector<std::pair<int, int>> ring;
    for (int k = 0; k < R; ++k) ring.push_back({R - k, k});              // left -> top-left
    for (int k = 0; k < R; ++k) ring.push_back({0, R + 2 * k});          // top edge
    for (int k = 0; k < R; ++k) ring.push_back({k, 3 * R + k});          // top-right -> right
    for (int k = 0; k < R; ++k) ring.push_back({R + k, 4 * R - k});      // right -> bottom-right
    for (int k = 0; k < R; ++k) ring.push_back({2 * R, 3 * R - 2 * k});  // bottom edge
    for (int k = 0; k < R; ++k) ring.push_back({2 * R - k, R - k});      // bottom-left -> left
    return ring;
}

void solve(const Input& in, std::ostream& out) {
    for (const auto& prog : in.cases) {
        std::vector<std::string> grid = buildHexPattern(prog.radius);
        const auto ring = borderRing(prog.radius);
        const int ringSize = static_cast<int>(ring.size());

        std::array<int, 6> pos{};
        std::array<bool, 6> active{};
        for (int e = 0; e < 6; ++e) pos[e] = e * prog.radius;

        for (const auto& line : prog.lines) {
            for (int e = 0; e < 6; ++e) {
                switch (line[e]) {
                    case 'F': pos[e] = (pos[e] + 1) % ringSize; break;
                    case 'B': pos[e] = (pos[e] + ringSize - 1) % ringSize; break;
                    case 'T': active[e] = !active[e]; break;
                    default: break;
                }
            }

            std::vector<std::vector<int>> hits(grid.size());
            for (std::size_t r = 0; r < grid.size(); ++r) hits[r].assign(grid[r].size(), 0);
            for (int e = 0; e < 6; ++e)
                if (active[e]) fireLaser(grid, hits, ring[pos[e]].first, ring[pos[e]].second, e + 1);

            // 2 beams create matter, 3+ destroy it, 0-1 leave the cell alone.
            for (std::size_t r = 0; r < grid.size(); ++r)
                for (std::size_t c = 0; c < grid[r].size(); ++c) {
                    if (hits[r][c] == 2)
                        grid[r][c] = 'M';
                    else if (hits[r][c] >= 3)
                        grid[r][c] = 'O';
                }
        }

        for (const auto& row : grid) out << row << '\n';
        out << '\n';
    }
}

int main(int argc, char** argv) {
    return ccc::run_level<Input>(LEVEL, argc, argv, parse, solve);
}
