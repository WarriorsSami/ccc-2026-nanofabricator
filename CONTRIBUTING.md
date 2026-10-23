# CONTRIBUTING.md

Contributions = shorter valid programs for boards in `notebook/boards/<input>/<idx>.prog`.

1. Fork, then build the validator: `cd notebook/tools && c++ -std=c++23 -O2 -I. -o validate merge.cpp`.
2. Produce a program (any method; see AGENTS.md code map), then `python3 notebook/tools/nb.py import-board <input> <idx> <program.txt> <your-handle>`.
   It validates, keeps it only if strictly shorter, updates `index.tsv`, and commits.
3. Open a PR touching only `notebook/boards/**` (code/strategy PRs: separate PR, no board changes).

PR gate (`.github/workflows/validate-boards.yml`, validator built from the base branch):
- every changed `.prog` is valid and strictly shorter than on `main`;
- `index.tsv` line counts match; `.in` files and the board set are unchanged.
Conflicts on `index.tsv`: rebase on `main` and re-run `nb.py import-board` for your boards.
