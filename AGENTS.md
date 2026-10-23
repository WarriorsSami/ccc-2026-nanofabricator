# AGENTS.md — CCC 2026 "Nanofabricator" (levels 1-6)

## Task (level 6, optimisation)
Per board (hex radius R, target pattern, count): emit a program of lines `x x x x x x` (one of F/B/T/- per emitter 1..6).
Score = total lines over all programs; lower is better. Program valid iff the pattern gets submitted >= count times.
Rules: emitter i starts at rail index (i-1)*R, inactive; F/B move 1 rail cell cw/ccw, T toggles; beams fixed dirs 1→ 2↘ 3↙ 4← 5↖ 6↗.
Per tick: exactly 2 beam hits -> matter; >=3 -> empty; matter persists. Submit when all pattern cells have matter and no matter on halo (non-pattern neighbours); submitted cells clear.
Ground truth simulator: `simulate()` in `level-6/main.cpp` (matches the server on all accepted submissions).

## State store (source of truth): `notebook/`
- `notebook/boards/<input>/<idx>.prog` best program; `<idx>.in` single-board input; `index.tsv` = idx, lines, source.
- `python3 notebook/tools/nb.py status|import-full|import-mini|import-board|assemble|show` — validates, keeps only strictly shorter, git-commits; lock-safe.
- Build validator first: `cd notebook/tools && c++ -std=c++23 -O2 -I. -o validate merge.cpp`.
- Inputs: 1-small 2-medium 3-large 4-enormous. Totals at push: small 107, medium 5991, large 5451, enormous 4861 = 16410 lines.

## Build / run
`c++ -std=c++23 -O2 -march=native -I<repo root> -o sol <file.cpp>`; generators: `./sol <input-id> <dir with in_level-6_*.txt>` -> `<dir>/out/out_level-6_<id>.txt`.

## Code map (level-6)
- `main.cpp` baseline generator: static config search, 2-emitter pen BFS, roll(doubled line)+erase, 6-emitter greedy (4^6 lines/tick, bit-sliced hits, env L6_PARAMS="gamma lambda ...", L6_DEBUG, L6_GREEDY_ONLY).
- `strategies/beam-search/` beam search over ticks, pair-aware reach potential (strongest generator).
- `strategies/anchor-sweep/` doubled anchor line sweep + 4 cutters by beam search, period-2 cycles.
- `strategies/cycles/` periodic cycles (closed walks per emitter) and loops through a fixed config.
- `strategies/cycle-sa/` simulated annealing for steady-state cycles (`cyc`), program builders (`build`, `buildd`).
- `strategies/micro/` window re-planning k->k-1 lines to an equivalent state (`micro`), suffix re-planning (`replan`).
- `strategies/window-opt/` exact window search with lower bounds (`wopt`).
- `strategies/suffix-resolve/` random cut + randomized beam search on the suffix (`resolve`).
- `strategies/hybrid/` variant generator used for the large input.
- `tools/merge.cpp` per-board min over candidate files; `tools/polish.cpp` DSL edits (delete/dissolve/shift lines); `tools/extract.py`/`splice.py` mini-input cut/paste.
Equivalent-state rule for local rewrites: same emitter offsets + active flags + matter on pattern∪halo + accepted count.

## Protocol
- Repo root is the source of truth. Read boards from `notebook/`, improve, import with `nb.py` (it validates + commits). Never edit notebook files by hand.
- Never run `git` (orchestrator commits/pushes) and never run `ccc` (orchestrator submits).
- Scratch: `work/<agent-name>/` (gitignored). Rust code: `rust/<crate>/` (one owner per crate; `rust/nano` = shared core).
- Assignments: `work/assignments/<name>.txt` = `<input-id> idx idx ...` (boards sorted by current cost, descending).
- Binary names must be prefixed with your agent name (a `killall` clash killed another agent's runs).
- CPU: 8 cores shared by all agents -> max 2 concurrent heavy processes per agent; no background loops left running when you stop.

## Rust (`rust/nano`)
Core lib for speed: parse, geometry (cell ids, offFor, beam bitsets), unsafe hot loops (`get_unchecked`, raw ptrs, fixed `[u64; 7]` bitsets), simulator identical to `simulate()`.
Bins: `nano-validate` (same CLI/stderr contract as `notebook/tools/merge.cpp`: `<input> <out> <cand...>` -> `merged total N`), then generators/optimisers ported from C++.
Build: `cargo build --release --manifest-path rust/nano/Cargo.toml`.

## Findings so far
- 6-emitter greedy (all 4^6 lines per tick) >> pen; beam search over ticks with pair-aware reach potential is the strongest generator.
- Period-k cycles (each emitter a closed walk) win big on high-count medium boards; deep searches beyond k=3 found nothing more.
- Window re-planning k->k-1 to an equivalent state: 0.05-0.3% per pass, keeps finding lines after notebook updates.
- Per-board min over many runs/params is where most gains came from; spend compute on the most expensive boards.
