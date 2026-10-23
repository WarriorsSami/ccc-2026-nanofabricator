#!/usr/bin/env python3
"""Level-6 notebook: the best known program for every board of every input.

Every candidate is validated with the exact simulator (tools/validate) and only
replaces the stored program when it is strictly shorter. Each import is one git
commit. A file lock makes concurrent imports from several agents safe.

usage:
  nb.py status                                              totals per input
  nb.py import-full  <input-id> <full_output.txt> <source>  one program per board, whole input
  nb.py import-mini  <input-id> <mini_output.txt> <source> <idx> [idx ...]
                                                            programs for the listed boards, in that order
  nb.py import-board <input-id> <idx> <program.txt> <source>
  nb.py assemble <outdir>                                   writes out_level-6_<input-id>.txt files
  nb.py show <input-id> <idx>                               board, current program and its source

input-id is one of: 1-small 2-medium 3-large 4-enormous; idx is the 0-based board index.
"""
import fcntl
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INPUTS = ['1-small', '2-medium', '3-large', '4-enormous']
VALIDATE = os.path.join(ROOT, 'tools', 'validate')


def board_dir(inp):
    return os.path.join(ROOT, 'boards', inp)


def split_programs(text):
    return [p.strip('\n') for p in text.split('\n\n') if p.strip()]


def read_index(inp):
    index = {}
    path = os.path.join(board_dir(inp), 'index.tsv')
    if os.path.exists(path):
        for line in open(path):
            if line.startswith('#') or not line.strip():
                continue
            idx, lines, source = line.rstrip('\n').split('\t')
            index[int(idx)] = (int(lines), source)
    return index


def write_index(inp, index):
    with open(os.path.join(board_dir(inp), 'index.tsv'), 'w') as f:
        f.write('# board\tlines\tsource\n')
        for idx in sorted(index):
            f.write(f'{idx}\t{index[idx][0]}\t{index[idx][1]}\n')


def ensure_board_inputs(inp):
    """Splits the input into one single-board input file per board."""
    d = board_dir(inp)
    os.makedirs(d, exist_ok=True)
    L = open(os.path.join(ROOT, 'inputs', f'in_level-6_{inp}.txt')).read().split('\n')
    n, i = int(L[0]), 1
    for k in range(n):
        while not L[i].strip():
            i += 1
        rows = int(L[i].split()[1])
        path = os.path.join(d, f'{k:03d}.in')
        if not os.path.exists(path):
            open(path, 'w').write('1\n' + '\n'.join(L[i:i + 1 + rows]) + '\n')
        i += 1 + rows
    return n


def validate(inp, idx, program):
    """Returns (lines, normalised program) if valid, else None."""
    with tempfile.TemporaryDirectory() as tmp:
        cand = os.path.join(tmp, 'cand.txt')
        out = os.path.join(tmp, 'out.txt')
        open(cand, 'w').write(program.strip('\n') + '\n')
        res = subprocess.run([VALIDATE, os.path.join(board_dir(inp), f'{idx:03d}.in'), out, cand],
                             capture_output=True, text=True)
        if res.returncode != 0 or 'merged total' not in res.stderr:
            return None
        lines = int(res.stderr.split('merged total')[1].split()[0])
        return lines, open(out).read().strip('\n')


def git(*args):
    return subprocess.run(['git', '-C', ROOT, *args], capture_output=True, text=True)


def import_programs(inp, pairs, source):
    """pairs: list of (idx, program text). Keeps strictly shorter valid ones and commits."""
    lock = open(os.path.join(ROOT, '.lock'), 'w')
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        ensure_board_inputs(inp)
        index = read_index(inp)
        improved, saved, invalid = 0, 0, 0
        for idx, prog in pairs:
            cur = index.get(idx, (10 ** 9, ''))[0]
            if len(prog.strip().split('\n')) >= cur:
                continue  # cannot be shorter even before validation trims it
            res = validate(inp, idx, prog)
            if res is None:
                invalid += 1
                continue
            lines, norm = res
            if lines < cur:
                open(os.path.join(board_dir(inp), f'{idx:03d}.prog'), 'w').write(norm + '\n')
                saved += 0 if cur == 10 ** 9 else cur - lines
                index[idx] = (lines, source)
                improved += 1
        write_index(inp, index)
        total = sum(v[0] for v in index.values())
        print(f'{inp}: {improved} boards improved by {saved} lines, {invalid} invalid candidates, total now {total}')
        if improved:
            git('add', '-A', 'boards')
            git('commit', '-q', '-m',
                f'{source}: {inp} {improved} boards, -{saved} lines (total {total})\n\n'
                'Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>')
    finally:
        fcntl.flock(lock, fcntl.LOCK_UN)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    cmd = sys.argv[1]
    if cmd == 'status':
        grand = 0
        for inp in INPUTS:
            index = read_index(inp)
            total = sum(v[0] for v in index.values())
            grand += total
            print(f'{inp}: {len(index)} boards, {total} lines')
        print(f'TOTAL: {grand}')
    elif cmd == 'import-full':
        inp, path, source = sys.argv[2:5]
        import_programs(inp, list(enumerate(split_programs(open(path).read()))), source)
    elif cmd == 'import-mini':
        inp, path, source = sys.argv[2:5]
        idxs = [int(x) for x in sys.argv[5:]]
        progs = split_programs(open(path).read())
        if len(progs) != len(idxs):
            print(f'warning: {len(progs)} programs for {len(idxs)} indices; importing the first {min(len(progs), len(idxs))}')
        import_programs(inp, list(zip(idxs, progs)), source)
    elif cmd == 'import-board':
        inp, idx, path, source = sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5]
        import_programs(inp, [(idx, open(path).read())], source)
    elif cmd == 'assemble':
        outdir = sys.argv[2]
        os.makedirs(outdir, exist_ok=True)
        for inp in INPUTS:
            n = ensure_board_inputs(inp)
            progs = [open(os.path.join(board_dir(inp), f'{k:03d}.prog')).read().strip('\n') for k in range(n)]
            open(os.path.join(outdir, f'out_level-6_{inp}.txt'), 'w').write('\n\n'.join(progs) + '\n')
            print(f'{inp}: {sum(len(p.split(chr(10))) for p in progs)} lines -> {outdir}')
    elif cmd == 'show':
        inp, idx = sys.argv[2], int(sys.argv[3])
        print(open(os.path.join(board_dir(inp), f'{idx:03d}.in')).read())
        print('lines/source:', read_index(inp).get(idx))
        print(open(os.path.join(board_dir(inp), f'{idx:03d}.prog')).read())
    else:
        print(__doc__)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
