#!/usr/bin/env python3
"""PR gate for notebook/boards: every changed board program must be valid and strictly shorter.

usage: ci_check.py <base_checkout> <pr_checkout>
The validator binary comes from the BASE checkout (base/notebook/tools/validate), so a PR
cannot change the rules it is judged by. Fails on: invalid program, not strictly shorter,
index.tsv line count mismatch, modified .in files, or new/removed boards.
"""
import os
import subprocess
import sys
import tempfile

base, pr = sys.argv[1], sys.argv[2]
validate = os.path.join(base, 'notebook', 'tools', 'validate')
errors, improved, saved = [], 0, 0


def index(root, inp):
    out = {}
    path = os.path.join(root, 'notebook', 'boards', inp, 'index.tsv')
    for line in open(path):
        if line.startswith('#') or not line.strip():
            continue
        idx, lines, _ = line.rstrip('\n').split('\t')
        out[int(idx)] = int(lines)
    return out


for inp in sorted(os.listdir(os.path.join(base, 'notebook', 'boards'))):
    bdir = os.path.join(base, 'notebook', 'boards', inp)
    pdir = os.path.join(pr, 'notebook', 'boards', inp)
    if not os.path.isdir(bdir):
        continue
    bfiles, pfiles = set(os.listdir(bdir)), set(os.listdir(pdir))
    if bfiles != pfiles:
        errors.append(f'{inp}: board files added/removed: {sorted(bfiles ^ pfiles)}')
        continue
    bidx, pidx = index(base, inp), index(pr, inp)
    for name in sorted(bfiles):
        b, p = open(os.path.join(bdir, name)).read(), open(os.path.join(pdir, name)).read()
        if b == p or name == 'index.tsv':
            continue
        if name.endswith('.in'):
            errors.append(f'{inp}/{name}: board inputs are immutable')
            continue
        if not name.endswith('.prog'):
            errors.append(f'{inp}/{name}: unexpected file')
            continue
        k = int(name[:-5])
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'out.txt')
            res = subprocess.run([validate, os.path.join(bdir, f'{k:03d}.in'), out, os.path.join(pdir, name)],
                                 capture_output=True, text=True)
        if res.returncode != 0 or 'merged total' not in res.stderr:
            errors.append(f'{inp}/{name}: invalid program')
            continue
        lines = int(res.stderr.split('merged total')[1].split()[0])
        if len(p.strip().split('\n')) != lines:
            errors.append(f'{inp}/{name}: program has trailing lines after the last needed submission')
        if lines >= bidx[k]:
            errors.append(f'{inp}/{name}: {lines} lines, not shorter than {bidx[k]}')
        elif pidx.get(k) != lines:
            errors.append(f'{inp}/index.tsv: board {k} says {pidx.get(k)}, program has {lines}')
        else:
            improved += 1
            saved += bidx[k] - lines

print(f'boards improved: {improved}, lines saved: {saved}')
for e in errors:
    print('ERROR', e)
sys.exit(1 if errors or improved == 0 else 0)
