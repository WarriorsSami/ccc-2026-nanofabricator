#!/usr/bin/env python3
"""extract.py <input.txt> <out_mini_input.txt> <idx> [idx ...]
Writes a mini input containing only the given pattern indices (0-based), in that order."""
import sys
src, dst, idxs = sys.argv[1], sys.argv[2], [int(x) for x in sys.argv[3:]]
L = open(src).read().split('\n'); i = 1; pats = []
for _ in range(int(L[0])):
    while not L[i].strip(): i += 1
    rows = int(L[i].split()[1]); pats.append('\n'.join(L[i:i + 1 + rows])); i += 1 + rows
open(dst, 'w').write(f"{len(idxs)}\n" + '\n\n'.join(pats[k] for k in idxs) + '\n')
