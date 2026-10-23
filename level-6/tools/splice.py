#!/usr/bin/env python3
"""splice.py <full_output.txt> <mini_output.txt> <idx> [idx ...]
Replaces programs at the given indices of full_output (in place) with the programs of
mini_output (same order as the idx list). Validate afterwards with the merge tool."""
import sys
full, mini, idxs = sys.argv[1], sys.argv[2], [int(x) for x in sys.argv[3:]]
F = [p.strip('\n') for p in open(full).read().split('\n\n') if p.strip()]
M = [p.strip('\n') for p in open(mini).read().split('\n\n') if p.strip()]
assert len(M) == len(idxs), (len(M), len(idxs))
for k, p in zip(idxs, M): F[k] = p
open(full, 'w').write('\n\n'.join(F) + '\n')
