#!/usr/bin/env python3
"""Map a harness PROFILE dump (byte address, cycles) to functions via avr-nm."""
import subprocess, sys, bisect
elf, prof = sys.argv[1], sys.argv[2]
syms = []
for line in subprocess.run(["avr-nm", "-n", "-S", "--defined-only", elf], capture_output=True, text=True).stdout.splitlines():
    p = line.split()
    if len(p) >= 3 and p[-2] in "tTwW" and not p[-1].startswith("__stack"):
        syms.append((int(p[0], 16), p[-1]))
syms.sort()
addrs = [a for a, _ in syms]
tot = {}
total = 0
for line in open(prof):
    a, c = line.split()
    a, c = int(a, 16), int(c)
    i = bisect.bisect_right(addrs, a) - 1
    name = syms[i][1] if i >= 0 else "?"
    tot[name] = tot.get(name, 0) + c
    total += c
for name, c in sorted(tot.items(), key=lambda x: -x[1])[:25]:
    print("%6.2f%%  %10d  %s" % (100.0 * c / total, c, name))
