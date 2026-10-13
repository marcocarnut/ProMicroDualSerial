#!/usr/bin/env python3
"""Instruction-level view of one function from a harness PROFILE dump."""
import sys, subprocess
elf, prof, fn = sys.argv[1:4]
cnt = {}
for l in open(prof):
    a, c = l.split()
    cnt[int(a, 16)] = int(c)
on = False
for l in subprocess.run(["avr-objdump", "-d", elf], capture_output=True, text=True).stdout.splitlines():
    if l.endswith("<%s>:" % fn):
        on = True
        continue
    if on and not l.strip():
        break
    if on:
        a = int(l.split(':')[0], 16)
        print("%8d %s" % (cnt.get(a, 0), l.strip()[:75]))
