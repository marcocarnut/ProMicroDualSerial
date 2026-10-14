#!/usr/bin/env python3
"""
Profile a PROFILER=1 build (make clean && make EXTRA=-DPROFILER=1) on the
board: reset the counts, run traffic, read them back and map them to
functions.

    tools/fwprofile.py [--elf ProMicroDualSerial.elf] [--baud 115200] [--bytes 20000]
                       [--mode both|a|b|idle] [--fill 0x55] [/dev/ttyACM0 /dev/ttyACM1]

Traffic is random data (or --fill: that byte throughout) over the loopback rig (see loopback_test.py): 'a' is
ACM0 -> ACM1 (soft RX), 'b' ACM1 -> ACM0 (soft TX), 'both' at once, 'idle'
just waits. Setting ACM0 to 110 baud makes the firmware dump the profile.

The Timer1 ISRs can't be sampled; samples due while one runs land late,
right after it. The lateness histogram gives the share of time spent there
(and in other code with interrupts off).
"""
import os
import random
import select
import subprocess
import sys
import termios
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loopback_test as lt


def trigger(path):
    """Make the firmware dump its profile (and reset it); return the lines."""
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    lt.configure(fd, 9600, 8, "N", False)
    attr = termios.tcgetattr(fd)
    attr[4] = attr[5] = termios.B110
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    out = b""
    end = time.time() + 5
    while time.time() < end and not out.endswith(b"E\n"):
        if select.select([fd], [], [], 0.1)[0]:
            out += os.read(fd, 4096)
    os.close(fd)
    if not out.endswith(b"E\n"):
        sys.exit("no profile from %s (not a PROFILER=1 build?)" % path)
    return out.decode().split("\n")


BUCKET = 32


def attribute(buckets, elf):
    """Split each bucket's count over the (innermost, possibly inlined)
    functions of a few addresses in it, via avr-addr2line."""
    addrs = [a + o for a, _ in buckets for o in range(0, BUCKET, 8)]
    out = subprocess.run(["avr-addr2line", "-a", "-f", "-i", "-e", elf] + ["%x" % a for a in addrs],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    # each address: "0x...." then (function, file:line) pairs, innermost first
    names = []
    for i, line in enumerate(out):
        if line.startswith("0x"):
            names.append(out[i + 1] if i + 1 < len(out) else "?")
    funcs = {}
    per = BUCKET // 8
    for k, (addr, count) in enumerate(buckets):
        for name in names[k * per:(k + 1) * per]:
            funcs[name] = funcs.get(name, 0) + count / float(per)
    return funcs


def main():
    args = sys.argv[1:]
    opt = {"--elf": "ProMicroDualSerial.elf", "--baud": "115200", "--bytes": "20000", "--mode": "both", "--fill": None}
    ports = []
    while args:
        a = args.pop(0)
        if a in opt:
            opt[a] = args.pop(0)
        else:
            ports.append(a)
    pa = ports[0] if ports else "/dev/ttyACM0"
    pb = ports[1] if len(ports) > 1 else "/dev/ttyACM1"
    baud, n, mode = int(opt["--baud"]), int(opt["--bytes"]), opt["--mode"]

    fa, fb = lt.open_port(pa), lt.open_port(pb)
    for f in (fa, fb):
        lt.configure(f, baud, 8, "N", False)
        lt.drain(f)
    trigger(pa)                              # discard (resets the counts)
    lt.configure(fa, baud, 8, "N", False)
    lt.drain(fa)

    random.seed(3)
    fill = opt["--fill"]
    gen = (lambda: bytes([int(fill, 0)]) * n) if fill else (lambda: bytes(random.getrandbits(8) for _ in range(n)))
    da = gen() if mode in ("both", "a") else b""
    db = gen() if mode in ("both", "b") else b""
    t0 = time.time()
    if mode == "idle":
        time.sleep(n / (baud / 10.0))
        ga, gb = b"", b""
    else:
        ga, gb = lt.exchange(fa, fb, da, db, baud)
    dt = time.time() - t0
    lines = trigger(pa)
    os.close(fa)
    os.close(fb)
    ok = gb == da and ga == db
    print("traffic: %s at %d, %d bytes each way, %.2f s, data %s" %
          (mode, baud, n if mode != "idle" else 0, dt, "ok" if ok else "MISMATCH"))

    total, buckets, late, stats = 0, [], {}, {}
    for l in lines:
        p = l.split()
        if not p:
            continue
        if p[0] == "N":
            total = int(p[1], 16) * 65536 + int(p[2])
        elif p[0] == "P":
            buckets.append((int(p[1], 16), int(p[2])))
        elif p[0] == "L":
            late[int(p[1], 16)] = int(p[2])
        elif p[0] == "S":
            stats[int(p[1], 16)] = int(p[2])
    if not total:
        sys.exit("no samples")

    # lateness: the smallest well populated bin is the stub's fixed overhead;
    # anything two bins (32 cycles) later was held off by an ISR / cli
    base = min(b for b, c in late.items() if c > total * 0.02)
    held = sum(c for b, c in late.items() if b >= base + 2)
    print("samples: %d; held off by non-nestable ISRs / cli: %.1f%%" % (total, 100.0 * held / total))
    print("lateness (16-cycle bins from the fastest): " +
          ", ".join("%d:%d" % (b - base, late[b]) for b in sorted(late)))
    if stats:
        print("soft TX encoder, per call: %d calls; stopped by RX backlog gate %d, batch limit %d, "
              "empty tx ring %d, full frame word queue %d; restarts from idle %d (the counts saturate at 65535)"
              % tuple(stats.get(i, 0) for i in range(6)))
        print("soft RX overruns: edge ring %d, rx ring %d" % (stats.get(6, 0), stats.get(7, 0)))
    print()

    funcs = attribute(buckets, opt["--elf"])
    print("%7s  %s" % ("share", "function (samples taken in main code and nested ISR bodies)"))
    for name, c in sorted(funcs.items(), key=lambda x: -x[1]):
        if c / total >= 0.003:
            print("%6.1f%%  %s" % (100.0 * c / total, name))


if __name__ == "__main__":
    main()
