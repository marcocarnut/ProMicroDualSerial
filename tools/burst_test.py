#!/usr/bin/env python3
"""
Bursty full duplex traffic over the loopback rig (see loopback_test.py),
like PPP carrying TCP: each port sends chunks of bytes with random gaps
between them, both at once, and every received byte and the line error
counters are checked.

    tools/burst_test.py [--baud 115200] [--coding 8N1] [--seconds 20]
                        [--fill 0x55] [A GA B GB] [/dev/ttyACM0 /dev/ttyACM1]

A GA B GB: ACM0 (into the soft UART's RX) sends chunks of A bytes every
~GA ms and ACM1 (out of the soft UART's TX) chunks of B bytes every ~GB ms
(each gap is drawn from 0.5..1.5 times the given one). Without them, a
suite of patterns is run. --fill sends that byte throughout instead of
random data: 0x55 in 8N1 has an edge at every bit, the soft UART's worst
case.

Steady streams (loopback_test.py) don't catch everything: bursts let the
transmitter queue up frames while the receiver is idle, which then compete
with the receiver once it gets busy again.
"""
import os
import random
import select
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import loopback_test as lt

SUITE = [
    (1500, 140, 1500, 300),     # both ways bulk, ACM0 faster
    (1500, 100, 1500, 100),     # both ways bulk, same rate
    (1500, 60, 600, 50),        # both ways, smaller chunks out of the soft UART
    (300, 20, 1500, 200),       # small chunks into the soft UART
    (1500, 140, 60, 40),        # bulk one way, "ACKs" the other
]


def run(fa, fb, a, ga, b, gb, seconds, fill, bits):
    mask = (1 << bits) - 1

    def chunk(n):
        if fill is not None:
            return bytes([fill & mask]) * n
        return bytes(random.getrandbits(8) & mask for _ in range(n))

    size, gap = {fa: a, fb: b}, {fa: ga / 1000.0, fb: gb / 1000.0}
    sent = {fa: bytearray(), fb: bytearray()}
    got = {fa: bytearray(), fb: bytearray()}
    pend, nxt = {fa: b"", fb: b""}, {fa: 0.0, fb: 0.0}
    c0 = {f: lt.icount(f) for f in (fa, fb)}
    t0 = time.time()
    while True:
        now = time.time() - t0
        for f in (fa, fb):
            if size[f] and not pend[f] and now < seconds and now >= nxt[f]:
                pend[f] = chunk(size[f])
                sent[f] += pend[f]
                nxt[f] = now + gap[f] * random.uniform(0.5, 1.5)
        done = len(got[fb]) >= len(sent[fa]) and len(got[fa]) >= len(sent[fb])
        if now >= seconds and not pend[fa] and not pend[fb] and done:
            break
        if now > seconds + 30:
            break
        r, w, _ = select.select([fa, fb], [f for f in (fa, fb) if pend[f]], [], 0.002)
        for f in w:
            try:
                k = os.write(f, pend[f])
                pend[f] = pend[f][k:]
            except BlockingIOError:
                pass
        for f in r:
            try:
                got[f] += os.read(f, 4096)
            except BlockingIOError:
                pass

    ok = True
    for f, name, want in ((fb, "ACM0 -> ACM1 (soft RX)", sent[fa]), (fa, "ACM1 -> ACM0 (soft TX)", sent[fb])):
        errs = {k: v - c0[f][k] for k, v in lt.icount(f).items()
                if k in ("frame", "overrun", "parity", "brk", "buf_overrun") and v != c0[f][k]}
        good = bytes(got[f]) == bytes(want) and not errs
        ok = ok and good
        print("    %s: %d bytes, %s%s" % (name, len(want), "ok" if good else "FAILED",
                                          "" if good else " (got %d, errors %s)" % (len(got[f]), errs)))
    return ok


def main():
    args = sys.argv[1:]
    opt = {"--baud": "115200", "--coding": "8N1", "--seconds": "20", "--fill": None}
    rest = []
    while args:
        a = args.pop(0)
        if a in opt:
            opt[a] = args.pop(0)
        else:
            rest.append(a)
    ports = [a for a in rest if a.startswith("/")]
    nums = [a for a in rest if not a.startswith("/")]
    if len(nums) not in (0, 4):
        sys.exit(__doc__)
    pa = ports[0] if ports else "/dev/ttyACM0"
    pb = ports[1] if len(ports) > 1 else "/dev/ttyACM1"
    baud, seconds = int(opt["--baud"]), float(opt["--seconds"])
    coding = opt["--coding"].upper()
    bits, parity, stop2 = int(coding[0]), coding[1], coding[2:] == "2"
    fill = int(opt["--fill"], 0) if opt["--fill"] else None
    patterns = [tuple(float(x) if i % 2 else int(x) for i, x in enumerate(nums))] if nums else SUITE

    fa, fb = lt.open_port(pa), lt.open_port(pb)
    for f in (fa, fb):
        lt.configure(f, baud, bits, parity, stop2)
        lt.drain(f)
    random.seed(5)

    failed = 0
    for a, ga, b, gb in patterns:
        print("%d %s, %s: ACM0 %d bytes / %g ms, ACM1 %d bytes / %g ms, %g s"
              % (baud, coding, "0x%02x" % fill if fill is not None else "random", a, ga, b, gb, seconds))
        if not run(fa, fb, a, ga, b, gb, seconds, fill, bits):
            failed += 1
    print("%d failures" % failed)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
