#!/usr/bin/env python3
"""
Hardware test for ProMicroDualUART (stdlib only, Linux).

Wire the two UARTs to each other like a null modem cable:

    TX0 (PD3)            ->  pin 4 (PD4, soft RX)
    pin 5 (PC6, soft TX) ->  RX1 (PD2)
    A3 (port 0 RTS)      ->  A0 (port 1 CTS)
    A1 (port 1 RTS)      ->  A2 (port 0 CTS)

and optionally, for --modem (and --dcd, with the DCD wires too):

    15 (port 0 DTR)      ->  9 (port 1 DSR)  [and 7 (port 1 DCD)]
    6  (port 1 DTR)      ->  14 (port 0 DSR) [and 16 (port 0 DCD)]

Then everything written to one port should come out of the other, through
both the hardware USART and the soft UART, in both directions at once.

    tools/loopback_test.py [--modem [--dcd]] [--no-flow] [/dev/ttyACM0] [/dev/ttyACM1]

For each line coding it sends random data both ways simultaneously and
checks the received bytes and the line error counters (TIOCGICOUNT), then
sends a break each way and checks it is reported as a break. It then checks
RTS/CTS flow control (dropping RTS on one port must stop the other port's
transmitter) and, with --modem, that DTR shows up as DSR and DCD on the
other side. Use --no-flow when the RTS/CTS wires are not fitted (with the
default FLOW_AUTO the data tests then still pass, CTS never gets armed) or
for firmware built with FLOW_OFF.
"""
import array
import fcntl
import os
import random
import select
import struct
import sys
import termios
import time

CMSPAR = 0o10000000000
TIOCGICOUNT = 0x545D

BAUDS = {b: getattr(termios, "B%d" % b) for b in
         (300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400)}
SIZES = {5: termios.CS5, 6: termios.CS6, 7: termios.CS7, 8: termios.CS8}


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd


def configure(fd, baud, bits, parity, stop2):
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(fd)
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = termios.CREAD | termios.CLOCAL | SIZES[bits]
    if parity != "N":
        cflag |= termios.PARENB
        if parity in "OM":
            cflag |= termios.PARODD
        if parity in "MS":
            cflag |= CMSPAR
    if stop2:
        cflag |= termios.CSTOPB
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW,
                      [iflag, oflag, cflag, lflag, BAUDS[baud], BAUDS[baud], cc])
    time.sleep(0.05)
    termios.tcflush(fd, termios.TCIOFLUSH)


def icount(fd):
    # struct serial_icounter_struct: cts dsr rng dcd rx tx frame overrun parity brk buf_overrun reserved[9]
    buf = array.array("i", [0] * 20)
    fcntl.ioctl(fd, TIOCGICOUNT, buf, True)
    names = ("cts", "dsr", "rng", "dcd", "rx", "tx", "frame", "overrun", "parity", "brk", "buf_overrun")
    return dict(zip(names, buf))


def drain(fd, timeout=0.2):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if not os.read(fd, 4096):
                time.sleep(0.01)
        except BlockingIOError:
            time.sleep(0.01)


def exchange(fa, fb, data_a, data_b, baud):
    """Send data_a on fa and data_b on fb at the same time; return what each received."""
    got_a, got_b = bytearray(), bytearray()
    pa, pb = 0, 0
    frame_time = 12.0 / baud
    deadline = time.time() + 2 + len(max(data_a, data_b, key=len)) * frame_time * 1.5
    while time.time() < deadline and (len(got_a) < len(data_b) or len(got_b) < len(data_a)):
        w = []
        if pa < len(data_a):
            w.append(fa)
        if pb < len(data_b):
            w.append(fb)
        r, w, _ = select.select([fa, fb], w, [], 0.05)
        for fd in w:
            try:
                if fd == fa:
                    pa += os.write(fa, data_a[pa:pa + 256])
                else:
                    pb += os.write(fb, data_b[pb:pb + 256])
            except BlockingIOError:
                pass
        for fd in r:
            try:
                d = os.read(fd, 4096)
            except BlockingIOError:
                continue
            (got_a if fd == fa else got_b).extend(d)
    return bytes(got_a), bytes(got_b)


def compare(name, sent, got):
    if sent == got:
        return True
    first = next((i for i in range(min(len(sent), len(got))) if sent[i] != got[i]), min(len(sent), len(got)))
    print("    %s: sent %d bytes, got %d, first difference at %d" % (name, len(sent), len(got), first))
    return False


def run_case(fa, fb, baud, bits, parity, stop2, nbytes):
    configure(fa, baud, bits, parity, stop2)
    configure(fb, baud, bits, parity, stop2)
    drain(fa)
    drain(fb)
    mask = (1 << bits) - 1
    data_a = bytes(random.getrandbits(8) & mask for _ in range(nbytes))
    data_b = bytes(random.getrandbits(8) & mask for _ in range(nbytes))
    ca0, cb0 = icount(fa), icount(fb)
    t0 = time.time()
    got_a, got_b = exchange(fa, fb, data_a, data_b, baud)
    dt = time.time() - t0
    ca1, cb1 = icount(fa), icount(fb)
    ok = compare("hw->soft (ACM0->ACM1)", data_a, got_b)
    ok &= compare("soft->hw (ACM1->ACM0)", data_b, got_a)
    for name, c0, c1 in (("ACM0", ca0, ca1), ("ACM1", cb0, cb1)):
        errs = {k: c1[k] - c0[k] for k in ("frame", "overrun", "parity", "brk") if c1[k] != c0[k]}
        if errs:
            print("    %s line errors: %s" % (name, errs))
            ok = False

    # break each way
    for (src, dst, name) in ((fa, fb, "ACM0->ACM1"), (fb, fa, "ACM1->ACM0")):
        c0 = icount(dst)
        termios.tcsendbreak(src, 0)       # 250..500 ms
        time.sleep(0.3)
        drain(dst, 0.1)
        c1 = icount(dst)
        if c1["brk"] - c0["brk"] != 1:
            print("    break %s: %d breaks reported (want 1), frame %+d" %
                  (name, c1["brk"] - c0["brk"], c1["frame"] - c0["frame"]))
            ok = False

    rate = 2 * nbytes / dt if dt else 0
    print("  %-7d %d%s%s  %s  (%.0f bytes/s total)" %
          (baud, bits, parity, "2" if stop2 else "1", "ok" if ok else "FAIL", rate))
    return ok


def set_line(fd, bit, on):
    fcntl.ioctl(fd, termios.TIOCMBIS if on else termios.TIOCMBIC, struct.pack("I", bit))


def get_lines(fd):
    return struct.unpack("I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0)))[0]


def read_for(fd, seconds):
    got = bytearray()
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.02)
        if r:
            try:
                got.extend(os.read(fd, 4096))
            except BlockingIOError:
                pass
    return bytes(got)


def test_flow(fa, fb):
    """Dropping RTS on the receiving port must hold off the other transmitter."""
    ok = True
    for (src, dst, name) in ((fa, fb, "ACM0->ACM1"), (fb, fa, "ACM1->ACM0")):
        configure(fa, 115200, 8, "N", False)
        configure(fb, 115200, 8, "N", False)
        drain(dst)
        set_line(dst, termios.TIOCM_RTS, False)
        time.sleep(0.05)
        data = bytes(random.getrandbits(8) for _ in range(500))
        os.write(src, data)
        held = read_for(dst, 0.5)
        set_line(dst, termios.TIOCM_RTS, True)
        rest = read_for(dst, 0.5)
        got = held + rest
        # at most the frame(s) already committed may slip through
        good = len(held) <= 2 and got == data
        print("  flow %s: %d bytes leaked while held, %d/%d delivered after release  %s" %
              (name, len(held), len(got), len(data), "ok" if good else "FAIL"))
        ok &= good
    return ok


def test_modem(fa, fb, dcd_wired):
    """DTR on one side must show up as DSR (and DCD, if wired) on the other."""
    ok = True
    for (src, dst, name) in ((fa, fb, "ACM0->ACM1"), (fb, fa, "ACM1->ACM0")):
        for on in (True, False, True):
            set_line(src, termios.TIOCM_DTR, on)
            time.sleep(0.1)
            lines = get_lines(dst)
            dsr = bool(lines & termios.TIOCM_DSR)
            dcd = bool(lines & termios.TIOCM_CAR)
            good = dsr == on and (dcd == on or not dcd_wired)
            print("  modem %s: DTR %d -> DSR %d DCD %d  %s" % (name, on, dsr, dcd, "ok" if good else "FAIL"))
            ok &= good
    return ok


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = [a for a in sys.argv[1:] if a.startswith("--")]
    pa = args[0] if len(args) > 0 else "/dev/ttyACM0"
    pb = args[1] if len(args) > 1 else "/dev/ttyACM1"
    fa, fb = open_port(pa), open_port(pb)
    random.seed(1)

    cases = []
    for baud in (115200, 57600, 38400, 9600):
        cases.append((baud, 8, "N", False))
    for parity in "OEMS":
        for bits in (7, 8):
            cases.append((115200, bits, parity, False))
    for bits in (5, 6):
        cases.append((115200, bits, "N", True))
    cases += [(2400, 8, "E", True), (300, 7, "O", False)]

    failed = 0
    for baud, bits, parity, stop2 in cases:
        nbytes = max(20, min(4000, baud // 20))
        if not run_case(fa, fb, baud, bits, parity, stop2, nbytes):
            failed += 1
    if "--no-flow" not in opts and not test_flow(fa, fb):
        failed += 1
    if "--modem" in opts and not test_modem(fa, fb, "--dcd" in opts):
        failed += 1
    print("%d failures" % failed)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
