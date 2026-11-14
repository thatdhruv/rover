#!/usr/bin/env python3
"""Hardware-in-the-loop tests, run on the Raspberry Pi against the real board.

Talks to the rover firmware's Zephyr shell (the ST-Link virtual COM port, or any UART wired to
the console) and checks the safety and control behaviour that unit tests cannot:

  alive           the shell answers and reports sane state
  imu             gravity magnitude and heading look plausible (rover at rest)
  jitter          the 1 kHz loop kept its timing (no >1 ms wakeups)
  link-cutoff     commands stop -> motors cut off within the timeout, fault logged, recovery works
  tracking        (--with-motors, wheels off the ground) wheel velocity follows the setpoint

    python3 hil.py --port /dev/ttyACM0 [--with-motors] [--only alive,jitter]

Exit status is non-zero if any check fails, so it can gate a deployment script.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
import time
from dataclasses import dataclass, field

PROMPT = "uart:~$ "
LINK_TIMEOUT_S = 0.25  # keep in sync with LINK_TIMEOUT_MS in firmware/app/src/control.c


# ---------------------------------------------------------------- shell client


class Shell:
    """Minimal Zephyr-shell client. `port` is anything with write/read/in_waiting (pyserial)."""

    def __init__(self, port, timeout: float = 2.0):
        self.port = port
        self.timeout = timeout

    def run(self, command: str) -> str:
        """Send one command and return its output, without echo or prompt."""
        self.port.reset_input_buffer()
        self.port.write((command + "\n").encode())
        buf = ""
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            chunk = self.port.read(self.port.in_waiting or 1)
            if chunk:
                buf += chunk.decode(errors="replace")
                if buf.endswith(PROMPT):
                    break
            else:
                time.sleep(0.005)
        else:
            raise TimeoutError(f"no prompt after {command!r}; got {buf!r}")
        lines = [ln.rstrip("\r") for ln in buf.split("\n")]
        # drop the echoed command and the trailing prompt; drop async log lines
        body = [ln for ln in lines if ln.strip() and command not in ln and "<inf>" not in ln]
        return "\n".join(ln.replace(PROMPT, "") for ln in body if ln.strip() != PROMPT.strip())


# --------------------------------------------------------------------- parsers


@dataclass
class Wheel:
    cmd: float
    meas: float
    duty: float
    count: int


@dataclass
class State:
    link: bool
    cutoff: bool
    uptime_ms: int
    wheels: list[Wheel] = field(default_factory=list)
    heading: float = 0.0
    gyro_z: float = 0.0
    accel: tuple[float, float, float] = (0.0, 0.0, 0.0)


_NUM = r"(-?\d+(?:\.\d+)?)"


def parse_state(text: str) -> State:
    head = re.search(r"link=(\d) cutoff=(\d) up=(\d+)ms", text)
    if not head:
        raise ValueError(f"cannot parse state: {text!r}")
    wheels = [
        Wheel(float(c), float(m), float(d), int(n))
        for c, m, d, n in re.findall(
            rf"wheel\d cmd={_NUM} meas={_NUM} duty={_NUM} count=(-?\d+)", text
        )
    ]
    hd = re.search(rf"heading={_NUM} rad gyro_z={_NUM} rad/s", text)
    acc = re.search(rf"accel={_NUM} {_NUM} {_NUM} m/s2", text)
    return State(
        link=head.group(1) == "1",
        cutoff=head.group(2) == "1",
        uptime_ms=int(head.group(3)),
        wheels=wheels,
        heading=float(hd.group(1)) if hd else 0.0,
        gyro_z=float(hd.group(2)) if hd else 0.0,
        accel=tuple(float(acc.group(i)) for i in (1, 2, 3)) if acc else (0.0, 0.0, 0.0),
    )


@dataclass
class Jitter:
    samples: int
    mean_ns: int
    min_ns: int
    max_ns: int
    max_abs_ns: int
    buckets: list[int]


def parse_jitter(text: str) -> Jitter:
    m = re.search(
        r"samples=(\d+) mean=(-?\d+)ns min=(-?\d+)ns max=(-?\d+)ns max\|e\|=(\d+)ns", text
    )
    if not m:
        raise ValueError(f"cannot parse jitter: {text!r}")
    buckets = [int(x) for x in re.findall(r"^\s+\S+\s+(\d+)$", text, re.MULTILINE)]
    return Jitter(*(int(g) for g in m.groups()), buckets=buckets)


def parse_faults(text: str) -> list[dict]:
    return [
        {"seq": int(s), "uptime_ms": int(t), "code": int(c), "arg0": int(a0), "arg1": int(a1)}
        for s, t, c, a0, a1 in re.findall(
            r"#(\d+) t=(\d+)ms code=(\d+) arg0=(\d+) arg1=(\d+)", text
        )
    ]


FAULT_LINK_LOST = 3

# --------------------------------------------------------------------- checks


class Rover:
    def __init__(self, shell: Shell):
        self.sh = shell

    def state(self) -> State:
        return parse_state(self.sh.run("rover state"))

    def vel(self, left: float, right: float) -> None:
        self.sh.run(f"rover vel {left} {right}")

    def heartbeat(self) -> None:
        self.sh.run("rover hb")

    def jitter(self) -> Jitter:
        return parse_jitter(self.sh.run("rover jitter"))

    def faults(self) -> list[dict]:
        return parse_faults(self.sh.run("faults"))


class Failure(AssertionError):
    pass


def expect(cond: bool, msg: str) -> None:
    if not cond:
        raise Failure(msg)


def check_alive(r: Rover, _: argparse.Namespace) -> str:
    s1 = r.state()
    time.sleep(0.2)
    s2 = r.state()
    expect(s2.uptime_ms > s1.uptime_ms, "uptime is not advancing")
    expect(len(s2.wheels) == 2, f"expected 2 wheels, got {len(s2.wheels)}")
    return f"up {s2.uptime_ms} ms"


def check_imu(r: Rover, _: argparse.Namespace) -> str:
    s = r.state()
    g = math.sqrt(sum(a * a for a in s.accel))
    expect(8.8 < g < 10.8, f"|accel| = {g:.2f} m/s2, expected ~9.81 (rover at rest?)")
    expect(abs(s.gyro_z) < 0.1, f"gyro_z = {s.gyro_z:.3f} rad/s at rest")
    expect(-math.pi <= s.heading <= math.pi, f"heading {s.heading} out of range")
    return f"|a|={g:.2f} m/s2 gyro_z={s.gyro_z:.3f} heading={s.heading:.2f}"


def check_jitter(r: Rover, args: argparse.Namespace) -> str:
    j = r.jitter()
    expect(j.samples > 1000, f"only {j.samples} loop samples recorded")
    expect(
        j.max_abs_ns < args.max_jitter_us * 1000,
        f"max |jitter| {j.max_abs_ns / 1000:.0f} us exceeds {args.max_jitter_us} us",
    )
    expect(j.buckets and j.buckets[-1] == 0, f"{j.buckets[-1] if j.buckets else '?'} wakeups >= 1 ms")
    return f"max {j.max_abs_ns / 1000:.0f} us over {j.samples} ticks"


def check_link_cutoff(r: Rover, args: argparse.Namespace) -> str:
    # establish a live link with zero motion
    r.vel(0, 0)
    time.sleep(0.05)
    s = r.state()
    expect(s.link and not s.cutoff, "link should be alive right after a command")

    # stop talking; the firmware must notice and cut the motors by itself
    start = time.monotonic()
    cut = False
    while time.monotonic() - start < LINK_TIMEOUT_S + 0.5:
        time.sleep(0.02)
        s = r.state()
        if s.cutoff:
            cut = True
            break
    elapsed = time.monotonic() - start
    expect(cut, f"motors not cut off {elapsed:.2f} s after the link went quiet")
    expect(not s.link, "link reported alive after timeout")
    expect(all(w.duty == 0.0 for w in s.wheels), "duty not zero after cutoff")
    expect(
        any(f["code"] == FAULT_LINK_LOST for f in r.faults()[:4]),
        "link-lost fault was not recorded in the flash log",
    )

    # the link coming back must re-arm the motors
    r.vel(0, 0)
    time.sleep(0.1)
    s = r.state()
    expect(s.link and not s.cutoff, "motors did not re-arm after the link returned")
    return f"cut off after ~{elapsed * 1000:.0f} ms, re-armed on recovery"


def check_tracking(r: Rover, args: argparse.Namespace) -> str:
    if not args.with_motors:
        raise Skip("needs --with-motors (wheels must be off the ground)")
    target = args.speed
    r.sh.run("rover vel 0 0")
    t0 = time.monotonic()
    s = r.state()
    while time.monotonic() - t0 < 2.5:
        r.vel(target, target)  # also keeps the link alive
        time.sleep(0.05)
        s = r.state()
    meas = [w.meas for w in s.wheels]
    r.vel(0, 0)
    for i, m in enumerate(meas):
        expect(
            abs(m - target) <= 0.15 * target,
            f"wheel {i}: measured {m:.2f} rad/s, wanted {target:.2f} +-15%",
        )
    return f"wheels at {meas[0]:.2f} / {meas[1]:.2f} rad/s (target {target:.2f})"


class Skip(Exception):
    pass


CHECKS = {
    "alive": check_alive,
    "imu": check_imu,
    "jitter": check_jitter,
    "link-cutoff": check_link_cutoff,
    "tracking": check_tracking,
}


def run_checks(rover: Rover, names: list[str], args: argparse.Namespace) -> int:
    failed = 0
    for name in names:
        try:
            detail = CHECKS[name](rover, args)
            print(f"PASS  {name:12s} {detail}")
        except Skip as e:
            print(f"SKIP  {name:12s} {e}")
        except (Failure, TimeoutError, ValueError) as e:
            failed += 1
            print(f"FAIL  {name:12s} {e}")
    try:  # never leave the wheels commanded
        rover.vel(0, 0)
    except Exception:
        pass
    print(f"\n{len(names) - failed} of {len(names)} checks passed (failures: {failed})")
    return 1 if failed else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--only", help="comma-separated subset of: " + ",".join(CHECKS))
    ap.add_argument("--with-motors", action="store_true", help="run checks that spin the wheels")
    ap.add_argument("--speed", type=float, default=6.0, help="tracking test setpoint, rad/s")
    ap.add_argument("--max-jitter-us", type=int, default=250)
    args = ap.parse_args(argv)

    import serial  # pyserial, only needed on the Pi

    names = args.only.split(",") if args.only else list(CHECKS)
    unknown = [n for n in names if n not in CHECKS]
    if unknown:
        ap.error(f"unknown checks: {unknown}")

    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        shell = Shell(port)
        shell.run("")  # sync to the prompt
        return run_checks(Rover(shell), names, args)


if __name__ == "__main__":
    sys.exit(main())
