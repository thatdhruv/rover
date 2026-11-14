"""Tests for the HIL script itself, against a fake board that mimics the firmware's shell.

This validates the parsers and the check logic (including that a firmware which does NOT cut
the motors on link loss is caught). It does not replace running hil.py on real hardware.
"""
import argparse
import time

import pytest

import hil

PROMPT = hil.PROMPT


class FakeBoard:
    """Serial-port stand-in modelling the firmware shell and its link-loss behaviour."""

    def __init__(self, cutoff_works=True, max_jitter_ns=40_000, slow_buckets=0, accel=(0.1, -0.2, 9.8)):
        self.cutoff_works = cutoff_works
        self.max_jitter_ns = max_jitter_ns
        self.slow_buckets = slow_buckets
        self.accel = accel
        self.t0 = time.monotonic()
        self.last_cmd = None
        self.cutoff = False
        self.cmd = (0.0, 0.0)
        self.faults = []  # newest last
        self.out = b""

    # -- pyserial surface
    @property
    def in_waiting(self):
        return len(self.out)

    def reset_input_buffer(self):
        self.out = b""

    def read(self, n=1):
        chunk, self.out = self.out[:n], self.out[n:]
        return chunk

    def write(self, data):
        line = data.decode().strip()
        self.out += (line + "\r\n" + self._handle(line) + PROMPT).encode()

    # -- model
    def _alive(self):
        return self.last_cmd is not None and time.monotonic() - self.last_cmd <= hil.LINK_TIMEOUT_S

    def _tick(self):
        if self.cutoff_works:
            if self.last_cmd is not None and not self._alive() and not self.cutoff:
                self.cutoff = True
                self.faults.append(hil.FAULT_LINK_LOST)
            elif self._alive() and self.cutoff:
                self.cutoff = False

    def _handle(self, line):
        self._tick()
        if line == "rover state":
            up = int((time.monotonic() - self.t0) * 1000)
            meas = (0.0, 0.0) if self.cutoff else self.cmd
            return (
                f"link={int(self._alive())} cutoff={int(self.cutoff)} up={up}ms\r\n"
                f"wheel0 cmd={self.cmd[0]:.2f} meas={meas[0]:.2f} duty=0.00 count=0\r\n"
                f"wheel1 cmd={self.cmd[1]:.2f} meas={meas[1]:.2f} duty=0.00 count=0\r\n"
                "heading=0.012 rad gyro_z=0.001 rad/s\r\n"
                f"accel={self.accel[0]:.2f} {self.accel[1]:.2f} {self.accel[2]:.2f} m/s2\r\n"
            )
        if line.startswith("rover vel"):
            _, _, l, r_ = line.split()
            self.cmd = (float(l), float(r_))
            self.last_cmd = time.monotonic()
            self._tick()
            return ""
        if line == "rover hb":
            self.last_cmd = time.monotonic()
            return ""
        if line == "rover jitter":
            hist = [9000, 800, 5, 0, 0, 0, self.slow_buckets]
            labels = ["<10us", "<50us", "<100us", "<250us", "<500us", "<1ms", ">=1ms"]
            rows = "".join(f"  {lab:<7} {n}\r\n" for lab, n in zip(labels, hist))
            return (
                f"samples={sum(hist)} mean=1200ns min=-30000ns max={self.max_jitter_ns}ns "
                f"max|e|={self.max_jitter_ns}ns\r\n{rows}"
            )
        if line == "faults":
            return "".join(
                f"#{i + 1} t={100 * i}ms code={c} arg0=250 arg1=0\r\n"
                for i, c in reversed(list(enumerate(self.faults)))
            )
        return ""


def args(**kw):
    ns = argparse.Namespace(with_motors=False, speed=6.0, max_jitter_us=250)
    ns.__dict__.update(kw)
    return ns


def rover(board):
    return hil.Rover(hil.Shell(board, timeout=1.0))


def test_parse_state_and_jitter_roundtrip():
    b = FakeBoard()
    r = rover(b)
    s = r.state()
    assert len(s.wheels) == 2 and s.accel == (0.1, -0.2, 9.8)
    j = r.jitter()
    assert j.samples == 9805 and j.buckets[-1] == 0 and j.max_abs_ns == 40_000


def test_all_default_checks_pass_on_a_healthy_board():
    b = FakeBoard()
    assert hil.run_checks(rover(b), ["alive", "imu", "jitter", "link-cutoff", "tracking"], args()) == 0


def test_tracking_runs_with_motors():
    assert hil.run_checks(rover(FakeBoard()), ["tracking"], args(with_motors=True)) == 0


def test_link_cutoff_catches_firmware_that_keeps_driving():
    b = FakeBoard(cutoff_works=False)
    assert hil.run_checks(rover(b), ["link-cutoff"], args()) == 1


def test_jitter_check_fails_on_slow_wakeups():
    assert hil.run_checks(rover(FakeBoard(slow_buckets=3)), ["jitter"], args()) == 1
    assert hil.run_checks(rover(FakeBoard(max_jitter_ns=900_000)), ["jitter"], args()) == 1


def test_imu_check_catches_wrong_gravity():
    assert hil.run_checks(rover(FakeBoard(accel=(0.0, 0.0, 1.0))), ["imu"], args()) == 1


def test_timeout_when_board_silent():
    class Dead(FakeBoard):
        def write(self, data):
            pass

    with pytest.raises(TimeoutError):
        hil.Shell(Dead(), timeout=0.2).run("rover state")
