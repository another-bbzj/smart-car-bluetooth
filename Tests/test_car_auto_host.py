import importlib.util
import math
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'Tools'))
import car_host
import car_auto


class Clock:
    value = 0.0
    def now(self): return self.value
    def sleep(self, seconds): self.value += seconds


class Controller:
    """Serial peer with firmware-like command results, not a mocked controller method."""
    def __init__(self, clock, reject=False, silent=False, short=False, rolling=False, fault=False):
        self.clock = clock
        self.reject, self.silent, self.short, self.rolling, self.inject_fault = reject, silent, short, rolling, fault
        self.state = self.result = self.frames = self.fault = self.armed_count = self.stops = 0
        self.sent_arm_after_stops = 0
        self.next_sample = 0
        self.heartbeats = 0
    def reset_input_buffer(self): pass
    def write(self, data):
        assert data[-1] == car_host.crc8(data[2:-1])
        cmd = data[2]
        if cmd == 6 and self.short: return len(data) - 1
        self.frames += 1
        self.result = 0
        if cmd == 2: self.state = self.fault = 0; self.stops += 1
        elif cmd == 6:
            self.armed_count += 1; self.sent_arm_after_stops = self.stops
            self.result = 1 if self.reject else 0
            self.state = 0 if self.reject else 1
        elif cmd == 5:
            self.heartbeats += 1
            if self.inject_fault: self.state = 2; self.fault = 7
        return len(data)
    def read(self, size):
        if self.clock.now() < self.next_sample or (self.silent and self.armed_count): return b''
        self.next_sample = self.clock.now() + .1
        speed = 8 if self.rolling else 0
        values = [self.state, self.fault, 0, speed, 0, speed, 0, 0, self.result, self.frames] + [0] * 6
        return struct.pack('<16f', *values) + car_host.TAIL


class AutoHostTests(unittest.TestCase):
    def test_geometry_packet_and_validation(self):
        packet = car_auto.auto_arm_packet(60, 150, 80)
        self.assertEqual(packet[2:10], bytes.fromhex('06 06 00 3c 00 96 00 50'))
        for params in [(0,150,80), (60,0,80), (60,150,151), (60.1,150,80), (True,150,80)]:
            with self.assertRaises(ValueError): car_auto.auto_arm_packet(*params)

    def run_peer(self, **options):
        clock = Clock(); peer = Controller(clock, **options)
        return clock, peer

    def test_confirm_stop_arm_heartbeat_and_final_stop(self):
        clock, peer = self.run_peer()
        car_auto.drive(peer, 60, 150, 80, .5, clock.now, clock.sleep, lambda s: None)
        self.assertEqual(peer.state, 0)
        self.assertEqual(peer.armed_count, 1)
        self.assertGreaterEqual(peer.sent_arm_after_stops, 3)
        self.assertGreater(peer.heartbeats, 0)

    def test_rejected_arm_does_not_retry(self):
        clock, peer = self.run_peer(reject=True)
        with self.assertRaises(RuntimeError):
            car_auto.drive(peer,60,150,80,.5,clock.now,clock.sleep,lambda s: None)
        self.assertEqual(peer.armed_count, 1)
        self.assertEqual(peer.state, 0)

    def test_silent_ack_and_partial_write_stop(self):
        for options in [dict(silent=True), dict(short=True), dict(fault=True)]:
            clock, peer = self.run_peer(**options)
            with self.assertRaises((RuntimeError, OSError)):
                car_auto.drive(peer,60,150,80,.5,clock.now,clock.sleep,lambda s: None)
            self.assertEqual(peer.state, 0)
            self.assertLessEqual(peer.armed_count, 1)

    def test_rolling_wheels_cannot_arm(self):
        clock, peer = self.run_peer(rolling=True)
        with self.assertRaises(RuntimeError):
            car_auto.drive(peer,60,150,80,.5,clock.now,clock.sleep,lambda s: None)
        self.assertEqual(peer.armed_count, 0)

    def test_bad_duration_has_no_serial_side_effect(self):
        for duration in (0, -1, math.nan, math.inf, 61):
            clock, peer = self.run_peer()
            with self.assertRaises(ValueError):
                car_auto.drive(peer,60,150,80,duration,clock.now,clock.sleep,lambda s: None)
            self.assertEqual(peer.frames, 0)


if __name__ == '__main__': unittest.main()
