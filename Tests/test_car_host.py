import importlib.util
from pathlib import Path
import struct
import unittest
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location('car_host', Path(__file__).resolve().parents[1] / 'Tools/car_host.py')
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)

class ProtocolTests(unittest.TestCase):
    def test_crc_golden(self):
        self.assertEqual(host.crc8(b'123456789'), 0xf4)
        self.assertEqual(host.packet(2), bytes.fromhex('c3 3c 02 00 2a'))

    def test_signed_and_range(self):
        self.assertEqual(host.wheels(80, -80)[4:8], struct.pack('>hh', 80, -80))
        with self.assertRaises(ValueError): host.wheels(181, 0)

    def test_pid_validation(self):
        self.assertEqual(host.pid_packet(1, '2', '.12', '0')[4:-1], bytes.fromhex('01 00 c8 00 0c 00 00'))
        for value in ['nan', 'inf', '-1', '0.001', '656']:
            with self.assertRaises(ValueError): host.pid_packet(0, value, '0', '0')

    def test_differential_mix(self):
        self.assertEqual(host.mix(1, 0, 80), (80, 80))
        self.assertEqual(host.mix(0, 1, 80), (-80, 80))
        self.assertEqual(host.mix(0, 0, 80), (0, 0))
        l, r = host.mix(1, 1, 180)
        self.assertLess(l, r)
        self.assertLessEqual(max(abs(l), abs(r)), 180)

    def test_fragmented_telemetry(self):
        frame = struct.pack('<16f', *range(16)) + host.TAIL
        reader = host.TelemetryReader()
        self.assertEqual(reader.feed(b'garbage' + frame[:12]), [])
        result = reader.feed(frame[12:] + frame)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0][3], 3)
        reader.feed(b'x' * 10000)
        self.assertLessEqual(len(reader.buffer), 67)

class Var:
    def __init__(self, value): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value

class Port:
    def __init__(self): self.sent = []; self.input = b''; self.closed = False
    def write(self, data): self.sent.append(data); return len(data)
    def read(self, count): data, self.input = self.input[:count], self.input[count:]; return data
    @property
    def in_waiting(self): return len(self.input)
    def close(self): self.closed = True

class DrivingSafetyTests(unittest.TestCase):
    def setUp(self):
        self.app = host.Console.__new__(host.Console)
        self.app.port = Port()
        self.app.armed = False
        self.app.pending_arm = False
        self.app.stop_retries = 0
        self.app.stop_retry_at = 0
        self.app.keys = set()
        self.app.sample = None
        self.app.last_rx = host.time.monotonic()
        self.app.next_tx = 0
        self.app.rpm = Var(60)
        self.app.status = Var('')
        self.app.values = Var('')
        self.app.actual_pid = Var('')
        self.app.log_file = None
        self.app.reader = host.TelemetryReader()
        self.app.root = SimpleNamespace(after=lambda *a: None, focus_get=lambda: None)
        self.app.tk = SimpleNamespace(TclError=RuntimeError)

    def telemetry(self, state, fault=0):
        self.app.port.input += struct.pack('<16f', state, fault, *([0] * 14)) + host.TAIL
        self.app.poll()

    def test_cannot_arm_without_stopped_telemetry(self):
        self.app.arm()
        self.assertEqual(self.app.port.sent, [])
        self.telemetry(2, 1)
        self.app.arm()
        self.assertEqual(self.app.port.sent, [])

    def test_requires_confirmation_then_release_brakes(self):
        self.telemetry(0)
        self.app.arm()
        self.assertFalse(self.app.armed)
        self.app.set_key('w', True)
        self.assertEqual(self.app.port.sent, [host.packet(1)])
        self.telemetry(1)
        self.app.set_key('w', True)
        self.assertEqual(self.app.port.sent[-1], host.wheels(60, 60))
        self.app.set_key('w', False)
        self.assertEqual(self.app.port.sent[-1], host.wheels(0, 0))

    def test_fault_and_focus_loss_never_resume(self):
        self.app.armed = True
        self.app.keys.add('w')
        self.telemetry(2, 1)
        self.assertFalse(self.app.armed)
        self.assertFalse(self.app.keys)
        self.telemetry(1)
        self.assertFalse(self.app.armed)
        self.app.armed = True
        self.app.focus_check()
        self.assertEqual(self.app.port.sent[-1], host.packet(2))
        self.assertFalse(self.app.armed)

    def test_stale_telemetry_disarms(self):
        self.app.armed = True
        self.app.last_rx -= 1
        self.app.poll()
        self.assertEqual(self.app.port.sent[-1], host.packet(2))
        self.assertFalse(self.app.armed)

    def test_stop_retries_after_fragment_timeout_and_blocks_arm(self):
        self.app.armed = True
        self.app.stop()
        self.telemetry(0)
        self.app.arm()
        self.assertNotIn(host.packet(1), self.app.port.sent)
        self.app.stop_retry_at = 0
        self.app.poll()
        self.assertEqual(self.app.port.sent, [host.packet(2), host.packet(2)])

if __name__ == '__main__': unittest.main()
