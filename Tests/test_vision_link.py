import binascii
import importlib.util
from pathlib import Path
import struct
import unittest

spec = importlib.util.spec_from_file_location('vision_link', Path(__file__).resolve().parents[1] / 'Tools/vision_link.py')
link = importlib.util.module_from_spec(spec)
spec.loader.exec_module(link)

class VisionWireTests(unittest.TestCase):
    def test_crc_and_wire_layout(self):
        self.assertEqual(binascii.crc_hqx(b'123456789', 0xffff), 0x29b1)
        data = link.frame(1, 1, 123, link.track_payload(valid_mask=15, confidence=900, lateral_mm=50, heading_mrad=100, curvature_milli=20, element=6, distance_mm=80), 30)
        self.assertEqual(len(data), 32)
        self.assertEqual(data[:14], bytes.fromhex('a5 5a 01 01 00 10 00 01 00 00 00 7b 00 1e'))
        self.assertEqual(struct.unpack('>H', data[-2:])[0], binascii.crc_hqx(data[2:-2], 0xffff))

    def test_invalid_fields(self):
        for args in [dict(confidence=1001), dict(tag_id=8), dict(heading_mrad=4000), dict(valid_mask=16)]:
            with self.assertRaises(ValueError): link.track_payload(**args)
        with self.assertRaises(ValueError): link.tile_payload(0, [7] * 9)
        with self.assertRaises(ValueError): link.frame(1, 0, 123, link.track_payload(), 201)

    def test_sender_session_and_sequence_wrap(self):
        class Port:
            def __init__(self): self.frames = []
            def write(self, data): self.frames.append(data); return len(data)
        port = Port(); sender = link.VisionSender(port, 123)
        sender.hello(); sender.sequence = 65535; sender.send_track()
        self.assertEqual(port.frames[0][3], 0)
        self.assertEqual(port.frames[1][6:8], b'\0\0')
        sender.send_tile(2, [0] * 9)
        self.assertEqual(port.frames[2][14], 2)

if __name__ == '__main__': unittest.main()
