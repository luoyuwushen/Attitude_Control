"""New FPGA diagnostic bytes remain readable by the released host decoder."""
import struct
import unittest

from host.core import StreamDecoder
from tools.telemetry_monitor import crc16, decode


def frame(diagnostic, fault=0):
    data = b'\xaa\x55\x01\x18' + struct.pack('<HHhhhhh', 17, 240, 3210, 0, 0, 0, 0)
    data += bytes([0, 1, fault, diagnostic])
    return data + crc16(data).to_bytes(2, 'little')


class StartDiagnosticsProtocolTests(unittest.TestCase):
    def test_nonzero_diagnostic_is_backward_compatible_and_retained_in_raw_frame(self):
        for diagnostic in (0x80, 0x90, 0xB1, 0xC4, 0xD8, 0xE0, 0xF0):
            with self.subTest(diagnostic=diagnostic):
                data = frame(diagnostic, fault=1)
                self.assertEqual(decode(data)['fault'], 1)
                rows = StreamDecoder().feed(data)
                self.assertEqual(len(rows), 1)
                self.assertEqual((rows[0]['state'], rows[0]['calibrated'], rows[0]['fault']), (0, 1, 1))
                self.assertEqual(bytes.fromhex(rows[0]['raw_hex'])[21], diagnostic)

    def test_legacy_reserved_zero_frame_remains_valid(self):
        rows = StreamDecoder().feed(frame(0))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['fault'], 0)
        self.assertEqual(bytes.fromhex(rows[0]['raw_hex'])[21], 0)

    def test_diagnostic_byte_is_covered_by_crc(self):
        damaged = bytearray(frame(0xB1, fault=1))
        damaged[21] ^= 0x10
        with self.assertRaises(ValueError):
            decode(bytes(damaged))
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(damaged), [])
        self.assertEqual(decoder.stats['crc_errors'], 1)


if __name__ == '__main__':
    unittest.main()
