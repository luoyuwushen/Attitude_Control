import struct
import unittest
from tools.telemetry_monitor import crc16, decode


class TelemetryTests(unittest.TestCase):
    def test_standard_crc_vector(self):
        self.assertEqual(crc16(b'123456789'), 0x29b1)

    def test_signed_frame_and_corruption(self):
        payload = b'\xaa\x55\x01\x18' + struct.pack('<HHhhhhh', 7, 800, -1024, 2048, 0, -100, -250)
        payload += bytes([2, 1, 0, 0])
        frame = payload + crc16(payload).to_bytes(2, 'little')
        result = decode(frame)
        self.assertEqual(result['omega_rad_s'], 2)
        self.assertEqual(result['command_permille'], -250)
        self.assertEqual(result['calibrated'], 1)
        self.assertLess(result['theta_deg'], 0)
        damaged = bytearray(frame)
        damaged[8] ^= 1
        with self.assertRaises(ValueError):
            decode(bytes(damaged))


if __name__ == '__main__':
    unittest.main()
