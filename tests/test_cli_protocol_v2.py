"""CLI compatibility against the RTL wire vector and interrupted mixed streams."""
import contextlib
import csv
import io
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from host.core import BASE_FIELDS, EXTENDED_FIELDS
from tools import telemetry_monitor as monitor


ROOT = Path(__file__).resolve().parents[1]
# Independently emitted by tb_telemetry_v2, including its wire CRC.
RTL_V2 = bytes.fromhex(
    "aa5502570000fd0209fd0c007afe6000e80303012195010478563412"
    "0204fffffeff0902e8030a0213000b02db000c02ff020d02ff030e020000"
    "0f02ff031002e01f110238ff120120130400000200140498badcfeb735")


def legacy_frame(sequence):
    payload = b"\xaa\x55\x01\x18" + struct.pack(
        "<HHhhhhh", sequence, 800, -1024, 2048, 0, -100, -250)
    payload += bytes((2, 1, 0, 0))
    return payload + monitor.crc16(payload).to_bytes(2, "little")


class FakeSerial:
    def __init__(self, chunks, clock):
        self.chunks = iter(chunks)
        self.clock = clock
        self.writes = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.closed = True

    def read(self, size):
        if size != 128:
            raise AssertionError("unexpected serial read size")
        self.clock[0] += 1
        return next(self.chunks, b"")

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)


class CliProtocolV2Tests(unittest.TestCase):
    def test_public_decode_reads_rtl_v2_golden_and_keeps_time(self):
        with patch.object(monitor, "time", types.SimpleNamespace(time=lambda: 123.5)):
            result = monitor.decode(RTL_V2)
        expected = {
            "time": 123.5, "sequence": 0, "adc": 765, "theta_q10": -759,
            "omega_q10": 12, "arm_q10": -390, "arm_speed_q10": 96,
            "command_permille": 1000, "state": 3, "calibrated": 1,
            "fault": 0x21, "protocol_version": 2, "raw_hex": RTL_V2.hex(),
            "device_time_ms": 0x12345678, "encoder_count": -65537,
            "control_loop_us": 1000, "sensor_flags": 0x13,
            "adc_down": 219, "adc_up": 767, "adc_raw": 1023,
            "adc_window_min": 0, "adc_window_max": 1023, "adc_mean_q4": 8160,
            "motor_command_permille": -200, "first_fault": 0x20,
            "firmware_version": 0x00020000, "sample_counter": 0xFEDCBA98,
        }
        for name, value in expected.items():
            with self.subTest(field=name):
                self.assertEqual(result[name], value)
        self.assertNotIn("observer_confidence", result)

    def test_public_decode_rejects_corrupt_or_incomplete_v2(self):
        corrupt = bytearray(RTL_V2)
        corrupt[30] ^= 1
        for frame in (bytes(corrupt), RTL_V2[:-1], RTL_V2 + b"\x00"):
            with self.subTest(frame=frame.hex()), self.assertRaises(ValueError):
                monitor.decode(frame)

    def test_cli_mixed_fragmented_crc_recovery_and_fixed_csv_columns(self):
        corrupt = bytearray(RTL_V2)
        corrupt[8] ^= 1
        wire = (b"noise\xaa" + legacy_frame(65535) + b"\xaa\x55\x01\x19junk"
                + corrupt + RTL_V2 + legacy_frame(1))
        chunks = [wire[offset:offset + 13] for offset in range(0, len(wire), 13)]
        clock = [0]
        port = FakeSerial(chunks, clock)
        factory = unittest.mock.Mock(return_value=port)
        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0])
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "mixed.csv"
            argv = ["telemetry_monitor", "--port", "TEST", "--seconds", str(len(chunks)),
                    "--csv", str(csv_path)]
            with patch.object(sys, "argv", argv), patch.object(monitor, "time", fake_time), \
                    patch.dict(sys.modules, {"serial": types.SimpleNamespace(Serial=factory)}), \
                    contextlib.redirect_stdout(output):
                monitor.main()
            with csv_path.open(encoding="utf-8-sig", newline="") as stream:
                reader = csv.DictReader(stream)
                rows = list(reader)
                self.assertEqual(reader.fieldnames, BASE_FIELDS + EXTENDED_FIELDS)
        factory.assert_called_once_with("TEST", 115200, timeout=0.1)
        self.assertTrue(port.closed)
        self.assertEqual(port.writes, [])
        self.assertEqual([row["sequence"] for row in rows], ["65535", "0", "1"])
        self.assertEqual([row["protocol_version"] for row in rows], ["1", "2", "1"])
        self.assertEqual([row["adc_raw"] for row in rows], ["", "1023", ""])
        self.assertEqual([row["motor_command_permille"] for row in rows], ["", "-200", ""])
        self.assertEqual(rows[1]["raw_hex"], RTL_V2.hex())
        self.assertTrue(all(row["observer_confidence"] == "" for row in rows))
        self.assertTrue(all(None not in row for row in rows))
        lines = output.getvalue().splitlines()
        self.assertEqual(len(lines), 3)
        self.assertIn("seq=0 state=3 ADC=765", lines[1])
        self.assertIn("fault=0x21", lines[1])

    def test_cli_only_sends_the_explicit_command_once(self):
        for command in ("D", "U", "G", "H", "S", "R", "F", "B"):
            with self.subTest(command=command):
                port = FakeSerial([], [0])
                argv = ["telemetry_monitor", "--port", "TEST", "--seconds", "0",
                        "--command", command]
                with patch.object(sys, "argv", argv), patch.dict(
                        sys.modules, {"serial": types.SimpleNamespace(Serial=lambda *a, **k: port)}):
                    monitor.main()
                self.assertEqual(port.writes, [command.encode("ascii")])
                self.assertTrue(port.closed)

    def test_direct_script_help_works_outside_project_without_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-B", str(ROOT / "tools/telemetry_monitor.py"), "--help"],
                cwd=directory, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--command {D,U,G,H,S,R,F,B,J,K,L,M}", result.stdout)


if __name__ == "__main__":
    unittest.main()
