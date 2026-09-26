import json
import math
import struct
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from host.core import Metrics, Replay, SessionRecorder, StreamDecoder, crc16, demo_record


def frame(sequence=0, version=1, tlv=b"", state=2, theta=-1024, command=-250):
    length = 24 + len(tlv)
    payload = bytes((0xaa, 0x55, version, length))
    payload += struct.pack("<HHhhhhh", sequence, 800, theta, 2048, 0, -100, command)
    payload += bytes((state, 1, 0, 0)) + tlv
    return payload + crc16(payload).to_bytes(2, "little")


class DecoderTests(unittest.TestCase):
    def test_crc_and_signed_current_protocol(self):
        self.assertEqual(crc16(b"123456789"), 0x29b1)
        with patch("host.core.time.monotonic", side_effect=[20.0, 20.5]):
            decoder = StreamDecoder()
            first, second = decoder.feed(frame(7) + frame(8))
        self.assertAlmostEqual(first["theta_deg"], -180 / math.pi)
        self.assertEqual(first["omega_rad_s"], 2)
        self.assertEqual(first["command_permille"], -250)
        self.assertEqual(first["theta_q10"], -1024)
        self.assertEqual(first["omega_q10"], 2048)
        self.assertEqual(first["arm_q10"], 0)
        self.assertEqual(first["arm_speed_q10"], -100)
        self.assertEqual(first["elapsed_s"], 0)
        self.assertEqual(second["elapsed_s"], 0.5)
        self.assertEqual(first["source"], "serial")
        self.assertNotIn("encoder_count", first)

    def test_split_glued_garbage_crc_resynchronization(self):
        decoder = StreamDecoder()
        damaged = bytearray(frame(1))
        damaged[8] ^= 1
        wire = b"garbage" + damaged + frame(2) + frame(3)
        results = []
        for byte in wire:
            results += decoder.feed(bytes([byte]))
        self.assertEqual([record["sequence"] for record in results], [2, 3])
        self.assertEqual(decoder.stats["crc_errors"], 1)
        self.assertEqual(decoder.stats["discarded_bytes"], 7 + 24)
        self.assertEqual(decoder.stats["frames"], 2)
        self.assertEqual(decoder.feed(b"\xaa"), [])
        self.assertEqual(decoder.feed(frame(4)[1:])[0]["sequence"], 4)

    def test_wrap_gap_duplicate_reset(self):
        decoder = StreamDecoder()
        records = decoder.feed(b"".join(frame(seq) for seq in (65534, 65535, 0, 3, 3, 1, 2)))
        self.assertEqual(decoder.stats["missing_frames"], 2)
        self.assertEqual(decoder.stats["duplicates"], 1)
        self.assertEqual(decoder.stats["resets"], 1)
        self.assertEqual(records[3]["sequence_gap"], 2)
        self.assertTrue(records[4]["sequence_duplicate"])
        self.assertTrue(records[5]["sequence_reset"])
        decoder.reset()
        self.assertEqual(decoder.stats["frames"], 0)
        self.assertEqual(decoder.feed(frame(500))[0]["sequence_gap"], 0)

    def test_v2_known_unknown_and_absent_fields(self):
        tlvs = b"\x01\x04" + struct.pack("<I", 123456)
        tlvs += b"\x02\x04" + struct.pack("<i", -30)
        tlvs += b"\x03\x02" + struct.pack("<h", -1024)
        tlvs += b"\x06\x04" + struct.pack("<f", 0.75)
        tlvs += b"\x63\x03abc"
        wire = frame(1, version=2, tlv=tlvs)
        result = StreamDecoder().feed(wire)[0]
        self.assertEqual(result["protocol_version"], 2)
        self.assertEqual(result["device_time_ms"], 123456)
        self.assertEqual(result["encoder_count"], -30)
        self.assertAlmostEqual(result["target_arm_deg"], -180 / math.pi)
        self.assertEqual(result["observer_confidence"], 0.75)
        self.assertNotIn("control_loop_us", result)
        self.assertEqual(result["raw_hex"], wire.hex())

    def test_reject_bad_sizes_truncation_duplicates_nonfinite(self):
        tlvs = [b"\x01\x01\x00", b"\x01", b"\x01\x04\x00",
                b"\x06\x04" + struct.pack("<f", float("nan")),
                b"\x06\x04" + struct.pack("<f", float("inf")),
                (b"\x09\x02\x01\x00") * 2]
        for tlv in tlvs:
            with self.subTest(tlv=tlv):
                decoder = StreamDecoder()
                results = decoder.feed(frame(1, version=2, tlv=tlv) + frame(2))
                self.assertEqual([record["sequence"] for record in results], [2])
                self.assertEqual(decoder.stats["unsupported_frames"], 1)


class RecorderReplayTests(unittest.TestCase):
    def test_roundtrip_dynamic_columns_events_raw_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder(tmp, {"source": "demo", "port": "DEMO"})
            one, two = demo_record(1, 0), demo_record(2, 1)
            two["future_counter"] = 40
            recorder.write(one)
            recorder.event("command", "S", one)
            recorder.write(two)
            recorder.extra_summary = {"recovery_delay_s": 0.3, "recovery_count": 1}
            summary_path = recorder.close()
            self.assertEqual(summary_path, recorder.close())
            saved = Replay.load(recorder.directory / "samples.csv")
            self.assertEqual(len(saved), 2)
            self.assertEqual(saved[0]["elapsed_s"], 0)
            self.assertEqual(saved[1]["elapsed_s"], 1)
            self.assertNotIn("future_counter", saved[0])
            self.assertNotIn("observer_confidence", saved[0])
            self.assertEqual(saved[1]["future_counter"], 40)
            self.assertEqual(saved[0]["theta_q10"], one["theta_q10"])
            self.assertEqual(saved[0]["source"], "replay")
            self.assertEqual(saved[0]["original_source"], "demo")
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["samples"], 2)
            self.assertEqual(summary["raw_frames"], 2)
            self.assertEqual(summary["recovery_delay_s"], 0.3)
            self.assertEqual(summary["recovery_count"], 1)
            raw = (recorder.directory / "raw_frames.bin").read_bytes()
            self.assertEqual(len(StreamDecoder().feed(raw)), 2)
            events = (recorder.directory / "events.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(json.loads(events[0])["kind"], "command")
            self.assertEqual(list(recorder.directory.glob(".samples_*.tmp")), [])

    def test_bad_crc_never_written_to_raw(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder(tmp, {})
            record = demo_record(0, 0)
            record["raw_hex"] = record["raw_hex"][:-4] + "0000"
            recorder.write(record)
            recorder.close()
            self.assertEqual((recorder.directory / "raw_frames.bin").read_bytes(), b"")

    def test_legacy_csv_nonfinite_fields_and_backwards_time(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "legacy.csv"
            path.write_text("time,sequence,theta_deg,arm_deg,state,calibrated,fault\n"
                            "100,0,1,NaN,1,1,0\n102,1,2,3,2,1,0\n"
                            "50,2,3,4,2,1,0\n51,3,4,5,2,1,0\n"
                            "52,4,nan,6,2,1,0\n", encoding="utf-8")
            records = Replay.load(path)
            self.assertEqual(len(records), 4)
            self.assertEqual([r["elapsed_s"] for r in records], [0, 2, 2, 3])
            self.assertTrue(records[2]["time_reset"])
            self.assertNotIn("arm_deg", records[0])
            self.assertEqual(records[0]["protocol_version"], 1)

    def test_skipped_csv_row_cannot_hide_gap_with_saved_zero_flags(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "damaged.csv"
            path.write_text("elapsed_s,sequence,theta_deg,state,calibrated,fault,sequence_gap,sequence_reset\n"
                            "0,0,1,1,1,0,0,False\n.02,1,nan,1,1,0,0,False\n"
                            ".04,2,1,2,1,0,0,False\n", encoding="utf-8")
            records = Replay.load(path)
            self.assertEqual(len(records), 2)
            self.assertEqual(records[1]["sequence_gap"], 1)
            metrics = Metrics()
            for record in records:
                metrics.add(record)
            self.assertIsNone(metrics.snapshot()["capture_delay_s"])

    def test_csv_without_finite_time_rejected_instead_of_zero_capture(self):
        cases = (
            "sequence,theta_deg,state,calibrated,fault\n0,1,1,1,0\n1,1,2,1,0\n",
            "time,sequence,theta_deg,state,calibrated,fault\nNaN,0,1,1,1,0\nNaN,1,1,2,1,0\n",
            "elapsed_s,host_monotonic,time,sequence,theta_deg,state\ninf,-inf,NaN,0,1,1\n",
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "untimed.csv"
            for content in cases:
                with self.subTest(content=content):
                    path.write_text(content, encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "时间"):
                        Replay.load(path)

    def test_csv_missing_selected_clock_cannot_bridge_continuity(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "partial_time.csv"
            for missing in ("", "NaN", "inf"):
                with self.subTest(missing=missing):
                    # Another clock does not silently replace the selected elapsed
                    # clock mid-session and create a fabricated continuous interval.
                    path.write_text("elapsed_s,time,sequence,theta_deg,state,calibrated,fault\n"
                                    "0,100,0,1,2,1,0\n"
                                    f"{missing},100.02,1,1,2,1,0\n"
                                    ".04,100.04,2,1,2,1,0\n", encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "第 3 行.*elapsed_s"):
                        Replay.load(path)

    def test_recorded_replay_preserves_earliest_demo_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            with SessionRecorder(tmp, {"source": "demo"}) as original:
                original.write(demo_record(0, 0))
            first_replay = Replay.load(original.directory / "samples.csv")
            self.assertEqual(first_replay[0]["original_source"], "demo")
            with SessionRecorder(tmp, {"source": "replay"}) as rerecorded:
                rerecorded.write(first_replay[0])
            second_replay = Replay.load(rerecorded.directory / "samples.csv")
            self.assertEqual(second_replay[0]["source"], "replay")
            self.assertEqual(second_replay[0]["original_source"], "demo")

    def test_demo_frame_is_decodable_and_labelled(self):
        record = demo_record(8.5, 65538)
        decoded = StreamDecoder().feed(bytes.fromhex(record["raw_hex"]))[0]
        self.assertEqual(decoded["sequence"], 2)
        self.assertEqual(record["source"], "demo")
        self.assertEqual(record["theta_deg"], decoded["theta_deg"])

    def test_demo_idle_has_zero_velocity_and_command(self):
        for t in (0, 0.5, 24.5):
            record = demo_record(t, 0)
            self.assertEqual(record["state"], 0)
            self.assertEqual(record["omega_rad_s"], 0)
            self.assertEqual(record["arm_speed_rad_s"], 0)
            self.assertEqual(record["command_permille"], 0)

    def test_periodic_flush_including_idle_without_each_sample_flush(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(SessionRecorder, "FLUSH_INTERVAL_S", 0.02):
            recorder = SessionRecorder(tmp, {})
            try:
                with patch.object(recorder, "_flush_files", wraps=recorder._flush_files) as flush:
                    for seq in range(10):
                        recorder.write(demo_record(seq * 0.02, seq))
                    self.assertLess(flush.call_count, 10)
                    # A quiet recorder must flush without depending on another sample.
                    threading.Event().wait(0.08)
                    self.assertGreaterEqual(flush.call_count, 1)
                saved = Replay.load(recorder.directory / "samples.csv")
                self.assertEqual(len(saved), 10)
                self.assertEqual(len((recorder.directory / "raw_frames.bin").read_bytes()), 240)
            finally:
                recorder.close()
            self.assertFalse(recorder._flush_thread.is_alive())

    def test_dynamic_column_rewrite_streams_rows_and_failure_keeps_old_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder(tmp, {})
            try:
                recorder.write(demo_record(0, 0))
                extended = demo_record(0.02, 1)
                extended["new_counter"] = 42
                with patch.object(Path, "replace", side_effect=OSError("simulated replace failure")):
                    with self.assertRaises(OSError):
                        recorder.write(extended)
                recorder.write(demo_record(0.04, 2))
                recorder.write(extended)
            finally:
                recorder.close()
            saved = Replay.load(recorder.directory / "samples.csv")
            self.assertEqual(len(saved), 3)
            self.assertEqual([r["sequence"] for r in saved], [0, 2, 1])
            self.assertEqual(saved[2]["new_counter"], 42)
            self.assertEqual(list(recorder.directory.glob(".samples_*.tmp")), [])

    def test_summary_failure_can_be_retried_after_samples_are_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SessionRecorder(tmp, {})
            recorder.write(demo_record(0, 0))
            with patch.object(Path, "write_text", side_effect=OSError("summary failure")):
                with self.assertRaises(OSError):
                    recorder.close()
            self.assertEqual(len(Replay.load(recorder.directory / "samples.csv")), 1)
            path = recorder.close()
            self.assertTrue(path.exists())
            self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["samples"], 1)


class MetricsTests(unittest.TestCase):
    @staticmethod
    def record(sequence, t, state=2, theta=3, calibrated=1, fault=0, command=0):
        return dict(sequence=sequence, elapsed_s=t, state=state, theta_deg=theta,
                    calibrated=calibrated, fault=fault, command_permille=command, arm_deg=sequence)

    def test_valid_balance_rms_saturation_and_capture(self):
        metrics = Metrics()
        for record in (self.record(0, 0, state=1, command=1000),
                       self.record(1, .02, state=1), self.record(2, .04, theta=3),
                       self.record(3, .06, theta=4, command=-1000),
                       self.record(4, .08, theta=100, calibrated=0),
                       self.record(5, .1, theta=100, fault=1)):
            metrics.add(record)
        result = metrics.snapshot()
        self.assertEqual(result["samples"], 6)
        self.assertEqual(result["balance_samples"], 2)
        self.assertAlmostEqual(result["balance_rms_deg"], math.sqrt(12.5))
        self.assertEqual(result["balance_peak_deg"], 4)
        self.assertEqual(result["balance_mean_deg"], 3.5)
        self.assertEqual(result["capture_delay_s"], .04)
        self.assertEqual(result["capture_count"], 1)
        self.assertEqual(result["total_duration_s"], .1)
        self.assertAlmostEqual(result["longest_balance_s"], .02)
        self.assertIsNone(result["longest_in_band_s"])
        self.assertEqual(result["saturation_ratio"], 2 / 6)
        self.assertEqual(result["swing_saturation_ratio"], 1 / 2)
        self.assertEqual(result["balance_saturation_ratio"], 1 / 4)

    def test_gap_reset_and_fault_prevent_false_capture(self):
        for records in ((self.record(0, 0, state=1), self.record(2, .02)),
                        (self.record(100, 0, state=1), self.record(0, .02)),
                        (self.record(0, 0, state=1), self.record(1, .02, state=1, fault=1), self.record(2, .04)),
                        (self.record(0, 0, state=1), self.record(1, .11))):
            metrics = Metrics()
            for record in records:
                metrics.add(record)
            self.assertIsNone(metrics.snapshot()["capture_delay_s"])
        metrics = Metrics()
        for record in (self.record(0, 0, state=1), self.record(2, .02, state=1), self.record(3, .04)):
            metrics.add(record)
        self.assertIsNone(metrics.snapshot()["capture_delay_s"])
        self.assertEqual(metrics.snapshot()["missing_frames"], 1)

    def test_duplicates_not_double_counted_and_empty_measurement_none(self):
        metrics = Metrics()
        self.assertIsNone(metrics.snapshot()["balance_rms_deg"])
        metrics.add(self.record(0, 0))
        metrics.add(self.record(0, 0))
        self.assertEqual(metrics.snapshot()["balance_samples"], 1)
        self.assertEqual(metrics.snapshot()["duplicate_count"], 1)

    def test_continuous_balance_and_two_degree_band(self):
        metrics = Metrics()
        for record in (self.record(0, 0, theta=1), self.record(1, .02, theta=-2),
                       self.record(2, .04, theta=2), self.record(3, .06, theta=2.01),
                       self.record(4, .08, theta=1), self.record(5, .1, theta=1)):
            metrics.add(record)
        self.assertAlmostEqual(metrics.snapshot()["longest_balance_s"], .1)
        self.assertAlmostEqual(metrics.snapshot()["longest_in_band_s"], .04)
        self.assertAlmostEqual(metrics.snapshot()["balance_mean_deg"], 5.01 / 6)

    def test_every_discontinuity_splits_continuous_intervals(self):
        breakers = (
            self.record(4, .04, theta=1),             # missing sequence
            self.record(0, .04, theta=1),             # reset/backwards sequence
            self.record(2, .14, theta=1),             # receive timeout
            self.record(2, .01, theta=1),             # receive clock backwards
            self.record(2, .04, theta=1, fault=1),
            self.record(2, .04, theta=1, calibrated=0),
            self.record(2, .04, theta=1, state=0),
        )
        for breaker in breakers:
            with self.subTest(breaker=breaker):
                metrics = Metrics()
                metrics.add(self.record(0, 0, theta=1))
                metrics.add(self.record(1, .02, theta=1))
                metrics.add(breaker)
                metrics.add(self.record((breaker["sequence"] + 1) & 65535, breaker["elapsed_s"] + .02, theta=1))
                result = metrics.snapshot()
                self.assertAlmostEqual(result["longest_balance_s"], .02)
                self.assertAlmostEqual(result["longest_in_band_s"], .02)

    def test_capture_count_and_phase_specific_saturation(self):
        metrics = Metrics()
        for record in (self.record(0, 0, state=1, command=800),
                       self.record(1, .02, state=1, command=-799),
                       self.record(2, .04, theta=1, command=1000),
                       self.record(3, .06, state=0),
                       self.record(4, .08, state=1, command=-800),
                       self.record(5, .1, theta=1, command=-999)):
            metrics.add(record)
        result = metrics.snapshot()
        self.assertEqual(result["capture_count"], 2)
        self.assertAlmostEqual(result["capture_delay_s"], .04)
        self.assertEqual(result["swing_saturation_ratio"], 2 / 3)
        self.assertEqual(result["balance_saturation_ratio"], 1 / 2)
        self.assertEqual(result["saturation_ratio"], 1 / 6)

    def test_receive_gap_boundary_and_missing_elapsed(self):
        for delay, captures in ((.1, 1), (.10001, 0)):
            metrics = Metrics()
            metrics.add(self.record(0, 0, state=1))
            metrics.add(self.record(1, delay))
            self.assertEqual(metrics.snapshot()["capture_count"], captures)
        metrics = Metrics()
        metrics.add(self.record(0, 0, theta=1))
        invalid = self.record(1, .02, theta=1)
        invalid.pop("elapsed_s")
        metrics.add(invalid)
        metrics.add(self.record(2, .04, theta=1))
        self.assertEqual(metrics.snapshot()["longest_balance_s"], 0)


if __name__ == "__main__":
    unittest.main()
