"""Version-3 historical capture codec tests, including independent wire vectors."""
import json
import struct
import unittest

from host.control_trace import decode_trace_frame, TraceCaptureCollector


# Fixed wire vectors, generated once with the bit-at-a-time CCITT reference,
# not the production binascii decoder/collector. The row integral is -1.25.
GOLD_ROW = bytes.fromhex("ffffffffc01dfeff3d3000fc00089dff00800000c0fe85ff7b00000203000c00")
GOLD_META = bytes.fromhex("aa5503280101341200000100000078563412e40d3d309dff50c300004030201001000000010091f5")
GOLD_DATA = bytes.fromhex("aa55033002013412000001000100ffffffffc01dfeff3d3000fc00089dff00800000c0fe85ff7b00000203000c000c61")
GOLD_END = bytes.fromhex("aa55031203013412010001000000083d5a11")


def reference_crc(data, initial=0xffff):
    value = initial
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xffff
    return value


def packet(kind, index=0, total=1, count=0, payload=b"", capture=0x1234, schema=1, flags=0):
    body = bytes([0xaa, 0x55, 3, 16 + len(payload)])
    body += struct.pack("<BBHHHBB", kind, schema, capture, index, total, count, flags) + payload
    return body + reference_crc(body).to_bytes(2, "little")


def metadata(total=1, capture=0x1234, **fields):
    values = dict(firmware=0x12345678, down=3556, up=12349, arm=-99, period=50000,
                  freeze_time=0x10203040, committed=total, reason=1, status=0)
    values.update(fields)
    payload = struct.pack("<IHHhIIIBB", *values.values())
    return packet(1, total=total, capture=capture, payload=payload)


def row(**fields):
    values = dict(sample=0xffffffff, encoder=-123456, adc=12349, theta=-1024,
                  omega=2048, arm=-99, speed=-32768, integral=-20971520,
                  command=-123, motor=123, age=512, h_flags=3,
                  quality=0, sensor_flags=12, fault=0)
    values.update(fields)
    return struct.pack("<IiHhhhhihhHBBBB", *values.values())


def data(rows, index=0, total=None, capture=0x1234):
    return packet(2, index=index, total=len(rows) if total is None else total,
                  count=len(rows), capture=capture, payload=b"".join(rows))


def end(rows, total=None, capture=0x1234, crc=None):
    total = len(rows) if total is None else total
    crc = reference_crc(b"".join(rows)) if crc is None else crc
    return packet(3, index=total, total=total, capture=capture,
                  payload=crc.to_bytes(2, "little"))


def rewrite(frame, offset, replacement):
    body = bytearray(frame[:-2])
    body[offset:offset + len(replacement)] = replacement
    return bytes(body) + reference_crc(body).to_bytes(2, "little")


class TraceDecoderTests(unittest.TestCase):
    def test_reference_crc_standard_vector_and_fixed_packets(self):
        self.assertEqual(reference_crc(b"123456789"), 0x29b1)
        self.assertEqual(row(), GOLD_ROW)
        self.assertEqual(metadata(), GOLD_META)
        self.assertEqual(data([GOLD_ROW]), GOLD_DATA)
        self.assertEqual(end([GOLD_ROW]), GOLD_END)
        self.assertEqual(reference_crc(GOLD_ROW), 0x3d08)

    def test_independent_golden_row_signed_fields_and_q24(self):
        decoded = decode_trace_frame(GOLD_DATA)
        self.assertEqual(decoded["protocol_version"], 3)
        self.assertEqual(decoded["capture_id"], 0x1234)
        self.assertEqual(decoded["schema"], 1)
        r = decoded["rows"][0]
        expected = dict(sample_counter=0xffffffff, encoder_count=-123456,
                        adc_mean_q4=12349, theta_q10=-1024, omega_q10=2048,
                        arm_q10=-99, arm_speed_q10=-32768,
                        integral_q24=-20971520, command_permille=-123,
                        motor_command_permille=123, control_age_ms=512,
                        h_flags=3, adc_quality=0, sensor_flags=12, fault=0)
        for key, value in expected.items():
            self.assertEqual(r[key], value, key)
        self.assertEqual(r["integral_permille"], -1.25)
        self.assertEqual(r["omega_rad_s"], 2)
        self.assertEqual(r["arm_speed_rad_s"], -32)
        self.assertEqual(r["raw_hex"], GOLD_ROW.hex())
        self.assertEqual(decoded["raw_hex"], GOLD_DATA.hex())
        self.assertNotIn("state", r)
        self.assertNotIn("host_monotonic", r)

    def test_golden_metadata_and_end(self):
        decoded = decode_trace_frame(GOLD_META)
        self.assertEqual(decoded["metadata"], dict(
            firmware_id=0x12345678, adc_down_q4=3556, adc_up_q4=12349,
            capture_arm_q10=-99, period_cycles=50000, freeze_device_ms=0x10203040,
            total_committed=1, freeze_reason=1, status=0))
        self.assertEqual(decode_trace_frame(GOLD_END)["record_crc16"], 0x3d08)

    def test_q24_preserves_all_signed_32_bits(self):
        values = [-2147483648, -1677721600, -1, 0, 1, 1677721600, 2147483647]
        decoded = decode_trace_frame(data([row(integral=v) for v in values]))
        self.assertEqual([r["integral_q24"] for r in decoded["rows"]], values)
        self.assertEqual([r["integral_permille"] for r in decoded["rows"]],
                         [v / 16777216 for v in values])

    def test_nonzero_chunk_index_and_seven_rows(self):
        decoded = decode_trace_frame(data([row(sample=i) for i in range(7)], index=9, total=16))
        self.assertEqual(len(bytes.fromhex(decoded["raw_hex"])), 240)
        self.assertEqual([r["index"] for r in decoded["rows"]], list(range(9, 16)))
        self.assertEqual([r["sample_counter"] for r in decoded["rows"]], list(range(7)))

    def test_raw_warning_and_fault_bits_are_preserved(self):
        decoded = decode_trace_frame(data([row(quality=0x20, sensor_flags=0x1f, fault=0xff, h_flags=0xf)]))
        self.assertEqual(decoded["rows"][0]["adc_quality"], 0x20)
        self.assertEqual(decoded["rows"][0]["fault"], 0xff)

    def test_bytes_like_inputs(self):
        for frame in (GOLD_DATA, bytearray(GOLD_DATA), memoryview(GOLD_DATA)):
            self.assertEqual(decode_trace_frame(frame)["rows"][0]["integral_q24"], -20971520)

    def test_every_truncation_and_trailing_byte_rejected(self):
        for frame in (GOLD_META, GOLD_DATA, GOLD_END):
            for length in range(len(frame)):
                with self.subTest(length=length, kind=frame[4]):
                    with self.assertRaises(ValueError):
                        decode_trace_frame(frame[:length])
            with self.assertRaises(ValueError):
                decode_trace_frame(frame + b"\x00")

    def test_bad_crc_and_unrecognised_envelope(self):
        cases = [b"", None, "aa55", bytes(241), rewrite(GOLD_META, 0, b"\x00"),
                 rewrite(GOLD_META, 2, b"\x02"), rewrite(GOLD_META, 3, b"\x27"),
                 rewrite(GOLD_META, 4, b"\x04"), rewrite(GOLD_META, 5, b"\x02"),
                 rewrite(GOLD_META, 13, b"\x01"), GOLD_DATA[:-1] + bytes([GOLD_DATA[-1] ^ 1])]
        for frame in cases:
            with self.subTest(frame=frame):
                with self.assertRaises(ValueError):
                    decode_trace_frame(frame)

    def test_metadata_field_validation(self):
        cases = [metadata(total=4097), metadata(down=16384), metadata(up=65535),
                 metadata(period=0), metadata(reason=0), metadata(reason=4),
                 metadata(status=2), metadata(total=2, committed=1),
                 rewrite(GOLD_META, 8, b"\x01\x00"), rewrite(GOLD_META, 12, b"\x01"),
                 packet(1, payload=bytes(23))]
        for frame in cases:
            with self.subTest(frame=frame.hex()):
                with self.assertRaises(ValueError):
                    decode_trace_frame(frame)

    def test_data_field_and_payload_validation(self):
        cases = [packet(2, count=0), packet(2, count=8, total=8, payload=b""),
                 packet(2, count=1, payload=bytes(31)), data([row()], index=1, total=1),
                 data([row(adc=16384)]), data([row(command=1001)]),
                 data([row(motor=-1001)]), data([row(age=513)]),
                 data([row(h_flags=0x10)]), data([row(sensor_flags=0x80)])]
        for frame in cases:
            with self.subTest(frame=frame.hex()):
                with self.assertRaises(ValueError):
                    decode_trace_frame(frame)

    def test_end_field_validation(self):
        cases = [packet(3, index=0, total=1, payload=b"\xff\xff"),
                 packet(3, index=1, count=1, payload=b"\xff\xff"),
                 packet(3, index=1, payload=b"\xff"), packet(3, index=1, payload=b"\xff" * 3)]
        for frame in cases:
            with self.subTest(frame=frame.hex()):
                with self.assertRaises(ValueError):
                    decode_trace_frame(frame)


class TraceCollectorTests(unittest.TestCase):
    def assert_failed(self, results, error=None):
        self.assertEqual(len(results), 1)
        result = results[0]
        self.assertFalse(result["complete"])
        self.assertEqual(result["status"], "failed")
        self.assertTrue(result["errors"])
        if error is not None:
            self.assertIn(error, result["errors"])
        return result

    def test_golden_complete_capture_contains_wire_evidence(self):
        collector = TraceCaptureCollector()
        self.assertEqual(collector.feed(GOLD_META), [])
        self.assertEqual(collector.feed(GOLD_DATA), [])
        results = collector.feed(GOLD_END)
        self.assertEqual(len(results), 1)
        result = results[0]
        self.assertTrue(result["complete"])
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["errors"], [])
        self.assertEqual(result["received_rows"], 1)
        self.assertEqual(result["raw_frames_hex"], [p.hex() for p in (GOLD_META, GOLD_DATA, GOLD_END)])
        self.assertEqual(result["rows"][0]["raw_hex"], GOLD_ROW.hex())
        self.assertEqual(result["record_crc16"], result["computed_record_crc16"])
        self.assertEqual(json.loads(json.dumps(result)), result)
        self.assertIsNone(collector.snapshot())
        self.assertEqual(collector.finish(), [])

    def test_empty_capture_needs_metadata_and_end_crc_ffff(self):
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=0))
        self.assertFalse(collector.snapshot()["complete"])
        result = collector.feed(end([]))[0]
        self.assertTrue(result["complete"])
        self.assertEqual(result["rows"], [])
        self.assertEqual(result["computed_record_crc16"], 0xffff)
        collector.feed(metadata(total=0))
        self.assert_failed(collector.feed(end([], crc=0)), "record_crc_mismatch")

    def test_chunk_boundaries_preserve_order_and_whole_record_crc(self):
        rows = [row(sample=i, integral=-i) for i in range(17)]
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=17))
        for start, count in ((0, 7), (7, 3), (10, 7)):
            self.assertEqual(collector.feed(data(rows[start:start + count], index=start, total=17)), [])
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        self.assertEqual([r["index"] for r in result["rows"]], list(range(17)))
        self.assertEqual([r["sample_counter"] for r in result["rows"]], list(range(17)))
        self.assertEqual(result["record_crc16"], reference_crc(b"".join(rows)))

    def test_sample_counter_gaps_and_wrap_are_not_filled(self):
        samples = [0xfffffffe, 0xffffffff, 0, 7]
        rows = [row(sample=v) for v in samples]
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=4))
        collector.feed(data(rows))
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        self.assertEqual([r["sample_counter"] for r in result["rows"]], samples)
        continuity = result["sample_counter_continuity"]
        self.assertFalse(continuity["contiguous"])
        self.assertEqual(continuity["checked_intervals"], 3)
        self.assertEqual(continuity["wrap_count"], 1)
        self.assertEqual(continuity["missing_sample_count"], 6)
        self.assertEqual(continuity["gaps"], [dict(previous_index=2, index=3,
            previous_counter=0, counter=7, delta=7, kind="forward_gap", missing_samples=6)])

    def test_sample_counter_normal_wrap_is_contiguous_across_chunks(self):
        rows = [row(sample=v) for v in [0xfffffffe, 0xffffffff, 0, 1]]
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=4))
        collector.feed(data(rows[:2], total=4))
        collector.feed(data(rows[2:], index=2, total=4))
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        self.assertTrue(result["sample_counter_continuity"]["contiguous"])
        self.assertEqual(result["sample_counter_continuity"]["missing_sample_count"], 0)
        self.assertEqual(result["sample_counter_continuity"]["wrap_count"], 1)

    def test_duplicate_and_backward_samples_are_explicit_without_fabrication(self):
        rows = [row(sample=v) for v in [100, 100, 2]]
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=3))
        collector.feed(data(rows))
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        continuity = result["sample_counter_continuity"]
        self.assertFalse(continuity["contiguous"])
        self.assertEqual(continuity["duplicate_count"], 1)
        self.assertEqual(continuity["backward_or_reset_count"], 1)
        self.assertEqual([gap["kind"] for gap in continuity["gaps"]], ["duplicate", "backward_or_reset"])
        self.assertEqual(len(result["rows"]), 3)

    def test_complete_export_does_not_make_measurements_valid(self):
        rows = [row(sample=1, sensor_flags=0), row(sample=2, sensor_flags=0x13)]
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=2))
        collector.feed(data(rows))
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        self.assertTrue(result["sample_counter_continuity"]["contiguous"])
        self.assertFalse(any(r["measurement_valid"] for r in result["rows"]))
        self.assertFalse(any(r["measurement_ready"] for r in result["rows"]))
        self.assertTrue(result["rows"][1]["sample_bad"])
        self.assertTrue(result["rows"][1]["sample_otr"])
        self.assertTrue(result["rows"][1]["live_otr"])

    def test_maximum_capture_is_bounded_and_ring_metadata_preserved(self):
        collector = TraceCaptureCollector()
        rows = [row(sample=i) for i in range(4096)]
        collector.feed(metadata(total=4096, committed=5000, status=1))
        for start in range(0, 4096, 7):
            self.assertEqual(collector.feed(data(rows[start:start + 7], index=start, total=4096)), [])
        result = collector.feed(end(rows))[0]
        self.assertTrue(result["complete"])
        self.assertEqual(len(result["rows"]), 4096)
        self.assertEqual(len(result["frame_evidence"]), 588)
        self.assertEqual(result["metadata"]["status"], 1)
        self.assertEqual(result["metadata"]["total_committed"], 5000)

    def test_custom_limit_and_invalid_constructor_limits(self):
        for limit in (0, -1, 4097, 1.5, True):
            with self.subTest(limit=limit):
                with self.assertRaises(ValueError):
                    TraceCaptureCollector(limit)
        collector = TraceCaptureCollector(1)
        self.assert_failed(collector.feed(metadata(total=2)), "collector_row_limit_exceeded")
        self.assertIsNone(collector.snapshot())

    def test_data_or_end_without_metadata_cannot_complete(self):
        collector = TraceCaptureCollector()
        for frame in (GOLD_DATA, GOLD_END, end([])):
            self.assert_failed(collector.feed(frame), "missing_metadata")

    def test_missing_first_chunk_and_middle_chunk_cannot_complete(self):
        collector = TraceCaptureCollector()
        collector.feed(metadata(total=3))
        self.assert_failed(collector.feed(data([row()], index=1, total=3)), "noncontiguous_row_index")
        collector.feed(metadata(total=3))
        collector.feed(data([row()], total=3))
        self.assert_failed(collector.feed(data([row()], index=2, total=3)), "noncontiguous_row_index")

    def test_duplicate_chunk_and_overlap_cannot_complete(self):
        for first_count, next_index in ((1, 0), (2, 1)):
            collector = TraceCaptureCollector()
            collector.feed(metadata(total=3))
            collector.feed(data([row()] * first_count, total=3))
            self.assert_failed(collector.feed(data([row()], index=next_index, total=3)), "noncontiguous_row_index")
            self.assert_failed(collector.feed(end([row()] * 3)), "missing_metadata")

    def test_cross_generation_or_total_count_change_cannot_complete(self):
        for frame, error in ((data([row()], capture=7), "capture_id_changed"),
                             (data([row()], total=2), "total_rows_changed")):
            collector = TraceCaptureCollector()
            collector.feed(GOLD_META)
            self.assert_failed(collector.feed(frame), error)

    def test_out_of_range_row_fails_and_preserves_bad_packet(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        frame = data([row()], index=1, total=1)
        result = self.assert_failed(collector.feed(frame), "row_index_out_of_range")
        self.assertEqual(result["raw_frames_hex"][-1], frame.hex())

    def test_end_before_all_rows_and_absent_end_cannot_complete(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        self.assert_failed(collector.feed(GOLD_END), "noncontiguous_row_index")
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        result = self.assert_failed(collector.finish("end_of_file"), "end_of_file")
        self.assertEqual(result["received_rows"], 1)
        self.assertIsNone(result["record_crc16"])

    def test_wrong_end_record_crc_with_valid_packet_crc_cannot_complete(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        self.assert_failed(collector.feed(end([row()], crc=0x3d09)), "record_crc_mismatch")

    def test_packet_crc_failure_is_sticky_until_new_metadata(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        damaged = GOLD_DATA[:-1] + bytes([GOLD_DATA[-1] ^ 1])
        self.assert_failed(collector.feed(damaged), "packet_crc_mismatch")
        self.assert_failed(collector.feed(GOLD_DATA), "missing_metadata")
        self.assert_failed(collector.feed(GOLD_END), "missing_metadata")
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        self.assertTrue(collector.feed(GOLD_END)[0]["complete"])

    def test_schema_change_truncation_and_parser_discard_fail_current_export(self):
        for frame, error in ((rewrite(GOLD_DATA, 5, b"\x02"), "unsupported_trace_schema"),
                             (GOLD_DATA[:-1], "declared_length_mismatch")):
            collector = TraceCaptureCollector()
            collector.feed(GOLD_META)
            self.assert_failed(collector.feed(frame), error)
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        result = self.assert_failed(collector.fail("parser_discard", b"\xaa\x55\x03"), "parser_discard")
        self.assertEqual(result["raw_frames_hex"][-1], "aa5503")

    def test_disconnect_and_retry_with_reused_capture_id_are_separate_results(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        failed = self.assert_failed(collector.finish(), "disconnected")
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        complete = collector.feed(GOLD_END)[0]
        self.assertTrue(complete["complete"])
        self.assertEqual(failed["capture_id"], complete["capture_id"])
        self.assertIsNot(failed, complete)
        self.assertFalse(failed["complete"])

    def test_new_metadata_explicitly_fails_previous_and_starts_new_export(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        failed = self.assert_failed(collector.feed(GOLD_META), "superseded_by_metadata_before_end")
        self.assertEqual(failed["received_rows"], 1)
        self.assertEqual(collector.snapshot()["received_rows"], 0)
        collector.feed(GOLD_DATA)
        self.assertTrue(collector.feed(GOLD_END)[0]["complete"])

    def test_terminal_end_does_not_create_another_complete_capture(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        self.assertTrue(collector.feed(GOLD_END)[0]["complete"])
        self.assert_failed(collector.feed(GOLD_END), "missing_metadata")

    def test_snapshot_is_detached_and_partial_not_complete(self):
        collector = TraceCaptureCollector()
        collector.feed(GOLD_META)
        collector.feed(GOLD_DATA)
        snapshot = collector.snapshot()
        self.assertFalse(snapshot["complete"])
        self.assertEqual(snapshot["status"], "receiving")
        snapshot["rows"].clear()
        snapshot["metadata"]["adc_up_q4"] = 0
        self.assertEqual(collector.snapshot()["received_rows"], 1)
        self.assertEqual(collector.snapshot()["metadata"]["adc_up_q4"], 12349)
        self.assertTrue(collector.feed(GOLD_END)[0]["complete"])

    def test_oversized_bad_packet_evidence_is_bounded(self):
        collector = TraceCaptureCollector()
        frame = b"x" * 10000
        result = self.assert_failed(collector.feed(frame), "frame_length_out_of_range")
        self.assertLessEqual(len(result["raw_frames_hex"][0]), 480)
        evidence = result["frame_evidence"][0]
        self.assertTrue(evidence["truncated"])
        self.assertEqual(evidence["length"], 10000)
        self.assertEqual(len(evidence["sha256"]), 64)
        self.assertIsNone(collector.snapshot())

    def test_invalid_nonbytes_is_explicit_failure(self):
        result = self.assert_failed(TraceCaptureCollector().feed(None), "frame_not_bytes")
        self.assertFalse(result["complete"])


if __name__ == "__main__":
    unittest.main()
