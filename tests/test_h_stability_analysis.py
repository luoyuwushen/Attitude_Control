"""Acceptance evidence must survive raw-wire, continuity and zero-bias checks."""
import binascii
import copy
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest

from host.control_trace import TraceCaptureCollector
from tools.analyze_h_stability import analyze_file, analyze_telemetry, analyze_trace


def frame(i, *, theta=0, arm=0, command=0, firmware=0x20009, flags=12,
          fault=0, state=2, quality=0, seq=None, clock=None, counter=None,
          reference=0, age=512, omit=None, diagnostic=0x90, motor=None):
    body = bytearray(b'\xaa\x55\x02\x00')
    body.extend(struct.pack('<HHhhhhhBBBB', (i if seq is None else seq) & 0xffff,
                            780, theta, 0, arm, 0, command, state, 1, fault, diagnostic))
    items = [(1, '<I', (i*20 if clock is None else clock) & 0xffffffff),
             (9, '<H', 1000), (10, '<H', flags), (19, '<I', firmware),
             (20, '<I', (i*20 if counter is None else counter) & 0xffffffff),
             (21, '<B', quality), (17, '<h', -command if motor is None else motor)]
    for kind, fmt, value in items:
        if kind != omit:
            data = struct.pack(fmt, value)
            body.extend(bytes((kind, len(data)))+data)
    body.extend(b'\x20\x07'+struct.pack('<hhHB', 0, reference, age, 3))
    body[3] = len(body)+2
    body.extend(binascii.crc_hqx(body, 0xffff).to_bytes(2, 'little'))
    return dict(raw_hex=body.hex(), theta_deg=0, arm_deg=0, active=False,
                time_utc='unsupported host-clock evidence')


def samples(n=3001, **kwargs):
    return [frame(i, **kwargs) for i in range(n)]


CONFIRMED = dict(segment=1, offset_s=0, evidence='Independent operator release confirmation')


def trace_archive(n=4096, total=60000):
    raw_row = struct.Struct('<IiHhhhhihhHBBBB')
    rows = [raw_row.pack(i, 0, 12480, 0, 0, 0, 0, 0, 0, 0, 512, 3, 0, 12, 0) for i in range(n)]
    def packet(kind, payload, index=0, count=0):
        body = b'\xaa\x55\x03'+bytes((16+len(payload),))
        body += struct.pack('<BBHHHBB', kind, 1, 1, index, n, count, 0)+payload
        return body+binascii.crc_hqx(body, 0xffff).to_bytes(2, 'little')
    wire = [packet(1, struct.pack('<IHHhIIIBB', 0x20009, 3500, 12480, 0, 50000, 60000, total, 1, 1))]
    for i in range(0, n, 7):
        batch = rows[i:i+7]
        wire.append(packet(2, b''.join(batch), i, len(batch)))
    wire.append(packet(3, binascii.crc_hqx(b''.join(rows), 0xffff).to_bytes(2, 'little'), n))
    collector, finished = TraceCaptureCollector(), []
    for raw in wire:
        finished.extend(collector.feed(raw))
    assert len(finished) == 1 and finished[0]['complete']
    return finished[0]


class StabilityEvidenceTests(unittest.TestCase):
    def test_60s_requires_external_hands_off_evidence(self):
        data = samples()
        missing = analyze_telemetry(data)
        self.assertFalse(missing['any_logged_target_met'])
        self.assertIn('hands_off_unconfirmed', missing['segments'][0]['insufficient_evidence'])
        confirmed = analyze_telemetry(data, hands_off=CONFIRMED)
        segment = confirmed['segments'][0]
        self.assertTrue(segment['logged_target_met'])
        self.assertEqual(segment['confirmed_hands_off_s'], 60)
        self.assertEqual(segment['tail_observed']['samples'], 1001)
        self.assertEqual(segment['tail_observed']['duration_s'], 20)

    def test_constant_bias_is_not_erased_by_standard_deviation(self):
        report = analyze_telemetry(samples(theta=54), hands_off=CONFIRMED)
        segment = report['segments'][0]
        self.assertFalse(segment['logged_target_met'])
        self.assertGreater(segment['tail_observed']['theta_zero_rms_deg'], 3)
        self.assertEqual(segment['tail_observed']['theta_stddev_deg'], 0)
        self.assertIn('theta_zero_rms_above_2deg', segment['failed_targets'])

    def test_arm_peak_to_peak_and_drift_use_decoded_values(self):
        data = [frame(i, arm=round(i*108/3000)) for i in range(3001)]
        report = analyze_telemetry(data, hands_off=CONFIRMED)
        metrics = report['segments'][0]['whole_segment']
        self.assertAlmostEqual(metrics['arm_peak_to_peak_deg'], 108*180/(1024*math.pi))
        self.assertGreater(metrics['arm_drift_deg'], 5.8)
        self.assertGreater(metrics['arm_linear_drift_deg_s'], 0.1)
        self.assertTrue(report['any_logged_target_met'])  # Last 20 s cover only one third.

    def test_tail_large_periodic_arm_motion_fails(self):
        data = [frame(i, arm=100 if i%100 < 50 else -100) for i in range(3001)]
        result = analyze_telemetry(data, hands_off=CONFIRMED)['segments'][0]
        self.assertIn('arm_peak_to_peak_above_5deg', result['failed_targets'])
        self.assertFalse(result['logged_target_met'])

    def test_saturation_before_tail_still_fails(self):
        data = samples()
        data[30] = frame(30, command=-1000)
        result = analyze_telemetry(data, hands_off=CONFIRMED)['segments'][0]
        self.assertEqual(result['tail_observed']['saturated_samples'], 0)
        self.assertIn('saturated_hands_off_samples', result['failed_targets'])

    def test_fewer_than_60_seconds_never_rounded_up(self):
        result = analyze_telemetry(samples(3000), hands_off=CONFIRMED)['segments'][0]
        self.assertEqual(result['confirmed_hands_off_s'], 59.98)
        self.assertFalse(result['logged_target_met'])

    def test_tail_boundary_uses_integer_device_time(self):
        result = analyze_telemetry(samples(1481))['segments'][0]
        self.assertEqual(result['continuous_healthy_h_s'], 29.6)
        self.assertEqual(result['tail_observed']['samples'], 1001)
        self.assertEqual(result['tail_observed']['duration_s'], 20)

    def test_late_release_does_not_count_hand_held_period(self):
        confirmation = dict(CONFIRMED, offset_s=1)
        result = analyze_telemetry(samples(), hands_off=confirmation)['segments'][0]
        self.assertEqual(result['confirmed_hands_off_s'], 59)
        self.assertFalse(result['logged_target_met'])

    def test_sequence_and_clock_and_sample_gaps_split_segments(self):
        for changed in (dict(seq=5000), dict(clock=5000), dict(counter=5000)):
            with self.subTest(changed=changed):
                data = samples()
                data[1500] = frame(1500, **changed)
                report = analyze_telemetry(data, hands_off=CONFIRMED)
                self.assertEqual(len(report['segments']), 3)
                self.assertFalse(report['any_logged_target_met'])
                self.assertLess(max(s['continuous_healthy_h_s'] for s in report['segments']), 30)

    def test_counter_wraps_are_contiguous(self):
        data = [frame(i, seq=65534+i, clock=0xffffffe0+i*20, counter=0xffffffe0+i*20) for i in range(4)]
        report = analyze_telemetry(data)
        self.assertEqual(len(report['segments']), 1)
        self.assertEqual(report['segments'][0]['continuous_healthy_h_s'], .06)

    def test_reference_change_restart_and_reserved_flags_are_not_stitched(self):
        for changed in (dict(reference=1), dict(age=0), dict(flags=0x800c), dict(quality=0x80)):
            with self.subTest(changed=changed):
                report = analyze_telemetry([frame(0), frame(1, **changed), frame(2)])
                self.assertGreater(len(report['segments']), 1)
                self.assertFalse(report['any_logged_target_met'])

    def test_bad_crc_cannot_be_hidden_by_derived_fields(self):
        data = samples(4)
        raw = bytearray.fromhex(data[1]['raw_hex'])
        raw[-1] ^= 1
        data[1]['raw_hex'] = raw.hex()
        report = analyze_telemetry(data)
        self.assertEqual(report['crc_decoded_frames'], 3)
        self.assertEqual(report['excluded_records']['invalid_raw_frame'], 1)
        self.assertEqual(len(report['segments']), 2)

    def test_unknown_or_incomplete_firmware_never_infers_h(self):
        for data, cause in [(samples(3, firmware=0x40000), 'unknown_firmware'),
                            (samples(3, omit=20), 'missing_required_fields')]:
            report = analyze_telemetry(data)
            self.assertEqual(report['segments'], [])
            self.assertEqual(report['excluded_records'][cause], 3)
            self.assertFalse(report['any_logged_target_met'])

    def test_fault_ending_invalidates_long_segment(self):
        data = samples()+[frame(3001, fault=1, state=3)]
        result = analyze_telemetry(data, hands_off=CONFIRMED)['segments'][0]
        self.assertFalse(result['logged_target_met'])
        self.assertIn('segment_ended_by_fault', result['insufficient_evidence'])

    def test_idle_exit_is_not_a_fault(self):
        data = samples()+[frame(3001, state=0)]
        result = analyze_telemetry(data, hands_off=CONFIRMED)['segments'][0]
        self.assertTrue(result['logged_target_met'])

    def test_conditioned_raw_warning_is_allowed_but_blind_is_invalid(self):
        report = analyze_telemetry(samples(4, firmware=0x30000, flags=0x5e, quality=0x80))
        self.assertEqual(len(report['segments']), 1)
        self.assertEqual(analyze_telemetry(samples(4, firmware=0x30000, flags=0x6c))['segments'], [])
        self.assertEqual(analyze_telemetry(samples(4, flags=14))['segments'], [])

    def test_stop_and_gated_command_cannot_count_as_healthy_h(self):
        for changed in (dict(diagnostic=0x98), dict(diagnostic=0x92), dict(diagnostic=0),
                        dict(command=100, motor=0)):
            with self.subTest(changed=changed):
                report = analyze_telemetry(samples(4, **changed))
                self.assertEqual(report['segments'], [])

    def test_invalid_confirmation_is_rejected(self):
        for marker in [dict(CONFIRMED, evidence=''), dict(CONFIRMED, offset_s=float('nan')),
                       dict(CONFIRMED, segment=2), dict(CONFIRMED, offset_s=1)]:
            with self.subTest(marker=marker), self.assertRaises(ValueError):
                analyze_telemetry(samples(4), hands_off=marker)

    def test_file_is_read_only_and_hash_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'telemetry.jsonl'
            path.write_text('\n'.join(json.dumps(row) for row in samples(4)), encoding='utf-8')
            before = path.read_bytes()
            report = analyze_file(path)
            self.assertEqual(report['source']['bytes'], len(before))
            self.assertEqual(path.read_bytes(), before)

    def test_four_second_trace_cannot_claim_60_seconds_from_total_committed(self):
        archive = trace_archive()
        report = analyze_trace(archive)
        self.assertEqual(report['row_count'], 4096)
        self.assertEqual(report['total_committed_not_retained'], 60000)
        self.assertAlmostEqual(report['retained_time_span_s'], 4.095)
        self.assertFalse(report['any_logged_target_met'])
        corrupt = copy.deepcopy(archive)
        corrupt['metadata']['total_committed'] = 90000
        with self.assertRaises(ValueError):
            analyze_trace(corrupt)


if __name__ == '__main__':
    unittest.main()
