"""Independent integer/FFT oracles and production-RTL wire analysis coverage."""
import binascii
import copy
import hashlib
import math
from pathlib import Path
import struct
import unittest

from host.control_trace import TraceCaptureCollector
from tools import analyze_control_trace as analysis


ROOT = Path(__file__).resolve().parents[1]
WIRE = ROOT / 'tests/fixtures/trace_top_v029_wire.txt'
ROW = struct.Struct('<IiHhhhhihhHBBBB')


def packet(kind, payload, total, *, index=0, count=0):
    body = bytes((0xaa, 0x55, 3, 16 + len(payload)))
    body += struct.pack('<BBHHHBB', kind, 1, 17, index, total, count, 0) + payload
    return body + binascii.crc_hqx(body, 0xffff).to_bytes(2, 'little')


def make_archive(rows, *, firmware=0x20009, down=3840, up=12800, target=0, period=50000):
    raw_rows = [ROW.pack(*(r[k] for k in (
        'sample_counter', 'encoder_count', 'adc_mean_q4', 'theta_q10', 'omega_q10',
        'arm_q10', 'arm_speed_q10', 'integral_q24', 'command_permille',
        'motor_command_permille', 'control_age_ms', 'h_flags', 'adc_quality', 'sensor_flags', 'fault')))
        for r in rows]
    total = len(rows)
    meta = struct.pack('<IHHhIIIBB', firmware, down, up, target, period, 1234, total, 1, 0)
    frames = [packet(1, meta, total)]
    for start in range(0, total, 7):
        chunk = raw_rows[start:start+7]
        frames.append(packet(2, b''.join(chunk), total, index=start, count=len(chunk)))
    frames.append(packet(3, binascii.crc_hqx(b''.join(raw_rows), 0xffff).to_bytes(2, 'little'),
                         total, index=total))
    collector = TraceCaptureCollector()
    completed = []
    for frame in frames:
        completed.extend(collector.feed(frame))
    assert len(completed) == 1 and completed[0]['complete']
    result = completed[0]
    result['storage_complete'] = True
    return result


def model_row(*, sample=100, encoder=100, adc=12800, theta=0, omega=0, arm=0,
              speed=0, integral=0, age=512, target=0, sensor_flags=12, quality=0):
    """Small independent mathematical oracle, using floor division explicitly."""
    products = [904758*theta, 77909*omega, -34213*(arm-target), -61768*speed]
    if age < 128:
        products[2] = math.floor(products[2] / 4)
    elif age < 256:
        products[2] = math.floor(products[2] / 2)
    elif age < 384:
        products[2] = math.floor(products[2] / 2) + math.floor(products[2] / 4)
    integral_integer = math.floor(abs(integral)/(2**24) + .5) * (-1 if integral < 0 else 1)
    request = -math.floor(sum(products)/(2**20)) + integral_integer
    delta = max(-335544, min(335544, 237*(arm-target)))
    allowed = age >= 384 and -268 < theta < 268 and -3584 < omega < 3584 and -10240 < speed < 10240
    freeze = allowed and ((request >= 1000 and delta > 0) or (request <= -1000 and delta < 0))
    flags = 1 + 2*int(allowed and not freeze) + 4*int(freeze) + 8*int(abs(integral) >= 1677721600)
    command = max(-1000, min(1000, request))
    return dict(sample_counter=sample, encoder_count=encoder, adc_mean_q4=adc,
                theta_q10=theta, omega_q10=omega, arm_q10=arm, arm_speed_q10=speed,
                integral_q24=integral, command_permille=command, motor_command_permille=-command,
                control_age_ms=age, h_flags=flags, adc_quality=quality,
                sensor_flags=sensor_flags, fault=0)


def decaying_rows(samples, *, first_omega=80, first_speed=-80, first_age=509, integral=0):
    rows = []
    omega, speed = first_omega, first_speed
    for i, sample in enumerate(samples):
        rows.append(model_row(sample=sample, omega=omega, speed=speed,
                              age=min(512, first_age+i), integral=integral))
        omega += (-omega)//8
        speed += (-speed)//8
    return rows


class ProductionWireAnalysisTests(unittest.TestCase):
    def test_all_four_real_rtl_captures_290_rows(self):
        self.assertTrue(WIRE.is_file(), 'Required production RTL wire evidence is absent')
        before = hashlib.sha256(WIRE.read_bytes()).hexdigest()
        grouped = {case: [] for case in range(1, 5)}
        expected = {case: [] for case in range(1, 5)}
        for line in WIRE.read_text(encoding='ascii').splitlines():
            parts = line.split()
            case = int(parts[1])
            if parts[0] == 'WIRE' and case in grouped:
                frame = bytes.fromhex(parts[2])
                if frame[2] == 3:
                    grouped[case].append(frame)
            elif parts[0] == 'EXPECTED' and case in expected:
                expected[case].append(bytes.fromhex(parts[3])[::-1].hex())
        counts = []
        for case in range(1, 5):
            with self.subTest(case=case):
                collector = TraceCaptureCollector()
                completed = []
                for frame in grouped[case]:
                    completed.extend(collector.feed(frame))
                completed.extend(collector.finish('fixture_intentionally_interrupted'))
                self.assertEqual(len(completed), 1)
                archive = completed[0]
                archive['storage_complete'] = True
                report, rows = analysis.analyze_capture(archive)
                counts.append(len(rows))
                self.assertEqual([r['raw_hex'] for r in rows], expected[case][:len(rows)])
                self.assertTrue(report['digital_model_consistent'])
                self.assertEqual(report['transfer_complete'], case in (1, 4))
                self.assertEqual(report['metadata']['period_cycles'], 1280)
                self.assertEqual(report['calibration_slope'], 367)
                self.assertTrue(all(c['mismatches'] == 0 for c in report['checks'].values()))
                self.assertFalse(report['spectrum']['available'])
                self.assertEqual(report['spectrum']['reason'], 'period_not_production_1ms')
        self.assertEqual(counts, [256, 7, 7, 20])
        self.assertEqual(sum(counts), 290)
        self.assertEqual(hashlib.sha256(WIRE.read_bytes()).hexdigest(), before)


class IntegerAnalysisTests(unittest.TestCase):
    def test_integral_nearest_rounding_is_symmetric_with_half_away_from_zero(self):
        pairs = [(0, 0), (1, 0), (-1, 0), (8388607, 0), (-8388607, 0),
                 (8388608, 1), (-8388608, -1), (25165824, 2), (-25165824, -2),
                 (1677721600, 100), (-1677721600, -100), (-2147483648, -128)]
        for value, expected in pairs:
            with self.subTest(value=value):
                self.assertEqual(analysis.round_q24(value), expected)

    def test_age_boundaries_use_separate_negative_shifts(self):
        # p_arm=-34213; at 3/4 stiffness the two floors add to -25661,
        # whereas flooring a single 3/4 product would incorrectly give -25660.
        expected = {0: 8554, 127: 8554, 128: 17107, 255: 17107,
                    256: 25661, 383: 25661, 384: 34213, 512: 34213}
        for age, numerator in expected.items():
            with self.subTest(age=age):
                terms = analysis.state_terms(model_row(arm=1, age=age), 0)
                self.assertEqual(terms['arm_term'], numerator/(1 << 20))
                self.assertEqual(terms['reconstructed_command'], 1)
                self.assertEqual(terms['expected_flags'], 3 if age >= 384 else 1)

    def test_feedback_flooring_is_not_truncation_toward_zero(self):
        positive = analysis.state_terms(model_row(theta=1), 0)
        negative = analysis.state_terms(model_row(theta=-1), 0)
        self.assertEqual(positive['reconstructed_command'], 0)
        self.assertEqual(negative['reconstructed_command'], 1)
        for terms in (positive, negative):
            self.assertAlmostEqual(sum(terms[k] for k in analysis.TERMS), terms['reconstructed_command'])

    def test_exact_gate_boundaries_and_capture_target(self):
        for field, boundary in [('theta', 268), ('omega', 3584), ('speed', 10240)]:
            for sign in (-1, 1):
                inside = analysis.state_terms(model_row(**{field: sign*(boundary-1)}), 0)
                edge = analysis.state_terms(model_row(**{field: sign*boundary}), 0)
                self.assertEqual(inside['expected_flags'] & 2, 2)
                self.assertEqual(edge['expected_flags'] & 2, 0)
        terms = analysis.state_terms(model_row(arm=100, target=99, integral=123), 99)
        self.assertEqual(terms['expected_next_integral_q24'], 360)

    def test_saturation_freeze_and_old_integral_at_limit(self):
        forward = analysis.state_terms(model_row(theta=-267, omega=-3583, arm=4096, speed=10239), 0)
        reverse = analysis.state_terms(model_row(theta=267, omega=3583, arm=-4096, speed=-10239), 0)
        self.assertEqual(forward['reconstructed_request'], 1234)
        self.assertEqual(reverse['reconstructed_request'], -1233)
        self.assertEqual(forward['reconstructed_command'], 1000)
        self.assertEqual(reverse['reconstructed_command'], -1000)
        for terms in (forward, reverse):
            self.assertEqual(terms['expected_flags'], 5)
            self.assertEqual(terms['expected_next_integral_q24'], 0)
        at_limit = analysis.state_terms(model_row(arm=1, integral=1677721600), 0)
        self.assertEqual(at_limit['expected_flags'], 11)
        self.assertEqual(at_limit['expected_next_integral_q24'], 1677721600)

    def test_rate_clipping_before_accumulation(self):
        for arm, step in [(1500, 335544), (-1500, -335544), (100, 23700), (-100, -23700)]:
            terms = analysis.state_terms(model_row(arm=arm), 0)
            self.assertEqual(terms['expected_next_integral_q24'], step)

    def test_calibration_numerator_discriminating_span_in_both_directions(self):
        # 3294199//520=6334, while the incorrect 3294208 yields 6335.
        # A 43 Q4 step must therefore map to theta=265, not 266.
        for down, up, adc in [(4600, 5120, 5077), (5120, 4600, 4643)]:
            archive = make_archive([model_row(adc=adc, theta=265)], down=down, up=up)
            report, _ = analysis.analyze_capture(archive)
            self.assertEqual(report['calibration_slope'], 6334)
            self.assertTrue(report['digital_model_consistent'])
            self.assertEqual(report['checks']['theta_mapping']['mismatches'], 0)

    def test_first_retained_nonzero_filters_are_observed_not_zero_initialized(self):
        original = decaying_rows([500, 501, 502, 503], integral=-8388608)
        self.assertEqual([r['omega_q10'] for r in original], [80, 70, 61, 53])
        self.assertEqual([r['arm_speed_q10'] for r in original], [-80, -70, -62, -55])
        report, rows = analysis.analyze_capture(make_archive(original))
        self.assertTrue(report['digital_model_consistent'])
        self.assertEqual(rows[0]['omega_q10'], 80)
        self.assertEqual(rows[0]['arm_speed_q10'], -80)
        self.assertEqual(report['checks']['omega_iir']['checked'], 3)
        self.assertEqual(report['checks']['arm_speed_iir']['checked'], 3)
        self.assertEqual(report['checks']['age_transition']['checked'], 3)

    def test_serial_counter_wrap_plus_one_is_contiguous(self):
        report, rows = analysis.analyze_capture(make_archive(decaying_rows([0xfffffffe, 0xffffffff, 0, 1])))
        self.assertTrue(report['digital_model_consistent'])
        self.assertTrue(report['sample_continuity']['contiguous'])
        self.assertEqual(report['sample_continuity']['wrap_count'], 1)
        self.assertEqual(report['checks']['integral_transition']['checked'], 3)
        self.assertEqual([r['continuous_from_previous'] for r in rows], [False, True, True, True])

    def test_gaps_skip_all_recurrences_and_keep_rows_without_interpolation(self):
        first = decaying_rows([1], first_age=10, integral=8388608)
        later = decaying_rows([100, 101], first_omega=500, first_speed=-500, first_age=511, integral=-8388608)
        report, rows = analysis.analyze_capture(make_archive(first + later))
        self.assertTrue(report['digital_model_consistent'])
        self.assertFalse(report['sample_continuity']['contiguous'])
        self.assertEqual(report['sample_continuity']['missing_sample_count'], 98)
        self.assertEqual(len(rows), 3)
        self.assertEqual(report['longest_healthy_segment_rows'], 2)
        for name in ('integral_transition', 'age_transition', 'omega_iir', 'arm_speed_iir'):
            self.assertEqual(report['checks'][name]['checked'], 1, name)

    def test_adc_warning_bit_does_not_become_hard_failure(self):
        report, _ = analysis.analyze_capture(make_archive([model_row(quality=0x20)]))
        self.assertEqual(report['healthy_rows'], 1)
        self.assertTrue(report['digital_model_consistent'])
        report, _ = analysis.analyze_capture(make_archive([model_row(quality=1)]))
        self.assertEqual(report['healthy_rows'], 0)

    def test_zero_gated_request_is_distinguished_from_wrong_motor_sign(self):
        source = model_row(integral=16777216)
        source['motor_command_permille'] = 0
        report, _ = analysis.analyze_capture(make_archive([source]))
        self.assertEqual(report['zero_gated_nonzero_requests'], 1)
        self.assertTrue(report['digital_model_consistent'])
        source['motor_command_permille'] = source['command_permille']
        report, _ = analysis.analyze_capture(make_archive([source]))
        self.assertFalse(report['digital_model_consistent'])
        self.assertEqual(report['checks']['motor_mapping']['mismatches'], 1)


class EvidenceAnalysisTests(unittest.TestCase):
    def test_v030_uses_conditioned_input_and_keeps_raw_warnings_in_evidence(self):
        row = model_row(sensor_flags=0x5e, quality=0x63)
        report, rows = analysis.analyze_capture(make_archive([row], firmware=0x30000))
        self.assertTrue(report['known_profile'])
        self.assertTrue(report['digital_model_consistent'])
        self.assertEqual(report['healthy_rows'], 1)
        self.assertEqual(report['adc_mean_semantics'], 'conditioned_control_input_q4')
        self.assertEqual(rows[0]['adc_quality'], 0x63)
        old, _ = analysis.analyze_capture(make_archive([row]))
        self.assertEqual(old['healthy_rows'], 0)

    def test_v030_blind_bad_unready_and_motion_bounds_are_not_healthy(self):
        for flag in (0x2c, 0x0d, 0x04):
            self.assertFalse(analysis.healthy(model_row(sensor_flags=flag), 0x30000))
        for field, value in [('omega_q10',30721),('arm_speed_q10',20481),('arm_q10',6145)]:
            row = model_row()
            row[field] = value
            self.assertFalse(analysis.healthy(row, 0x30000))

    def test_unknown_firmware_does_not_claim_model_or_derive_terms(self):
        report, rows = analysis.analyze_capture(make_archive([model_row()], firmware=0x2000a))
        self.assertFalse(report['known_profile'])
        self.assertIsNone(report['digital_model_consistent'])
        self.assertIsNone(report['parameter_source'])
        self.assertIsNone(report['calibration_slope'])
        self.assertEqual(report['terms'], {})
        self.assertFalse(report['spectrum']['available'])
        self.assertTrue(all(c['checked'] == 0 for c in report['checks'].values()))
        self.assertNotIn('reconstructed_command', rows[0])

    def test_metadata_tampering_is_rejected(self):
        archive = make_archive([model_row()])
        for field, value in [('firmware_id', 0x2000a), ('adc_up_q4', 12799),
                             ('capture_arm_q10', 1), ('period_cycles', 1)]:
            changed = copy.deepcopy(archive)
            changed['metadata'][field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'metadata_disagrees'):
                analysis.analyze_capture(changed)

    def test_row_raw_hex_tampering_is_rejected(self):
        archive = make_archive([model_row()])
        archive['rows'][0]['raw_hex'] = '00'*32
        with self.assertRaisesRegex(ValueError, 'rows_disagree'):
            analysis.analyze_capture(archive)

    def test_wire_tampering_or_deleted_end_cannot_claim_complete(self):
        archive = make_archive([model_row()])
        changed = copy.deepcopy(archive)
        damaged = bytearray.fromhex(changed['raw_frames_hex'][1])
        damaged[18] ^= 1
        changed['raw_frames_hex'][1] = damaged.hex()
        with self.assertRaises(ValueError):
            analysis.analyze_capture(changed)
        changed = copy.deepcopy(archive)
        changed['raw_frames_hex'].pop()
        with self.assertRaisesRegex(ValueError, 'declared_complete'):
            analysis.analyze_capture(changed)

    def test_derived_row_fields_are_reconstructed_from_original_bytes(self):
        archive = make_archive([model_row(integral=-8388608)])
        archive['rows'][0].update(theta_q10=30000, integral_q24=12345, command_permille=999,
                                  integral_permille=123, target=7, theta_deg=999)
        report, rows = analysis.analyze_capture(archive)
        self.assertTrue(report['digital_model_consistent'])
        self.assertEqual(rows[0]['theta_q10'], 0)
        self.assertEqual(rows[0]['integral_q24'], -8388608)
        self.assertEqual(rows[0]['command_permille'], -1)
        self.assertEqual(rows[0]['theta_deg'], 0)

    def test_transport_and_storage_downgrades_remain_explicit(self):
        archive = make_archive([model_row()])
        archive.update(complete=False, storage_complete=False)
        report, _ = analysis.analyze_capture(archive)
        self.assertFalse(report['transfer_complete'])
        self.assertFalse(report['storage_complete'])
        self.assertTrue(report['digital_model_consistent'])

    def test_empty_capture_is_not_a_performed_model_verification(self):
        report, rows = analysis.analyze_capture(make_archive([]))
        self.assertTrue(report['transfer_complete'])
        self.assertIsNone(report['digital_model_consistent'])
        self.assertEqual(rows, [])
        self.assertEqual(report['checks']['command']['checked'], 0)


class SpectrumAnalysisTests(unittest.TestCase):
    @staticmethod
    def signal_rows(function, n=1000):
        channels = ('command_permille', 'omega_q10', 'arm_speed_q10', *analysis.TERMS[:4])
        return [dict.fromkeys(channels, function(i/1000, i)) for i in range(n)]

    def test_known_single_tones_land_in_correct_bands(self):
        for frequency, band in [(10, 2), (50, 3), (250, 4)]:
            with self.subTest(frequency=frequency):
                rows = self.signal_rows(lambda t, _: 12 + math.sin(2*math.pi*frequency*t))
                result = analysis.spectrum(rows, 50000)
                self.assertTrue(result['available'])
                self.assertEqual(result['sample_rate_hz'], 1000)
                fractions = [b['power_fraction'] for b in result['channels']['command_permille']['bands']]
                self.assertGreater(fractions[band], .99999)
                self.assertAlmostEqual(sum(fractions), 1, places=12)

    def test_two_known_amplitudes_give_power_ratio_not_amplitude_ratio(self):
        rows = self.signal_rows(lambda t, _: 3*math.sin(2*math.pi*10*t) + 4*math.sin(2*math.pi*250*t))
        bands = analysis.spectrum(rows, 50000)['channels']['command_permille']['bands']
        self.assertAlmostEqual(bands[2]['power_fraction'], 9/25, places=5)
        self.assertAlmostEqual(bands[4]['power_fraction'], 16/25, places=5)

    def test_nyquist_weight_and_dc_demeaning(self):
        rows = self.signal_rows(lambda t, i: math.sin(2*math.pi*10*t) + (-1)**i)
        bands = analysis.spectrum(rows, 50000)['channels']['command_permille']['bands']
        self.assertAlmostEqual(bands[2]['power_fraction'], 1/3, places=5)
        self.assertAlmostEqual(bands[4]['power_fraction'], 2/3, places=5)
        constant = analysis.spectrum(self.signal_rows(lambda t, i: 13), 50000)
        channel = constant['channels']['command_permille']
        self.assertEqual(channel['demeaned_windowed_power'], 0)
        self.assertTrue(all(b['power_fraction'] is None for b in channel['bands']))

    def test_short_or_accelerated_rtl_period_is_not_called_1khz_spectrum(self):
        self.assertFalse(analysis.spectrum(self.signal_rows(lambda t, i: i, 511), 50000)['available'])
        self.assertEqual(analysis.spectrum(self.signal_rows(lambda t, i: i), 1280)['reason'],
                         'period_not_production_1ms')

    def test_only_longest_contiguous_healthy_segment_is_transformed(self):
        rows = [model_row(sample=i) for i in range(600)]
        rows += [model_row(sample=i) for i in range(700, 1200)]
        report, _ = analysis.analyze_capture(make_archive(rows))
        self.assertTrue(report['digital_model_consistent'])
        self.assertEqual(report['longest_healthy_segment_rows'], 600)
        self.assertTrue(report['spectrum']['available'])
        self.assertEqual(report['spectrum']['n'], 600)
        self.assertEqual(report['sample_continuity']['missing_sample_count'], 100)


if __name__ == '__main__':
    unittest.main()
