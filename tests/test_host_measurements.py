import json
import math
import tempfile
import unittest
from pathlib import Path

from host.measurements import (
    MeasurementError, blind_zone_scan, encoder_measurement, free_decay,
    jog_response, save_measurement, static_measurement, validate_records,
)


Q10_DEG = 180 / math.pi / 1024


def record(index, count=0, theta=180, configured_cpr=1040, sign=1, **extra):
    arm_raw = max(-32768, min(32767, count * sign * (6588397 // configured_cpr) // 1024))
    theta_raw = max(-3217, min(3217, round(((theta + 180) % 360 - 180) / Q10_DEG)))
    sample = dict(elapsed_s=index * 0.02, sequence=index % 65536,
                  adc=700, theta_q10=theta_raw, theta_deg=theta_raw * Q10_DEG,
                  arm_q10=arm_raw, arm_deg=arm_raw * Q10_DEG,
                  omega_rad_s=0.0, arm_speed_rad_s=0.0,
                  state=0, calibrated=1, fault=0, command_permille=0,
                  protocol_version=1, source="serial", sequence_gap=0,
                  sequence_reset=False, sequence_duplicate=False)
    sample.update(extra)
    return sample


def encoder_run(delta=1000, configured_cpr=1040, sign=1):
    direction = 1 if delta > 0 else -1
    counts = [0] * 51 + list(range(direction, delta + direction, direction)) + [delta] * 51
    return [record(i, count, configured_cpr=configured_cpr, sign=sign) for i, count in enumerate(counts)]


def decay_records(amplitude=8, decay=0.12, period=0.8, seconds=15):
    return [record(i, theta=180 + amplitude * math.exp(-decay*i*0.02) * math.cos(2*math.pi*i*0.02/period))
            for i in range(round(seconds/0.02))]


class ValidationTests(unittest.TestCase):
    def test_sequence_wrap_and_required_strict_fields(self):
        self.assertEqual(len(validate_records([record(65535), record(65536)])), 2)
        for key, value in (("calibrated", None), ("calibrated", 1.0), ("state", 0.5),
                           ("fault", 0.1), ("sequence", True), ("adc", 1024),
                           ("theta_deg", float("nan")), ("arm_q10", 32768)):
            with self.subTest(key=key, value=value), self.assertRaises(MeasurementError):
                validate_records([record(0), record(1, **{key: value})])

    def test_gaps_resets_duplicates_and_mixed_origins_are_rejected(self):
        invalid = [dict(elapsed_s=0.11), dict(elapsed_s=0), dict(sequence=2),
                   dict(sequence_reset=True), dict(sequence_duplicate=True),
                   dict(time_reset=True), dict(sequence_gap=1), dict(source="demo"),
                   dict(source="replay", original_source="demo")]
        for change in invalid:
            with self.subTest(change=change), self.assertRaises(MeasurementError):
                validate_records([record(0), record(1, **change)])
        with self.assertRaises(MeasurementError):
            validate_records([record(0)])

    def test_stationary_requires_state_and_zero_command(self):
        for change in (dict(state=4), dict(command_permille=100)):
            with self.assertRaises(MeasurementError):
                validate_records([record(0), record(1, **change)], require_stopped=True)


class StaticScanTests(unittest.TestCase):
    def test_unmarked_adc_calibration_and_averaged_code_statistics(self):
        result = static_measurement([record(i, calibrated=0, adc=code) for i, code in enumerate([700, 701, 699, 700])])
        self.assertEqual(result["adc_mean"], 700)
        self.assertAlmostEqual(result["adc_std"], math.sqrt(0.5))
        self.assertEqual(result["adc_ptp"], 2)
        self.assertNotIn("theta_mean_deg", result)
        with self.assertRaises(MeasurementError):
            static_measurement([record(0, calibrated=None), record(1, calibrated=None)])

    def test_wrapped_static_angle_is_not_zero(self):
        result = static_measurement([record(i, theta=theta) for i, theta in enumerate([179.8, 180.2, 179.8, 180.2])])
        self.assertAlmostEqual(abs(result["theta_mean_deg"]), 180)
        self.assertLess(result["theta_rms_deg"], 0.3)

    def test_scan_marks_rails_jumps_and_fault_without_angle_claim(self):
        result = blind_zone_scan([record(i, calibrated=0, adc=code, fault=int(i == 4))
                                  for i, code in enumerate([500, 510, 520, 1023, 0])])
        self.assertEqual(result["max_adjacent_jump_codes"], 1023)
        self.assertEqual(result["near_rail_samples"], 2)
        self.assertEqual(result["fault_samples"], 1)
        self.assertEqual(result["suspected_segments"][-1]["elapsed_s"], 0.08)
        self.assertNotIn("blind_zone_angle_deg", result)
        self.assertNotIn("electrical_travel_deg", result)

    def test_demo_and_demo_replay_remain_synthetic(self):
        for extras in (dict(source="demo"), dict(source="replay", original_source="demo")):
            result = static_measurement([record(i, **extras) for i in range(5)])
            self.assertEqual(result["evidence"], "synthetic")
            self.assertFalse(result["is_physical_measurement"])

    def test_physical_origin_replay_is_not_a_current_physical_measurement(self):
        result = static_measurement([record(i, source="replay", original_source="serial") for i in range(5)])
        self.assertEqual(result["original_source"], "serial")
        self.assertEqual(result["source"], "replay")
        self.assertFalse(result["is_physical_measurement"])


class EncoderTests(unittest.TestCase):
    def test_real_cpr_is_independent_of_configured_value_both_signs(self):
        for delta, sign in ((1000, 1), (-1000, 1), (1000, -1), (-1000, -1)):
            records = encoder_run(delta=delta, sign=sign)
            result = encoder_measurement(records[:50], records[-50:], encoder_sign=sign, all_records=records)
            self.assertEqual(result["delta_count"], delta)
            self.assertEqual(result["cpr_estimate"], 1000)
            self.assertNotEqual(result["cpr_estimate"], result["configured_cpr"])
            self.assertEqual(result["provenance"], "inverted")
            self.assertTrue(result["full_interval_checked"])

    def test_negative_count_inverse_and_fractional_mechanical_reference(self):
        records = [record(i, -20 if i < 5 else -280) for i in range(10)]
        result = encoder_measurement(records[:5], records[5:], physical_angle_deg=-90)
        self.assertEqual(result["relative_count_start"], -20)
        self.assertEqual(result["relative_count_end"], -280)
        self.assertEqual(result["cpr_estimate"], 1040)

    def test_direct_count_preferred_and_noninvertible_configuration(self):
        records = [record(i, protocol_version=2, encoder_count=5 if i < 5 else 1305) for i in range(10)]
        result = encoder_measurement(records[:5], records[5:], configured_cpr=10000)
        self.assertEqual(result["cpr_estimate"], 1300)
        self.assertEqual(result["provenance"], "direct")
        for sample in records:
            del sample["encoder_count"]
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], records[5:], configured_cpr=10000)

    def test_stationary_endpoint_count_not_other_sensors(self):
        records = encoder_run()
        records[-1]["adc"] = 690
        result = encoder_measurement(records[:5], records[-5:])
        self.assertEqual(result["delta_count"], 1000)
        records[-1].update(arm_q10=6180, arm_deg=6180 * Q10_DEG)
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], records[-5:])

    def test_full_interval_rejects_calibration_gap_origin_and_parameter_changes(self):
        for changes in (dict(calibrated=0), dict(sequence_gap=1), dict(origin_reset=True),
                        dict(fault=1), dict(sequence_reset=True)):
            records = encoder_run()
            records[100].update(changes)
            with self.subTest(change=changes), self.assertRaises(MeasurementError):
                encoder_measurement(records[:50], records[-50:], all_records=records)
        records = encoder_run()
        records[10]["parameter_version"] = 1
        records[100]["parameter_version"] = 2
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:50], records[-50:], all_records=records)
        records = encoder_run()
        records[300].update(arm_q10=0, arm_deg=0)
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:50], records[-50:], all_records=records)

    def test_saturation_mapping_rounding_and_truncated_full_history(self):
        records = encoder_run()
        for invalid in (dict(arm_q10=32767, arm_deg=32767*Q10_DEG),
                        dict(arm_q10=1, arm_deg=Q10_DEG)):
            end = [dict(item, **invalid) for item in records[-5:]]
            with self.assertRaises(MeasurementError):
                encoder_measurement(records[:5], end)
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], records[-5:], all_records=records[5:])
        end = [{key: value for key, value in item.items() if key != "arm_q10"} for item in records[-5:]]
        for item in end:
            item["arm_deg"] = round(item["arm_deg"], 2)
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], end)

    def test_context_sample_mismatch_and_rails_are_rejected(self):
        records = encoder_run()
        end = [dict(sample) for sample in records[-5:]]
        for sample in end:
            raw = 999 * (6588397 // 1040) // 1024
            sample.update(arm_q10=raw, arm_deg=raw*Q10_DEG)
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], end, all_records=records)
        records[100]["adc"] = 1023
        with self.assertRaises(MeasurementError):
            encoder_measurement(records[:5], records[-5:], all_records=records)


class DecayTests(unittest.TestCase):
    def test_50_hz_q10_decay_and_zero_stopped_speed(self):
        result = free_decay(decay_records(), noise_std_deg=0.03)
        self.assertAlmostEqual(result["damped_period_s"], 0.8, delta=0.015)
        self.assertAlmostEqual(result["decay_rate_s_inv"], 0.12, delta=0.01)
        self.assertAlmostEqual(result["b_over_J_s_inv"], 0.24, delta=0.02)
        self.assertAlmostEqual(result["mgd_over_J_s_inv2"], (2*math.pi/0.8)**2 + 0.12**2, delta=1)
        self.assertGreater(result["fit_r2"], 0.99)
        self.assertGreaterEqual(result["peak_count"], 4)
        self.assertNotIn("b", result)
        self.assertNotIn("J", result)

    def test_insufficient_peaks_large_angle_coupling_noise_and_no_decay(self):
        invalid = [decay_records(seconds=2), decay_records(amplitude=20), decay_records(decay=0)]
        coupled = decay_records()
        for i, sample in enumerate(coupled):
            sample["arm_deg"] = 3*math.sin(i*0.05)
        invalid.append(coupled)
        for records in invalid:
            with self.assertRaises(MeasurementError):
                free_decay(records)
        with self.assertRaises(MeasurementError):
            free_decay(decay_records(), noise_std_deg=2)

    def test_nonexponential_peaks_are_rejected(self):
        samples = []
        for i in range(750):
            t = i*0.02
            amplitude = 8 if t < 6 else 2*math.exp(-0.15*(t-6))
            samples.append(record(i, theta=180+amplitude*math.cos(2*math.pi*t/0.8)))
        with self.assertRaises(MeasurementError):
            free_decay(samples)

    def test_derived_angles_inconsistent_with_raw_q10_are_rejected(self):
        records = decay_records()
        for sample in records:
            sample["theta_q10"] = -3217  # Raw sensor angle says fully static.
        with self.assertRaises(MeasurementError):
            free_decay(records)


class JogAndReportTests(unittest.TestCase):
    @staticmethod
    def jog():
        counts = [0]*5 + [1, 3, 6, 10, 15, 19] + [22, 24, 25, 25, 25, 25]
        return [record(i, count, state=4 if 5 <= i <= 10 else 0,
                       command_permille=100 if 5 <= i <= 10 else 0)
                for i, count in enumerate(counts)]

    def test_angle_derivative_measures_coast_when_reported_speed_is_zero(self):
        result = jog_response(self.jog())
        self.assertGreater(result["peak_speed_from_angle_rad_s"], 0)
        self.assertGreater(result["coast_change_deg"], 0)
        self.assertTrue(result["direction_matches_command"])
        self.assertAlmostEqual(result["jog_duration_estimate_s"], 0.12)
        self.assertNotIn("mechanical_time_constant_s", result)
        self.assertNotIn("torque", result)

    def test_jog_requires_before_after_single_episode_and_direction(self):
        for records in (self.jog()[5:], self.jog()[:11], [record(i) for i in range(10)]):
            with self.assertRaises(MeasurementError):
                jog_response(records)

    def test_batched_host_timestamps_do_not_create_microsecond_speed_spikes(self):
        regular = self.jog()
        batched = self.jog()
        for i, sample in enumerate(batched):
            sample["elapsed_s"] = (i//3)*0.06 + (i % 3)*0.000001
        result = jog_response(batched)
        reference = jog_response(regular)
        self.assertLess(result["peak_speed_from_angle_rad_s"], reference["peak_speed_from_angle_rad_s"]*2)

    def test_jog_origin_reset_and_parameter_changes_are_rejected(self):
        for changes in (dict(origin_reset=True), dict(calibration_reset=True)):
            samples = self.jog()
            samples[13].update(changes)
            with self.subTest(change=changes), self.assertRaises(MeasurementError):
                jog_response(samples)
        samples = self.jog()
        for index, sample in enumerate(samples):
            sample["parameter_version"] = 1 if index < 12 else 2
        with self.assertRaises(MeasurementError):
            jog_response(samples)
        for change in (dict(state=2), dict(command_permille=-100), dict(fault=1)):
            records = self.jog()
            records[8].update(change)
            with self.assertRaises(MeasurementError):
                jog_response(records)

    def test_atomic_json_metadata_and_nonfinite_rejection(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/"报告.json"
            result = static_measurement([record(0), record(1)])
            self.assertEqual(save_measurement(path, result, {"固件": {"COUNTS_PER_REV": 1040}, "量具": "待测"}), path)
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(saved["metadata"]["量具"], "待测")
            self.assertEqual(saved["measurement"]["adc_mean"], 700)
            before = path.read_bytes()
            with self.assertRaises(MeasurementError):
                save_measurement(path, {"value": float("nan")})
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual([item.name for item in Path(temporary).iterdir()], ["报告.json"])


if __name__ == "__main__":
    unittest.main()
