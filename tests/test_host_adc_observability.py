"""Conversion diagnostics are evidence, never additional motor authorization."""
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from host.app import Window, EXTENSIONS
from host.core import EXTENDED_FIELDS, Replay, SessionRecorder, StreamDecoder
from host.diagnostics import (adc_detail_text, adc_diagnostic_text,
    adc_observability_text, measurement_block_reason, recovery_hint)
from host.operation_log import OperationLog
from test_host_adc_diagnostics import frame, tlv, RTL_GOLDEN as GOLDEN_87
from test_host_sensor_recovery import recovery_frame
from test_host_ui import WorkerStub


# Actual serialized UART bytes emitted by tb_telemetry_v2, supplied by the RTL
# agent. Deliberately includes high bits/unphysical values to test wire fidelity.
RTL_GOLDEN_190 = bytes.fromhex(
    'aa5502be0000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
    '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
    '110238ff120120130402000200140498badcfe15012916022913170211001802eb031902f137'
    '1a22010000010080ffff34127856e803e110c3001601a20d88132000feffff80efcdab89'
    '1b22cdabff031000f4016400c8002c019001f4015802bc0220038403e803dcfe10325476'
    '1c043412dcfe1d04cdab6587c7ce')
FIELDS = ('reference_q4', 'filtered_min', 'filtered_max', 'contribution_min',
    'contribution_max', 'max_step', 'outlier_count', 'outlier_longest',
    'first_outlier_index', 'first_outlier_edge_ticks', 'max_step_index',
    'max_step_edge_ticks', 'edge_outlier_count', 'conversion_count',
    'detail_flags', 'filtered_sum')
CURRENT = (1, 256, 32768, 65535, 4660, 22136, 1000, 4321, 195, 278,
           3490, 5000, 32, 65534, 33023, 2309737967)
FAULT = (43981, 1023, 16, 500, 100, 200, 300, 400, 500, 600,
         700, 800, 900, 1000, 65244, 1985229328)
OBSERVABLE = {prefix + name: value for prefix, values in
              (('adc_', CURRENT), ('adc_fault_', FAULT)) for name, value in zip(FIELDS, values)}
OBSERVABLE.update(adc_fault_time_ms=0xfedc1234, adc_fault_sample_counter=0x8765abcd)
REALISTIC = (3520, 204, 260, 214, 235, 31, 12, 7, 112, 99,
             110, 98, 4, 5000, 0x05ad, 1100000)


def group(kind, values=REALISTIC):
    return bytes((kind, 34)) + struct.pack('<15HI', *values)


def observation_frame(**kwargs):
    extras = group(26) + group(27)
    extras += tlv(21, '<B', 0) + tlv(22, '<H', 0x24)
    extras += tlv(23, '<H', 191) + tlv(24, '<H', 275) + tlv(25, '<H', 3504)
    extras += tlv(28, '<I', 118655) + tlv(29, '<I', 118654)
    return frame(firmware=0x20002, extra=extras, **kwargs)


class ADCObservabilityProtocolTests(unittest.TestCase):
    def test_real_uart_golden_all_fragment_boundaries_and_bytewise(self):
        self.assertEqual(len(RTL_GOLDEN_190), 190)
        for split in range(1, 190):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(RTL_GOLDEN_190[:split]), [])
            rows = decoder.feed(RTL_GOLDEN_190[split:])
            self.assertEqual(len(rows), 1)
            self.assertEqual({key: rows[0][key] for key in OBSERVABLE}, OBSERVABLE)
            self.assertEqual(rows[0]['firmware_version'], 0x20002)
            self.assertEqual(decoder.stats['crc_errors'], 0)
        decoder = StreamDecoder()
        rows = []
        for value in RTL_GOLDEN_190:
            rows.extend(decoder.feed(bytes([value])))
        self.assertEqual(rows[0]['raw_hex'], RTL_GOLDEN_190.hex())

    def test_group_lengths_duplicates_truncation_and_scalar_duplicates_rejected(self):
        for kind in (26, 27):
            good = group(kind)
            for bad in (good + good, bytes((kind, 0)), bytes((kind, 33)) + good[2:-1],
                        bytes((kind, 35)) + good[2:] + b'\x00', good[:-1]):
                with self.subTest(kind=kind, length=len(bad)):
                    decoder = StreamDecoder()
                    rows = decoder.feed(frame(extensions=False, extra=bad) + frame(sequence=2))
                    self.assertEqual([r['sequence'] for r in rows], [2])
                    self.assertEqual(decoder.stats['unsupported_frames'], 1)
        for kind in (28, 29):
            good = tlv(kind, '<I', 1)
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(frame(extensions=False, extra=good + good)), [])
            self.assertEqual(decoder.stats['unsupported_frames'], 1)

    def test_unknown_tlvs_remain_skippable_and_missing_groups_are_not_invented(self):
        row = StreamDecoder().feed(frame(extensions=False,
            extra=b'\xfe\x03abc\xfe\x02xy' + group(26)))[0]
        self.assertEqual(row['adc_conversion_count'], 5000)
        self.assertNotIn('adc_fault_conversion_count', row)
        self.assertNotIn('adc_fault_time_ms', row)
        minimal = StreamDecoder().feed(frame(extensions=False))[0]
        self.assertTrue(all(key not in minimal for key in OBSERVABLE))

    def test_crc_resync_and_old_24_87_106_byte_protocols(self):
        bad = bytearray(RTL_GOLDEN_190)
        bad[140] ^= 0x80
        decoder = StreamDecoder()
        rows = decoder.feed(bytes(bad) + frame(version=1) + GOLDEN_87 + recovery_frame() + RTL_GOLDEN_190)
        self.assertEqual(len(rows), 4)
        self.assertEqual(decoder.stats['crc_errors'], 1)
        self.assertEqual([r.get('firmware_version') for r in rows], [None, 0x20000, 0x20001, 0x20002])
        for row in rows[:3]:
            self.assertNotIn('adc_conversion_count', row)

    def test_every_raw_statistic_survives_csv_replay_and_json_log(self):
        row = StreamDecoder().feed(RTL_GOLDEN_190)[0]
        self.assertTrue(set(OBSERVABLE).issubset(EXTENDED_FIELDS))
        self.assertEqual(len(EXTENDED_FIELDS), len(set(EXTENDED_FIELDS)))
        with tempfile.TemporaryDirectory() as tmp:
            with SessionRecorder(tmp, {'source': 'offline_simulation'}) as recorder:
                recorder.write(row)
            replay = Replay.load(recorder.directory / 'samples.csv')[0]
            self.assertEqual({key: replay[key] for key in OBSERVABLE}, OBSERVABLE)
            logger = OperationLog(Path(tmp), {'source': 'offline_simulation'}, directory_name='logs')
            logger.write_telemetry(row)
            directory = logger.directory
            logger.close()
            stored = json.loads((directory / 'telemetry.jsonl').read_text(encoding='utf-8'))
            self.assertEqual({key: stored[key] for key in OBSERVABLE}, OBSERVABLE)

    def test_no_event_flags_differ_from_saturated_edge_age(self):
        row = dict(zip(('adc_' + key for key in FIELDS), REALISTIC))
        row.update(adc_detail_flags=1, adc_first_outlier_index=65535,
                   adc_first_outlier_edge_ticks=65535)
        self.assertIn('首离群：未报告事件', adc_detail_text(row))
        row.update(adc_detail_flags=0x0bad, adc_first_outlier_index=23)
        text = adc_detail_text(row)
        self.assertIn('首离群：中值输出索引 23', text)
        self.assertIn('≥1310.70 μs 或无先前可追溯边沿', text)
        self.assertIn('EN/BN2/BN1/PWM=1010', text)
        self.assertIn('EN/BN2/BN1/PWM=1011', text)
        self.assertIn('已建立（不代表窗口健康）', text)
        self.assertIn('不是模拟毛刺宽度', text)

    def test_saturation_and_bypass_mean_never_replace_control_mean(self):
        row = dict(zip(('adc_' + key for key in FIELDS), REALISTIC))
        row.update(adc_raw=220, adc_mean_q4=3488)
        text = adc_detail_text(row)
        self.assertIn('220.000 code（全中值输出旁路均值，不替代控制均值）', text)
        self.assertIn('均值 218.000', adc_diagnostic_text(row))
        row['adc_detail_flags'] |= 2
        self.assertIn('不计算旁路均值', adc_detail_text(row))
        self.assertIn('计数/和存在饱和', adc_observability_text(row))
        self.assertEqual(row['adc_mean_q4'], 3488)

    def test_missing_fault_group_is_not_filled_from_current_group(self):
        row = dict(zip(('adc_' + key for key in FIELDS), REALISTIC))
        row.update(sensor_fault_reason=0x24, adc_fault_time_ms=118655)
        text = adc_observability_text(row)
        self.assertIn('设备 118655 ms', text)
        self.assertIn('首次异常转换级统计未提供', text)
        self.assertNotIn('滤后 204', text)
        self.assertEqual(adc_detail_text({}), '转换级统计：未提供。')

    def test_diagnostics_do_not_reclassify_health_or_regress_recovery(self):
        # Even saturated/correlated statistics neither veto good sensor_flags
        # nor rescue invalid/unready measurements.
        details = dict(OBSERVABLE, firmware_version=0x20002)
        for flags in (0, 4, 12, 13, 14, 28):
            self.assertEqual(measurement_block_reason(dict(sensor_flags=flags, **details)),
                             measurement_block_reason({'sensor_flags': flags}))
        base = dict(sensor_fault=1, fault=1, calibrated=1)
        self.assertIn('R 只清控制器故障', recovery_hint(dict(base, firmware_version=0x20000)))
        for version in (0x20001, 0x20002):
            self.assertIn('连续可信采样后按 R', recovery_hint(dict(base, firmware_version=version)))
            self.assertIn('清除不启动电机', recovery_hint(dict(base, firmware_version=version)))


class ADCObservabilityUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        with patch('host.app.list_ports.comports', return_value=[]):
            self.window = Window(Path(self.tmp.name))
        self.window.timer.stop()
        self.window.plot_timer.stop()

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.tmp.cleanup()

    def test_main_summary_and_expanded_raw_values_replace_stale_diagnostics(self):
        w = self.window
        w.reset_data('replay')
        w.receive(StreamDecoder().feed(observation_frame()))
        w.update_display()
        text = w.adc_diagnostic_label.text()
        for expected in ('project-v0.2.2', '首次观测异常：设备 118655 ms / 窗 118654',
                         '滤后 204～260', '贡献 214～235', '离群 12/5000',
                         '滤后连续最长 1.4 μs', '近驱动边沿 4', '仅相关统计'):
            self.assertIn(expected, text)
        self.assertTrue(set(OBSERVABLE).issubset(key for key, _, _ in EXTENSIONS))
        index = next(i for i, item in enumerate(EXTENSIONS) if item[0] == 'adc_fault_detail_flags')
        self.assertIn('不参与控制判据', w.ext_table.item(index, 1).toolTip())
        self.assertIn('首次观察到传感器故障时最近发布窗口', w.ext_table.item(index, 1).toolTip())
        self.assertIn('可能晚于判失败的输入一窗', w.ext_table.item(index, 1).toolTip())
        w.receive(StreamDecoder().feed(frame(version=1)))
        w.update_display()
        self.assertNotIn('首次观测异常：设备', w.adc_diagnostic_label.text())
        self.assertEqual(w.ext_table.item(index, 1).text(), '未提供')
        self.assertEqual(w.ext_table.item(index, 1).toolTip(), '转换级统计：未提供。')
        self.assertFalse((Path(self.tmp.name) / 'fpga_logs').exists())

    def test_new_statistics_never_automatically_authorize_or_start_motor(self):
        w = self.window
        w.reset_data('serial')
        w.worker = WorkerStub()
        w.serial_ready = True
        row = StreamDecoder().feed(frame(firmware=0x20002))[0]
        w.receive([dict(row, **OBSERVABLE)])
        w.update_display()
        self.assertFalse(w.direction_check.isChecked())
        self.assertFalse(w.allowed('G'))
        self.assertFalse(w.allowed('H'))
        w.direction_check.setChecked(True)
        self.assertTrue(w.allowed('G'))
        self.assertTrue(w.allowed('H'))
        self.assertEqual(w.worker.sent, [])
        w.latest['sensor_flags'] = 4
        self.assertFalse(w.allowed('G'))
        self.assertFalse(w.allowed('H'))
        self.assertTrue(w.allowed('S'))


if __name__ == '__main__':
    unittest.main()
