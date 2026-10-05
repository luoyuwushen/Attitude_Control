import struct
import unittest

from host.core import StreamDecoder, crc16
from host.diagnostics import (adc_diagnostic_text, adc_quality_names,
                              measurement_block_reason, recovery_hint)


def recovery_frame():
    frame = bytearray(bytes.fromhex(
        'aa5502570000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
        '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
        '110238ff120120130400000200140498badcfe'))
    frame[75:79] = struct.pack('<I', 0x00020001)
    for tag, value, fmt in [(21, 0x20, '<B'), (22, 0x108, '<H'),
                             (23, 204, '<H'), (24, 251, '<H'), (25, 3519, '<H')]:
        payload = struct.pack(fmt, value)
        frame.extend(bytes([tag, len(payload)]) + payload)
    frame[3] = len(frame) + 2
    return bytes(frame) + struct.pack('<H', crc16(frame))


class SensorRecoveryTests(unittest.TestCase):
    def test_actual_rtl_uart_golden(self):
        # Emitted by tb_telemetry_v2 UART simulation, not the Python fixture.
        frame = bytes.fromhex(
            'aa55026a0000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
            '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
            '110238ff120120130401000200140498badcfe15012916022913170211001802eb031902f1379436')
        decoder = StreamDecoder()
        rows = []
        for byte in frame:
            rows.extend(decoder.feed(bytes([byte])))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['firmware_version'], 0x20001)
        self.assertEqual(rows[0]['adc_quality_reason'], 0x29)
        self.assertEqual(rows[0]['sensor_fault_reason'], 0x1329)
        self.assertEqual(rows[0]['adc_fault_window_min'], 17)
        self.assertEqual(rows[0]['adc_fault_window_max'], 1003)
        self.assertEqual(rows[0]['adc_fault_mean_q4'], 14321)

    def test_reason_and_trigger_window_all_split_boundaries(self):
        frame = recovery_frame()
        self.assertEqual(len(frame), 106)
        for split in range(1, len(frame)):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(frame[:split]), [])
            rows = decoder.feed(frame[split:])
            self.assertEqual(len(rows), 1)
            row = rows[0]
            self.assertEqual(row['firmware_version'], 0x00020001)
            self.assertEqual(row['adc_quality_reason'], 0x20)
            self.assertEqual(row['sensor_fault_reason'], 0x108)
            self.assertEqual(row['adc_fault_window_min'], 204)
            self.assertEqual(row['adc_fault_window_max'], 251)
            self.assertEqual(row['adc_fault_mean_q4'], 3519)

    def test_current_warning_and_latched_fault_window_are_distinct(self):
        row = StreamDecoder().feed(recovery_frame())[0]
        text = adc_diagnostic_text(row)
        self.assertIn('project-v0.2.1', text)
        self.assertIn('当前质量：原码跨度警示', text)
        self.assertIn('连续中值采样突变', text)
        self.assertIn('相邻均值突变', text)
        self.assertIn('首次传感器故障观测时原码窗口 204～251', text)
        self.assertIn('滤后均值 219.938', text)

    def test_raw_span_warning_alone_does_not_veto_trusted_measurement(self):
        self.assertEqual(measurement_block_reason({'sensor_flags': 12, 'adc_quality_reason': 32}), '')
        self.assertTrue(measurement_block_reason({'sensor_flags': 13, 'adc_quality_reason': 8}))

    def test_recovery_instruction_matches_actual_firmware(self):
        base = {'sensor_fault': 1, 'fault': 1, 'calibrated': 1}
        legacy = recovery_hint(dict(base, firmware_version=0x00020000))
        current = recovery_hint(dict(base, firmware_version=0x00020001))
        self.assertIn('R 只清控制器故障', legacy)
        self.assertIn('连续可信采样后按 R', current)
        self.assertIn('清除不启动电机', current)
        self.assertIn('ADC 超量程', recovery_hint(dict(base, firmware_version=0x00020001, adc_over_range=1)))

    def test_missing_and_unknown_reasons_not_claimed_healthy(self):
        self.assertEqual(adc_quality_names(None), '未提供')
        self.assertIn('未知原因位', adc_quality_names(0x8000))
        self.assertEqual(adc_quality_names(0), '无')
        text = adc_diagnostic_text({'adc_raw': 220, 'sensor_fault_reason': 8})
        self.assertIn('首次传感器故障观测时原码窗口 未提供～未提供', text)


if __name__ == '__main__':
    unittest.main()
