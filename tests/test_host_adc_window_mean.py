"""Versioned ADC descriptions preserve the meanings of the unchanged wire fields."""
import copy
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from host.app import Window, EXTENSIONS
from host.core import StreamDecoder
from host.diagnostics import (adc_detail_text, adc_diagnostic_text, adc_filter_text,
    adc_observability_text, adc_quality_names, measurement_block_reason, motion_limit_reason)
from test_host_motor_test import RTL_GOLDEN_199 as GOLDEN_023


# Actual UART golden from tb_telemetry_v2 for project-v0.2.4, CRC 0x0592.
GOLDEN_024 = bytes.fromhex(
    'aa5502c70000fd0209fd0c007afe6000e803030121950104785634120204fffffeff'
    '0902e8030a0213000b02db000c02ff020d02ff030e0200000f02ff031002e01f'
    '110238ff120120130404000200140498badcfe15012916022913170211001802eb031902f137'
    '1a22010000010080ffff34127856e803e110c3001601a20d88132000feffff80efcdab89'
    '1b22cdabff031000f4016400c8002c019001f4015802bc0220038403e803dcfe10325476'
    '1c043412dcfe1d04cdab65871e01051f04eb32a4f89205')


class ContinuousMeanDescriptionsTests(unittest.TestCase):
    def test_real_new_uart_golden_every_fragment_preserves_all_old_payload_fields(self):
        old = StreamDecoder().feed(GOLDEN_023)[0]
        self.assertEqual(len(GOLDEN_024), 199)
        self.assertEqual([i for i,(a,b) in enumerate(zip(GOLDEN_023,GOLDEN_024)) if a!=b], [75,197,198])
        ignored = {'time','host_monotonic','firmware_version','raw_hex'}
        for split in range(1, 199):
            decoder=StreamDecoder()
            self.assertEqual(decoder.feed(GOLDEN_024[:split]), [])
            row=decoder.feed(GOLDEN_024[split:])[0]
            self.assertEqual(row['firmware_version'], 0x20004)
            self.assertEqual({k:v for k,v in row.items() if k not in ignored},
                             {k:v for k,v in old.items() if k not in ignored})

    def test_filter_and_quality_descriptions_are_version_specific(self):
        for version, name in ((0x20001,'3 点中值 + 16 点平均'), (0x20002,'3 点中值 + 16 点平均'),
                              (0x20003,'7 点中值 + 16 点平均'), (0x20004,'7 点中值 + 1 ms 全窗连续平均')):
            self.assertEqual(adc_filter_text({'firmware_version':version}), name)
        old=adc_quality_names(12, {'firmware_version':0x20003})
        self.assertIn('滤后窗口跨度异常',old)
        self.assertIn('连续中值采样突变',old)
        new=adc_quality_names(0x4c, version=0x20004)
        self.assertIn('均值块窗跨度 >32 code',new)
        self.assertIn('相邻均值块步进 >32 code',new)
        self.assertIn('RMS >16 code',new)
        self.assertIn('未知原因位',adc_quality_names(0x40, version=0x20003))
        generic=adc_quality_names(12)
        self.assertNotIn('中值',generic)
        self.assertNotIn('均值块',generic)
        self.assertIn('窗口 OTR',adc_quality_names(1, version=0x20004))
        self.assertIn('0/1023',adc_quality_names(2, version=0x20004))

    def test_continuous_mean_is_independent_crosscheck_not_old_bypass(self):
        row=dict(firmware_version=0x20004,adc_detail_flags=1,adc_conversion_count=5000,
                 adc_filtered_sum=1100200,adc_mean_q4=3521,adc_raw=220,
                 adc_contribution_min=215,adc_contribution_max=225)
        before=copy.deepcopy(row)
        detail=adc_detail_text(row)
        for text in ('全流核对均值：220.040','同一 1 ms 窗独立计算','Q4 舍入','稳态 5000',
                     '16 点诊断抽样范围不参与控制平均','40 个输出 / 8 μs','不是模拟毛刺宽度'):
            self.assertIn(text,detail)
        self.assertNotIn('旁路均值',detail)
        self.assertIn('均值 220.062',adc_diagnostic_text(row))
        self.assertIn('16 点诊断抽样 215～225',adc_observability_text(row))
        self.assertEqual(row,before)
        row['adc_detail_flags'] |= 2
        self.assertIn('不计算核对均值',adc_detail_text(row))
        row['adc_detail_flags']=1
        row['adc_conversion_count']=0
        self.assertIn('未提供有效的和/计数',adc_detail_text(row))
        row['firmware_version']=0x20003
        row['adc_conversion_count']=5000
        self.assertIn('旁路均值，不替代控制均值',adc_detail_text(row))
        self.assertIn('贡献 215～225',adc_observability_text(row))

    def test_current_and_latched_cause_use_version_without_changing_admission(self):
        row=dict(firmware_version=0x20004,adc_raw=222,adc_quality_reason=0x4c,sensor_fault_reason=0x4c,
                 arm_q10=0,arm_speed_q10=0,omega_q10=0)
        text=adc_diagnostic_text(row)
        self.assertIn('project-v0.2.4',text)
        self.assertEqual(text.count('均值块窗跨度 >32 code'),2)
        self.assertEqual(text.count('RMS >16 code'),2)
        for flags in (0,4,12,13,14,28):
            self.assertEqual(measurement_block_reason(dict(row,sensor_flags=flags)),
                             measurement_block_reason({'sensor_flags':flags}))
        self.assertEqual(motion_limit_reason(row,'G'),'')


class ContinuousMeanUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.app=QApplication.instance() or QApplication([])

    def test_main_and_table_labels_switch_versions_without_rewriting_telemetry(self):
        with tempfile.TemporaryDirectory() as tmp:
            with patch('host.app.list_ports.comports',return_value=[]):w=Window(Path(tmp))
            w.timer.stop();w.plot_timer.stop();w.reset_data('replay')
            for raw,expected in ((GOLDEN_024,'16 点诊断抽样'),(GOLDEN_023,'贡献')):
                row=StreamDecoder().feed(raw)[0]
                w.receive([row]);w.update_display()
                self.assertIn(expected,w.adc_diagnostic_label.text())
                for i,(key,_,_) in enumerate(EXTENSIONS):
                    if key.endswith(('contribution_min','contribution_max')):
                        self.assertIn(expected,w.ext_table.item(i,0).text())
                        self.assertEqual(w.ext_table.item(i,1).text(),str(row[key]))
                self.assertEqual(w.latest['raw_hex'],raw.hex())
            w.reset_data('replay');w.update_display()
            for i,(_,name,_) in enumerate(EXTENSIONS):self.assertEqual(w.ext_table.item(i,0).text(),name)
            w.close();w.deleteLater();self.app.processEvents()
            self.assertFalse((Path(tmp)/'fpga_logs').exists())


if __name__=='__main__':unittest.main()
