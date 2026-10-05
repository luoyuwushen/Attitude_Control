"""Trace/live stream separation, bounded failure tails, storage and operator UI."""
import csv
import json
import os
from pathlib import Path
import struct
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PySide6.QtWidgets import QApplication
from host.app import Window
from host.core import StreamDecoder, crc16, TRACE_STATUS_FIELDS
from host.control_trace import TraceCaptureCollector
from host.trace_recording import TraceArchiveWriter
from host.transport import SerialWorker
from host.tuning import H_LQI_TRACE_PROFILE, TUNING_SCHEMA
from test_control_trace import metadata, row, data, end, GOLD_DATA, GOLD_META, GOLD_END
from test_host_handover_control import full_frame
from test_host_ui import WorkerStub


def live(sequence=0, state=0, firmware=0x20009, flags=2, count=1, capture=0x1234):
    body = bytearray(full_frame((0, 0, 512, 3), firmware=firmware)[:-2])
    body[3] = 215
    body[4:6] = struct.pack('<H', sequence)
    body[18:21] = bytes((state, 1, 0))
    body[40:42] = struct.pack('<H', 12)
    body.extend(bytes((33, 5)) + struct.pack('<BHH', flags, count, capture))
    return bytes(body) + crc16(body).to_bytes(2, 'little')


def captures(decoder):
    return [e['capture'] for e in decoder.take_trace_events() if e['type'] == 'result']


def collected(*, firmware=0x20009, complete=True, rows=None):
    rows = [row()] if rows is None else rows
    collector = TraceCaptureCollector()
    collector.feed(metadata(total=len(rows), firmware=firmware))
    for i in range(0, len(rows), 7):
        collector.feed(data(rows[i:i+7], index=i, total=len(rows)))
    return (collector.feed(end(rows)) if complete else collector.finish('disconnected'))[0]


class TraceStreamTests(unittest.TestCase):
    def test_215_live_frame_every_split_and_unknown_profile(self):
        raw = live()
        self.assertEqual(len(raw), 215)
        for split in range(1, 215):
            decoder = StreamDecoder()
            self.assertEqual(decoder.feed(raw[:split]), [])
            result = decoder.feed(raw[split:])[0]
            self.assertEqual(tuple(result[k] for k in TRACE_STATUS_FIELDS), (2, 1, 0x1234))
            self.assertEqual(result['raw_hex'], raw.hex())
            self.assertEqual(decoder.trace_stats['frames'], 0)
        known = StreamDecoder().feed(live(state=2))[0]
        self.assertEqual(known['parameter_profile'], H_LQI_TRACE_PROFILE)
        self.assertEqual(TUNING_SCHEMA['profile_definitions'][H_LQI_TRACE_PROFILE]['gain_q10'],
                         [904758, 77909, -34213, -61768])
        unknown = StreamDecoder().feed(live(state=2, firmware=0x2000a))[0]
        self.assertIsNone(unknown['parameter_profile'])
        self.assertIsNone(unknown['target'])

    def test_mixed_live_history_fragments_do_not_modify_live_sequence_or_clock(self):
        decoder = StreamDecoder()
        wire = live(10) + GOLD_META + live(11) + GOLD_DATA + GOLD_END + live(12)
        records = []
        for i in range(0, len(wire), 13):
            records.extend(decoder.feed(wire[i:i+13]))
        self.assertEqual([r['sequence'] for r in records], [10, 11, 12])
        self.assertEqual(decoder.stats['frames'], 3)
        self.assertEqual(decoder.stats['missing_frames'], 0)
        self.assertEqual(decoder.stats['duplicates'], 0)
        self.assertEqual(decoder.stats['resets'], 0)
        self.assertEqual(decoder.stats['discarded_bytes'], 0)
        result = captures(decoder)
        self.assertEqual(len(result), 1)
        self.assertTrue(result[0]['complete'])
        self.assertNotIn('host_monotonic', result[0]['rows'][0])

    def test_history_alone_never_becomes_a_live_measurement(self):
        decoder = StreamDecoder()
        self.assertEqual(decoder.feed(GOLD_META + GOLD_DATA + GOLD_END), [])
        self.assertIsNone(decoder._first_time)
        self.assertIsNone(decoder._sequence)
        self.assertEqual(decoder.stats['frames'], 0)

    def test_bad_crc_and_585_remaining_chunks_form_one_failure_without_live_damage(self):
        rows = [row(sample=i) for i in range(4096)]
        decoder = StreamDecoder()
        decoder.feed(metadata(total=4096))
        damaged = bytearray(data(rows[:7], total=4096))
        damaged[-1] ^= 1
        self.assertEqual(decoder.feed(damaged), [])
        records = []
        live_sequence = 0
        for start in range(7, 4096, 7):
            wire = data(rows[start:start+7], index=start, total=4096)
            if start % 140 == 7:
                wire += live(live_sequence)
                live_sequence += 1
            records.extend(decoder.feed(wire))
        self.assertFalse(captures(decoder), 'Failure must stay aggregated until end/timeout/disconnect')
        decoder.feed(end(rows))
        result = captures(decoder)
        self.assertEqual(len(result), 1)
        self.assertFalse(result[0]['complete'])
        self.assertIn('packet_crc_mismatch', result[0]['errors'])
        self.assertEqual(result[0]['failed_tail_frames_seen'], 586)
        self.assertTrue(result[0]['failed_tail_end_seen'])
        self.assertFalse(result[0]['failed_tail_truncated'])
        self.assertEqual(len(result[0]['failed_tail_raw_frames_hex']), 586)
        self.assertEqual(len(records), live_sequence)
        self.assertEqual(decoder.stats['frames'], live_sequence)
        self.assertEqual(decoder.stats['crc_errors'], 0)
        self.assertEqual(decoder.stats['missing_frames'], 0)
        self.assertEqual(decoder.stats['discarded_bytes'], 0)
        self.assertEqual(decoder.trace_stats['crc_errors'], 1)
        for _ in range(10):
            decoder.feed(GOLD_DATA + GOLD_END)
        self.assertEqual(captures(decoder), [])
        decoder.feed(GOLD_META + GOLD_DATA + GOLD_END)
        self.assertTrue(captures(decoder)[0]['complete'])

    def test_missing_chunk_tail_evidence_is_bounded_and_disconnect_saves_once(self):
        decoder = StreamDecoder()
        decoder.feed(metadata(total=4))
        decoder.feed(data([row()], index=2, total=4))
        for _ in range(9000):
            decoder.feed(data([row()], index=3, total=4))
        decoder.finish_trace('disconnected')
        result = captures(decoder)
        self.assertEqual(len(result), 1)
        self.assertFalse(result[0]['complete'])
        self.assertTrue(result[0]['failed_tail_truncated'])
        self.assertLessEqual(result[0]['failed_tail_bytes_stored'], 262144)
        self.assertEqual(result[0]['failed_tail_frames_seen'], 9000)

    def test_timeout_before_metadata_and_after_data_cannot_claim_complete(self):
        for payload in (b'', GOLD_META + GOLD_DATA, GOLD_META + GOLD_DATA[:12]):
            decoder = StreamDecoder()
            decoder.begin_trace(now=time.monotonic() - 10)
            decoder.feed(payload)
            decoder.check_trace_timeout(now=time.monotonic() + 10)
            result = captures(decoder)
            self.assertEqual(len(result), 1)
            self.assertFalse(result[0]['complete'])
            self.assertTrue(result[0]['errors'])

    def test_motion_interrupt_does_not_drop_following_live_frames(self):
        decoder = StreamDecoder()
        decoder.feed(GOLD_META + GOLD_DATA)
        records = decoder.feed(live(1, state=2) + GOLD_END + live(2, state=2))
        self.assertEqual([r['sequence'] for r in records], [1, 2])
        result = captures(decoder)
        self.assertEqual(len(result), 1)
        self.assertFalse(result[0]['complete'])
        self.assertIn('motion_interrupted_export', result[0]['errors'])

    def test_retry_meta_flushes_old_failure_and_new_capture_is_independent(self):
        decoder = StreamDecoder()
        bad = GOLD_DATA[:-1] + bytes([GOLD_DATA[-1] ^ 1])
        decoder.feed(GOLD_META + bad + GOLD_META + GOLD_DATA + GOLD_END)
        result = captures(decoder)
        self.assertEqual(len(result), 2)
        self.assertFalse(result[0]['complete'])
        self.assertTrue(result[1]['complete'])
        self.assertEqual(result[0]['capture_id'], result[1]['capture_id'])


class TraceArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.writers = []

    def tearDown(self):
        for writer in self.writers:
            writer.request_close()
            self.assertTrue(writer.join(3))
        self.temp.cleanup()

    def writer(self, **kwargs):
        writer = TraceArchiveWriter(self.temp.name, **kwargs)
        self.writers.append(writer)
        return writer

    def finish(self, writer):
        writer.request_close()
        self.assertTrue(writer.join(3))
        return writer.take_events()

    def test_complete_and_incomplete_exports_get_separate_json_csv_files(self):
        writer = self.writer()
        for complete in (True, False):
            writer.submit(collected(complete=complete))
        events = self.finish(writer)
        self.assertEqual(len(events), 2)
        self.assertEqual([e['complete'] for e in events], [True, False])
        self.assertTrue(all(e['storage_complete'] for e in events))
        for event in events:
            saved = json.loads(Path(event['json_path']).read_text(encoding='utf8'))
            self.assertEqual(saved['complete'], event['complete'])
            self.assertTrue(saved['historical'])
            self.assertTrue(saved['storage_complete'])
            self.assertEqual(saved['parameter_profile'], H_LQI_TRACE_PROFILE)
            self.assertEqual(saved['profile_definition']['gain_q10'], [904758, 77909, -34213, -61768])
            with Path(event['csv_path']).open(encoding='utf-8-sig', newline='') as stream:
                records = list(csv.DictReader(stream))
            self.assertEqual(records[0]['integral_q24'], '-20971520')
            self.assertEqual(records[0]['target'], '0.0')
            self.assertEqual(records[0]['out'], '-123')

    def test_unknown_firmware_and_invalid_measurement_do_not_get_targets(self):
        writer = self.writer()
        writer.submit(collected(firmware=0x2000a))
        writer.submit(collected(rows=[row(sensor_flags=0)]))
        for event in self.finish(writer):
            saved = json.loads(Path(event['json_path']).read_text(encoding='utf8'))
            self.assertIsNone(saved['rows'][0]['target'])
            if saved['metadata']['firmware_id'] == 0x2000a:
                self.assertIsNone(saved['parameter_profile'])
                self.assertIsNone(saved['profile_definition'])

    def test_blocked_writer_does_not_block_submission_or_claim_saved(self):
        entered, release = threading.Event(), threading.Event()
        thread_ids = []
        def slow(capture):
            thread_ids.append(threading.get_ident())
            entered.set()
            if not release.wait(3):
                raise TimeoutError('test release missing')
            return dict(status='saved', storage_complete=True, complete=capture['complete'])
        writer = self.writer(write_capture=slow)
        started = time.monotonic()
        writer.submit(collected())
        self.assertLess(time.monotonic() - started, .2)
        self.assertTrue(entered.wait(1))
        self.assertEqual(writer.take_events(), [])
        self.assertNotEqual(thread_ids[0], threading.get_ident())
        writer.request_close()
        self.assertFalse(writer.done.is_set())
        release.set()
        self.assertTrue(self.finish(writer)[0]['storage_complete'])

    def test_csv_failure_keeps_raw_json_and_reports_storage_failure(self):
        writer = self.writer()
        original = Path.open
        def fail_csv(path, *args, **kwargs):
            if path.name == 'rows.csv.part':
                raise OSError('simulated disk full')
            return original(path, *args, **kwargs)
        with patch.object(Path, 'open', fail_csv):
            writer.submit(collected())
            event = self.finish(writer)[0]
        self.assertEqual(event['status'], 'storage_failed')
        self.assertFalse(event['storage_complete'])
        failures = list(Path(self.temp.name).glob('*/storage_error.json'))
        self.assertEqual(len(failures), 1)
        saved = json.loads(failures[0].read_text(encoding='utf8'))
        self.assertFalse(saved['storage_complete'])
        self.assertTrue(saved['raw_frames_hex'])
        self.assertIn('simulated disk full', saved['storage_error'])


class MemoryPort:
    def __init__(self, worker, chunks):
        self.worker, self.chunks = worker, list(chunks)
        self.is_open = False
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def open(self):
        self.is_open = True

    def write(self, payload):
        self.writes.append(payload)
        return len(payload)

    def read(self, count):
        if self.chunks:
            return self.chunks.pop(0)
        self.worker.request_close(False)
        return b''

    def close(self):
        self.is_open = False


class TraceWorkerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_worker_live_and_archive_separate_and_disconnect_drains_writer(self):
        with tempfile.TemporaryDirectory() as directory:
            worker = SerialWorker('FAKE', trace_directory=directory)
            worker.set_trace_context({'runtime_log_directory': 'example/fpga_logs/session'})
            statuses, records = [], []
            worker.trace_status.connect(statuses.append)
            worker.records.connect(records.extend)
            worker.send('T')
            port = MemoryPort(worker, [live(7), metadata(firmware=0x20009), GOLD_DATA, GOLD_END, live(8)])
            with patch('host.transport.serial.Serial', return_value=port):
                worker.run()
            self.assertEqual(port.writes, [b'T'])
            self.assertEqual([r['sequence'] for r in records], [7, 8])
            saved = [s for s in statuses if s['status'] == 'saved']
            self.assertEqual(len(saved), 1)
            self.assertTrue(saved[0]['complete'])
            self.assertTrue(saved[0]['storage_complete'])
            self.assertFalse(any('rows' in s for s in statuses))
            content = json.loads(Path(saved[0]['json_path']).read_text(encoding='utf8'))
            self.assertEqual(content['connection_context']['runtime_log_directory'], 'example/fpga_logs/session')
            self.assertTrue(worker._trace_writer.done.is_set())

    def test_disconnect_missing_end_saves_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            worker = SerialWorker('FAKE', trace_directory=directory)
            statuses = []
            worker.trace_status.connect(statuses.append)
            port = MemoryPort(worker, [GOLD_META + GOLD_DATA])
            with patch('host.transport.serial.Serial', return_value=port):
                worker.run()
            saved = [s for s in statuses if s['status'] == 'saved']
            self.assertEqual(len(saved), 1)
            self.assertFalse(saved[0]['complete'])
            self.assertTrue(saved[0]['storage_complete'])

    def test_worker_bad_first_data_keeps_live_connection_and_writes_one_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            rows = [row(sample=i) for i in range(4096)]
            first = bytearray(data(rows[:7], total=4096))
            first[-1] ^= 1
            chunks = [metadata(total=4096), bytes(first)]
            sequence = 0
            for start in range(7, 4096, 7):
                chunk = data(rows[start:start+7], index=start, total=4096)
                if start % 140 == 7:
                    chunk += live(sequence)
                    sequence += 1
                chunks.append(chunk)
            chunks.extend([end(rows), live(sequence)])
            worker = SerialWorker('FAKE', trace_directory=directory)
            statuses, records, failures = [], [], []
            worker.trace_status.connect(statuses.append)
            worker.records.connect(records.extend)
            worker.failed.connect(failures.append)
            port = MemoryPort(worker, chunks)
            with patch('host.transport.serial.Serial', return_value=port):
                worker.run()
            self.assertEqual(failures, [])
            self.assertEqual(len(records), sequence + 1)
            saved = [s for s in statuses if s['status'] == 'saved']
            self.assertEqual(len(saved), 1)
            self.assertFalse(saved[0]['complete'])
            self.assertEqual(len(list(Path(directory).glob('*/capture.json'))), 1)
            self.assertLess(len(statuses), 10)
            self.assertEqual(worker.delivery_overflows, 0)

    def test_stop_supersedes_queued_trace_without_hardware_access(self):
        worker = SerialWorker('FAKE')
        statuses = []
        worker.trace_status.connect(statuses.append)
        self.assertTrue(worker.send('T'))
        self.assertTrue(worker.send('S'))
        self.assertEqual(worker.commands.get_nowait()[0], 'S')
        self.assertTrue(worker.commands.empty())
        self.assertEqual(statuses[0]['status'], 'request_cancelled')


class TraceUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.window = Window(Path(self.temp.name))
        self.window.timer.stop()
        self.window.plot_timer.stop()
        self.window.reset_data('serial')
        self.window.worker = WorkerStub()
        self.window.serial_ready = True
        self.window.receive(StreamDecoder().feed(live()))

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.temp.cleanup()

    def test_export_gate_firmware_freshness_motion_and_frozen_metadata(self):
        self.assertTrue(self.window.allowed('T'))
        original = dict(self.window.latest)
        for changes in ({'firmware_version': 0x20008}, {'firmware_version': 0x2000a},
                        {'state': 2}, {'state': 4}, {'state': 5}, {'trace_flags': 0},
                        {'trace_flags': 3}, {'trace_flags': 6}, {'trace_flags': 0x82},
                        {'trace_row_count': 4097}, {'trace_capture_id': None}):
            self.window.latest = dict(original, **changes)
            self.assertFalse(self.window.allowed('T'), changes)
        self.window.latest = dict(original, state=3, fault=1, trace_row_count=0)
        self.assertTrue(self.window.allowed('T'))
        self.window.last_received = time.monotonic() - 2
        self.assertFalse(self.window.allowed('T'))
        self.assertTrue(self.window.allowed('S'))

    def test_historical_progress_does_not_refresh_live_state_or_claim_disk_success(self):
        latest, received, history = self.window.latest, self.window.last_received, len(self.window.history)
        self.window.receive_trace_status(dict(status='receiving', received_rows=4096, total_rows=4096))
        self.window.receive_trace_status(dict(status='saving', received_rows=4096, total_rows=4096,
                                               complete=True, storage_complete=False))
        self.assertIs(self.window.latest, latest)
        self.assertEqual(self.window.last_received, received)
        self.assertEqual(len(self.window.history), history)
        self.assertIn('尚未确认写盘成功', self.window.trace_progress.text())
        self.assertTrue(self.window.trace_busy)
        self.assertTrue(self.window.allowed('S'))
        self.window.receive_trace_status(dict(status='saved', received_rows=4096, total_rows=4096,
            complete=True, storage_complete=True, sample_gap_count=2, json_path='example/capture.json'))
        self.assertIn('2 处间断', self.window.trace_progress.text())
        self.assertIn('已保存', self.window.trace_progress.text())
        self.assertFalse(self.window.trace_busy)

    def test_incomplete_saved_and_storage_failure_are_distinct(self):
        self.window.receive_trace_status(dict(status='saved', received_rows=2, total_rows=7,
            complete=False, storage_complete=True, json_path='example/capture.json'))
        self.assertIn('导出不完整', self.window.trace_progress.text())
        self.window.receive_trace_status(dict(status='storage_failed', error='disk full', storage_complete=False))
        self.assertIn('保存失败', self.window.trace_progress.text())
        self.assertNotIn('已保存', self.window.trace_progress.text())


if __name__ == '__main__':
    unittest.main()
