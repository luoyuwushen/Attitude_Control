"""Deterministic blocked-writer tests for the bounded experiment queue."""
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from host import core
from host.core import Replay, demo_record
from host.recording import AsyncSessionRecorder


class ControlledFactory:
    def __init__(self, block=(), errors=None, write_delay=0):
        self.entered = {name: threading.Event() for name in ('init', 'write', 'flush', 'close')}
        self.release = {name: threading.Event() for name in self.entered}
        for name, event in self.release.items():
            if name not in block:
                event.set()
        self.errors = errors or {}
        self.write_delay = write_delay
        self.operations = []
        self.thread_ids = []
        self.writer = None

    def step(self, name):
        self.thread_ids.append(threading.get_ident())
        self.entered[name].set()
        if not self.release[name].wait(3):
            raise TimeoutError('Test did not release ' + name)
        if name in self.errors:
            raise OSError(self.errors[name])

    def __call__(self, directory, metadata, *, background_flush):
        if background_flush is not False:
            raise AssertionError('Second flush thread would violate file ownership')
        self.step('init')
        factory = self
        class Writer:
            def __init__(self):
                self.directory = Path(directory)/'fake_session'
                self.metadata = metadata
                self.extra_summary = {}
                self.invalid = None

            def write(self, record):
                factory.step('write')
                if factory.write_delay:
                    time.sleep(factory.write_delay)
                factory.operations.append(('sample', record))

            def event(self, kind, detail, record=None, *, timestamp=None):
                factory.thread_ids.append(threading.get_ident())
                factory.operations.append(('event', kind, detail, record, timestamp))

            def flush(self):
                factory.step('flush')
                factory.operations.append(('flush',))

            def invalidate(self, error):
                factory.thread_ids.append(threading.get_ident())
                self.invalid = self.invalid or error
                factory.operations.append(('invalidate', str(error)))

            def close(self):
                factory.step('close')
                factory.operations.append(('close',))
                if self.invalid:
                    raise self.invalid
                return self.directory/'summary.json'
        self.writer = Writer()
        return self.writer


class AsyncRecordingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.recorders = []
        self.factories = []

    def tearDown(self):
        for factory in self.factories:
            for release in factory.release.values():
                release.set()
        for recorder in self.recorders:
            recorder.request_close()
            self.assertTrue(recorder.join(3), 'Writer did not finish cleanup')
        self.directory.cleanup()

    def create(self, factory=None, **kwargs):
        factory = factory or ControlledFactory()
        self.factories.append(factory)
        recorder = AsyncSessionRecorder(self.directory.name, {'source': 'offline'},
                                         writer_factory=factory, **kwargs)
        self.recorders.append(recorder)
        return recorder, factory

    def promptly(self, action):
        completed = threading.Event()
        errors = []
        def call():
            try:
                action()
            except Exception as error:
                errors.append(error)
            finally:
                completed.set()
        caller = threading.Thread(target=call, daemon=True)
        caller.start()
        self.assertTrue(completed.wait(.25), 'Producer waited for blocked disk operation')
        caller.join(1)
        if errors:
            raise errors[0]

    def finish(self, recorder):
        recorder.request_close()
        self.assertTrue(recorder.done.wait(2))
        self.assertTrue(recorder.join(1))
        return recorder.poll()

    def test_blocked_initialization_does_not_block_submission_poll_or_close_request(self):
        factory = ControlledFactory(block=('init',))
        recorder, factory = self.create(factory)
        self.assertTrue(factory.entered['init'].wait(1))
        self.assertEqual(recorder.poll()['state'], 'starting')
        self.assertIsNone(recorder.poll()['directory'])
        self.promptly(lambda: recorder.write({'sequence': 1}))
        self.promptly(lambda: recorder.event('marker', 'during init'))
        self.promptly(recorder.request_close)
        self.assertEqual(recorder.poll()['pending_count'], 2)
        self.assertFalse(recorder.done.is_set())
        factory.release['init'].set()
        status = self.finish(recorder)
        self.assertEqual((status['state'], status['written']), ('closed', 2))

    def test_blocked_write_counts_inflight_and_keeps_callers_responsive(self):
        recorder, factory = self.create(ControlledFactory(block=('write',)))
        recorder.write({'sequence': 1})
        self.assertTrue(factory.entered['write'].wait(1))
        first = recorder.poll()
        self.assertEqual(first['pending_count'], 1)
        self.assertGreater(first['pending_bytes'], 0)
        self.promptly(lambda: recorder.write({'sequence': 2}))
        self.promptly(lambda: recorder.event('marker', 'responsive'))
        self.promptly(recorder.request_close)
        self.assertEqual(recorder.poll()['pending_count'], 3)
        factory.release['write'].set()
        status = self.finish(recorder)
        self.assertEqual((status['accepted'], status['written'], status['pending_bytes']), (3, 3, 0))

    def test_blocked_idle_flush_does_not_hold_producer_or_state_lock(self):
        with patch.object(core.SessionRecorder, 'FLUSH_INTERVAL_S', .01):
            recorder, factory = self.create(ControlledFactory(block=('flush',)))
            self.assertTrue(factory.entered['flush'].wait(1))
            self.promptly(lambda: recorder.write({'sequence': 1}))
            self.promptly(lambda: recorder.poll())
            self.promptly(recorder.request_close)
            self.assertFalse(recorder.done.is_set())
            factory.release['flush'].set()
            self.assertEqual(self.finish(recorder)['state'], 'closed')

    def test_blocked_close_does_not_claim_done_or_saved_and_rejects_new_items(self):
        recorder, factory = self.create(ControlledFactory(block=('close',)))
        self.promptly(recorder.request_close)
        self.assertTrue(factory.entered['close'].wait(1))
        self.assertFalse(recorder.done.is_set())
        self.assertEqual(recorder.poll()['state'], 'closing')
        self.assertIsNone(recorder.poll()['summary'])
        self.promptly(recorder.request_close)
        with self.assertRaises(OSError):
            recorder.write({'sequence': 1})
        with self.assertRaises(OSError):
            recorder.event('marker', 'late')
        self.assertEqual(recorder.poll()['rejected'], 2)
        factory.release['close'].set()
        self.assertEqual(self.finish(recorder)['state'], 'closed')

    def test_deep_snapshots_fifo_timestamps_and_first_close_metadata(self):
        factory = ControlledFactory(block=('init',))
        metadata = {'notes': ['before']}
        recorder = AsyncSessionRecorder(self.directory.name, metadata, writer_factory=factory)
        self.recorders.append(recorder)
        self.factories.append(factory)
        first = {'sequence': 1, 'nested': {'values': [7]}}
        detail = {'text': ['release']}
        event_record = {'sequence': 1, 'nested': [9]}
        extra = {'communication': {'missing': [0]}}
        recorder.write(first)
        with patch('host.recording.time.time', return_value=1234.5):
            recorder.event('marker', detail, event_record)
        recorder.write({'sequence': 2})
        recorder.request_close(extra)
        metadata['notes'][0] = 'changed'
        first['nested']['values'][0] = 99
        detail['text'][0] = 'changed'
        event_record['nested'][0] = 99
        extra['communication']['missing'][0] = 99
        recorder.request_close({'replaced': True})
        factory.release['init'].set()
        self.assertEqual(self.finish(recorder)['state'], 'closed')
        rows = [item for item in factory.operations if item[0] in ('sample', 'event')]
        self.assertEqual([x[0] for x in rows], ['sample', 'event', 'sample'])
        self.assertEqual(rows[0][1]['nested']['values'], [7])
        self.assertEqual(rows[1][2:], ({'text': ['release']}, {'sequence': 1, 'nested': [9]}, 1234.5))
        self.assertEqual(factory.writer.metadata['notes'], ['before'])
        self.assertEqual(factory.writer.extra_summary, {'communication': {'missing': [0]}})
        self.assertEqual(len(set(factory.thread_ids)), 1)
        self.assertNotIn(threading.get_ident(), factory.thread_ids)

    def test_count_overflow_drains_accepted_prefix_but_never_reports_success(self):
        recorder, factory = self.create(ControlledFactory(block=('write',)), capacity=2)
        recorder.write({'sequence': 1})
        self.assertTrue(factory.entered['write'].wait(1))
        recorder.write({'sequence': 2})
        with self.assertRaisesRegex(OSError, 'queue is full'):
            recorder.event('marker', 'overflow')
        status = recorder.poll()
        self.assertEqual((status['state'], status['pending_count'], status['accepted']), ('failed', 2, 2))
        self.assertFalse(recorder.done.is_set())
        factory.release['write'].set()
        status = self.finish(recorder)
        self.assertEqual((status['written'], status['rejected'], status['pending_count']), (2, 1, 0))
        self.assertIsNone(status['summary'])
        self.assertEqual([x[1]['sequence'] for x in factory.operations if x[0] == 'sample'], [1, 2])
        self.assertIn('queue is full', str(factory.writer.invalid))

    def test_byte_overflow_includes_inflight_multibyte_payload(self):
        recorder, factory = self.create(ControlledFactory(block=('write',)), max_pending_bytes=500)
        recorder.write({'note': '界'*100})
        self.assertTrue(factory.entered['write'].wait(1))
        self.assertGreater(recorder.poll()['pending_bytes'], 300)
        with self.assertRaisesRegex(OSError, 'queue is full'):
            recorder.write({'note': '界'*100})
        factory.release['write'].set()
        status = self.finish(recorder)
        self.assertEqual((status['accepted'], status['written'], status['rejected']), (1, 1, 1))
        self.assertEqual(status['state'], 'failed')

    def test_one_oversized_item_is_rejected_without_unbounded_queue(self):
        recorder, factory = self.create(ControlledFactory(block=('init',)), max_pending_bytes=16)
        with self.assertRaises(OSError):
            recorder.write({'large': 'x'*100})
        self.assertEqual((recorder.poll()['accepted'], recorder.poll()['pending_bytes']), (0, 0))
        factory.release['init'].set()
        self.assertEqual(self.finish(recorder)['state'], 'failed')

    def test_io_failure_discards_remaining_queue_and_keeps_first_error_until_cleanup(self):
        factory = ControlledFactory(block=('write', 'close'), errors={'write': 'first disk failure', 'close': 'later cleanup failure'})
        recorder, factory = self.create(factory)
        recorder.write({'sequence': 1})
        self.assertTrue(factory.entered['write'].wait(1))
        recorder.write({'sequence': 2})
        recorder.event('marker', 'queued')
        factory.release['write'].set()
        self.assertTrue(factory.entered['close'].wait(1))
        status = recorder.poll()
        self.assertEqual((status['state'], status['pending_count'], status['written']), ('failed', 0, 0))
        self.assertEqual(status['accepted'], 3)
        self.assertFalse(recorder.done.is_set())
        with self.assertRaisesRegex(OSError, 'first disk failure'):
            recorder.write({'sequence': 3})
        factory.release['close'].set()
        status = self.finish(recorder)
        self.assertEqual(status['error'], 'first disk failure')
        self.assertEqual(status['written'], 0)
        self.assertIsNone(status['summary'])

    def test_idle_flush_failure_is_visible_without_any_following_write(self):
        with patch.object(core.SessionRecorder, 'FLUSH_INTERVAL_S', .01):
            recorder, factory = self.create(ControlledFactory(errors={'flush': 'idle disk failure'}))
            self.assertTrue(recorder.done.wait(1))
            status = recorder.poll()
            self.assertEqual((status['state'], status['error']), ('failed', 'idle disk failure'))
            self.assertIsNone(status['summary'])

    def test_final_close_failure_does_not_certify_completed_writes(self):
        recorder, factory = self.create(ControlledFactory(errors={'close': 'final close failure'}))
        recorder.write({'sequence': 1})
        status = self.finish(recorder)
        self.assertEqual((status['state'], status['written']), ('failed', 1))
        self.assertEqual(status['error'], 'final close failure')
        self.assertIsNone(status['summary'])

    def test_initialization_failure_completes_without_hidden_worker_exception(self):
        recorder, factory = self.create(ControlledFactory(block=('init',), errors={'init': 'cannot create files'}))
        recorder.write({'sequence': 1})
        factory.release['init'].set()
        self.assertTrue(recorder.done.wait(1))
        status = recorder.poll()
        self.assertEqual((status['state'], status['error']), ('failed', 'cannot create files'))
        self.assertEqual((status['accepted'], status['written'], status['pending_count']), (1, 0, 0))
        self.assertIsNone(status['directory'])

    def test_continuous_backlog_does_not_starve_periodic_flush(self):
        with patch.object(core.SessionRecorder, 'FLUSH_INTERVAL_S', .01):
            recorder, factory = self.create(ControlledFactory(block=('init',), write_delay=.002))
            for sequence in range(40):
                recorder.write({'sequence': sequence})
            recorder.request_close()
            factory.release['init'].set()
            self.assertEqual(self.finish(recorder)['written'], 40)
            operations = factory.operations
            self.assertGreaterEqual(sum(item[0] == 'flush' for item in operations), 3)
            first_flush = next(i for i, item in enumerate(operations) if item[0] == 'flush')
            self.assertTrue(any(item[0] == 'sample' for item in operations[first_flush+1:]))

    def test_real_writer_dynamic_columns_raw_events_and_close_tail(self):
        recorder = AsyncSessionRecorder(self.directory.name, {'source': 'offline'})
        self.recorders.append(recorder)
        one, two = demo_record(0, 0), demo_record(.02, 1)
        two['future_counter'] = 42
        recorder.write(one)
        before = time.time()
        recorder.event('marker', {'label': ['release']}, one)
        after = time.time()
        recorder.write(two)
        recorder.request_close({'test_metadata': {'ok': True}})
        status = self.finish(recorder)
        self.assertEqual((status['state'], status['accepted'], status['written']), ('closed', 3, 3))
        path = status['directory']
        rows = Replay.load(path/'samples.csv')
        self.assertEqual([row['sequence'] for row in rows], [0, 1])
        self.assertNotIn('future_counter', rows[0])
        self.assertEqual(rows[1]['future_counter'], 42)
        self.assertEqual((path/'raw_frames.bin').read_bytes(), bytes.fromhex(one['raw_hex']+two['raw_hex']))
        events = [json.loads(line) for line in (path/'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(events), 1)
        self.assertLessEqual(before, events[0]['time'])
        self.assertLessEqual(events[0]['time'], after)
        self.assertEqual(events[0]['sequence'], 0)
        summary = json.loads(status['summary'].read_text(encoding='utf-8'))
        self.assertEqual((summary['samples'], summary['raw_frames']), (2, 2))
        self.assertEqual(summary['test_metadata'], {'ok': True})
        self.assertEqual(list(path.glob('.samples_*.tmp')), [])

    def test_real_writer_overflow_drains_prefix_and_invalidates_success_summary(self):
        entered, release = threading.Event(), threading.Event()
        def delayed_factory(*args, **kwargs):
            entered.set()
            if not release.wait(2):
                raise TimeoutError('Test did not release real factory')
            return core.SessionRecorder(*args, **kwargs)
        recorder = AsyncSessionRecorder(self.directory.name, {}, capacity=1,
                                         writer_factory=delayed_factory)
        self.recorders.append(recorder)
        try:
            self.assertTrue(entered.wait(1))
            recorder.write(demo_record(0, 0))
            with self.assertRaisesRegex(OSError, 'queue is full'):
                recorder.write(demo_record(.02, 1))
        finally:
            release.set()
        status = self.finish(recorder)
        self.assertEqual((status['state'], status['accepted'], status['written'], status['rejected']),
                         ('failed', 1, 1, 1))
        self.assertEqual([row['sequence'] for row in Replay.load(status['directory']/'samples.csv')], [0])
        self.assertEqual(len((status['directory']/'raw_frames.bin').read_bytes()), 24)
        self.assertFalse((status['directory']/'summary.json').exists())

    def test_invalid_limits_and_prequeue_input_do_not_start_failed_sessions(self):
        for kwargs in ({'capacity': 0}, {'capacity': True}, {'max_pending_bytes': 0}):
            with self.assertRaises(ValueError):
                AsyncSessionRecorder(self.directory.name, {}, **kwargs)
        recorder, factory = self.create(ControlledFactory(block=('init',)))
        with self.assertRaises(TypeError):
            recorder.write(None)
        self.assertEqual((recorder.poll()['accepted'], recorder.poll()['rejected']), (0, 0))
        recorder.write({'sequence': 1})
        factory.release['init'].set()
        self.assertEqual(self.finish(recorder)['state'], 'closed')


if __name__ == '__main__':
    unittest.main()
