"""Serial settings and generic byte transport verified without hardware."""
from dataclasses import FrozenInstanceError
import unittest
from unittest.mock import patch

from host.serial_config import SerialConfig

try:
    import serial
    from host.core import demo_record
    from host.transport import SerialWorker
except ModuleNotFoundError as error:
    if error.name not in {'PySide6', 'serial'}:
        raise
    DEPENDENCY_ERROR = str(error)
else:
    DEPENDENCY_ERROR = None


class SerialConfigTests(unittest.TestCase):
    def test_default_is_immutable_project_configuration(self):
        config = SerialConfig()
        self.assertTrue(config.is_project_default)
        self.assertEqual(config.display_label, '115200 / 8N1')
        self.assertEqual(config.as_dict(), dict(baudrate=115200, bytesize=8, parity='N', stopbits=1.0))
        with self.assertRaises(FrozenInstanceError):
            config.baudrate = 9600

    def test_custom_values_and_accepted_ranges(self):
        for parity in 'NEOMS':
            for bytesize in (5, 6, 7, 8):
                for stopbits in (1, 1.5, 2):
                    config = SerialConfig(123456, bytesize, parity, stopbits)
                    self.assertIs(config.validate(), config)
                    self.assertFalse(config.is_project_default)
        self.assertEqual(SerialConfig(4_000_000).baudrate, 4_000_000)

    def test_invalid_settings_are_rejected_before_open(self):
        cases = [dict(baudrate=value) for value in (0, -1, 4_000_001, 9600.0, '9600', True)]
        cases += [dict(bytesize=value) for value in (4, 9, 7.0, '8', True)]
        cases += [dict(parity=value) for value in ('n', '', None, [], 'X')]
        cases += [dict(stopbits=value) for value in (0, 1.1, 3, '1', True, float('nan'))]
        for settings in cases:
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                SerialConfig(**settings)


class MemoryPort:
    def __init__(self, worker, data=b'', short_write=None, read_error=None, close_error=None):
        self.worker = worker
        self.data = data
        self.short_write = short_write
        self.read_error = read_error
        self.close_error = close_error
        self.is_open = False
        self.in_waiting = len(data)
        self.writes = []
        self.open_lines = None
        self.closed = False

    def open(self):
        self.open_lines = (self.port, self.dtr, self.rts)
        self.is_open = True

    def write(self, data):
        self.writes.append(data)
        return len(data) if self.short_write is None else self.short_write

    def read(self, count):
        if self.read_error:
            raise self.read_error
        # Drain accepted payloads, then close. The default close request also
        # checks that generic and incompatible project modes never append S.
        if self.worker.commands.empty():
            self.worker.request_close(True)
        data, self.data = self.data[:count], self.data[count:]
        self.in_waiting = len(self.data)
        return data

    def close(self):
        self.closed = True
        self.is_open = False
        if self.close_error:
            raise self.close_error


@unittest.skipIf(DEPENDENCY_ERROR, f'上位机依赖未安装：{DEPENDENCY_ERROR}')
class SerialTransportTests(unittest.TestCase):
    def run_memory(self, worker, port):
        events = {name: [] for name in ('raw_received', 'raw_sent', 'records', 'statistics',
                                      'command_sent', 'failed', 'status')}
        for name, values in events.items():
            getattr(worker, name).connect(values.append)
        with patch('host.transport.serial.Serial', return_value=port) as factory:
            worker.run()
        self.assertFalse(port.is_open)
        self.assertTrue(port.closed)
        self.assertEqual(events['status'][-1], 'disconnected')
        return events, factory

    def test_full_line_configuration_is_passed_before_open(self):
        config = SerialConfig(123456, 7, 'E', 1.5)
        worker = SerialWorker('TEST-COM', config=config, mode='raw')
        port = MemoryPort(worker)
        events, factory = self.run_memory(worker, port)
        self.assertEqual(factory.call_args.kwargs, dict(port=None, **config.as_dict(),
                         timeout=0.02, write_timeout=0.25, rtscts=False, dsrdtr=False))
        self.assertEqual(port.open_lines, ('TEST-COM', False, False))
        self.assertEqual(port.writes, [])
        self.assertEqual(events['failed'], [])
        self.assertIsNone(worker.decoder)

    def test_raw_preserves_bytes_and_never_appends_terminator_or_stop(self):
        worker = SerialWorker('TEST-COM', mode='raw', config=SerialConfig(parity='O', stopbits=2))
        self.assertFalse(worker.stop_on_close)
        payload = b'\x00\xff\x80G\r\n'
        self.assertTrue(worker.send_raw(payload))
        port = MemoryPort(worker, data=b'\x00\xaa\x55\xff\r\n')
        events, _ = self.run_memory(worker, port)
        self.assertEqual(port.writes, [payload])
        self.assertEqual(events['raw_sent'], [payload])
        self.assertEqual(events['raw_received'], [b'\x00\xaa\x55\xff\r\n'])
        self.assertEqual(events['records'], [])
        self.assertEqual(events['command_sent'], [])
        stats = events['statistics'][-1]
        self.assertEqual((stats['rx_bytes'], stats['tx_bytes'], stats['frames']), (6, 6, 0))

    def test_raw_never_sends_stop_on_read_failure(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        port = MemoryPort(worker, read_error=serial.SerialException('removed'))
        events, _ = self.run_memory(worker, port)
        self.assertEqual(port.writes, [])
        self.assertEqual(events['failed'], ['removed'])

    def test_project_also_emits_original_bytes_and_retains_decoder(self):
        worker = SerialWorker('TEST-COM')
        data = bytes.fromhex(demo_record(8, 2)['raw_hex'])
        port = MemoryPort(worker, data=data)
        events, _ = self.run_memory(worker, port)
        self.assertEqual(events['raw_received'], [data])
        self.assertEqual(len(events['records'][0]), 1)
        self.assertEqual(events['statistics'][-1]['frames'], 1)
        self.assertEqual(events['statistics'][-1]['rx_bytes'], len(data))
        self.assertEqual(events['statistics'][-1]['tx_bytes'], 1)
        self.assertEqual(port.writes, [b'S'])

    def test_modes_reject_each_others_send_api(self):
        generic = SerialWorker('TEST-COM', mode='raw')
        project = SerialWorker('TEST-COM')
        for command in 'DUGSRFB':
            self.assertFalse(generic.send(command))
        self.assertFalse(project.send_raw(b'G'))
        for command in ('', 'GS', b'S', None):
            self.assertFalse(project.send(command))

    def test_incompatible_project_settings_block_all_commands_and_final_stop(self):
        for config in (SerialConfig(9600), SerialConfig(bytesize=7),
                       SerialConfig(parity='E'), SerialConfig(stopbits=2)):
            with self.subTest(config=config):
                worker = SerialWorker('TEST-COM', config=config)
                for command in 'DUGSRFB':
                    self.assertFalse(worker.send(command))
                port = MemoryPort(worker)
                events, _ = self.run_memory(worker, port)
                self.assertEqual(port.writes, [])
                self.assertEqual(events['command_sent'], [])

    def test_narrow_data_bits_reject_truncation(self):
        for bits in (5, 6, 7):
            with self.subTest(bits=bits):
                worker = SerialWorker('TEST-COM', config=SerialConfig(bytesize=bits), mode='raw')
                self.assertFalse(worker.send_raw(bytes([1 << bits])))
                payload = bytes([0, (1 << bits) - 1])
                self.assertTrue(worker.send_raw(payload))
                port = MemoryPort(worker)
                events, _ = self.run_memory(worker, port)
                self.assertEqual(port.writes, [payload])
                self.assertEqual(events['raw_sent'], [payload])

    def test_raw_size_type_queue_and_close_limits(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        for payload in (b'', b'x' * 4097, 'hello', 8, None):
            self.assertFalse(worker.send_raw(payload))
        self.assertTrue(worker.send_raw(b'x' * 4096))
        for _ in range(7):
            self.assertTrue(worker.send_raw(b'x'))
        self.assertFalse(worker.send_raw(b'x'))
        worker.request_close(True)
        self.assertFalse(worker.send_raw(b'x'))
        port = MemoryPort(worker)
        self.run_memory(worker, port)
        self.assertEqual(port.writes, [])

    def test_raw_queues_immutable_snapshot_and_writes_only_when_worker_runs(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        payload = bytearray(b'abc')
        port = MemoryPort(worker)
        self.assertTrue(worker.send_raw(payload))
        payload[:] = b'def'
        self.assertEqual(port.writes, [])
        self.run_memory(worker, port)
        self.assertEqual(port.writes, [b'abc'])

    def test_raw_partial_write_reports_failure_without_success_event_or_stop(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        self.assertTrue(worker.send_raw(b'abc'))
        port = MemoryPort(worker, short_write=1)
        events, _ = self.run_memory(worker, port)
        self.assertEqual(port.writes, [b'abc'])
        self.assertEqual(events['raw_sent'], [])
        self.assertEqual(events['statistics'][-1]['tx_bytes'], 1)
        self.assertTrue(any('未完整写入' in message for message in events['failed']))

    def test_large_raw_packet_is_line_rate_bounded_and_receives_between_chunks(self):
        for config in (SerialConfig(), SerialConfig(9600, 7, 'E', 2)):
            with self.subTest(config=config):
                worker = SerialWorker('TEST-COM', config=config, mode='raw')
                payload = b'x' * 4096
                self.assertTrue(worker.send_raw(payload))
                port = MemoryPort(worker)
                reads = []
                frame_bits = 1 + config.bytesize + (config.parity != 'N') + config.stopbits

                def write(data):
                    # Model a driver which waits for bytes to leave the UART.
                    # The old whole-packet write fails this legitimate case.
                    if len(data) * frame_bits / config.baudrate > .25:
                        raise serial.SerialTimeoutException('packet exceeds write timeout')
                    port.writes.append(data)
                    return len(data)

                def read(count):
                    reads.append(sum(map(len, port.writes)))
                    if reads[-1] == len(payload):
                        worker.request_close(False)
                    return b'r'

                port.write, port.read = write, read
                events, _ = self.run_memory(worker, port)
                self.assertEqual(b''.join(port.writes), payload)
                self.assertGreater(len(port.writes), 1)
                self.assertEqual(events['raw_sent'], [payload])
                self.assertEqual(events['failed'], [])
                self.assertEqual(events['statistics'][-1]['tx_bytes'], len(payload))
                self.assertTrue(any(0 < offset < len(payload) for offset in reads))
                self.assertEqual(len(events['raw_received']), len(reads))

    def test_close_cancels_remaining_raw_chunks_without_claiming_complete_packet(self):
        worker = SerialWorker('TEST-COM', config=SerialConfig(9600), mode='raw')
        payload = b'x' * 4096
        self.assertTrue(worker.send_raw(payload))
        port = MemoryPort(worker)
        activity = []
        worker.wire_activity.connect(activity.append)
        # Close on the first interleaved read, after only one chunk was sent.
        port.read = lambda count: worker.request_close(False) or b''
        events, _ = self.run_memory(worker, port)
        self.assertEqual(len(port.writes), 1)
        self.assertLess(len(port.writes[0]), len(payload))
        self.assertEqual(events['raw_sent'], [])
        self.assertEqual(events['failed'], [])
        self.assertEqual(events['statistics'][-1]['tx_bytes'], len(port.writes[0]))
        self.assertTrue(activity[-1]['cancelled'])
        self.assertEqual(activity[-1]['requested_bytes'], len(payload) - len(port.writes[0]))

    def test_raw_close_failure_still_reports_disconnected(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        port = MemoryPort(worker, close_error=OSError('close removed'))
        events, _ = self.run_memory(worker, port)
        self.assertTrue(any('close removed' in message for message in events['failed']))
        self.assertEqual(port.writes, [])

    def test_invalid_worker_parameters_never_open_device(self):
        with patch('host.transport.serial.Serial') as factory:
            for parameters in (dict(mode='invalid'), dict(config={}), dict(config=9600)):
                with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                    SerialWorker('TEST-COM', **parameters)
            factory.assert_not_called()

    def test_open_error_always_reports_disconnected(self):
        worker = SerialWorker('TEST-COM', mode='raw')
        statuses, failures = [], []
        worker.status.connect(statuses.append)
        worker.failed.connect(failures.append)
        with patch('host.transport.serial.Serial', side_effect=serial.SerialException('unsupported')):
            worker.run()
        self.assertEqual(statuses, ['disconnected'])
        self.assertEqual(failures, ['unsupported'])


if __name__ == '__main__':
    unittest.main()
