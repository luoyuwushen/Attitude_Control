"""Offline operator-safety and serial-failure acceptance checks."""
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    import serial
    from PySide6.QtWidgets import QApplication
    from host.app import Window
    from host.transport import SerialWorker
    from host.core import demo_record
except ModuleNotFoundError as error:
    if error.name not in {"PySide6", "pyqtgraph", "serial", "numpy"}:
        raise
    DEPENDENCY_ERROR = str(error)
else:
    DEPENDENCY_ERROR = None


class WorkerStub:
    """Capture attempted commands without any physical device access."""
    def __init__(self):
        self.closing = threading.Event()
        self.sent = []
        self.close_requests = []

    def send(self, command):
        self.sent.append(command)
        return True

    def request_close(self, send_stop=True):
        self.close_requests.append(send_stop)
        self.closing.set()

    def isRunning(self):
        return False


class FakePort:
    """One deterministic worker iteration, including real error branches."""
    def __init__(self, on_read=None, write_error=None, read_error=None,
                 short_write=False, close_error=None):
        self.on_read = on_read
        self.write_error = write_error
        self.read_error = read_error
        self.short_write = short_write
        self.close_error = close_error
        self.is_open = False
        self.in_waiting = 0
        self.writes = []
        self.closed = False
        self.open_settings = None

    def open(self):
        self.open_settings = (self.port, self.dtr, self.rts)
        self.is_open = True

    def write(self, data):
        self.writes.append(data)
        if self.write_error:
            self.write_error(data)
        return 0 if self.short_write else len(data)

    def read(self, count):
        if self.read_error:
            raise self.read_error
        if self.on_read:
            self.on_read()
        return b""

    def close(self):
        self.closed = True
        self.is_open = False
        if self.close_error:
            raise self.close_error


@unittest.skipIf(DEPENDENCY_ERROR, f"上位机依赖未安装：{DEPENDENCY_ERROR}")
class SerialSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def run_worker(self, worker, port):
        failures, statuses, commands = [], [], []
        worker.failed.connect(failures.append)
        worker.status.connect(statuses.append)
        worker.command_sent.connect(commands.append)
        with patch("host.transport.serial.Serial", return_value=port) as factory:
            worker.run()
        return failures, statuses, commands, factory

    def test_connect_does_not_send_an_automatic_command(self):
        worker = SerialWorker("TEST-COM")
        port = FakePort(on_read=lambda: worker.request_close(False))
        failures, statuses, commands, factory = self.run_worker(worker, port)
        self.assertEqual(port.writes, [])
        self.assertEqual(commands, [])
        self.assertEqual(failures, [])
        self.assertEqual(statuses, ["connected", "disconnected"])
        self.assertTrue(port.closed)
        self.assertEqual(port.open_settings, ("TEST-COM", False, False))
        self.assertEqual(factory.call_args.kwargs["baudrate"], 115200)

    def test_stop_replaces_queued_movement_and_calibration(self):
        worker = SerialWorker("TEST-COM")
        for command in ("G", "F", "B", "D", "U"):
            self.assertTrue(worker.send(command))
        self.assertTrue(worker.send("S"))
        port = FakePort(on_read=lambda: worker.request_close(False))
        failures, _, commands, _ = self.run_worker(worker, port)
        self.assertEqual(port.writes, [b"S"])
        self.assertEqual(commands, ["S"])
        self.assertEqual(failures, [])

    def test_stop_is_accepted_even_when_queue_is_full(self):
        worker = SerialWorker("TEST-COM")
        for _ in range(worker.commands.maxsize):
            self.assertTrue(worker.send("G"))
        self.assertFalse(worker.send("F"))
        self.assertTrue(worker.send("S"))
        port = FakePort(on_read=lambda: worker.request_close(False))
        self.run_worker(worker, port)
        self.assertEqual(port.writes, [b"S"])

    def test_disconnect_sends_stop_without_sending_pending_motion(self):
        worker = SerialWorker("TEST-COM")
        worker.send("G")
        worker.request_close()
        port = FakePort()
        failures, statuses, commands, _ = self.run_worker(worker, port)
        self.assertEqual(port.writes, [b"S"])
        self.assertEqual(commands, ["S"])
        self.assertEqual(failures, [])
        self.assertEqual(statuses[-1], "disconnected")
        self.assertFalse(worker.send("F"))

    def test_write_failure_is_reported_and_shutdown_attempts_stop(self):
        worker = SerialWorker("TEST-COM")
        worker.send("G")

        def fail_motion(data):
            if data != b"S":
                raise serial.SerialException("write failed")

        port = FakePort(write_error=fail_motion)
        failures, statuses, commands, _ = self.run_worker(worker, port)
        self.assertEqual(port.writes, [b"G", b"S"])
        self.assertEqual(commands, ["S"])
        self.assertTrue(any("write failed" in item for item in failures))
        self.assertEqual(statuses[-1], "disconnected")
        self.assertTrue(port.closed)

    def test_short_write_never_claims_command_was_sent(self):
        worker = SerialWorker("TEST-COM")
        worker.send("G")
        port = FakePort(short_write=True)
        failures, statuses, commands, _ = self.run_worker(worker, port)
        self.assertEqual(commands, [])
        self.assertEqual(port.writes, [b"G", b"S"])
        self.assertTrue(any("未完整写入" in item for item in failures))
        self.assertEqual(statuses[-1], "disconnected")

    def test_unplug_reports_failed_stop_and_closes_device(self):
        worker = SerialWorker("TEST-COM")

        def unplugged_write(_):
            raise serial.SerialException("device removed")

        port = FakePort(read_error=serial.SerialException("read unplugged"),
                        write_error=unplugged_write)
        failures, statuses, commands, _ = self.run_worker(worker, port)
        self.assertEqual(commands, [])
        self.assertTrue(any("read unplugged" in item for item in failures))
        self.assertTrue(any("SW3" in item for item in failures))
        self.assertEqual(statuses[-1], "disconnected")
        self.assertTrue(port.closed)

    def test_device_close_error_still_reports_disconnected(self):
        worker = SerialWorker("TEST-COM")
        port = FakePort(on_read=lambda: worker.request_close(False),
                        close_error=OSError("removed during close"))
        failures, statuses, _, _ = self.run_worker(worker, port)
        self.assertTrue(any("removed during close" in item for item in failures))
        self.assertEqual(statuses[-1], "disconnected")


@unittest.skipIf(DEPENDENCY_ERROR, f"上位机依赖未安装：{DEPENDENCY_ERROR}")
class OperatorAcceptanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QApplication.instance() or QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="host-ui-test-")
        with patch("host.app.list_ports.comports", return_value=[]):
            self.window = Window(self.directory.name)
        self.window.timer.stop()

    def tearDown(self):
        self.window.worker = None
        self.window.close()
        self.window.deleteLater()
        self.application.processEvents()
        self.directory.cleanup()

    def record(self, t=0, sequence=0, **changes):
        result = demo_record(8, sequence)
        result.update(dict(elapsed_s=t, state=0, source="serial", calibrated=1,
                           fault=0, theta_deg=0) | changes)
        return result

    def live(self, **changes):
        self.window.reset_data("serial")
        worker = self.window.worker = WorkerStub()
        self.window.serial_ready = True
        self.window.receive([self.record(**changes)])
        return worker

    def test_start_and_jog_require_calibration_idle_clear_fault_and_verified_direction(self):
        worker = self.live()
        for command in "GFB":
            self.assertFalse(self.window.send_command(command))
        self.window.direction_check.setChecked(True)
        for command in "GFB":
            self.assertTrue(self.window.send_command(command))
        self.assertEqual(worker.sent, list("GFB"))
        for changes in ({"calibrated": 0}, {"fault": 1}, {"state": 1},
                        {"state": 2}, {"state": 3}, {"state": 4}, {"state": 99}):
            self.window.receive([self.record(**changes)])
            for command in "GFB":
                self.assertFalse(self.window.send_command(command), (changes, command))
        self.assertEqual(worker.sent, list("GFB"))

    def test_calibrate_and_clear_only_while_idle_or_faulted(self):
        worker = self.live(calibrated=0)
        for state in (0, 3):
            self.window.receive([self.record(state=state, calibrated=0, fault=1)])
            for command in "DUR":
                self.assertTrue(self.window.send_command(command))
        for state in (1, 2, 4, 99):
            self.window.receive([self.record(state=state)])
            for command in "DUR":
                self.assertFalse(self.window.send_command(command))
        self.assertEqual(worker.sent, list("DURDUR"))

    def test_stale_or_absent_telemetry_blocks_operation_but_stop_remains_available(self):
        worker = self.live()
        self.window.direction_check.setChecked(True)
        self.window.last_received = time.monotonic() - 2
        self.window.update_display()
        for command in "DUGRFB":
            self.assertFalse(self.window.send_command(command))
            self.assertFalse(self.window.command_buttons[command].isEnabled())
        self.assertTrue(self.window.stop_button.isEnabled())
        self.assertTrue(self.window.send_command("S"))
        self.window.latest = None
        self.assertTrue(self.window.send_command("S"))
        self.assertEqual(worker.sent, ["S", "S"])

    def test_demo_and_replay_never_send_device_commands_even_with_residual_connection(self):
        worker = self.live()
        self.window.direction_check.setChecked(True)
        for source in ("demo", "replay"):
            self.window.source = source
            for command in "DUGSRFB":
                self.assertFalse(self.window.send_command(command), (source, command))
        self.assertEqual(worker.sent, [])

    def test_gui_backlog_does_not_refresh_stale_worker_telemetry(self):
        worker = self.live()
        self.window.direction_check.setChecked(True)
        self.window.receive([self.record(host_monotonic=time.monotonic() - 10)])
        self.window.update_display()
        self.assertFalse(self.window.is_fresh())
        for command in "DUGRFB":
            self.assertFalse(self.window.send_command(command))
            self.assertFalse(self.window.command_buttons[command].isEnabled())
        self.assertTrue(self.window.send_command("S"))
        self.assertEqual(worker.sent, ["S"])

    def test_missing_invalid_or_future_receive_clock_cannot_enable_movement(self):
        worker = self.live()
        self.window.direction_check.setChecked(True)
        for received in (None, "bad clock", float("nan"), float("inf"),
                         True, time.monotonic() + 10):
            self.window.receive([self.record(host_monotonic=received)])
            self.assertFalse(self.window.is_fresh(), received)
            self.assertFalse(self.window.send_command("G"), received)
        self.assertEqual(worker.sent, [])

    def test_late_signals_from_old_connection_cannot_replace_current_data(self):
        old_worker = SerialWorker("TEST-COM")
        old_worker.records.connect(self.window.receive_serial)
        self.window.worker = old_worker
        self.window.reset_data("serial")
        old_worker.records.emit([self.record(sequence=1)])
        self.assertEqual(self.window.latest["sequence"], 1)
        self.live(sequence=2)
        current = self.window.latest
        old_worker.records.emit([self.record(sequence=3)])
        self.assertIs(self.window.latest, current)
        self.window.start_demo()
        self.assertEqual(self.window.source, "serial")  # Active worker blocks switching.
        self.window.worker = None
        self.window.start_demo()
        old_worker.records.emit([self.record(sequence=4)])
        self.assertIsNone(self.window.latest)

    def test_closing_device_blocks_all_new_commands(self):
        worker = self.live()
        self.window.direction_check.setChecked(True)
        self.window.disconnect_source()
        self.assertEqual(worker.close_requests, [True])
        for command in "DUGSRFB":
            self.assertFalse(self.window.send_command(command))
        self.assertEqual(worker.sent, [])

    def csv(self, name="minimal.csv", start=0):
        path = Path(self.directory.name) / name
        path.write_text("time,sequence,theta_deg,state\n"
                        f"{100 + start},10,1,2\n{100.02 + start},11,0,2\n", encoding="utf-8")
        return path

    def test_legacy_minimal_csv_displays_missing_values_without_invented_data(self):
        self.assertTrue(self.window.load_replay(self.csv()))
        self.window.last_tick = time.monotonic()
        self.window.tick()
        self.assertEqual(self.window.latest["source"], "replay")
        self.assertEqual(self.window.cards["arm_deg"][0].text(), "—")
        self.assertEqual(self.window.cards["command_permille"][0].text(), "—")
        self.assertTrue(all(self.window.ext_table.item(row, 1).text() == "未提供"
                            for row in range(self.window.ext_table.rowCount())))
        self.assertFalse(self.window.send_command("G"))

    def test_reopen_csv_resets_history_events_verification_and_recovery(self):
        self.window.start_demo()
        self.window.receive([self.record(t=5, sequence=200)])
        self.window.direction_check.setChecked(True)
        self.window.mark_recovery()
        self.assertTrue(self.window.load_replay(self.csv()))
        self.assertIsNone(self.window.latest)
        self.assertEqual(len(self.window.history), 0)
        self.assertIsNone(self.window.recovery)
        self.assertFalse(self.window.direction_check.isChecked())
        self.window.last_tick = time.monotonic()
        self.window.tick()
        self.assertTrue(self.window.history)
        self.assertTrue(all(row["source"] == "replay" for row in self.window.history))
        self.assertTrue(self.window.load_replay(self.csv("again.csv", start=50)))
        self.assertEqual(self.window.replay_index, 0)
        self.assertEqual(len(self.window.history), 0)
        self.assertEqual(self.window.metrics.snapshot()["samples"], 0)

    def test_active_serial_session_cannot_be_replaced_by_csv_replay(self):
        worker = self.live()
        original = self.window.latest
        self.assertFalse(self.window.load_replay(self.csv()))
        self.assertEqual(self.window.source, "serial")
        self.assertIs(self.window.latest, original)
        self.assertIs(self.window.worker, worker)

    def test_recovery_hold_does_not_bridge_missing_frames_or_long_gap(self):
        self.window.start_demo()
        self.window.hold_seconds.setValue(0.5)
        self.window.receive([self.record(t=0, sequence=0, state=2)])
        self.window.mark_recovery()
        for n in range(1, 21):
            self.window.receive([self.record(t=n * .02, sequence=n, state=2)])
        self.assertIsNone(self.window.recovery_result)
        self.window.receive([self.record(t=.44, sequence=22, state=2, sequence_gap=1)])
        self.window.receive([self.record(t=.46, sequence=23, state=2)])
        self.assertIsNone(self.window.recovery_result)
        self.window.receive([self.record(t=.8, sequence=24, state=2)])
        self.window.receive([self.record(t=.82, sequence=25, state=2)])
        self.assertIsNone(self.window.recovery_result)
        for n in range(1, 27):
            self.window.receive([self.record(t=.82 + n * .02, sequence=25 + n, state=2)])
        self.assertAlmostEqual(self.window.recovery_result, .8)

    def test_recovery_reset_cancels_measurement_and_fault_breaks_hold(self):
        self.window.start_demo()
        self.window.receive([self.record(t=0, sequence=0, state=2)])
        self.window.mark_recovery()
        self.window.receive([self.record(t=.02, sequence=1, state=2)])
        self.window.receive([self.record(t=.04, sequence=2, state=2, fault=1)])
        self.assertIsNone(self.window.recovery_start)
        self.window.receive([self.record(t=.06, sequence=0, state=2, sequence_reset=True)])
        self.assertIsNone(self.window.recovery)
        self.assertIsNone(self.window.recovery_result)

    def test_unknown_fault_or_invalid_calibration_cannot_prove_recovery(self):
        for changes in ({"fault": None}, {"calibrated": None}, {"calibrated": 2}):
            with self.subTest(changes=changes):
                self.window.start_demo()
                self.window.receive([self.record(t=0, sequence=0, state=2, **changes)])
                self.window.mark_recovery()
                for n in range(1, 31):
                    self.window.receive([self.record(t=n * .02, sequence=n, state=2, **changes)])
                self.assertIsNone(self.window.recovery_start)
                self.assertIsNone(self.window.recovery_result)

    def test_recorded_demo_remains_labelled_after_replay_and_source_change_closes_session(self):
        self.window.start_demo()
        self.window.receive([self.record(t=0, sequence=0)])
        self.assertTrue(self.window.start_recording({"experiment": "offline acceptance"}))
        recorder = self.window.recorder
        self.window.receive([self.record(t=.02, sequence=1)])
        self.assertTrue(self.window.load_replay(self.csv()))
        self.assertIsNone(self.window.recorder)
        metadata = json.loads((recorder.directory / "metadata.json").read_text(encoding="utf-8"))
        summary = json.loads((recorder.directory / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(metadata["source"], "demo")
        self.assertEqual(summary["source"], "demo")
        self.assertEqual(summary["samples"], 1)
        self.assertTrue(self.window.load_replay(recorder.directory / "samples.csv"))
        self.assertEqual(self.window.replay_rows[0]["original_source"], "demo")


if __name__ == "__main__":
    unittest.main()
