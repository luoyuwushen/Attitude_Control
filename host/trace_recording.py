"""Bounded background persistence for historical control exports.

The serial worker transfers ownership of each terminal collector result. No
large serialization or file operation runs in a GUI callback or serial read.
"""
from collections import deque
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import uuid
from host.tuning import H_LQI_TRACE_PROFILE, H_LQI_CONDITIONED_PROFILE, TUNING_SCHEMA
from host.adc_status import ADC_CONDITIONING_FIRMWARE, measurement_ready


class TraceArchiveWriter:
    def __init__(self, directory, capacity=8, *, write_capture=None):
        if type(capacity) is not int or capacity < 1:
            raise ValueError("trace archive capacity must be positive")
        self.directory = Path(directory)
        self.capacity = capacity
        self._write_capture = write_capture or self._persist
        self._condition = threading.Condition()
        self._queue = deque()
        self._events = deque()
        self._pending = 0
        self._closing = False
        self.done = threading.Event()
        self._thread = threading.Thread(target=self._run, name="J280 trace writer", daemon=True)
        self._thread.start()

    def submit(self, capture):
        """Transfer ownership without copying rows; caller must not mutate it."""
        with self._condition:
            if self._closing:
                raise OSError("连续记录写入器已关闭")
            accepted = self._pending < self.capacity
            if not accepted:
                # Reserve one bounded failure item so the rejected export's
                # bytes survive too. The serial owner must then close the link.
                capture = dict(capture, complete=False, status="failed",
                    errors=list(capture["errors"]) + ["trace_storage_queue_overflow"])
                self._closing = True
            self._queue.append(capture)
            self._pending += 1
            self._condition.notify()
            return accepted

    def request_close(self):
        with self._condition:
            self._closing = True
            self._condition.notify()

    def take_events(self):
        with self._condition:
            events = list(self._events)
            self._events.clear()
            return events

    def join(self, timeout=None):
        self._thread.join(timeout)
        return self.done.is_set()

    @staticmethod
    def _json(path, value):
        with path.open("w", encoding="utf-8", newline="\n") as target:
            json.dump(value, target, ensure_ascii=False, allow_nan=False, indent=2)
            target.write("\n")
            target.flush()
            os.fsync(target.fileno())

    def _persist(self, capture):
        directory = self.directory / (datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f_") + uuid.uuid4().hex[:8])
        directory.mkdir(parents=True, exist_ok=False)
        result = dict(capture, historical=True, storage_complete=False, storage_status="writing",
                      saved_utc=datetime.now(timezone.utc).isoformat())
        metadata = capture.get("metadata") or {}
        firmware = metadata.get("firmware_id")
        profile = {0x00020009: H_LQI_TRACE_PROFILE,
                   ADC_CONDITIONING_FIRMWARE: H_LQI_CONDITIONED_PROFILE}.get(firmware)
        known = profile is not None
        conditioned = firmware == ADC_CONDITIONING_FIRMWARE
        result["parameter_profile"] = profile
        result["profile_definition"] = TUNING_SCHEMA["profile_definitions"][profile] if known else None
        result["profile_source"] = "known_firmware_definition_not_runtime_readback" if known else "unknown"
        result["adc_mean_q4_semantics"] = ("conditioned_control_input" if conditioned else
                                            "median7_full_window_mean" if known else "unknown")
        result["rows"] = []
        for original in capture["rows"]:
            row = dict(original)
            valid = (measurement_ready(dict(row, firmware_version=firmware)) and row["fault"] == 0 and
                     (conditioned or row["adc_quality"] & 0x5f == 0))
            active = bool(known and row["h_flags"] & 1 and row["fault"] == 0)
            if conditioned:
                row.update(adc_control_q4=row["adc_mean_q4"],
                           blind_zone=bool(row["sensor_flags"] & 0x20),
                           spike_rejected=bool(row["sensor_flags"] & 0x40))
            row.update(actual=row["theta_deg"], actual_valid=valid,
                       target=0.0 if active and valid else None,
                       out=row["command_permille"],
                       algorithm="H_LQI" if active else "unknown",
                       parameter_profile=profile if active else None)
            result["rows"].append(row)
        json_path, csv_path = directory / "capture.json", directory / "rows.csv"
        try:
            # Preserve raw evidence before attempting a derived CSV. If the
            # second file fails, this JSON explicitly says storage incomplete.
            self._json(json_path, result)
            fields = list(result["rows"][0]) if result["rows"] else [
                "index", "sample_counter", "encoder_count", "adc_mean_q4", "theta_q10",
                "omega_q10", "arm_q10", "arm_speed_q10", "integral_q24",
                "command_permille", "motor_command_permille", "control_age_ms",
                "h_flags", "adc_quality", "sensor_flags", "fault", "raw_hex"]
            temporary_csv = directory / "rows.csv.part"
            with temporary_csv.open("w", encoding="utf-8-sig", newline="") as target:
                writer = csv.DictWriter(target, fieldnames=fields)
                writer.writeheader()
                writer.writerows(result["rows"])
                target.flush()
                os.fsync(target.fileno())
            os.replace(temporary_csv, csv_path)
            result.update(storage_complete=True, storage_status="saved")
            temporary_json = directory / "capture.json.part"
            self._json(temporary_json, result)
            os.replace(temporary_json, json_path)
        except Exception as error:
            result.update(storage_complete=False, storage_status="failed", storage_error=str(error))
            try:
                self._json(directory / "storage_error.json", result)
            except Exception:
                pass
            raise OSError(f"{directory}: {error}") from error
        continuity = capture["sample_counter_continuity"]
        return dict(status="saved", storage_complete=True, complete=capture["complete"],
                    capture_id=capture["capture_id"], received_rows=len(capture["rows"]),
                    total_rows=capture["total_rows"], errors=list(capture["errors"]),
                    sample_contiguous=continuity["contiguous"],
                    sample_gap_count=len(continuity["gaps"]),
                    missing_sample_count=continuity["missing_sample_count"],
                    json_path=str(json_path), csv_path=str(csv_path))

    def _run(self):
        try:
            while True:
                with self._condition:
                    while not self._queue and not self._closing:
                        self._condition.wait()
                    if not self._queue:
                        return
                    capture = self._queue.popleft()
                try:
                    event = self._write_capture(capture)
                except Exception as error:
                    event = dict(status="storage_failed", storage_complete=False,
                                 complete=capture["complete"], error=str(error),
                                 capture_id=capture["capture_id"], received_rows=len(capture["rows"]),
                                 total_rows=capture["total_rows"])
                with self._condition:
                    self._pending -= 1
                    self._events.append(event)
        finally:
            self.done.set()
