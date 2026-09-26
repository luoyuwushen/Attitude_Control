"""Incremental telemetry decoding, loss diagnostics and durable session files.

Version 1 matches the current RTL. Version 2 reserves optional TLV measurements;
no extended measurement exists in a record unless the device transmitted it.
"""
from __future__ import annotations

import csv
import json
import math
import struct
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path


BASE_FIELDS = [
    "time", "host_monotonic", "elapsed_s", "sequence", "adc", "theta_deg",
    "omega_rad_s", "arm_deg", "arm_speed_rad_s", "command_permille", "state",
    "calibrated", "fault", "protocol_version", "source", "sequence_gap",
    "sequence_reset", "sequence_duplicate", "raw_hex", "theta_q10", "omega_q10",
    "arm_q10", "arm_speed_q10",
]
# TLV type, public name, little-endian format, conversion factor.
TLV_FIELDS = {
    1: ("device_time_ms", "<I", 1),
    2: ("encoder_count", "<i", 1),
    3: ("target_arm_deg", "<h", 180 / math.pi / 1024),
    4: ("target_speed_rad_s", "<h", 1 / 1024),
    5: ("observer_theta_deg", "<h", 180 / math.pi / 1024),
    6: ("observer_confidence", "<f", 1),
    7: ("energy_error", "<f", 1),
    8: ("parameter_version", "<I", 1),
    9: ("control_loop_us", "<H", 1),
    10: ("sensor_flags", "<H", 1),
    11: ("adc_down", "<H", 1),
    12: ("adc_up", "<H", 1),
}
EXTENDED_FIELDS = [value[0] for value in TLV_FIELDS.values()]


def crc16(data):
    """CRC-16/CCITT-FALSE (initial FFFF, polynomial 1021)."""
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xffff
    return crc


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _sequence_info(previous, sequence):
    if previous is None:
        return 0, False, False
    delta = (sequence - previous) & 0xffff
    if delta == 0:
        return 0, False, True
    if delta < 32768:
        return delta - 1, False, False
    return 0, True, False


def _decode_frame(frame):
    if len(frame) < 24 or frame[:2] != b"\xaa\x55" or frame[3] != len(frame):
        raise ValueError("Invalid frame header or length")
    version = frame[2]
    if version not in (1, 2) or (version == 1 and len(frame) != 24):
        raise ValueError("Unsupported protocol version or length")
    if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
        raise ValueError("CRC mismatch")
    sequence, adc, theta, omega, arm, arm_speed, command = struct.unpack("<HHhhhhh", frame[4:18])
    record = {
        "sequence": sequence, "adc": adc,
        "theta_q10": theta, "omega_q10": omega,
        "arm_q10": arm, "arm_speed_q10": arm_speed,
        "theta_deg": theta / 1024 * 180 / math.pi,
        "omega_rad_s": omega / 1024, "arm_deg": arm / 1024 * 180 / math.pi,
        "arm_speed_rad_s": arm_speed / 1024, "command_permille": command,
        "state": frame[18], "calibrated": frame[19] & 1, "fault": frame[20],
        "protocol_version": version, "raw_hex": frame.hex(),
    }
    if version == 2:
        cursor, end, seen = 22, len(frame) - 2, set()
        while cursor < end:
            if cursor + 2 > end:
                raise ValueError("Truncated TLV header")
            kind, length = frame[cursor:cursor + 2]
            cursor += 2
            if cursor + length > end:
                raise ValueError("Truncated TLV value")
            value_bytes = frame[cursor:cursor + length]
            cursor += length
            if kind in TLV_FIELDS:
                name, fmt, factor = TLV_FIELDS[kind]
                if kind in seen or length != struct.calcsize(fmt):
                    raise ValueError("Duplicate TLV or invalid field size")
                seen.add(kind)
                value = struct.unpack(fmt, value_bytes)[0]
                if not _finite(value):
                    raise ValueError("Non-finite TLV measurement")
                record[name] = value if factor == 1 else value * factor
    return record


class StreamDecoder:
    """Parse arbitrary serial chunks and resynchronize after corrupted frames."""

    def __init__(self, protocol="project"):
        if protocol != "project":
            raise ValueError("Only the project telemetry protocol is supported")
        self.protocol = protocol
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.stats = dict.fromkeys(
            ("frames", "crc_errors", "discarded_bytes", "unsupported_frames",
             "missing_frames", "duplicates", "resets"), 0)
        self._first_time = None
        self._sequence = None

    def _discard(self, count):
        del self.buffer[:count]
        self.stats["discarded_bytes"] += count

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while self.buffer:
            header = self.buffer.find(b"\xaa\x55")
            if header < 0:
                self._discard(len(self.buffer) - (1 if self.buffer[-1] == 0xaa else 0))
                break
            if header:
                self._discard(header)
            if len(self.buffer) < 4:
                break
            version, length = self.buffer[2:4]
            if version not in (1, 2) or length < 24 or (version == 1 and length != 24):
                self.stats["unsupported_frames"] += 1
                self._discard(1)
                continue
            if len(self.buffer) < length:
                break
            frame = bytes(self.buffer[:length])
            if crc16(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                self.stats["crc_errors"] += 1
                self._discard(1)
                continue
            try:
                record = _decode_frame(frame)
            except ValueError:
                self.stats["unsupported_frames"] += 1
                self._discard(length)
                continue
            del self.buffer[:length]
            received = time.monotonic()
            if self._first_time is None:
                self._first_time = received
            gap, reset, duplicate = _sequence_info(self._sequence, record["sequence"])
            self._sequence = record["sequence"]
            record.update(time=time.time(), host_monotonic=received,
                          elapsed_s=max(0, received - self._first_time), source="serial",
                          sequence_gap=gap, sequence_reset=reset, sequence_duplicate=duplicate)
            self.stats["frames"] += 1
            self.stats["missing_frames"] += gap
            self.stats["resets"] += int(reset)
            self.stats["duplicates"] += int(duplicate)
            records.append(record)
        return records


class Metrics:
    """Measured sample statistics; continuous intervals require gaps <= 100 ms."""

    MAX_RECEIVE_GAP_S = 0.1
    IN_BAND_DEG = 2.0

    def __init__(self):
        self.samples = self.balance_samples = self.gap_count = self.missing_frames = 0
        self.reset_count = self.duplicate_count = 0
        self._sum = self._sum_square = self._peak = 0.0
        self._arm_min = self._arm_max = None
        self._command_samples = self._saturated = 0
        self._swing_commands = self._swing_saturated = 0
        self._balance_commands = self._balance_saturated = 0
        self._first = self._last = None
        self._previous_elapsed = None
        self._sequence = self._previous_state = self._swing_start = None
        self._balance_start = self._in_band_start = None
        self._longest_balance = self._longest_in_band = 0.0
        self._in_band_samples = 0
        self.capture_count = 0
        self.capture_delay_s = None

    def add(self, record):
        self.samples += 1
        sequence = record.get("sequence")
        inferred = _sequence_info(self._sequence, sequence) if isinstance(sequence, int) else (0, False, False)
        gap = record.get("sequence_gap", inferred[0])
        reset = bool(record.get("sequence_reset")) or inferred[1]
        duplicate = bool(record.get("sequence_duplicate")) or inferred[2]
        gap = max(inferred[0], max(0, int(gap)) if _finite(gap) else 0)
        self._sequence = sequence if isinstance(sequence, int) else self._sequence
        self.gap_count += int(gap > 0)
        self.missing_frames += gap
        self.reset_count += int(reset)
        self.duplicate_count += int(duplicate)
        elapsed = record.get("elapsed_s")
        time_reset = bool(record.get("time_reset")) or (_finite(elapsed) and self._previous_elapsed is not None and elapsed < self._previous_elapsed)
        receive_gap = (_finite(elapsed) and self._previous_elapsed is not None
                       and elapsed - self._previous_elapsed > self.MAX_RECEIVE_GAP_S + 1e-9)
        broken = bool(gap or reset or time_reset or receive_gap or not _finite(elapsed))
        if _finite(elapsed):
            if self._first is None:
                self._first = elapsed
            self._last = elapsed if self._last is None else max(self._last, elapsed)
        if broken:
            self._swing_start = None
            self._balance_start = self._in_band_start = None
        if duplicate:
            return
        self._previous_elapsed = elapsed if _finite(elapsed) else None
        arm, command, theta = record.get("arm_deg"), record.get("command_permille"), record.get("theta_deg")
        if _finite(arm):
            self._arm_min = arm if self._arm_min is None else min(self._arm_min, arm)
            self._arm_max = arm if self._arm_max is None else max(self._arm_max, arm)
        state = record.get("state")
        if _finite(command):
            self._command_samples += 1
            self._saturated += int(abs(command) >= 1000)
            if state == 1:
                self._swing_commands += 1
                self._swing_saturated += int(abs(command) >= 800)
            elif state == 2:
                self._balance_commands += 1
                self._balance_saturated += int(abs(command) >= 1000)
        healthy = record.get("calibrated") == 1 and record.get("fault") == 0
        if healthy and state == 2 and _finite(theta):
            self.balance_samples += 1
            self._sum += theta
            self._sum_square += theta * theta
            self._peak = max(self._peak, abs(theta))
            if _finite(elapsed):
                if self._balance_start is None:
                    self._balance_start = elapsed
                self._longest_balance = max(self._longest_balance, elapsed - self._balance_start)
                if abs(theta) <= self.IN_BAND_DEG:
                    self._in_band_samples += 1
                    if self._in_band_start is None:
                        self._in_band_start = elapsed
                    self._longest_in_band = max(self._longest_in_band, elapsed - self._in_band_start)
                else:
                    self._in_band_start = None
        else:
            self._balance_start = self._in_band_start = None
        if not healthy:
            self._swing_start = None
        elif state == 1 and self._previous_state != 1 and not broken:
            self._swing_start = elapsed
        elif state == 2 and self._previous_state == 1 and self._swing_start is not None and _finite(elapsed):
            self.capture_count += 1
            if self.capture_delay_s is None and elapsed >= self._swing_start:
                self.capture_delay_s = elapsed - self._swing_start
            self._swing_start = None
        elif state != 1:
            self._swing_start = None
        self._previous_state = state

    def snapshot(self):
        duration = max(0, self._last - self._first) if self._first is not None else 0.0
        return {
            "samples": self.samples, "total_duration_s": duration,
            "balance_samples": self.balance_samples,
            "balance_rms_deg": math.sqrt(self._sum_square / self.balance_samples) if self.balance_samples else None,
            "balance_mean_deg": self._sum / self.balance_samples if self.balance_samples else None,
            "balance_peak_deg": self._peak if self.balance_samples else None,
            "arm_min_deg": self._arm_min, "arm_max_deg": self._arm_max,
            "saturation_ratio": self._saturated / self._command_samples if self._command_samples else None,
            "swing_saturation_ratio": self._swing_saturated / self._swing_commands if self._swing_commands else None,
            "balance_saturation_ratio": self._balance_saturated / self._balance_commands if self._balance_commands else None,
            "capture_count": self.capture_count, "capture_delay_s": self.capture_delay_s,
            "longest_balance_s": self._longest_balance if self.balance_samples else None,
            "longest_in_band_s": self._longest_in_band if self._in_band_samples else None,
            "gap_count": self.gap_count,
            "missing_frames": self.missing_frames, "reset_count": self.reset_count,
            "duplicate_count": self.duplicate_count,
        }


def _clean_json(value):
    if isinstance(value, dict):
        return {str(k): _clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean_json(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


class SessionRecorder:
    """Record sessions with a one-second flush period, including idle sessions."""

    FLUSH_INTERVAL_S = 1.0

    def __init__(self, directory, metadata):
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        stem = datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
        candidate, number = root / stem, 1
        while True:
            try:
                candidate.mkdir()
                break
            except FileExistsError:
                candidate = root / f"{stem}_{number}"
                number += 1
        self.directory = self.path = candidate
        self.metadata = _clean_json(dict(metadata))
        self.metadata.setdefault("created_at", datetime.now().isoformat(timespec="seconds"))
        self.metadata.setdefault("format_version", 1)
        (candidate / "metadata.json").write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        self._csv_path = candidate / "samples.csv"
        self._fields = BASE_FIELDS + EXTENDED_FIELDS
        self._csv = self._csv_path.open("w", encoding="utf-8-sig", newline="")
        self._writer = csv.DictWriter(self._csv, fieldnames=self._fields)
        self._writer.writeheader()
        self._csv.flush()
        self._raw = (candidate / "raw_frames.bin").open("wb")
        self._events = (candidate / "events.jsonl").open("w", encoding="utf-8")
        self.metrics = Metrics()
        self.extra_summary = {}
        self._closed = False
        self._summary_written = False
        self._close_error = None
        self._raw_count = 0
        self._lock = threading.RLock()
        self._flush_stop = threading.Event()
        self._flush_error = None
        self._flush_thread = threading.Thread(target=self._flush_worker,
                                              name="telemetry-session-flush", daemon=True)
        self._flush_thread.start()

    def _flush_worker(self):
        while not self._flush_stop.wait(self.FLUSH_INTERVAL_S):
            with self._lock:
                if self._closed:
                    return
                try:
                    self._flush_files()
                except OSError as error:
                    # Surface disk failures to the main recorder call; never silently
                    # report a successful recording after a background flush failed.
                    self._flush_error = error
                    return

    def _flush_files(self):
        for stream in (self._csv, self._raw, self._events):
            stream.flush()

    def _check_open(self):
        if self._closed:
            raise ValueError("Session is already closed")
        if self._flush_error is not None:
            raise OSError("Session background flush failed") from self._flush_error

    def _extend_fields(self, new_fields):
        """Stream old CSV rows to a sibling file, then replace only when complete."""
        self._csv.close()
        temporary = None
        fields = self._fields + new_fields
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8-sig", newline="",
                                             dir=self.directory, prefix=".samples_",
                                             suffix=".tmp", delete=False) as output:
                temporary = Path(output.name)
                writer = csv.DictWriter(output, fieldnames=fields)
                writer.writeheader()
                with self._csv_path.open(encoding="utf-8-sig", newline="") as previous:
                    for row in csv.DictReader(previous):
                        writer.writerow(row)
            temporary.replace(self._csv_path)
            self._fields = fields
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
            self._csv = self._csv_path.open("a", encoding="utf-8", newline="")
            self._writer = csv.DictWriter(self._csv, fieldnames=self._fields)

    def write(self, record):
        with self._lock:
            self._check_open()
            record = _clean_json(dict(record))
            new_fields = [key for key in record if key not in self._fields]
            if new_fields:
                self._extend_fields(new_fields)
            self._writer.writerow(record)
            raw_hex = record.get("raw_hex")
            if isinstance(raw_hex, str):
                try:
                    frame = bytes.fromhex(raw_hex)
                    _decode_frame(frame)
                except (ValueError, TypeError):
                    pass
                else:
                    self._raw.write(frame)
                    self._raw_count += 1
            self.metrics.add(record)

    def event(self, kind, detail, record=None):
        with self._lock:
            self._check_open()
            event = {"time": time.time(), "kind": str(kind), "detail": detail}
            if record:
                for key in ("sequence", "elapsed_s", "state", "fault", "source"):
                    if key in record:
                        event[key] = record[key]
            self._events.write(json.dumps(_clean_json(event), ensure_ascii=False, allow_nan=False) + "\n")

    def close(self):
        summary = self.directory / "summary.json"
        with self._lock:
            if not self._closed:
                self._flush_stop.set()
                for stream in (self._csv, self._raw, self._events):
                    try:
                        stream.close()
                    except OSError as error:
                        self._close_error = self._close_error or error
                self._closed = True
            if not self._summary_written:
                if self._close_error is not None:
                    raise OSError("Session close failed") from self._close_error
                if self._flush_error is not None:
                    raise OSError("Session background flush failed") from self._flush_error
                result = self.metrics.snapshot()
                result.update(self.extra_summary)
                result.update(closed_at=datetime.now().isoformat(timespec="seconds"),
                              raw_frames=self._raw_count, source=self.metadata.get("source"),
                              data_file="samples.csv")
                summary.write_text(json.dumps(_clean_json(result), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
                self._summary_written = True
        self._flush_thread.join(timeout=self.FLUSH_INTERVAL_S)
        return summary

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class Replay:
    @staticmethod
    def load(csv_path):
        """Load current sessions or telemetry_monitor CSV; normalize time to zero."""
        records = []
        previous_clock = previous_sequence = None
        elapsed = 0.0
        clock_key = None
        text_fields = {"source", "original_source", "raw_hex"}
        bool_fields = {"sequence_reset", "sequence_duplicate", "time_reset"}
        with Path(csv_path).open(encoding="utf-8-sig", newline="") as source:
            reader = csv.DictReader(source)
            required = {"sequence", "theta_deg", "state"}
            if not required.issubset(reader.fieldnames or ()):
                raise ValueError("CSV 缺少遥测必要列：sequence、theta_deg、state")
            for line_number, row in enumerate(reader, start=2):
                record = {}
                for key, raw in row.items():
                    if key is None or raw is None or not raw.strip():
                        continue
                    if key in text_fields:
                        record[key] = raw
                    elif key in bool_fields and raw.lower() in ("true", "false"):
                        record[key] = raw.lower() == "true"
                    else:
                        try:
                            value = float(raw)
                        except ValueError:
                            continue
                        if not math.isfinite(value):
                            continue
                        record[key] = int(value) if value.is_integer() else value
                if not required.issubset(record) or not isinstance(record["sequence"], int) or not 0 <= record["sequence"] <= 65535:
                    continue
                if clock_key is None:
                    clock_key = next((key for key in ("elapsed_s", "host_monotonic", "time") if _finite(record.get(key))), None)
                    if clock_key is None:
                        raise ValueError(f"CSV 第 {line_number} 行缺少有限时间：需要 elapsed_s、host_monotonic 或 time")
                clock = record.get(clock_key)
                if not _finite(clock):
                    # Missing time must not become a zero-duration sample or bridge
                    # a continuous interval; keep all samples on one known clock.
                    raise ValueError(f"CSV 第 {line_number} 行的 {clock_key} 时间缺失或非有限，无法计算时长")
                if previous_clock is not None:
                    if clock < previous_clock:
                        record["time_reset"] = True
                    elapsed += max(0.0, clock - previous_clock)
                previous_clock = clock
                record["elapsed_s"] = elapsed
                gap, reset, duplicate = _sequence_info(previous_sequence, record["sequence"])
                previous_sequence = record["sequence"]
                recorded_gap = record.get("sequence_gap")
                record["sequence_gap"] = max(gap, max(0, int(recorded_gap)) if _finite(recorded_gap) else 0)
                record["sequence_reset"] = bool(record.get("sequence_reset")) or reset
                record["sequence_duplicate"] = bool(record.get("sequence_duplicate")) or duplicate
                record.setdefault("protocol_version", 1)
                if "source" in record:
                    record.setdefault("original_source", record["source"])
                record["source"] = "replay"
                records.append(record)
        return records


def demo_record(t, sequence):
    """Synthetic v1 telemetry for the visibly labelled demonstration mode."""
    t = max(0.0, float(t))
    cycle = t % 24
    state = 0 if cycle < 1 else (1 if cycle < 6 else 2)
    theta = math.pi * math.cos(cycle * 1.7) * math.exp(-max(0, cycle - 1) * 0.12) if state == 1 else (0.025 * math.sin(t * 3.5) if state == 2 else math.pi)
    omega = 2.0 * math.sin(t * 1.7) if state == 1 else (0.0875 * math.cos(t * 3.5) if state == 2 else 0.0)
    arm = 0.32 * math.sin(t * 0.8) if state else 0.0
    speed = 0.256 * math.cos(t * 0.8) if state else 0.0
    command = int(780 * math.sin(t * 1.7)) if state == 1 else (int(180 * math.sin(t * 3.5)) if state == 2 else 0)
    q = lambda value: max(-32768, min(32767, round(value * 1024)))
    payload = b"\xaa\x55\x01\x18" + struct.pack("<HHhhhhh", sequence & 0xffff,
                max(0, min(1023, round(512 + theta * 120))), q(theta), q(omega), q(arm), q(speed), command)
    payload += bytes((state, 1, 0, 0))
    frame = payload + crc16(payload).to_bytes(2, "little")
    record = _decode_frame(frame)
    record.update(time=time.time(), host_monotonic=time.monotonic(), elapsed_s=t,
                  source="demo", sequence_gap=0, sequence_reset=False, sequence_duplicate=False)
    return record
