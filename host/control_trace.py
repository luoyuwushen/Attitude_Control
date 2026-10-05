"""Offline version-3 control-history decoding and bounded export assembly.

This module deliberately does not import the live telemetry decoder. A trace
row is historical evidence, never a fresh measurement or a command response.
Only a validated metadata/data/end sequence can become a complete capture.
"""
from __future__ import annotations

import binascii
import copy
import hashlib
import math
import struct


TRACE_VERSION = 3
TRACE_SCHEMA = 1
MAX_TRACE_ROWS = 4096
MAX_TRACE_FRAME_BYTES = 240
TRACE_ROW_BYTES = 32
_HEADER = struct.Struct("<BBHHHBB")
_METADATA = struct.Struct("<IHHhIIIBB")
_ROW = struct.Struct("<IiHhhhhihhHBBBB")
_ROW_FIELDS = (
    "sample_counter", "encoder_count", "adc_mean_q4", "theta_q10",
    "omega_q10", "arm_q10", "arm_speed_q10", "integral_q24",
    "command_permille", "motor_command_permille", "control_age_ms",
    "h_flags", "adc_quality", "sensor_flags", "fault",
)
_METADATA_FIELDS = (
    "firmware_id", "adc_down_q4", "adc_up_q4", "capture_arm_q10",
    "period_cycles", "freeze_device_ms", "total_committed", "freeze_reason",
    "status",
)


class TraceDecodeError(ValueError):
    """A complete candidate packet failed the trace protocol contract."""


def _require(condition, message):
    if not condition:
        raise TraceDecodeError(message)


def decode_trace_frame(frame):
    """Decode one entire packet, raising ``ValueError`` on any invalid field.

    This accepts bytes, bytearray or memoryview, not a serial byte stream. The
    caller owns framing and must report discarded/truncated candidates to its
    collector. Reserved packet flags are zero in schema 1. Unknown firmware
    identities remain raw metadata; no control parameter profile is inferred.
    """
    _require(isinstance(frame, (bytes, bytearray, memoryview)), "frame_not_bytes")
    frame_length = frame.nbytes if isinstance(frame, memoryview) else len(frame)
    _require(16 <= frame_length <= MAX_TRACE_FRAME_BYTES, "frame_length_out_of_range")
    frame = bytes(frame)
    _require(frame[:2] == b"\xaa\x55", "invalid_magic")
    _require(frame[2] == TRACE_VERSION, "unsupported_trace_version")
    _require(frame[3] == len(frame), "declared_length_mismatch")
    _require(binascii.crc_hqx(frame[:-2], 0xffff) == int.from_bytes(frame[-2:], "little"),
             "packet_crc_mismatch")
    kind, schema, capture_id, index, total_rows, row_count, flags = _HEADER.unpack_from(frame, 4)
    _require(schema == TRACE_SCHEMA, "unsupported_trace_schema")
    _require(kind in (1, 2, 3), "unknown_trace_kind")
    _require(flags == 0, "reserved_packet_flags")
    _require(total_rows <= MAX_TRACE_ROWS, "total_rows_out_of_range")
    payload = frame[14:-2]
    result = dict(protocol_version=TRACE_VERSION, kind=kind, schema=schema,
                  capture_id=capture_id, index=index, total_rows=total_rows,
                  row_count=row_count, flags=flags, raw_hex=frame.hex(),
                  payload_hex=payload.hex())
    if kind == 1:
        _require(index == 0 and row_count == 0, "invalid_metadata_index_or_count")
        _require(len(payload) == _METADATA.size, "invalid_metadata_length")
        metadata = dict(zip(_METADATA_FIELDS, _METADATA.unpack(payload)))
        _require(metadata["adc_down_q4"] <= 0x3fff and metadata["adc_up_q4"] <= 0x3fff,
                 "metadata_adc_out_of_range")
        _require(metadata["period_cycles"] > 0, "invalid_period_cycles")
        _require(metadata["freeze_reason"] in (1, 2, 3), "unknown_freeze_reason")
        _require(metadata["status"] & ~1 == 0, "reserved_metadata_status")
        _require(metadata["total_committed"] >= total_rows, "committed_count_below_total_rows")
        result["metadata"] = metadata
    elif kind == 2:
        _require(1 <= row_count <= 7, "row_count_out_of_range")
        _require(len(payload) == row_count * TRACE_ROW_BYTES, "data_payload_length_mismatch")
        _require(index + row_count <= total_rows, "row_index_out_of_range")
        rows = []
        for offset in range(row_count):
            raw = payload[offset * TRACE_ROW_BYTES:(offset + 1) * TRACE_ROW_BYTES]
            row = dict(zip(_ROW_FIELDS, _ROW.unpack(raw)))
            _require(row["adc_mean_q4"] <= 0x3fff, "row_adc_out_of_range")
            _require(abs(row["command_permille"]) <= 1000 and
                     abs(row["motor_command_permille"]) <= 1000, "row_command_out_of_range")
            _require(row["control_age_ms"] <= 512, "row_control_age_out_of_range")
            _require(row["h_flags"] & ~0xf == 0, "reserved_h_flags")
            _require(row["sensor_flags"] & ~0x7f == 0, "reserved_sensor_flags")
            row.update(index=index + offset, raw_hex=raw.hex(),
                       integral_permille=row["integral_q24"] / (1 << 24),
                       theta_deg=row["theta_q10"] * 180 / (math.pi * 1024),
                       omega_rad_s=row["omega_q10"] / 1024,
                       arm_deg=row["arm_q10"] * 180 / (math.pi * 1024),
                       arm_speed_rad_s=row["arm_speed_q10"] / 1024,
                       sample_bad=bool(row["sensor_flags"] & 1),
                       sample_otr=bool(row["sensor_flags"] & 2),
                       measurement_valid=bool(row["sensor_flags"] & 4),
                       measurement_ready=bool(row["sensor_flags"] & 8),
                       live_otr=bool(row["sensor_flags"] & 16))
            rows.append(row)
        result["rows"] = rows
    else:
        _require(index == total_rows and row_count == 0, "invalid_end_index_or_count")
        _require(len(payload) == 2, "invalid_end_payload_length")
        result["record_crc16"] = int.from_bytes(payload, "little")
    return result


def _evidence(frame):
    """Bound evidence even when the caller passes an oversized invalid packet."""
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        return dict(raw_hex="", length=None, truncated=False, input_type=type(frame).__name__)
    if isinstance(frame, memoryview):
        # A candidate normally is at most 240 bytes; flatten a non-byte view
        # only after checking its byte size, to keep misuse bounded too.
        length = frame.nbytes
        if length > MAX_TRACE_FRAME_BYTES:
            return dict(raw_hex="", length=length, truncated=True,
                        evidence_error="oversized_memoryview")
    else:
        length = len(frame)
    raw = bytes(frame[:MAX_TRACE_FRAME_BYTES])
    result = dict(raw_hex=raw.hex(), length=length, truncated=length > MAX_TRACE_FRAME_BYTES)
    if length > MAX_TRACE_FRAME_BYTES:
        result["sha256"] = hashlib.sha256(frame).hexdigest()
    return result


class TraceCaptureCollector:
    """Collect one bounded export at a time; never retain an unbounded history.

    ``feed(packet)`` returns a list of terminal result dictionaries. Usually it
    returns an empty list; END or an error returns one result. A new metadata
    packet terminates any unfinished export as failed and starts a fresh one,
    even if its capture_id is unchanged. The caller must persist returned
    results. Failed exports can resume only through a new metadata packet.

    Call ``finish()`` on disconnect/end-of-file and ``fail(reason, evidence)``
    when a stream parser discards trace bytes. Neither operation claims that a
    capture missing its END packet is complete. ``snapshot()`` is a detached
    view of pending data. Results are historical and JSON serializable.
    ``complete`` describes export integrity, not sample continuity or sensor
    validity: those are separate counters and per-row sensor flags.
    """

    def __init__(self, max_rows=MAX_TRACE_ROWS):
        if isinstance(max_rows, bool) or not isinstance(max_rows, int) or not 1 <= max_rows <= MAX_TRACE_ROWS:
            raise ValueError("max_rows must be an integer from 1 to 4096")
        self.max_rows = max_rows
        self._capture = None
        self._crc = 0xffff

    @staticmethod
    def _empty(packet=None):
        packet = packet or {}
        return dict(protocol_version=TRACE_VERSION, schema=packet.get("schema"),
                    capture_id=packet.get("capture_id"), total_rows=packet.get("total_rows"),
                    metadata=None, rows=[], raw_frames_hex=[], frame_evidence=[],
                    complete=False, status="receiving", errors=[], received_rows=0,
                    record_crc16=None, computed_record_crc16=0xffff,
                    sample_counter_continuity=dict(
                        checked_intervals=0, contiguous=None, missing_sample_count=0,
                        duplicate_count=0, backward_or_reset_count=0,
                        wrap_count=0, gaps=[]))

    def _append_rows(self, rows):
        continuity = self._capture["sample_counter_continuity"]
        received = self._capture["rows"]
        for row in rows:
            if received:
                previous = received[-1]
                current_count, previous_count = row["sample_counter"], previous["sample_counter"]
                delta = (current_count - previous_count) & 0xffffffff
                continuity["checked_intervals"] += 1
                if 0 < delta < 0x80000000 and current_count < previous_count:
                    continuity["wrap_count"] += 1
                if delta != 1:
                    missing = None
                    if delta == 0:
                        kind = "duplicate"
                        continuity["duplicate_count"] += 1
                    elif delta < 0x80000000:
                        kind = "forward_gap"
                        missing = delta - 1
                        continuity["missing_sample_count"] += missing
                    else:
                        kind = "backward_or_reset"
                        continuity["backward_or_reset_count"] += 1
                    continuity["gaps"].append(dict(
                        previous_index=previous["index"], index=row["index"],
                        previous_counter=previous_count, counter=current_count,
                        delta=delta, kind=kind, missing_samples=missing))
                continuity["contiguous"] = not continuity["gaps"]
            received.append(row)

    def _append_evidence(self, frame):
        evidence = _evidence(frame)
        self._capture["frame_evidence"].append(evidence)
        self._capture["raw_frames_hex"].append(evidence["raw_hex"])

    def _terminal(self, error=None, complete=False):
        capture = self._capture
        if capture is None:
            return []
        if error:
            capture["errors"].append(str(error))
        capture["complete"] = bool(complete and not capture["errors"])
        capture["status"] = "complete" if capture["complete"] else "failed"
        capture["received_rows"] = len(capture["rows"])
        capture["computed_record_crc16"] = self._crc
        self._capture = None
        self._crc = 0xffff
        return [capture]

    def fail(self, reason, evidence=None):
        """Explicitly fail the current export, including a parser-level loss."""
        if self._capture is None:
            if evidence is None:
                return []
            self._capture = self._empty()
        if evidence is not None:
            self._append_evidence(evidence)
        return self._terminal(str(reason) or "capture_failed")

    def finish(self, reason="disconnected"):
        """End an unfinished export as failed; a successful END needs no finish."""
        return self.fail(reason or "missing_end")

    def snapshot(self):
        """Return an isolated pending result, or None after completion/failure."""
        return copy.deepcopy(self._capture)

    @property
    def active(self):
        return self._capture is not None

    def progress(self):
        """Small immutable-valued status; no copying historical rows for a GUI."""
        if self._capture is None:
            return None
        return {key: self._capture[key] for key in
                ("capture_id", "schema", "total_rows", "received_rows", "status")}

    def feed(self, frame):
        """Consume exactly one candidate packet; return zero or more results."""
        try:
            packet = decode_trace_frame(frame)
        except ValueError as error:
            if self._capture is None:
                self._capture = self._empty()
            return self.fail(str(error), evidence=frame)
        finished = []
        if packet["kind"] == 1:
            if self._capture is not None:
                finished.extend(self._terminal("superseded_by_metadata_before_end"))
            self._capture = self._empty(packet)
            self._capture["metadata"] = packet["metadata"]
            self._crc = 0xffff
            self._append_evidence(frame)
            if packet["total_rows"] > self.max_rows:
                finished.extend(self._terminal("collector_row_limit_exceeded"))
            return finished
        if self._capture is None:
            self._capture = self._empty(packet)
            self._append_evidence(frame)
            return self._terminal("missing_metadata")
        self._append_evidence(frame)
        for field in ("schema", "capture_id", "total_rows"):
            if packet[field] != self._capture[field]:
                return self._terminal(field + "_changed")
        expected_index = len(self._capture["rows"])
        if packet["index"] != expected_index:
            return self._terminal("noncontiguous_row_index")
        if packet["kind"] == 2:
            if expected_index + packet["row_count"] > self.max_rows:
                return self._terminal("collector_row_limit_exceeded")
            self._append_rows(packet["rows"])
            self._crc = binascii.crc_hqx(bytes.fromhex(packet["payload_hex"]), self._crc)
            self._capture["received_rows"] = len(self._capture["rows"])
            self._capture["computed_record_crc16"] = self._crc
            return []
        self._capture["record_crc16"] = packet["record_crc16"]
        if expected_index != self._capture["total_rows"]:
            return self._terminal("missing_rows")
        if packet["record_crc16"] != self._crc:
            return self._terminal("record_crc_mismatch")
        return self._terminal(complete=True)
