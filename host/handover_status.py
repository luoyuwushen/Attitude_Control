"""Operator guidance based on live observations, never a command ACK."""
import math
from host.adc_status import in_blind_zone, measurement_ready


HANDOVER_STEPS = '扶稳直立 → 点击 H → 确认直立平衡 → 再松手'


def _uint(value, bits):
    return type(value) is int and 0 <= value < (1 << bits)


class HandoverStatus:
    FRESH_SECONDS = 1.0
    WAIT_SECONDS = 2.0

    def __init__(self):
        self.reset()

    def reset(self):
        self.requested_at = None
        self.written = False
        self.stopping = False
        self.stop_at = None
        self.was_running = False
        self.local_rejection = None
        self.record = None
        self.previous = None

    def queued(self, command, now):
        if command == 'H':
            self.requested_at = now
            self.written = False
            self.stopping = False
            self.was_running = False
            self.local_rejection = None
            self.record = None
        elif command == 'S':
            self.requested_at = None
            self.written = False
            self.stopping = True
            self.local_rejection = None
            # Require a post-stop observation before showing stopped.
            self.record = None
            self.stop_at = now
        elif command in 'DUGRFBJKLM':
            self.requested_at = None
            self.written = False
            self.local_rejection = None
            self.stopping = False
            self.stop_at = None

    def sent(self, command):
        # A delayed H-written notification after S cannot revive the request.
        if command == 'H' and self.requested_at is not None and not self.stopping:
            self.written = True

    def rejected(self, reason):
        self.requested_at = None
        self.written = False
        self.local_rejection = reason

    def observe(self, record, now):
        """Accept only advancing, recent, ordinary serial telemetry."""
        received = record.get('host_monotonic')
        sequence, device_ms = record.get('sequence'), record.get('device_time_ms')
        if (record.get('source') != 'serial' or record.get('protocol_version') not in (1, 2)
                or not isinstance(received, (int, float)) or isinstance(received, bool)
                or not math.isfinite(received) or not 0 < received <= now
                or now - received >= self.FRESH_SECONDS
                or not _uint(sequence, 16) or not _uint(device_ms, 32)):
            self.record = None
            return
        marker = (received, sequence, device_ms)
        if record.get('sequence_reset'):
            # Rebase counters without reviving a cancelled H or dropping the
            # request-time boundary for already buffered observations.
            self.record = None
            self.previous = marker
            return
        if self.previous is not None:
            old_time, old_sequence, old_ms = self.previous
            if (received < old_time or not 0 < (sequence - old_sequence) % 65536 < 32768
                    or not 0 < (device_ms - old_ms) % 4294967296 < 2147483648):
                self.record = None
                return
        if record.get('sequence_duplicate'):
            self.record = None
            return
        self.previous = marker
        boundary = self.stop_at if self.stopping else self.requested_at
        if boundary is not None and received <= boundary:
            self.record = None
            return
        # Keep a small snapshot; no raw frames, histories or trace rows here.
        self.record = {key: record.get(key) for key in (
            'host_monotonic', 'state', 'fault', 'calibrated', 'h_control_flags',
            'start_result', 'stop_pressed', 'sensor_flags', 'firmware_version')}
        if self._running():
            self.was_running = True
            self.requested_at = None
            self.local_rejection = None

    def _running(self):
        r = self.record or {}
        flags = r.get('h_control_flags')
        sensor = r.get('sensor_flags')
        return (r.get('state') == 2 and r.get('fault') == 0 and r.get('calibrated') == 1
                and r.get('stop_pressed') == 0 and _uint(flags, 8)
                and _uint(sensor, 16) and measurement_ready(r)
                and not flags & 0xf0 and bool(flags & 1))

    def presentation(self, now, *, connected, live=True):
        """Return a stable status code, reader-facing text, and severity."""
        if not live:
            return 'offline', 'H 接管：演示／回放不能确认实物接管。', 'neutral'
        if not connected:
            return 'disconnected', 'H 接管：连接已断开，无法确认状态；运动中请按 SW3。', 'warning'
        r = self.record
        if r and not 0 <= now - r['host_monotonic'] < self.FRESH_SECONDS:
            return 'stale', '遥测超时，无法确认 H 状态；请继续扶稳，必要时按 S / SW3。', 'warning'
        if r and (r.get('state') == 3 or r.get('fault') not in (None, 0)):
            return 'fault', 'H 未运行：设备当前故障；请扶稳并检查故障提示。', 'error'
        if r and in_blind_zone(r):
            return 'blind_zone', 'H 未运行：摆杆位于水平附近盲区；扶离盲区并等待测量就绪后重新点击 H，无需按 R。', 'warning'
        if self.local_rejection:
            return 'not_sent', 'H 未发送：' + self.local_rejection + '；请继续扶稳。', 'warning'
        if self.stopping:
            if r and r.get('state') == 0:
                return 'stopped', '已观测到停止：设备当前待机，H 未运行。', 'neutral'
            return 'stopping', '停止 S 已请求；等待新鲜遥测确认停止，请继续扶稳。', 'warning'
        if self._running():
            return 'running', '已观测到 H 运行 · 直立平衡；请记录实际松手时刻。', 'success'
        if self.requested_at is not None:
            if now - self.requested_at >= self.WAIT_SECONDS:
                suffix = '当前仍待机' if r and r.get('state') == 0 else '尚无有效状态确认'
                return 'timeout', f'等待超时：未观测到接管，{suffix}；请继续扶稳并检查状态。', 'warning'
            if r and r.get('state') == 0:
                if r.get('start_result') in (2, 3, 4, 5, 6, 7):
                    return 'rejected', ('未观测到接管 · 当前仍待机；设备最近启动结果为拒收。'
                                        '请继续扶稳。'), 'warning'
                return 'waiting', '未观测到接管 · 当前仍待机；请继续扶稳。', 'warning'
            stage = '已发送' if self.written else '已排队'
            return ('written' if self.written else 'queued',
                    f'H {stage}，尚未观测到接管；请继续扶稳。', 'warning')
        if not r:
            return 'unverified', 'H 接管：等待新鲜、持续更新的实时状态；请继续扶稳。', 'warning'
        if r.get('state') == 0:
            prefix = 'H 已结束' if self.was_running else 'H 未运行'
            return 'idle', prefix + ' · 当前待机；点击 H 后请等待直立平衡提示。', 'neutral'
        return 'unverified', '未观测到 H 接管；请查看当前设备状态并继续扶稳。', 'warning'
