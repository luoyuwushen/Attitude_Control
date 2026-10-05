"""Fresh RTL compile/UART run plus independent Python wire oracle, no hardware.

Usage: python tests/run_control_trace_export.py [--output PATH]
Artifacts are test evidence, never captures/fpga_logs. Production sources are
only read. The temporary simulator binary is removed after recording its hash.
"""
from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import shutil
import struct
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [ROOT / "tests/tb_control_trace_export.sv"] + [
    ROOT / "Attitude_Control/src" / name for name in
    ("control_trace_buffer.v", "control_trace_export.v", "uart_tx_byte.v")]
# case: retained rows, first pattern, total commits, capture id, reason,
#       wire packets, rows delivered, END count, actual RAM requests
CASES = {
    1: (0, 0, 0, 1, 3, 2, 0, 1, 0),
    2: (1, 100, 1, 2, 1, 3, 1, 1, 1),
    3: (15, 200, 15, 3, 2, 5, 15, 1, 15),
    4: (15, 200, 15, 3, 2, 5, 15, 1, 15),
    5: (4096, 10009, 4105, 4, 1, 588, 4096, 1, 4096),
    6: (20, 300, 20, 5, 1, 0, 0, 0, 0),
    7: (20, 300, 20, 5, 1, 1, 0, 0, 0),
    8: (20, 300, 20, 5, 1, 2, 7, 0, 7),
    9: (20, 300, 20, 5, 1, 5, 20, 1, 20),
    10: (1, 400, 1, 6, 1, 2, 1, 0, 1),
    11: (1, 400, 1, 6, 1, 3, 1, 1, 1),
    12: (1, 400, 1, 6, 1, 1, 0, 0, 1),
    13: (1, 400, 1, 6, 1, 1, 0, 0, 1),
    14: (20, 500, 20, 7, 1, 2, 7, 0, 7),
    15: (8, 600, 8, 9, 2, 4, 8, 1, 8),
    16: (8, 600, 8, 9, 2, 0, 0, 0, 0),
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def signed(value: int, width: int) -> int:
    value &= (1 << width) - 1
    return value - (1 << width) if value & (1 << (width - 1)) else value


def row(n: int) -> bytes:
    command = n % 2001 - 1000
    return struct.pack(
        "<IiHhhhhihhHBBBB", (0xFFFFF000 + n) & 0xFFFFFFFF,
        signed(-123456789 + n * 131071, 32), n % 16384,
        signed(n * 71 - 32768, 16), signed(32767 - n * 43, 16),
        signed(n * 97 - 12345, 16), signed(-32768 + n * 113, 16),
        signed(0x80000001 ^ (n * 0x10203), 32), command, -command,
        n % 513, n % 16, n % 2 * 32, 12 + n % 4, n % 4)


def verify(path: Path) -> dict:
    captures: dict[int, dict] = {}
    for line in path.read_text(encoding="ascii").splitlines():
        items = line.split()
        case = int(items[1])
        spec = CASES[case]
        count, first, total, capture, reason, packets, received, ended, reads = spec
        metadata = struct.pack("<IHHhIIIBB", 0x12345678, 3557, 12535, -316,
                               50000, 0xFEDC0000 + total, total, reason, int(total > 4096))
        if items[0] == "CASE":
            assert case not in captures
            assert list(map(int, items[2:6])) == [count, first, total, capture]
            assert bytes.fromhex(items[6])[::-1] == metadata
            captures[case] = dict(packets=[], rows=b"", end=0, result=None)
        elif items[0] == "PACKET":
            c = captures[case]
            frame = bytes.fromhex(items[2])
            assert frame[:4] == bytes([0xAA, 0x55, 3, len(frame)])
            assert len(frame) <= 240
            assert int.from_bytes(frame[-2:], "little") == binascii.crc_hqx(frame[:-2], 0xFFFF)
            kind, schema, ident, index, rows, n, flags = struct.unpack_from("<BBHHHBB", frame, 4)
            assert schema == 1 and ident == capture and rows == count and flags == 0
            payload = frame[14:-2]
            assert not c["end"]
            if kind == 1:
                assert not c["packets"] and index == n == 0 and payload == metadata
            elif kind == 2:
                assert c["packets"] and 1 <= n <= 7 and len(payload) == n * 32
                assert index == len(c["rows"]) // 32
                assert n == min(7, count - index)
                assert payload == b"".join(row(first + k) for k in range(index, index + n))
                c["rows"] += payload
            elif kind == 3:
                assert index == count and n == 0 and len(payload) == 2
                assert len(c["rows"]) == count * 32
                assert int.from_bytes(payload, "little") == binascii.crc_hqx(c["rows"], 0xFFFF)
                c["end"] += 1
            else:
                raise AssertionError(f"unknown kind {kind}")
            c["packets"].append(frame)
        elif items[0] == "RESULT":
            c = captures[case]
            expected = [packets, received, ended, sum(map(len, c["packets"])), reads]
            assert list(map(int, items[2:])) == expected
            assert len(c["packets"]) == packets and len(c["rows"]) == received * 32 and c["end"] == ended
            c["result"] = dict(zip(("packets", "rows", "ends", "bytes", "ram_requests"), expected))
        else:
            raise AssertionError(f"unknown log line {line}")
    assert set(captures) == set(CASES) and all(c["result"] for c in captures.values())
    # Same-generation retry is bit-for-bit fresh META through END, not append.
    assert captures[3]["packets"] == captures[4]["packets"]
    return {
        "cases": {k: v["result"] for k, v in captures.items()},
        "wire_packets": sum(len(c["packets"]) for c in captures.values()),
        "wire_bytes": sum(sum(map(len, c["packets"])) for c in captures.values()),
        "raw_row_bytes_compared": sum(len(c["rows"]) for c in captures.values()),
        "crc_errors": 0, "row_byte_mismatches": 0, "retry_exact": True,
        "scope": "Real RTL UART at four clocks/bit; no top arbitration or physical serial port",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "tmp/control_trace_export_validation")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    wire = output / "wire.txt"
    if args.verify_only:
        print(json.dumps(verify(wire), indent=2))
        return
    compiler = Path(shutil.which("iverilog") or ROOT / ".tools/iverilog/bin/iverilog.exe")
    simulator = compiler.with_name("vvp.exe" if compiler.suffix == ".exe" else "vvp")
    binary = output / "export.vvp"
    files = SOURCES + [Path(__file__).resolve()]
    before = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
    result = {"source_sha256_before": before, "passed": False}
    try:
        for name, command in (
            ("compile", [str(compiler), "-g2012", "-Wall", "-s", "tb_control_trace_export",
                         "-o", str(binary), *map(str, SOURCES)]),
            ("run", [str(simulator), str(binary), f"+WIRE_LOG={wire.as_posix()}"]),
        ):
            start = time.monotonic()
            completed = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                                       encoding="utf-8", errors="replace", timeout=240)
            console = completed.stdout + completed.stderr
            (output / f"{name}.log").write_text(console, encoding="utf-8")
            result[name] = {"command": command, "exit_code": completed.returncode,
                            "wall_s": time.monotonic() - start, "log": str(output / f"{name}.log")}
            print(console, end="", flush=True)
            completed.check_returncode()
            if name == "run":
                assert "PASS control trace export:" in console
        result["oracle"] = verify(wire)
        result["source_sha256_after"] = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
        assert result["source_sha256_after"] == before
        result["passed"] = True
    finally:
        if binary.exists():
            result["removed_binary_sha256"] = sha(binary)
            binary.unlink()
        result["files_sha256"] = {p.name: sha(p) for p in output.iterdir()
                                  if p.is_file() and p.name != "summary.json"}
        (output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
