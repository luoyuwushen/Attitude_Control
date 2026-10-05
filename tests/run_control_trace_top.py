"""Independent top UART trace test runner and raw-wire consistency oracle.

Synthetic ADC/encoder inputs exercise production routing, not plant stability.
No serial port is opened. Default output is release validation evidence.
"""
from __future__ import annotations

import argparse
import binascii
import hashlib
import json
import re
import shutil
import struct
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "tests/tb_control_trace_top.sv"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(path: Path) -> dict:
    captures: dict[int, dict] = {}
    v2_count = v3_count = wire_bytes = compared = 0
    previous_seq = previous_ms = previous_sample = None
    for line in path.read_text(encoding="ascii").splitlines():
        parts = line.split()
        case = int(parts[1])
        if parts[0] == "CASE":
            assert case not in captures
            total, committed, capture = map(int, parts[2:5])
            metadata = bytes.fromhex(parts[5])[::-1]
            fw, down, up, arm, period, freeze_ms, count, reason, overwritten = struct.unpack("<IHHhIIIBB", metadata)
            assert fw in (0x20009, 0x30000) and down == 3840 and up == 12800 and period == 1280
            assert count == committed and total == min(committed, 256) and reason == 1
            assert overwritten == int(committed > 256)
            captures[case] = dict(total=total, committed=committed, id=capture, metadata=metadata,
                                  expected=[], rows=b"", packets=[], ends=0, result=None)
        elif parts[0] == "EXPECTED":
            c = captures[case]
            assert int(parts[2]) == len(c["expected"])
            row = bytes.fromhex(parts[3])[::-1]
            assert len(row) == 32
            c["expected"].append(row)
        elif parts[0] == "WIRE":
            frame = bytes.fromhex(parts[2])
            wire_bytes += len(frame)
            assert frame[:2] == b"\xaa\x55" and frame[3] == len(frame)
            assert int.from_bytes(frame[-2:], "little") == binascii.crc_hqx(frame[:-2], 0xFFFF)
            if frame[2] == 2:
                assert len(frame) in (215, 219) and frame[206:208] == b"\x21\x05"
                seq = int.from_bytes(frame[4:6], "little")
                ms = int.from_bytes(frame[24:28], "little")
                sample = int.from_bytes(frame[81:85], "little")
                assert int.from_bytes(frame[75:79], "little") in (0x20009, 0x30000)
                if previous_seq is not None:
                    assert seq == (previous_seq + 1) % 65536 and ms >= previous_ms and sample > previous_sample
                previous_seq, previous_ms, previous_sample = seq, ms, sample
                v2_count += 1
            else:
                assert frame[2] == 3 and case > 0
                c = captures[case]
                assert len(c["expected"]) == c["total"] and not c["ends"]
                kind, schema, ident, index, total, n, flags = struct.unpack_from("<BBHHHBB", frame, 4)
                assert schema == 1 and ident == c["id"] and total == c["total"] and flags == 0
                payload = frame[14:-2]
                if kind == 1:
                    assert not c["packets"] and index == n == 0 and payload == c["metadata"]
                elif kind == 2:
                    assert c["packets"] and index == len(c["rows"]) // 32
                    assert n == min(7, c["total"] - index) and 1 <= n <= 7
                    assert payload == b"".join(c["expected"][index:index + n])
                    c["rows"] += payload
                    compared += len(payload)
                elif kind == 3:
                    assert index == total and n == 0 and len(c["rows"]) == total * 32
                    assert payload == binascii.crc_hqx(c["rows"], 0xFFFF).to_bytes(2, "little")
                    c["ends"] += 1
                else:
                    raise AssertionError(f"kind {kind}")
                c["packets"].append(frame)
                v3_count += 1
        elif parts[0] == "RESULT":
            c = captures[case]
            packets, rows, ends, live = map(int, parts[2:])
            assert packets == len(c["packets"]) and rows == len(c["rows"]) // 32 and ends == c["ends"]
            assert ends == int(case in (1, 4))
            if ends:
                assert rows == c["total"]
            else:
                assert packets == 2 and rows == 7
            if case == 1:
                assert live >= 3 and c["total"] == 256 and c["committed"] >= 405
            c["result"] = dict(packets=packets, rows=rows, complete=bool(ends), live_frames=live)
        else:
            raise AssertionError(f"unrecognised wire log {line}")
    assert set(captures) == {1, 2, 3, 4} and all(c["result"] for c in captures.values())
    assert captures[1]["expected"] == captures[2]["expected"]
    assert captures[3]["expected"] == captures[4]["expected"]
    return dict(cases={k: v["result"] for k, v in captures.items()},
                wire_packets=v2_count + v3_count, current_v2_packets=v2_count,
                historical_v3_packets=v3_count, wire_bytes=wire_bytes,
                raw_committed_row_bytes_compared=compared, crc_errors=0,
                sequence_errors=0, row_mismatches=0,
                scope="Production top driven via simulated command UART/ADC/encoder pins; shortened acquisition period, no physical hardware")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "Release/project-v0.3.0/validation/trace_top_rtl")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    wire = output / "wire.txt"
    if args.verify_only:
        print(json.dumps(verify(wire), indent=2))
        return
    if (output / "summary.json").exists():
        raise RuntimeError("Existing evidence must be kept; choose a new --output directory")
    compiler = Path(shutil.which("iverilog") or ROOT / ".tools/iverilog/bin/iverilog.exe")
    simulator = compiler.with_name("vvp.exe" if compiler.suffix == ".exe" else "vvp")
    sources = sorted((ROOT / "Attitude_Control/src").glob("*.v"))
    files = [BENCH, Path(__file__).resolve(), *sources]
    before = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
    binary = output / "top.vvp"
    result = dict(source_sha256_before=before, passed=False)
    try:
        for name, command in (
            ("compile", [str(compiler), "-g2012", "-Wall", "-s", "tb_control_trace_top", "-o", str(binary), str(BENCH), *map(str, sources)]),
            ("run", [str(simulator), str(binary), f"+WIRE_LOG={wire.as_posix()}"]),
        ):
            begin = time.monotonic()
            p = subprocess.run(command, cwd=ROOT, capture_output=True, text=True,
                               encoding="utf-8", errors="replace", timeout=600)
            console = p.stdout + p.stderr
            (output / f"{name}.log").write_text(console, encoding="utf-8")
            result[name] = dict(command=command, exit_code=p.returncode,
                                wall_s=time.monotonic() - begin, log=str(output / f"{name}.log"))
            print(console, end="", flush=True)
            p.check_returncode()
            if name == "run":
                matched = re.search(r"PASS control trace top: checks=(\d+) v2=(\d+) v3=(\d+) stop_checks=(\d+) stop_max_clocks=(\d+) max_v2_gap_clocks=(\d+)", console)
                assert matched, "successful simulator exit without test completion"
                result["bench_metrics"] = dict(zip(("checks", "v2", "v3", "stop_checks", "stop_max_clocks", "max_v2_gap_clocks"), map(int, matched.groups())))
        result["oracle"] = verify(wire)
        result["source_sha256_after"] = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
        assert result["source_sha256_after"] == before
        result["passed"] = True
    finally:
        if binary.exists():
            result["removed_binary_sha256"] = sha(binary)
            binary.unlink()
        result["files_sha256"] = {p.name: sha(p) for p in output.iterdir() if p.is_file() and p.name != "summary.json"}
        (output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
