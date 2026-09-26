"""Build with Gowin, then reject any setup/hold failure even if gw_sh exits zero."""
import argparse
from html import unescape
import hashlib
import json
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / 'Project/Attitude_Control'


def copy_result(source, destination):
    if destination.exists():
        # Gowin marks its output read-only; allow repeat builds to replace it.
        destination.chmod(destination.stat().st_mode | stat.S_IWRITE)
    shutil.copy2(source, destination)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gw-sh', default=shutil.which('gw_sh'))
    parser.add_argument('--verify-only', action='store_true', help='Audit existing PnR outputs without rebuilding')
    args = parser.parse_args()
    if not args.gw_sh and not args.verify_only:
        parser.error('Provide --gw-sh with your Gowin IDE/bin/gw_sh executable')
    subprocess.run([sys.executable, ROOT / 'tools/verify_project.py'], check=True)
    if not args.verify_only:
        result = subprocess.run([args.gw_sh, 'build.tcl'], cwd=PROJECT, capture_output=True, timeout=600)
        output = result.stdout.decode('utf-8', errors='replace') + result.stderr.decode('utf-8', errors='replace')
        print(output, end='')
        if result.returncode or re.search(r'ERROR\s+\(', output):
            raise SystemExit('Gowin build failed')
    pnr = PROJECT / 'impl/pnr'
    if args.verify_only and not pnr.is_dir():
        pnr = ROOT / 'Release/reports'
    html = (pnr / 'Attitude_Control_tr_content.html').read_text(encoding='utf-8')
    text = re.sub(r'\s+', ' ', unescape(re.sub('<[^>]+>', ' ', html)))
    summary = {}
    for kind in ['Setup', 'Hold']:
        match = re.search('Numbers of ' + kind + r' Violated Endpoints\s+(\d+)', text)
        if not match:
            raise SystemExit('Missing timing summary')
        summary[kind.lower() + '_violations'] = int(match.group(1))
    match = re.search(r'sys_clk 50\.000\(MHz\)\s+([\d.]+)\(MHz\)', text)
    summary['fmax_mhz'] = float(match.group(1)) if match else None
    if any(summary[k] for k in ['setup_violations', 'hold_violations']):
        raise SystemExit('Timing failed: ' + json.dumps(summary))
    bitstream = pnr / 'Attitude_Control.fs'
    if args.verify_only and not bitstream.is_file():
        bitstream = ROOT / 'Release/Attitude_Control.fs'
    assert bitstream.is_file(), 'No bitstream generated'
    report = (pnr / 'Attitude_Control.rpt.txt').read_text(encoding='utf-8')
    pin_html = (pnr / 'Attitude_Control.pin.html').read_text(encoding='utf-8')
    section = pin_html.split('name="Pinout_by_Port_Name"')[1].split('</table>')[0]
    actual_pins = {}
    for row in re.findall(r'<tr>(.*?)</tr>', section, re.S):
        cells = [unescape(re.sub('<[^>]+>', '', cell)).strip()
                 for cell in re.findall(r'<td[^>]*>(.*?)</td>', row, re.S)]
        if len(cells) > 8 and cells[0] not in {'Port Name', 'Name'} and cells[3] == 'Y':
            actual_pins[cells[0]] = {'pad': cells[2].split('/')[0], 'standard': cells[7]}
    cst = (PROJECT / 'src/attitude_control.cst').read_text(encoding='utf-8')
    expected_pins = dict(re.findall(r'IO_LOC\s+"([^"]+)"\s+(\w+)\s*;', cst))
    assert actual_pins.keys() == expected_pins.keys(), 'Not all placed pins have constraints'
    assert all(actual_pins[port]['pad'] == pad for port, pad in expected_pins.items()), 'Placed pad mismatch'
    for port, value in actual_pins.items():
        assert value['standard'] == ('LVCMOS15' if port.startswith('key_sw') else 'LVCMOS33')
    summary['placed_pins'] = actual_pins
    summary['device'] = 'GW2A-LV55PG484C8/I7'
    summary['tool'] = re.search(r'Tool Version\s+([^\n]+)', text).group(1).split(' Part Number')[0]
    summary['bitstream_sha256'] = hashlib.sha256(bitstream.read_bytes()).hexdigest()
    summary['dsp_usage'] = re.search(r'DSP\s*\|\s*([^|]+)', report).group(1).strip()
    summary['logic_usage'] = re.search(r'\n\s*Logic\s*\|\s*([^|]+)', report).group(1).strip()
    summary['register_usage'] = re.search(r'\n\s*Register\s*\|\s*([^|]+)', report).group(1).strip()
    summary['created_time'] = re.search(r'<Created Time>:\s*([^\n]+)', report).group(1).strip()
    summary['source_sha256'] = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in sorted((PROJECT / 'src').iterdir())}
    if args.verify_only:
        previous = json.loads((ROOT / 'docs/build_validation.json').read_text(encoding='utf-8'))
        assert previous['source_sha256'] == summary['source_sha256'], 'Stale PnR: source changed'
        assert previous['bitstream_sha256'] == summary['bitstream_sha256'], 'Bitstream changed'
    release = ROOT / 'Release'
    release.mkdir(exist_ok=True)
    if not args.verify_only:
        copy_result(bitstream, release / bitstream.name)
    archive = release / 'reports'
    archive.mkdir(exist_ok=True)
    if pnr != archive:
        for report_file in pnr.iterdir():
            if report_file.suffix in {'.html', '.txt'}:
                copy_result(report_file, archive / report_file.name)
    (ROOT / 'docs/build_validation.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print('PASS: timing closed, Release bitstream verified, evidence saved to docs/build_validation.json')


if __name__ == '__main__':
    main()
