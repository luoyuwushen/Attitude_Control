"""Run all independent checks; a missing simulator is an error, never a pass."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile
import sys

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    print('+', ' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=ROOT, check=True, timeout=180, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--iverilog', help='Icarus bin directory, or iverilog executable')
    args = parser.parse_args()
    fallback = ROOT / '.tools/iverilog/bin/iverilog.exe'
    compiler = args.iverilog or shutil.which('iverilog') or (str(fallback) if fallback.exists() else None)
    if not compiler:
        parser.error('Icarus Verilog is required. Install it or provide --iverilog.')
    # Resolve against the caller's directory before subprocesses switch to ROOT.
    compiler = Path(compiler).resolve()
    if compiler.is_dir():
        compiler = compiler / ('iverilog.exe' if sys.platform == 'win32' else 'iverilog')
    simulator = compiler.with_name('vvp.exe' if sys.platform == 'win32' else 'vvp')
    if not simulator.is_file():
        simulator = Path(shutil.which('vvp') or simulator)
    run([sys.executable, 'tools/verify_project.py'])
    run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-p', 'test_*.py', '-v'])
    sources = sorted((ROOT / 'Attitude_Control/src').glob('*.v'))
    with tempfile.TemporaryDirectory(prefix='attitude_sim_') as directory:
        directory = Path(directory)
        run([compiler, '-g2005', '-Wall', '-s', 'top', '-o', directory / 'top.vvp', *sources])
        for bench in sorted((ROOT / 'tests').glob('tb_*.sv')):
            output = directory / (bench.stem + '.vvp')
            run([compiler, '-g2012', '-Wall', '-s', bench.stem, '-o', output, bench, *sources])
            run([simulator, output])
    print('PASS: full regression', flush=True)


if __name__ == '__main__':
    main()
