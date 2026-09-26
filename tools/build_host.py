"""Build a Windows standalone desktop program in Project/Release."""
from pathlib import Path
import argparse
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-smoke', action='store_true')
    parser.add_argument('--dist-dir', type=Path, default=ROOT / 'Release',
                        help='输出目录；可使用独立版本目录，避免覆盖正在运行的 EXE')
    args = parser.parse_args()
    if sys.platform != 'win32':
        parser.error('请在 Windows 上构建 Windows 上位机')
    build_dir = ROOT / 'build' / 'host'
    spec_dir = ROOT / 'build' / 'host_spec'
    dist_dir = args.dist_dir.resolve()
    try:
        subprocess.run([sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean',
                        '--onefile', '--windowed', '--name', 'J280_Monitor',
                        '--distpath', str(dist_dir), '--workpath', str(build_dir),
                        '--specpath', str(spec_dir), '--paths', str(ROOT),
                        '--exclude-module', 'scipy', '--exclude-module', 'matplotlib',
                        '--exclude-module', 'tkinter', '--exclude-module', 'PyQt5',
                        '--exclude-module', 'PyQt6', '--exclude-module', 'PySide2',
                        '--exclude-module', 'IPython', '--exclude-module', 'pytest',
                        '--exclude-module', 'numba', '--exclude-module', 'llvmlite',
                        '--exclude-module', 'pyqtgraph.opengl',
                        str(ROOT / 'tools' / 'host_monitor.py')], cwd=ROOT, check=True)
        executable = dist_dir / 'J280_Monitor.exe'
        if not args.skip_smoke:
            subprocess.run([str(executable), '--smoke-test'], cwd=ROOT, check=True, timeout=60)
        for name in ('README.md', 'PROTOCOL.md'):
            text = (ROOT / 'host' / name).read_text(encoding='utf-8')
            text = text.replace('(PROTOCOL.md)', '(HOST_PROTOCOL.md)').replace('(README.md)', '(HOST_README.md)')
            (dist_dir / ('HOST_README.md' if name == 'README.md' else 'HOST_PROTOCOL.md')).write_text(text, encoding='utf-8')
        print(f'PASS: {executable} ({executable.stat().st_size:,} bytes)')
    finally:
        # Both absolute targets are fixed descendants of this project's build directory.
        for directory in (build_dir, spec_dir):
            resolved = directory.resolve()
            if resolved.is_relative_to((ROOT / 'build').resolve()) and resolved.exists():
                shutil.rmtree(resolved)


if __name__ == '__main__':
    main()
