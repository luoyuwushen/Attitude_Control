"""Source and PyInstaller entry point for the measurement desk."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from host.app import main

if __name__ == '__main__':
    raise SystemExit(main())
