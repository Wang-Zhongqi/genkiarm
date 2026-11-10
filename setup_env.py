"""Create or sync the project environment from uv.lock using uv."""

from pathlib import Path
import os
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"


def main():
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv was not found. Install with 'winget install --id=astral-sh.uv -e' and reopen your terminal.")
    subprocess.run([uv, "sync", "--locked"], cwd=ROOT, check=True)
    python = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    print("Environment ready. Python:", python)
    print("Activate:", r".\.venv\Scripts\Activate.ps1" if os.name == "nt" else "source .venv/bin/activate")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"Environment setup failed: {exc}", file=sys.stderr)
        sys.exit(1)
