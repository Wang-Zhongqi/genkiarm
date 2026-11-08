"""Create a Python 3.12 virtual environment and install project dependencies."""

from pathlib import Path
import os
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parent
VENV = ROOT / ".venv"


def find_python312():
    candidates = [[sys.executable], ["py", "-3.12"], ["python3.12"]]
    for command in candidates:
        if not shutil.which(command[0]):
            continue
        result = subprocess.run(
            command + ["-c", "import sys; print(sys.version_info[:2] == (3, 12))"],
            capture_output=True, text=True,
        )
        if result.returncode == 0 and result.stdout.strip() == "True":
            return command
    raise RuntimeError("Python 3.12 was not found. Install Python 3.12 and run this script again.")


def main():
    python = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    if VENV.exists():
        if not python.is_file():
            raise RuntimeError(".venv exists but is incomplete. Rename it before retrying.")
        result = subprocess.check_output(
            [str(python), "-c", "import sys; print(sys.version_info[:2] == (3, 12))"],
            text=True,
        )
        if result.strip() != "True":
            raise RuntimeError("Existing .venv does not use Python 3.12. Rename it before retrying.")
    else:
        subprocess.run(find_python312() + ["-m", "venv", str(VENV)], check=True)

    subprocess.run([str(python), "-m", "pip", "install", "--upgrade", "pip"], check=True)
    subprocess.run(
        [str(python), "-m", "pip", "install", "-r", str(ROOT / "requirements.txt")],
        check=True,
    )
    subprocess.run([str(python), "-m", "pip", "check"], check=True)
    print("Environment ready. Python:", python)
    print("Activate:", r".\.venv\Scripts\Activate.ps1" if os.name == "nt" else "source .venv/bin/activate")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"Environment setup failed: {exc}", file=sys.stderr)
        sys.exit(1)
