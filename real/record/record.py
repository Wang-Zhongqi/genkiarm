"""Start LeRobot recording using the adjacent config.json."""

from pathlib import Path
import json
import runpy
import shutil
import sys


def prepare_calibration(config):
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    settings = json.loads(config.read_text(encoding="utf-8"))
    arguments = []
    for section, group, name in (
        ("robot", "robots", "so_follower"),
        ("teleop", "teleoperators", "so_leader"),
    ):
        directory = config.parent
        filename = f"{settings[section]['id']}.json"
        saved = directory / filename
        cached = HF_LEROBOT_CALIBRATION / group / name / filename
        if not saved.exists() and cached.is_file():
            shutil.copy2(cached, saved)
        status = "saved calibration" if saved.is_file() else "calibration required on connection"
        print(f"{section}: {status}: {saved}", flush=True)
        arguments.append(f"--{section}.calibration_dir={directory}")
    return arguments


def main():
    config = Path(__file__).resolve().with_name("config.json")
    calibration = prepare_calibration(config)
    sys.argv = ["lerobot-record", f"--config_path={config}", *calibration, *sys.argv[1:]]
    runpy.run_module("lerobot.scripts.lerobot_record", run_name="__main__")


if __name__ == "__main__":
    main()
