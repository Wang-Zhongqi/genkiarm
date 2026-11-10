"""Teleoperate a MuJoCo arm using joint positions read from a physical SO-101 arm.

python sim/real_teleop_sim.py --port COM3
Joint order for --signs and --offsets: pan, lift, elbow, wrist flex, wrist roll, gripper.
Joint directions are defined by the model's rotation axes; default signs are all +1.
All six angles use encoder center 2048 and 4096 ticks per revolution, not range midpoints.
Offsets and joint positions, including the gripper, are in degrees.
The simulated arm follows via qpos/mj_forward; no dynamics stepping is performed.
"""

import argparse
import json
import math
from pathlib import Path
import time

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def load_calibration(path):
    calibration = json.loads(path.read_text(encoding="utf-8"))
    for index, motor in enumerate(JOINT_NAMES, start=1):
        entry = calibration[motor]
        if entry["id"] != index or entry["range_min"] >= entry["range_max"]:
            raise ValueError(f"Invalid calibration for {motor}: {entry}")
        for field in ("drive_mode", "homing_offset"):
            if not isinstance(entry[field], int):
                raise ValueError(f"Invalid {field} for {motor}")
    return calibration


def read_leader_action(leader):
    positions = leader.bus.sync_read(
        "Present_Position", normalize=False, num_retry=leader.config.num_read_retries,
    )
    return {
        f"{name}.pos": (positions[name] - 2048) * 360 / 4096
        for name in JOINT_NAMES
    }


class SimArm:
    def __init__(self, model, signs, offsets):
        self.model = model
        self.data = mujoco.MjData(model)
        self.signs = np.asarray(signs, dtype=float)
        self.offsets = np.asarray(offsets, dtype=float)
        self.addresses = []
        self.limits = []
        for joint in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if joint_id < 0:
                raise ValueError(f"Model is missing joint {joint}")
            if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"Expected a hinge joint: {joint}")
            self.addresses.append(model.jnt_qposadr[joint_id])
            limits = [-np.inf, np.inf]
            if model.jnt_limited[joint_id]:
                limits = model.jnt_range[joint_id].copy()
            # The current arm defines its motion limits on actuators.
            for actuator in range(model.nu):
                if (model.actuator_trntype[actuator] == mujoco.mjtTrn.mjTRN_JOINT
                        and model.actuator_trnid[actuator, 0] == joint_id
                        and model.actuator_ctrllimited[actuator]):
                    low, high = model.actuator_ctrlrange[actuator]
                    limits = [max(limits[0], low), min(limits[1], high)]
            self.limits.append(limits)
        self.limits = np.asarray(self.limits)

    def update(self, action):
        angles = np.array([float(action[f"{motor}.pos"]) for motor in JOINT_NAMES])
        if not np.all(np.isfinite(angles)):
            raise ValueError("Leader returned a non-finite joint position")
        targets = np.deg2rad(angles * self.signs + self.offsets)
        targets = np.clip(targets, self.limits[:, 0], self.limits[:, 1])
        self.data.qpos[self.addresses] = targets
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM4", help="Leader serial port (default: COM4)")
    parser.add_argument("--calibration", type=Path,
                        default=ROOT / "real/record/genkiarm_leader.json")
    parser.add_argument("--model", type=Path, default=ROOT / "sim/arm/arm.xml")
    parser.add_argument("--fps", type=float, default=30)
    parser.add_argument("--signs", type=int, nargs=6, choices=(-1, 1), default=[1] * 6)
    parser.add_argument("--offsets", type=float, nargs=6, default=[0] * 6,
                        help="Joint zero offsets in degrees, after applying signs")
    parser.add_argument("--check", action="store_true",
                        help="Validate model and calibration without opening a serial port or viewer")
    return parser


def run(args):
    if not math.isfinite(args.fps) or args.fps <= 0:
        raise ValueError("--fps must be finite and positive")
    if not all(math.isfinite(value) for value in args.offsets):
        raise ValueError("--offsets must be finite")
    calibration_path = args.calibration.resolve()
    load_calibration(calibration_path)
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    sim_arm = SimArm(model, args.signs, args.offsets)
    if args.check:
        print(f"Model OK: {model.nq} coordinates; calibration OK: {calibration_path}")
        return

    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    from mujoco import viewer as mj_viewer

    leader = SO101Leader(SO101LeaderConfig(
        port=args.port,
        id=calibration_path.stem,
        calibration_dir=calibration_path.parent,
        use_degrees=True,
    ))
    try:
        leader.connect(calibrate=False)
        if not leader.is_calibrated:
            raise ValueError(
                "Motor calibration differs from the supplied JSON. "
                "Restore the matching leader calibration with LeRobot before teleoperating."
            )
        sim_arm.update(read_leader_action(leader))
        print(f"Reading {args.port} using {calibration_path}")
        print("Real-to-sim teleoperation; leader torque is disabled. Close the viewer or press Ctrl+C to stop.")
        with mj_viewer.launch_passive(model, sim_arm.data) as viewer:
            viewer.cam.lookat[:] = [0, 0, 0.25]
            viewer.cam.distance = 1.0
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            while viewer.is_running():
                started = time.perf_counter()
                action = read_leader_action(leader)
                with viewer.lock():
                    sim_arm.update(action)
                viewer.sync()
                time.sleep(max(0, 1 / args.fps - (time.perf_counter() - started)))
    finally:
        if leader.is_connected:
            leader.disconnect()


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\nTeleoperation stopped.")
    except (OSError, ValueError, KeyError, ImportError, RuntimeError) as exc:
        details = [f"Teleoperation failed: {exc}"]
        cause = exc.__cause__
        seen = {id(exc)}
        while cause is not None and id(cause) not in seen:
            seen.add(id(cause))
            details.append(f"Caused by {type(cause).__name__}: {cause}")
            cause = cause.__cause__
        if isinstance(exc, ConnectionError):
            details.append(
                "Close the servo utility/serial monitor before connecting; "
                "Windows serial ports allow only one application at a time."
            )
        parser.exit(1, "\n".join(details) + "\n")


if __name__ == "__main__":
    main()
