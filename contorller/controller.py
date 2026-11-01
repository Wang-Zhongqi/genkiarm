"""Configurable gamepad -> six motor angles, shared by sim and real."""
from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
import time
import numpy as np
from .gamepad import GamepadReader
from .kinematics.core import skew, vector

DEFAULT_CONFIG = Path(__file__).with_name('gamepad_config.json')


def rotate(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    k = skew(axis)
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def merge(base, update):
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


@dataclass
class MotorCommand:
    angles: np.ndarray
    mode: str
    tcp_pose: np.ndarray
    target_pose: np.ndarray
    accepted: bool = True
    connected: bool = True
    message: str = ''

    @property
    def degrees(self):
        return np.rad2deg(self.angles)


class Controller:
    """Stateful input processing, with injectable reader for deterministic tests.

    All angles are radians; input axes are in the current TCP-local frame.
    Local increments are transformed into the model frame for IK.
    A+stick changes tool direction at fixed TCP (free roll). A+shoulders
    changes tool-local roll at fixed TCP (other orientation components free).
    Combining them requests the full orientation; infeasible tasks are held.
    """
    def __init__(self, model, initial_angles=None, *, config=None, limits=None,
                 reader=None, device=None):
        if model.dof != 5:
            raise ValueError('this six-motor mapping expects five arm joints plus gripper')
        self.model = model
        self.config = json.loads(DEFAULT_CONFIG.read_text(encoding='utf-8'))
        if config is not None:
            update = config if isinstance(config, dict) else json.loads(Path(config).read_text(encoding='utf-8'))
            merge(self.config, deepcopy(update))
        c = self.config
        for key in ('cartesian_speed', 'rotation_speed', 'joint_speed', 'gripper_speed',
                    'position_tolerance', 'rotation_tolerance', 'rate'):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if not 0 <= c['deadzone'] < 1 or c['ik_iterations'] < 1:
            raise ValueError('invalid deadzone or IK iteration count')
        for pair in c['joint_pairs'].values():
            if len(pair) != 2 or any(not isinstance(i, int) or i not in range(6) for i in pair):
                raise ValueError('joint pairs use zero-based motor indices 0..5')
        for button in c['buttons'].values():
            if not isinstance(button, int) or not 1 <= button <= 32:
                raise ValueError('button numbers use WinMM indices 1..32')
        for axis in [*c['cartesian_axes'].values(), *c['rotation_axes'].values()]:
            if np.linalg.norm(vector(axis, 3, 'Cartesian axis')) < 1e-12:
                raise ValueError('Cartesian axes cannot be zero')
        if c['triggers']['mode'] not in ('combined', 'separate'):
            raise ValueError('trigger mode must be combined or separate')
        self.limits = np.vstack([model.limits, [0, np.pi / 2]]) if limits is None else np.asarray(limits, dtype=float)
        if (self.limits.shape != (6, 2) or not np.isfinite(self.limits).all()
                or np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError('limits must contain six finite lower/upper pairs')
        self.angles = np.zeros(6)
        self.mode = 'idle'
        self.reset(np.zeros(6) if initial_angles is None else initial_angles)
        self.reader = reader
        self.requested_device = device
        self.device = None
        self.caps = None
        self.next_discovery = 0.

    def reset(self, angles):
        angles = vector(angles, 6, 'motor angles')
        if np.any(angles < self.limits[:, 0] - 1e-7) or np.any(angles > self.limits[:, 1] + 1e-7):
            raise ValueError('initial motor angles exceed limits')
        self.angles = angles.copy()
        self.goal = self.model.forward(angles[:5])
        self.mode = 'idle'
        self.rotation_task = None

    def poll(self, dt):
        """Read once. Disconnection returns a hold command, never stale input."""
        if self.reader is None:
            self.reader = GamepadReader()
        if self.device is None and time.monotonic() >= self.next_discovery:
            self.next_discovery = time.monotonic() + 1.
            devices = self.reader.devices()
            selected = self.requested_device if self.requested_device is not None else next(iter(devices), None)
            if selected in devices:
                self.device, self.caps = selected, devices[selected]
        state = None if self.device is None else self.reader.read(self.device, self.caps, deadzone=0.)
        if state is None:
            self.device, self.caps = None, None
        return self.update(state, dt)

    def _axis(self, axes, name):
        value = float(axes.get(name, 0.))
        if not np.isfinite(value):
            raise ValueError('gamepad axis is not finite')
        value = float(np.clip(value, -1, 1))
        deadzone = self.config['deadzone']
        return np.sign(value) * max(0., abs(value) - deadzone) / (1 - deadzone)

    def _trigger(self, axes):
        t = self.config['triggers']
        if t['mode'] == 'combined':
            return t['open_sign'] * self._axis(axes, t['axis'])
        # Separate trigger axes normally rest at -1 and reach +1 when pressed.
        rest = t['rest']
        if rest not in (-1, 0):
            raise ValueError('separate trigger rest must be -1 or 0')
        def amount(name):
            value = float(axes.get(name, rest))
            if not np.isfinite(value):
                raise ValueError('trigger axis is not finite')
            value = np.clip((value - rest) / (1 - rest), 0, 1)
            return 0. if value <= self.config['deadzone'] else float(value)
        return amount(t['left_axis']) - amount(t['right_axis'])

    def update(self, state, dt):
        """Process a GamepadReader state; does not write motor commands."""
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be positive and finite')
        dt = float(dt)
        if state is None:
            self.mode = 'disconnected'
            self.goal = self.model.forward(self.angles[:5])
            return self._output(False, False, 'Gamepad disconnected; hold')
        c = self.config; axes = state.get('axes', {}); buttons = set(state.get('buttons', []))
        bindings = c['buttons']
        horizontal = self._axis(axes, c['axes']['horizontal'])
        vertical = c['axes']['vertical_sign'] * self._axis(axes, c['axes']['vertical'])
        forward = float(bindings['shoulder_right'] in buttons) - float(bindings['shoulder_left'] in buttons)
        pov = state.get('pov')
        if pov is not None and (not np.isfinite(pov) or not 0 <= pov < 360):
            raise ValueError('POV must be None or degrees in [0,360)')
        pair_key = 'y' if bindings['y'] in buttons else 'x' if bindings['x'] in buttons else 'default'
        mode = f'fk_{pair_key}' if pov is not None else 'ik_orbit' if bindings['a'] in buttons else 'ik_translate'
        rotation_task = ('mixed' if forward and (horizontal or vertical) else 'roll' if forward else 'direction') if mode == 'ik_orbit' else None
        if mode != self.mode or rotation_task != self.rotation_task:
            self.goal = self.model.forward(self.angles[:5])
        self.mode = mode
        self.rotation_task = rotation_task
        q = self.angles.copy()
        if not (pov is not None and 5 in c['joint_pairs'][pair_key]):
            q[5] = np.clip(q[5] + self._trigger(axes) * c['gripper_speed'] * dt, *self.limits[5])
        if pov is not None:
            radians = np.deg2rad(pov)
            delta = np.array([np.sin(radians), np.cos(radians)]) * c['joint_speed'] * dt
            pair = c['joint_pairs'][pair_key]
            q[pair] = np.clip(q[pair] + delta, self.limits[pair, 0], self.limits[pair, 1])
            self.angles = q
            self.goal = self.model.forward(q[:5])
            return self._output()
        moving = horizontal != 0 or vertical != 0 or forward != 0
        if not moving:
            self.angles = q
            return self._output()
        tool_rotation = self.model.forward(self.angles[:5])[:3, :3]
        # Retry smaller task increments when the requested pose cannot be solved.
        for scale in (1., .5, .25, .125):
            target = self.goal.copy()
            options = {}
            if mode == 'ik_translate':
                local_delta = sum(value * np.asarray(c['cartesian_axes'][key], dtype=float)
                                  for key, value in [('horizontal', horizontal), ('vertical', vertical), ('forward', forward)])
                target[:3, :3] = tool_rotation
                target[:3, 3] += tool_rotation @ local_delta * c['cartesian_speed'] * dt * scale
            else:
                amount = c['rotation_speed'] * dt * scale
                rotation = tool_rotation @ rotate(c['rotation_axes']['horizontal'], horizontal * amount)
                rotation = rotation @ rotate(c['rotation_axes']['vertical'], vertical * amount) @ rotate(c['rotation_axes']['roll'], forward * amount)
                target[:3, :3] = rotation
                if forward == 0:
                    options['direction'] = rotation[:, 2]
                else:
                    options['orientation'] = rotation
                    options['rotation_weights'] = [1, 1, 1] if horizontal or vertical else [0, 0, 1]
            result = self.model.solve(target[:3, 3], self.angles[:5],
                                      tolerance=c['position_tolerance'], iterations=c['ik_iterations'],
                                      direction_tolerance=c['rotation_tolerance'], restarts=0, **options)
            if (result.success and np.all(result.angles >= self.limits[:5, 0])
                    and np.all(result.angles <= self.limits[:5, 1])):
                q[:5] = result.angles
                self.angles = q
                self.goal = target
                return self._output(message='Reduced task increment' if scale < 1 else '')
        self.angles = q  # Independent trigger input can still open/release the gripper.
        return self._output(False, message='IK failed; arm target held')

    def _output(self, accepted=True, connected=True, message=''):
        return MotorCommand(self.angles.copy(), self.mode, self.model.forward(self.angles[:5]),
                            self.goal.copy(), accepted, connected, message)
