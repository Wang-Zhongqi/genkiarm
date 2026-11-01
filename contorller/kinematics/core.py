"""Product-of-exponentials FK, geometric Jacobian and bounded position IK."""
from dataclasses import dataclass
import json
from pathlib import Path
import numpy as np

DEFAULT_MODEL = Path(__file__).with_name('arm_model.json')


def vector(value, size, label):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{label} must contain {size} finite values')
    return result


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def rotation_vector(rotation):
    """SO(3) logarithm in world coordinates, including rotations near pi."""
    cosine = np.clip((np.trace(rotation) - 1) / 2, -1., 1.)
    angle = float(np.arccos(cosine))
    vee = np.array([rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]])
    if angle < 1e-7:
        return vee / 2
    if np.pi - angle < 1e-5:
        _, vectors = np.linalg.eigh((rotation + rotation.T) / 2)
        axis = vectors[:, -1]
        if axis @ vee < 0:
            axis = -axis
        return axis * angle
    return angle / (2 * np.sin(angle)) * vee


@dataclass
class IKResult:
    success: bool
    angles: np.ndarray
    error: float
    iterations: int = 0
    direction_error: float = 0.


class SerialArm:
    """Serial hinge/slide chain in one fixed base coordinate frame.

    Joint axes/anchors and home TCP transform refer to reference_angles.
    Position IK optionally constrains TCP +Z or a weighted orientation.
    No collision avoidance or dynamics is performed by this geometric solver.
    """

    def __init__(self, names, axes, anchors, home, limits, reference_angles=None,
                 joint_types=None):
        self.names = tuple(names)
        n = len(self.names)
        if not n or len(set(self.names)) != n:
            raise ValueError('joint names must be unique and nonempty')
        self.axes = np.asarray(axes, dtype=float).copy()
        self.anchors = np.asarray(anchors, dtype=float).copy()
        self.home = np.asarray(home, dtype=float).copy()
        self.limits = np.asarray(limits, dtype=float).copy()
        for array, shape in [(self.axes, (n, 3)), (self.anchors, (n, 3)),
                             (self.home, (4, 4)), (self.limits, (n, 2))]:
            if array.shape != shape or not np.isfinite(array).all():
                raise ValueError(f'invalid model array, expected {shape}')
        if np.any(self.limits[:, 0] >= self.limits[:, 1]):
            raise ValueError('joint lower limits must be less than upper limits')
        lengths = np.linalg.norm(self.axes, axis=1)
        if np.any(lengths < 1e-12):
            raise ValueError('joint axes cannot be zero')
        self.axes /= lengths[:, None]
        rotation = self.home[:3, :3]
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(rotation), 1)
                or not np.allclose(self.home[3], [0, 0, 0, 1])):
            raise ValueError('home must be a rigid transform')
        self.reference_angles = vector(np.zeros(n) if reference_angles is None
                                       else reference_angles, n, 'reference_angles').copy()
        self.joint_types = tuple(joint_types or ['hinge'] * n)
        if len(self.joint_types) != n or any(t not in ('hinge', 'slide') for t in self.joint_types):
            raise ValueError('joint types must be hinge or slide')

    @property
    def dof(self):
        return len(self.names)

    def _evaluate(self, q):
        q = vector(q, self.dof, 'joint angles')
        transform = np.eye(4)
        axes, anchors = [], []
        for axis, anchor, kind, delta in zip(self.axes, self.anchors, self.joint_types,
                                           q - self.reference_angles):
            axes.append(transform[:3, :3] @ axis)
            anchors.append(transform[:3, :3] @ anchor + transform[:3, 3])
            step = np.eye(4)
            if kind == 'hinge':
                k = skew(axis)
                rotation = np.eye(3) + np.sin(delta) * k + (1 - np.cos(delta)) * (k @ k)
                step[:3, :3] = rotation
                step[:3, 3] = anchor - rotation @ anchor
            else:
                step[:3, 3] = axis * delta
            transform = transform @ step
        return transform @ self.home, np.asarray(axes), np.asarray(anchors)

    def forward(self, q):
        """TCP homogeneous transform, in the model's base frame."""
        return self._evaluate(q)[0]

    def jacobian(self, q):
        """6 x N world/base geometric Jacobian, translation then rotation."""
        pose, axes, anchors = self._evaluate(q)
        jac = np.zeros((6, self.dof))
        for i, kind in enumerate(self.joint_types):
            if kind == 'hinge':
                jac[:3, i] = np.cross(axes[i], pose[:3, 3] - anchors[i])
                jac[3:, i] = axes[i]
            else:
                jac[:3, i] = axes[i]
        return jac

    def solve(self, target, seed=None, tolerance=.001, iterations=200,
              direction=None, direction_tolerance=.02, restarts=12,
              orientation=None, rotation_weights=None):
        """Return success=False and best bounded candidate if unreachable.

        direction is optional desired tool +Z in base coordinates. It adds
        two independent orientation constraints, suitable for a 5-DOF arm.
        orientation is an alternative target rotation matrix. rotation_weights
        select angular error components in that target's local frame; [0,0,1]
        controls axial roll while leaving other orientation components free.
        """
        target = vector(target, 3, 'target')
        seed = vector(self.reference_angles if seed is None else seed, self.dof, 'seed')
        if (not np.isfinite([tolerance, direction_tolerance]).all()
                or tolerance <= 0 or direction_tolerance <= 0
                or not isinstance(iterations, int) or iterations < 1
                or not isinstance(restarts, int) or restarts < 0):
            raise ValueError('invalid IK tolerances or iteration counts')
        if direction is not None:
            direction = vector(direction, 3, 'direction')
            length = np.linalg.norm(direction)
            if length < 1e-12:
                raise ValueError('direction cannot be zero')
            direction = direction / length
        if orientation is not None:
            orientation = np.asarray(orientation, dtype=float)
            if (direction is not None or orientation.shape != (3, 3)
                    or not np.isfinite(orientation).all()
                    or not np.allclose(orientation.T @ orientation, np.eye(3), atol=1e-7)
                    or not np.isclose(np.linalg.det(orientation), 1)):
                raise ValueError('orientation must be a rotation matrix, without direction')
            weights = vector(np.ones(3) if rotation_weights is None else rotation_weights, 3, 'rotation weights')
            if np.any(weights < 0) or not np.any(weights > 0):
                raise ValueError('rotation weights must be nonnegative and nonzero')
            weight = np.sqrt(weights)
        elif rotation_weights is not None:
            raise ValueError('rotation weights require orientation')
        lo, hi = self.limits.T
        initial = np.clip(seed, lo, hi)
        rng = np.random.default_rng(0)
        starts = [initial]
        if restarts:
            starts.extend([(lo + hi) / 2, *rng.uniform(lo, hi, (restarts, self.dof))])
        best, best_score, count = None, float('inf'), 0

        def residual(q):
            pose = self.forward(q)
            position_error = target - pose[:3, 3]
            angle = 0.
            error = position_error
            if direction is not None:
                angle = float(np.arccos(np.clip(direction @ pose[:3, 2], -1, 1)))
                error = np.r_[position_error, .05 * (direction - pose[:3, 2])]
            elif orientation is not None:
                angular_error = weight * (orientation.T @ rotation_vector(orientation @ pose[:3, :3].T))
                angle = float(np.linalg.norm(angular_error))
                error = np.r_[position_error, .05 * angular_error]
            return error, float(np.linalg.norm(position_error)), angle, pose

        for start in starts:
            q = np.clip(start, lo, hi)
            for _ in range(iterations):
                count += 1
                error, norm, angle, pose = residual(q)
                score = float(error @ error)
                success = norm <= tolerance and angle <= direction_tolerance
                if score < best_score or success:
                    best_score = score
                    best = IKResult(success, q.copy(), norm, count, angle)
                if success:
                    return best
                jac = self.jacobian(q)
                j = jac[:3]
                if direction is not None:
                    j = np.vstack([j, -.05 * skew(pose[:3, 2]) @ jac[3:]])
                elif orientation is not None:
                    j = np.vstack([j, .05 * weight[:, None] * (orientation.T @ jac[3:])])
                dq = j.T @ np.linalg.solve(j @ j.T + .003**2 * np.eye(len(error)), error)
                dq *= min(1., .15 / max(np.linalg.norm(dq), 1e-12))
                for scale in (1., .5, .25, .1):
                    candidate = np.clip(q + scale * dq, lo, hi)
                    trial = residual(candidate)[0]
                    if trial @ trial < score - 1e-18:
                        q = candidate
                        break
                else:
                    break
        return best

    def save(self, path):
        data = {'names': self.names, 'axes': self.axes.tolist(),
                'anchors': self.anchors.tolist(), 'home': self.home.tolist(),
                'limits': self.limits.tolist(), 'reference_angles': self.reference_angles.tolist(),
                'joint_types': self.joint_types}
        Path(path).write_text(json.dumps(data, indent=2), encoding='utf-8')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).read_text(encoding='utf-8')))
