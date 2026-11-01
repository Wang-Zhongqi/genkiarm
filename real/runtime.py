"""Run the physical arm, gamepad input and optional camera."""
from contextlib import ExitStack
import time
import numpy as np
from contorller.controller import Controller
from contorller.kinematics.core import DEFAULT_MODEL, SerialArm, vector
from .arm.servo_controller import ServoController
from .arm.camera import Camera


def run_real(args, *, reader=None):
    if args.camera_id is not None and args.camera_id < 0:
        raise ValueError('camera ID must be nonnegative')
    model = SerialArm.load(DEFAULT_MODEL)
    limits = np.vstack([model.limits, [0., np.pi / 2]])
    limits = np.column_stack([np.maximum(limits[:, 0], -np.pi / 2),
                              np.minimum(limits[:, 1], np.pi / 2)])
    model = SerialArm(model.names, model.axes, model.anchors, model.home,
                      limits[:5], model.reference_angles, model.joint_types)
    angles = np.zeros(6)  # Stored targets; there is no servo-position feedback.
    controller = Controller(model, angles, limits=limits, reader=reader)
    config = controller.config
    period = 1 / config['rate']
    with ExitStack() as stack:
        camera = None
        if args.camera_id is not None:
            camera = Camera(args.camera_id)
            stack.callback(camera.close)
            if not camera.poll():
                return 0
            print(f'Eye-in-hand camera {args.camera_id} connected.', flush=True)
        motors = ServoController(args.port)
        stack.callback(motors.close)
        for servo_id in range(1, 7):
            if not motors.enable_torque(servo_id):
                raise IOError(f'Cannot enable servo {servo_id}')
        print(f'Arm connected on {args.port}. Ctrl+C stops.', flush=True)
        previous = time.monotonic()
        last_print, previous_status, connected = 0., None, False
        while True:
            if camera is not None and not camera.poll():
                break
            tick = time.monotonic()
            dt = max(period if tick == previous else tick - previous, 1e-6)
            previous = tick
            command = controller.poll(dt)
            if command.connected and not connected:
                controller.reset(angles)
                command = controller.update({'axes': {}, 'buttons': [], 'pov': None}, dt)
            connected = command.connected
            if connected:
                targets = vector(command.angles, 6, 'motor angles')
                if np.any(targets < limits[:, 0]) or np.any(targets > limits[:, 1]):
                    raise ValueError('motor target exceeds travel')
                for index, angle in enumerate(targets):
                    if angle != angles[index]:
                        if not motors.set_servo_angle(index + 1, float(np.rad2deg(angle))):
                            raise IOError(f'Cannot write servo {index + 1}')
                        angles[index] = angle
            status = (command.mode, command.accepted, command.connected, command.message)
            if status != previous_status or tick - last_print >= .25:
                print(f'{command.mode}: deg={np.round(command.degrees, 2).tolist()} '
                      f'tcp_m={np.round(command.tcp_pose[:3, 3], 4).tolist()} {command.message}', flush=True)
                last_print, previous_status = tick, status
            time.sleep(max(0., period - (time.monotonic() - tick)))
    return 0
