"""GenkiArm entry point: option definitions, validation and mode dispatch.

python genkiarm.py real --port COM3 --camera-id 0
Starts continuous gamepad control of the physical arm. Simulation startup is TODO.
"""
import argparse

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['sim', 'real'], help='Simulation or physical arm')
    parser.add_argument('--port', help='Physical arm COM port; required in real mode')
    parser.add_argument('--camera-id', type=int, help='Eye-in-hand camera index; real mode only')
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.mode == 'real' and (args.port is None or not args.port.strip()):
        parser.error('real mode requires --port')
    if args.camera_id is not None and (args.mode != 'real' or args.camera_id < 0):
        parser.error('--camera-id requires real mode and a nonnegative index')
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        if args.mode == 'real':
            from real.runtime import run_real
            return run_real(args)
        # TODO: implement sim startup.
        return 0
    except KeyboardInterrupt:
        print('\nController stopped; no further motor targets sent.')
        return 130
    except (ValueError, IOError, ImportError, RuntimeError) as exc:
        build_parser().error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
