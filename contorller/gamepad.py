"""读取 Windows 已连接的蓝牙/USB 手柄，无需第三方依赖。

运行：python -m contorller.gamepad
枚举设备：python -m contorller.gamepad --list
指定设备：python -m contorller.gamepad --device 0
轴名称和按键编号取决于手柄驱动；本脚本不向机械臂发送指令。
接口文档：https://learn.microsoft.com/en-us/windows/win32/api/joystickapi/
"""

import argparse
import ctypes
from ctypes import wintypes
import sys
import time


class JOYINFOEX(ctypes.Structure):
    _fields_ = [(name, wintypes.DWORD) for name in (
        "dwSize", "dwFlags", "dwXpos", "dwYpos", "dwZpos", "dwRpos",
        "dwUpos", "dwVpos", "dwButtons", "dwButtonNumber", "dwPOV",
        "dwReserved1", "dwReserved2",
    )]


class JOYCAPSW(ctypes.Structure):
    _fields_ = [
        ("wMid", wintypes.WORD), ("wPid", wintypes.WORD),
        ("szPname", wintypes.WCHAR * 32),
        *[(name, wintypes.UINT) for name in (
            "wXmin", "wXmax", "wYmin", "wYmax", "wZmin", "wZmax",
            "wNumButtons", "wPeriodMin", "wPeriodMax", "wRmin", "wRmax",
            "wUmin", "wUmax", "wVmin", "wVmax", "wCaps", "wMaxAxes",
            "wNumAxes", "wMaxButtons",
        )],
        ("szRegKey", wintypes.WCHAR * 32),
        ("szOEMVxD", wintypes.WCHAR * 260),
    ]


class GamepadReader:
    """read() 返回当前状态字典；断开连接时返回 None。"""

    def __init__(self):
        if sys.platform != "win32":
            raise RuntimeError("该脚本使用 Windows 手柄接口，只支持 Windows。")
        self.api = ctypes.WinDLL("winmm")
        self.api.joyGetNumDevs.argtypes = []
        self.api.joyGetNumDevs.restype = wintypes.UINT
        self.api.joyGetPosEx.argtypes = [wintypes.UINT, ctypes.POINTER(JOYINFOEX)]
        self.api.joyGetPosEx.restype = wintypes.UINT
        self.api.joyGetDevCapsW.argtypes = [
            ctypes.c_size_t, ctypes.POINTER(JOYCAPSW), wintypes.UINT,
        ]
        self.api.joyGetDevCapsW.restype = wintypes.UINT

    def _position(self, device):
        info = JOYINFOEX()
        info.dwSize = ctypes.sizeof(info)
        info.dwFlags = 0xFF  # JOY_RETURNALL
        if self.api.joyGetPosEx(device, ctypes.byref(info)) != 0:
            return None
        return info

    def devices(self):
        result = {}
        for device in range(min(self.api.joyGetNumDevs(), 16)):
            if self._position(device) is None:
                continue
            caps = JOYCAPSW()
            if self.api.joyGetDevCapsW(device, ctypes.byref(caps), ctypes.sizeof(caps)) == 0:
                result[device] = caps
        return result

    def read(self, device, caps, deadzone=0.08):
        info = self._position(device)
        if info is None:
            return None
        axes = {}
        for axis, flag in (("X", 0), ("Y", 0), ("Z", 1), ("R", 2), ("U", 4), ("V", 8)):
            if flag and not caps.wCaps & flag:
                continue
            low, high = getattr(caps, f"w{axis}min"), getattr(caps, f"w{axis}max")
            raw = getattr(info, f"dw{axis}pos")
            value = 2 * (raw - low) / (high - low) - 1 if high > low else 0.0
            value = max(-1.0, min(1.0, value))
            axes[axis] = 0.0 if abs(value) <= deadzone else round(value, 2)
        return {
            "axes": axes,
            "buttons": [i + 1 for i in range(min(caps.wNumButtons, 32))
                        if info.dwButtons & (1 << i)],
            "pov": None if not caps.wCaps & 16 or info.dwPOV == 0xFFFF
                   else info.dwPOV / 100,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="列出当前手柄后退出")
    parser.add_argument("--device", type=int, choices=range(16), metavar="0-15",
                        help="指定设备编号；默认自动选择")
    parser.add_argument("--deadzone", type=float, default=0.08, help="轴中心死区，默认 0.08")
    parser.add_argument("--hz", type=int, default=50, help="读取频率，默认 50 Hz")
    args = parser.parse_args()
    if not 0 <= args.deadzone < 1 or not 1 <= args.hz <= 1000:
        parser.error("deadzone 必须在 [0, 1) 范围内，hz 必须在 1 到 1000 范围内")
    reader = GamepadReader()
    if args.list:
        devices = reader.devices()
        for device, caps in devices.items():
            print(f"[{device}] {caps.szPname}：{caps.wNumAxes} 轴，{caps.wNumButtons} 按键")
        if not devices:
            print("未找到手柄。请先连接蓝牙，并运行 joy.cpl 检查设备是否可用。")
        return

    print("等待手柄连接… Ctrl+C 退出。轴范围 -1 到 1；POV 为角度（0=上、90=右）。", flush=True)
    device = None
    previous = None
    while True:
        if device is None:
            devices = reader.devices()
            selected = args.device if args.device is not None else next(iter(devices), None)
            if selected not in devices:
                time.sleep(1)
                continue
            device, caps = selected, devices[selected]
            print(f"已连接 [{device}] {caps.szPname}；按键使用驱动编号，请逐个按下确认映射。", flush=True)
            previous = None
        state = reader.read(device, caps, args.deadzone)
        if state is None:
            print("手柄已断开，等待重新连接…", flush=True)
            device = None
            time.sleep(1)
            continue
        if state != previous:
            print(f"轴={state['axes']}  按下的按键={state['buttons']}  方向键={state['pov']}", flush=True)
            previous = state
        time.sleep(1 / args.hz)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已停止读取。")
    except (RuntimeError, OSError) as exc:
        print(f"无法读取手柄：{exc}", file=sys.stderr)
        sys.exit(1)
