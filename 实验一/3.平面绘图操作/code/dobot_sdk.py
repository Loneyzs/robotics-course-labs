"""随附 Dobot C SDK 的小型 ctypes 封装，直接访问 COM 口。

结构体与原型按 sdk_x64/DobotType.h、DobotDll.h 定义。
不自动回零，不改变工具偏移；连接和 status 查询不发送运动指令。
"""

import ctypes as C
import os
from pathlib import Path
import struct
import time
import zipfile

from geometry import Point


class Packed(C.Structure):
    _pack_ = 1


class Pose(Packed):
    _fields_ = [(name, C.c_float) for name in ("x", "y", "z", "r", "j1", "j2", "j3", "j4")]


class Offset(Packed):
    _fields_ = [(name, C.c_float) for name in ("x", "y", "z")]


class PTP(Packed):
    _fields_ = [("mode", C.c_uint8)] + [(name, C.c_float) for name in ("x", "y", "z", "r")]


class XYZR(Packed):
    _fields_ = [(name, C.c_float) for name in ("x", "y", "z", "r")]


class ARC(Packed):
    _fields_ = [("middle", XYZR), ("end", XYZR)]


class MotionParams(Packed):
    _fields_ = [(name, C.c_float) for name in ("xyz_velocity", "r_velocity", "xyz_acceleration", "r_acceleration")]


class Ratio(Packed):
    _fields_ = [("velocity", C.c_float), ("acceleration", C.c_float)]


class Dobot:
    def __init__(self, port="COM7"):
        if os.name != "nt" or struct.calcsize("P") != 8:
            raise RuntimeError("此封装使用随附 Windows x64 SDK，请使用 64 位 Windows Python")
        root = Path(__file__).resolve().parent
        sdk = root / "sdk_x64"
        if not (sdk / "DobotDll.dll").exists():
            archive = root.parents[2] / "docs/DobotDemoV2.0/DobotDll/Precompiled/Windows/x64/DobotDll.zip"
            sdk.mkdir(exist_ok=True)
            with zipfile.ZipFile(archive) as z:
                for entry in z.infolist():
                    # 随附压缩包是平铺文件，仅提取 DLL 和原始头文件。
                    if Path(entry.filename).name == entry.filename and entry.filename.endswith((".dll", ".h")):
                        z.extract(entry, sdk)
        self._directory = os.add_dll_directory(str(sdk))
        self.api = C.CDLL(str(sdk / "DobotDll.dll"))
        self.connected = False
        self.motion_started = False
        self.saved_params = None
        self.port = port
        self._bind()

    def _bind(self):
        q = C.POINTER(C.c_uint64)
        signatures = {
            "ConnectDobot": [C.c_char_p, C.c_uint32, C.c_char_p, C.c_char_p],
            "DisconnectDobot": [], "SetCmdTimeout": [C.c_uint32],
            "GetPose": [C.POINTER(Pose)],
            "GetEndEffectorParams": [C.POINTER(Offset)],
            "GetAlarmsState": [C.POINTER(C.c_uint8), C.POINTER(C.c_uint32), C.c_uint32],
            "GetQueuedCmdCurrentIndex": [q],
            "SetQueuedCmdStartExec": [], "SetQueuedCmdStopExec": [],
            "SetQueuedCmdForceStopExec": [], "SetQueuedCmdClear": [],
            "SetPTPCmd": [C.POINTER(PTP), C.c_bool, q],
            "SetARCCmd": [C.POINTER(ARC), C.c_bool, q],
        }
        for suffix, cls in (("PTPCoordinateParams", MotionParams), ("ARCParams", MotionParams),
                            ("PTPCommonParams", Ratio), ("ARCCommonParams", Ratio)):
            signatures["Get" + suffix] = [C.POINTER(cls)]
            signatures["Set" + suffix] = [C.POINTER(cls), C.c_bool, q]
        for name, args in signatures.items():
            fn = getattr(self.api, name)
            fn.argtypes = args
            fn.restype = C.c_int

    def call(self, name, *args):
        code = getattr(self.api, name)(*args)
        if code != 0:
            raise RuntimeError(f"{name} 失败，SDK 返回码 {code}（1=缓冲区满，2=通信超时，3=参数无效）")

    def __enter__(self):
        fw, version = C.create_string_buffer(100), C.create_string_buffer(100)
        code = self.api.ConnectDobot(self.port.encode("ascii"), 115200, fw, version)
        if code:
            reason = {1: "未找到设备", 2: "端口被占用；请先在 DobotStudio 断开连接"}.get(code, "未知错误")
            raise RuntimeError(f"连接 {self.port} 失败：{reason}（{code}）")
        self.connected = True
        self.firmware = fw.value.decode(errors="replace")
        try:
            self.call("SetCmdTimeout", 1500)
        except BaseException:
            self.api.DisconnectDobot()
            self.connected = False
            raise
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.motion_started:
                if exc_type:
                    self.api.SetQueuedCmdForceStopExec()
                    self.api.SetQueuedCmdClear()
                else:
                    self.api.SetQueuedCmdStopExec()
                self.motion_started = False
            if self.saved_params:
                for name, value in self.saved_params.items():
                    index = C.c_uint64()
                    result = getattr(self.api, "Set" + name)(C.byref(value), False, C.byref(index))
                    if result:
                        print(f"提示：恢复 {name} 失败，返回码 {result}")
        finally:
            if self.connected:
                self.api.DisconnectDobot()
                self.connected = False

    def pose(self):
        pose = Pose()
        self.call("GetPose", C.byref(pose))
        return {name: getattr(pose, name) for name, _ in pose._fields_}

    def offset(self):
        offset = Offset()
        self.call("GetEndEffectorParams", C.byref(offset))
        return {name: getattr(offset, name) for name, _ in offset._fields_}

    def alarms(self):
        data, length = (C.c_uint8 * 32)(), C.c_uint32()
        self.call("GetAlarmsState", data, C.byref(length), 32)
        return list(data[:length.value])

    def ensure_no_alarms(self):
        alarms = self.alarms()
        if any(alarms):
            raise RuntimeError(f"机械臂存在报警 {alarms}，请先检查机械臂状态")

    def prepare_motion(self, speed=15.0, acceleration=30.0):
        self.ensure_no_alarms()
        # 先停队列，再清旧指令，避免重新启动遗留任务。
        self.call("SetQueuedCmdStopExec")
        self.call("SetQueuedCmdClear")
        self.saved_params = {}
        for name, cls in (("PTPCoordinateParams", MotionParams), ("ARCParams", MotionParams),
                          ("PTPCommonParams", Ratio), ("ARCCommonParams", Ratio)):
            value = cls()
            self.call("Get" + name, C.byref(value))
            self.saved_params[name] = value
        for name, value in (("PTPCoordinateParams", MotionParams(speed, 10, acceleration, 20)),
                            ("ARCParams", MotionParams(speed, 10, acceleration, 20)),
                            ("PTPCommonParams", Ratio(100, 100)),
                            ("ARCCommonParams", Ratio(100, 100))):
            index = C.c_uint64()
            self.call("Set" + name, C.byref(value), False, C.byref(index))
        self.motion_started = True
        self.call("SetQueuedCmdStartExec")

    def _wait(self, target, point: Point, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = C.c_uint64()
            self.call("GetQueuedCmdCurrentIndex", C.byref(current))
            self.ensure_no_alarms()
            if current.value >= target:
                # 索引表示指令执行完成；再次核对位置，便于发现固件拒绝运动。
                actual = self.pose()
                if any(abs(actual[n] - getattr(point, n)) > 0.8 for n in ("x", "y", "z")):
                    raise RuntimeError(f"运动终点偏差过大：实际 {actual}，目标 {point}")
                return
            time.sleep(0.04)
        raise TimeoutError(f"运动等待超时，目标队列索引 {target}")

    def move(self, point: Point):
        cmd = PTP(2, point.x, point.y, point.z, point.r)  # PTPMOVLXYZMode
        index = C.c_uint64()
        self.call("SetPTPCmd", C.byref(cmd), True, C.byref(index))
        self._wait(index.value, point)

    def arc(self, middle: Point, end: Point):
        cmd = ARC(XYZR(middle.x, middle.y, middle.z, middle.r),
                  XYZR(end.x, end.y, end.z, end.r))
        index = C.c_uint64()
        self.call("SetARCCmd", C.byref(cmd), True, C.byref(index))
        self._wait(index.value, end)
