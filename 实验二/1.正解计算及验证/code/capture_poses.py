"""用本地 Dobot SDK 采集 X>0 的四个象限。默认只预览采样计划。"""

import argparse
import ctypes as C
from datetime import datetime
import json
from math import asin, cos, degrees, radians
from pathlib import Path
import sys
import time

EXPERIMENT = Path(__file__).resolve().parents[2]
SDK_WRAPPER = EXPERIMENT.parent / "实验一" / "3.平面绘图操作" / "code"
sys.path.insert(0, str(SDK_WRAPPER))
from dobot_sdk import Dobot, Packed, PTP, Ratio


class JointRates(Packed):
    _fields_ = [("velocity", C.c_float * 4), ("acceleration", C.c_float * 4)]


class SamplingArm(Dobot):
    def prepare_sampling(self):
        self.ensure_no_alarms()
        self.call("SetQueuedCmdStopExec")
        self.call("SetQueuedCmdClear")
        api = self.api
        api.GetPTPJointParams.argtypes = [C.POINTER(JointRates)]
        api.GetPTPJointParams.restype = C.c_int
        api.SetPTPJointParams.argtypes = [
            C.POINTER(JointRates), C.c_bool, C.POINTER(C.c_uint64)
        ]
        api.SetPTPJointParams.restype = C.c_int
        self.saved_params = {}
        rates = JointRates((10, 10, 10, 10), (20, 20, 20, 20))
        sampling_params = {"PTPJointParams": rates, "PTPCommonParams": Ratio(100, 100)}
        for name, value in sampling_params.items():
            previous = type(value)()
            self.call("Get" + name, C.byref(previous))
            self.saved_params[name] = previous
        self.motion_started = True
        self.call("SetQueuedCmdStartExec")
        self.apply_parameters(sampling_params)

    def apply_parameters(self, parameters):
        # 实机在队列停止时会确认 Set 指令，但读回仍可能是旧值。
        for name, expected in parameters.items():
            index = C.c_uint64()
            self.call("Set" + name, C.byref(expected), True, C.byref(index))
            current = C.c_uint64()
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                self.ensure_no_alarms()
                self.call("GetQueuedCmdCurrentIndex", C.byref(current))
                if current.value >= index.value:
                    break
                time.sleep(0.05)
            else:
                raise TimeoutError(f"{name} 未在期限内执行")
            actual = type(expected)()
            self.call("Get" + name, C.byref(actual))
            if bytes(actual) != bytes(expected):
                raise RuntimeError(f"{name} 回读值与设定不符")

    def restore_sampling(self):
        self.apply_parameters(self.saved_params)
        self.saved_params = None

    def move_joints(self, joints):
        if abs(joints[0]) > 12:
            raise ValueError(f"J1 超出本次前方采样范围 {joints[0]}")
        self.ensure_no_alarms()
        index = C.c_uint64()
        command = PTP(4, *joints)  # PTPMOVJANGLEMode
        self.call("SetPTPCmd", C.byref(command), True, C.byref(index))
        current = C.c_uint64()
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            self.call("GetQueuedCmdCurrentIndex", C.byref(current))
            self.ensure_no_alarms()
            if current.value >= index.value:
                pose = self.pose()
                if all(abs(pose[f"j{i}"] - target) < 0.35 for i, target in enumerate(joints, 1)):
                    return pose
            time.sleep(0.08)
        raise TimeoutError(f"关节目标未到达 {joints}，最后位姿 {self.pose()}")


def octant(pose):
    xyz = [pose[axis] for axis in "xyz"]
    if any(abs(value) < 0.01 for value in xyz):
        raise ValueError(f"有坐标落在分界面附近 {xyz}")
    return "".join("+" if value > 0 else "-" for value in xyz)


def now():
    return datetime.now().astimezone().isoformat()


def forearm_for_height(shoulder, z):
    return degrees(asin((135 * cos(radians(shoulder)) - z) / 147))


def scan_plan():
    return [
        (key, (azimuth, shoulder, forearm_for_height(shoulder, 8), 0),
         (azimuth, shoulder, forearm_for_height(shoulder, -4), 0))
        for key, azimuth, shoulder in (("++", 12, 12), ("+-", -12, 18))
    ]


def capture(port):
    log_dir = Path(__file__).resolve().parent / "records"
    log_dir.mkdir(exist_ok=True)
    path = log_dir / (datetime.now().strftime("live_%Y%m%d_%H%M%S_%f") + ".json")
    dataset = {"source": "live SDK GetPose", "port": port, "started_at": now(), "samples": []}

    def save():
        temporary = path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(dataset, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
        )
        temporary.replace(path)

    with SamplingArm(port) as robot:
        start = robot.pose()
        offset = robot.offset()
        alarms = robot.alarms()
        dataset["start_pose"] = start
        dataset["configured_offset_mm"] = offset
        dataset["start_alarm_bytes"] = alarms
        dataset["sampling_joint1_max_abs_deg"] = 12
        save()
        print(f"实机记录 {path}", flush=True)
        plan = scan_plan()
        try:
            if any(alarms):
                raise ValueError(f"当前存在报警 {alarms}")
            if any(abs(offset[axis] - target) > 0.1 for axis, target in zip("xyz", (59.7, 0, 0))):
                raise ValueError(f"当前末端偏置与采样模型不同 {offset}")
            if not (abs(start["j1"]) <= 8 and 0 <= start["j2"] <= 28
                    and 30 <= start["j3"] <= 75 and abs(start["j4"]) <= 5 and start["z"] >= -12):
                raise ValueError(f"起始姿态超出本次规划范围 {start}")
            robot.prepare_sampling()
            entry_forearm = min(start["j3"], 56)
            if start["j3"] > entry_forearm:
                robot.move_joints((start["j1"], start["j2"], entry_forearm, start["j4"]))
            robot.move_joints((0, 12, entry_forearm, 0))
            for xy, upper, lower in plan:
                for expected, target in ((xy + "+", upper), (xy + "-", lower)):
                    pose = robot.move_joints(target)
                    actual = octant(pose)
                    if actual != expected:
                        raise ValueError(f"实测象限 {actual} 与目标 {expected} 不符 {pose}")
                    sample = {"octant": actual, "timestamp": now(), "target_joints_deg": target, "pose": pose}
                    dataset["samples"].append(sample)
                    save()
                    print(f"{actual}  {pose}", flush=True)
                robot.move_joints(upper)
            _, (_, shoulder, forearm, wrist), _ = plan[-1]
            dataset["end_pose"] = robot.move_joints((0, shoulder, forearm, wrist))
            robot.restore_sampling()
            dataset["settings_restored"] = True
            dataset["completed_at"] = now()
        except BaseException as error:
            dataset["interrupted_at"] = now()
            dataset["error"] = str(error)
            raise
        finally:
            save()
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.execute:
        capture(args.port)
    else:
        for xy, high, low in scan_plan():
            print(f"{xy}+ {high}\n{xy}- {low}")


if __name__ == "__main__":
    main()
