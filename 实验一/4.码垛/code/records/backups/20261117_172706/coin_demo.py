"""实验四：单枚硬币取放、越障、旋转 180°、放币后抬起。

默认只生成计划和预览，不连接机械臂。
  python "4.码垛/code/coin_demo.py" plan
  python "4.码垛/code/coin_demo.py" run --execute
  python "4.码垛/code/coin_demo.py" stop
"""

import argparse
import ctypes as C
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
RESULTS = ROOT.parent / "results"
sys.path.insert(0, str(ROOT.parents[1] / "3.平面绘图操作" / "code"))
from geometry import Point
from dobot_sdk import Dobot, MotionParams, Packed, Ratio

STOP_FILE = ROOT / "STOP_REQUESTED"


class JointParams(Packed):
    _fields_ = [("velocity", C.c_float * 4), ("acceleration", C.c_float * 4)]


class Kinematics(Packed):
    _fields_ = [("velocity", C.c_float), ("acceleration", C.c_float)]


class StopRequested(RuntimeError):
    pass


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def finite_positive(value, label, maximum):
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError(f"{label}须在 0～{maximum} 之间")


def verify_applied_parameters(profile, applied):
    for key in ("xyz_velocity_mm_s", "xyz_acceleration_mm_s2", "r_velocity_deg_s",
                "r_acceleration_deg_s2", "velocity_ratio_percent", "acceleration_ratio_percent"):
        if abs(applied[key] - profile[key]) > 0.01:
            raise RuntimeError(f"运动参数回读不符：{key}={applied[key]}，要求{profile[key]}")
    for key in ("joint_velocity_deg_s", "joint_acceleration_deg_s2"):
        if any(abs(v - profile[key]) > 0.01 for v in applied[key]):
            raise RuntimeError(f"关节参数回读不符：{key}={applied[key]}")


def start_alignment(current, plan, config):
    """允许用户拖回 A 附近，吸币前先抬升并对齐准确的取币位置。"""
    a = plan["source"]
    if not all(math.isfinite(current[k]) for k in ("x", "y", "z", "r")):
        raise ValueError("当前坐标无效")
    deviation = {"xy_mm": math.hypot(current["x"] - a["x"], current["y"] - a["y"]),
                 "z_mm": current["z"] - a["z"], "r_deg": current["r"] - a["r"]}
    allowed = config["start_tolerance"]
    if (deviation["xy_mm"] > allowed["xy_mm"] or abs(deviation["z_mm"]) > allowed["z_mm"]
            or abs(deviation["r_deg"]) > allowed["r_deg"]):
        raise ValueError(f"请拖回A附近：允许XY距离{allowed['xy_mm']}mm、Z偏差±{allowed['z_mm']}mm、"
                         f"R偏差±{allowed['r_deg']}°；当前偏差{deviation}")
    if all(abs(current[k] - a[k]) <= 0.5 for k in ("x", "y", "z", "r")):
        return deviation, []
    high = max(current["z"], plan["travel_z_mm"])
    def move(label, x, y, z, r):
        return {"kind": "move", "label": label, "point": dict(x=x, y=y, z=z, r=r)}
    steps = [move("起点校正：先竖直抬升", current["x"], current["y"], high, current["r"]),
             move("起点校正：高位对准A和取币姿态", a["x"], a["y"], high, a["r"]),
             move("起点校正：竖直下降到A取币高度", a["x"], a["y"], a["z"], a["r"])]
    return deviation, steps


def choose_rotation(source_r, source_xy, target_xy, direction="auto", wrist_limits=(-150, 150)):
    """R=J1+J4；沿水平路线采样并检查未折返的 ±180° 腕部轨迹。"""
    if direction not in ("auto", "ccw", "cw"):
        raise ValueError("旋转方向须为 auto / ccw / cw")
    joints1 = [math.degrees(math.atan2(source_xy[1] + (target_xy[1] - source_xy[1]) * t / 100,
                                     source_xy[0] + (target_xy[0] - source_xy[0]) * t / 100))
               for t in range(101)]
    a1 = joints1[0]
    candidates = []
    for delta in (180.0, -180.0):
        target_r = source_r + delta
        wrist = [source_r - a1, target_r - a1] + [target_r - j1 for j1 in joints1]
        low, high = min(wrist), max(wrist)
        feasible = wrist_limits[0] <= low and high <= wrist_limits[1]
        candidates.append({"delta_deg": delta, "direction": "ccw" if delta > 0 else "cw",
                           "direction_cn": "逆时针" if delta > 0 else "顺时针",
                           "target_r_deg": target_r, "j4_range_deg": [low, high],
                           "feasible": feasible})
    allowed = [c for c in candidates if c["feasible"] and (direction == "auto" or c["direction"] == direction)]
    if not allowed:
        raise ValueError(f"所选旋转方向超出腕部范围：{candidates}")
    return allowed[0], candidates


def build_plan(source, destination, config):
    a, b = source["pose"], destination["pose"]
    if any(source.get("alarm_bytes", [])) or any(destination.get("alarm_bytes", [])):
        raise ValueError("采点记录存在报警，请重新采集正常位置")
    if config.get("coin_count") != 1:
        raise ValueError("当前脚本按单枚演示设计，coin_count 必须为 1")
    for name in ("xy_mm", "z_mm", "r_deg"):
        finite_positive(config["start_tolerance"][name], f"起点容差{name}", 50)
    for item in ("obstacle_height_mm", "clearance_mm", "release_offset_mm", "verification_lift_mm"):
        finite_positive(config[item], item, 100)
    if config["clearance_mm"] < 10:
        raise ValueError("越障余量至少保留 10 mm")
    for name in ("grip_wait_s", "release_wait_s", "verification_hold_s"):
        finite_positive(config[name], name, 10)
    profile = config["motion_profile"]
    for key in ("joint_velocity_deg_s", "joint_acceleration_deg_s2", "xyz_velocity_mm_s",
                "xyz_acceleration_mm_s2", "r_velocity_deg_s", "r_acceleration_deg_s2"):
        finite_positive(profile[key], key, 200)
    if profile["velocity_ratio_percent"] != 100 or profile["acceleration_ratio_percent"] != 100:
        raise ValueError("本实验按最大倍率运行，速度和加速度倍率均须为100%")
    if any(abs(source["tool_offset"][k] - destination["tool_offset"][k]) > 0.05 for k in ("x", "y", "z")):
        raise ValueError("A、B 采点时工具偏移不同")
    if not all(math.isfinite(p[k]) for p in (a, b) for k in ("x", "y", "z", "r", "j1", "j4")):
        raise ValueError("采点坐标无效")
    # 核对 SDK 记录的 J1 和 R/J4 关系，再使用 atan2 推断路径上的腕部需求。
    for p in (a, b):
        if abs(math.degrees(math.atan2(p["y"], p["x"])) - p["j1"]) > 1:
            raise ValueError("采点基座角与 XY 不一致，不能推断腕部角")
        if abs(p["r"] - p["j1"] - p["j4"]) > 1:
            raise ValueError("采点不满足 R=J1+J4，请核对工具和坐标系")
    rotation, candidates = choose_rotation(a["r"], (a["x"], a["y"]),
                                           (b["x"], b["y"]), config["rotation_direction"])
    high = max(a["z"], b["z"]) + config["obstacle_height_mm"] + config["clearance_mm"]
    release = b["z"] + config["release_offset_mm"]
    verify = release + config["verification_lift_mm"]
    reset_r = a["j4"] + b["j1"]
    # A 为硬币上表面，B 为空吸盘触桌面。B 上方释放，避免未知币厚引起下压。
    ra, rb = a["r"], rotation["target_r_deg"]

    def move(label, x, y, z, r):
        return {"kind": "move", "label": label, "point": dict(x=x, y=y, z=z, r=r)}

    steps = [
        {"kind": "suction", "label": "打开吸盘", "on": True},
        {"kind": "wait", "label": "等待吸附", "seconds": config["grip_wait_s"]},
        move("携币竖直抬升越障高度", a["x"], a["y"], high, ra),
        move("在高处旋转180度", a["x"], a["y"], high, rb),
        move("保持高位越过障碍到B上方", b["x"], b["y"], high, rb),
        move("下降至B释放高度", b["x"], b["y"], release, rb),
        {"kind": "suction", "label": "关闭吸盘释放硬币", "on": False},
        {"kind": "wait", "label": "等待泄压释放", "seconds": config["release_wait_s"]},
        move("释放后竖直抬起验证", b["x"], b["y"], verify, rb),
        {"kind": "wait", "label": "停留供观察和拍摄", "seconds": config["verification_hold_s"]},
        move("空吸盘反向恢复初始腕角", b["x"], b["y"], verify, reset_r),
    ]
    return {"coin_count": 1, "source": a, "destination": b,
            "start_tolerance": config["start_tolerance"],
            "start_alignment": "near A -> lift -> align above recorded A -> descend before suction",
            "distance_xy_mm": math.hypot(b["x"] - a["x"], b["y"] - a["y"]),
            "obstacle_height_mm": config["obstacle_height_mm"], "obstacle_height_source": "user estimate",
            "clearance_mm": config["clearance_mm"], "travel_z_mm": high,
            "release_z_mm": release, "verification_z_mm": verify,
            "rotation": rotation, "rotation_candidates": candidates,
            "empty_return_r_deg": reset_r, "empty_return_delta_r_deg": reset_r - rb,
            "motion_mode": "MOVL_XYZ", "tool_offset": source["tool_offset"], "steps": steps}


class CoinRobot(Dobot):
    def __init__(self, port, recorder):
        super().__init__(port)
        self.recorder = recorder
        self.phase = "连接"
        q = C.POINTER(C.c_uint64)
        bindings = {
            "GetPTPJointParams": [C.POINTER(JointParams)],
            "SetPTPJointParams": [C.POINTER(JointParams), C.c_bool, q],
            "GetKinematics": [C.POINTER(Kinematics)],
            "GetEndEffectorSuctionCup": [C.POINTER(C.c_bool), C.POINTER(C.c_bool)],
            "SetEndEffectorSuctionCup": [C.c_bool, C.c_bool, C.c_bool, q],
        }
        for name, signature in bindings.items():
            fn = getattr(self.api, name)
            fn.argtypes, fn.restype = signature, C.c_int

    def check_stop(self):
        if STOP_FILE.exists():
            self.api.SetQueuedCmdForceStopExec()
            self.api.SetQueuedCmdClear()
            raise StopRequested("收到停止请求；已停止并清队列，吸盘状态保持")

    def suction_state(self):
        enabled, on = C.c_bool(), C.c_bool()
        self.call("GetEndEffectorSuctionCup", C.byref(enabled), C.byref(on))
        return {"enabled": enabled.value, "on": on.value}

    def suction(self, on):
        self.check_stop()
        index = C.c_uint64()
        self.call("SetEndEffectorSuctionCup", True, on, False, C.byref(index))
        state = self.suction_state()
        if state["on"] != on:
            raise RuntimeError("吸盘开关状态与指令不一致")
        return state

    def parameters(self):
        j, p, r = JointParams(), MotionParams(), Ratio()
        for name, value in (("PTPJointParams", j), ("PTPCoordinateParams", p), ("PTPCommonParams", r)):
            self.call("Get" + name, C.byref(value))
        return {"joint_velocity_deg_s": list(j.velocity),
                "joint_acceleration_deg_s2": list(j.acceleration),
                "xyz_velocity_mm_s": p.xyz_velocity, "xyz_acceleration_mm_s2": p.xyz_acceleration,
                "r_velocity_deg_s": p.r_velocity, "r_acceleration_deg_s2": p.r_acceleration,
                "velocity_ratio_percent": r.velocity, "acceleration_ratio_percent": r.acceleration}

    def prepare_experiment(self, profile):
        self.ensure_no_alarms()
        self.check_stop()
        self.call("SetQueuedCmdStopExec")
        self.call("SetQueuedCmdClear")
        self.saved_params = {}
        for name, cls in (("PTPJointParams", JointParams), ("PTPCoordinateParams", MotionParams),
                          ("PTPCommonParams", Ratio)):
            old = cls()
            self.call("Get" + name, C.byref(old))
            self.saved_params[name] = old
        joint = JointParams()
        for i in range(4):
            joint.velocity[i] = profile["joint_velocity_deg_s"]
            joint.acceleration[i] = profile["joint_acceleration_deg_s2"]
        values = {"PTPJointParams": joint,
                  "PTPCoordinateParams": MotionParams(profile["xyz_velocity_mm_s"], profile["r_velocity_deg_s"],
                                                        profile["xyz_acceleration_mm_s2"], profile["r_acceleration_deg_s2"]),
                  "PTPCommonParams": Ratio(100, 100)}
        for name, value in values.items():
            index = C.c_uint64()
            self.call("Set" + name, C.byref(value), False, C.byref(index))
        self.motion_started = True
        self.call("SetQueuedCmdStartExec")

    def sample(self):
        p, k = self.pose(), Kinematics()
        self.call("GetKinematics", C.byref(k))
        self.recorder["samples"].append({"t_s": time.perf_counter() - self.recorder["_clock"],
                  "phase": self.phase, "pose": p, "tcp_velocity_mm_s": k.velocity,
                  "tcp_acceleration_mm_s2": k.acceleration})
        return p

    def _wait(self, target, point, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_stop()
            index = C.c_uint64()
            self.call("GetQueuedCmdCurrentIndex", C.byref(index))
            self.ensure_no_alarms()
            actual = self.sample()
            if index.value >= target:
                if any(abs(actual[k] - getattr(point, k)) > 0.8 for k in ("x", "y", "z", "r")):
                    raise RuntimeError(f"终点坐标或R角偏差过大：{actual}，目标{point}")
                return
            time.sleep(0.025)
        raise TimeoutError(f"指令 {target} 超时")

    def pause(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_stop()
            self.ensure_no_alarms()
            time.sleep(min(0.04, max(0, deadline - time.monotonic())))


def run_steps(bot, plan, recorder):
    for step in plan["steps"]:
        bot.check_stop()
        bot.phase = step["label"]
        event = {"phase": bot.phase, "start_s": time.perf_counter() - recorder["_clock"], "state": "running"}
        recorder["events"].append(event)
        print(bot.phase, flush=True)
        if step["kind"] == "move":
            bot.move(Point(**step["point"]))
        elif step["kind"] == "suction":
            event["suction_state"] = bot.suction(step["on"])
        else:
            bot.pause(step["seconds"])
        event.update(end_s=time.perf_counter() - recorder["_clock"], state="completed")
        if step["label"] == "打开吸盘":
            recorder["grip_started_s"] = event["end_s"]
        elif step["label"] == "等待泄压释放":
            recorder["release_finished_s"] = event["end_s"]
        elif step["label"] == "释放后竖直抬起验证":
            recorder["verification_lift_finished_s"] = event["end_s"]


def execute(plan, config):
    STOP_FILE.unlink(missing_ok=True)
    record = {"timestamp": datetime.now(timezone.utc).isoformat(), "completed": False,
              "plan": plan, "motion_profile": config["motion_profile"],
              "events": [], "samples": [], "visual_release_confirmation": "pending human observation"}
    output = ROOT / "records/runs" / datetime.now().strftime("%Y%m%d_%H%M%S")
    output.mkdir(parents=True, exist_ok=True)
    try:
        with CoinRobot(config["port"], record) as bot:
            bot.ensure_no_alarms()
            current, offset, suction = bot.pose(), bot.offset(), bot.suction_state()
            record.update(initial_pose=current, initial_suction=suction, original_parameters=bot.parameters())
            if suction["on"]:
                raise ValueError("开始时吸盘已开启，请先确认是否持币；程序保留当前状态")
            if any(abs(offset[k] - plan["tool_offset"][k]) > 0.05 for k in ("x", "y", "z")):
                raise ValueError("工具偏移与采点时不同")
            deviation, alignment = start_alignment(current, plan, config)
            record.update(start_deviation=deviation, start_alignment_steps=alignment)
            print(f"起点偏差：XY={deviation['xy_mm']:.2f} mm，Z={deviation['z_mm']:.2f} mm，"
                  f"R={deviation['r_deg']:.2f}°；自动校正步骤 {len(alignment)} 条", flush=True)
            bot.prepare_experiment(config["motion_profile"])
            record["applied_parameters"] = bot.parameters()
            verify_applied_parameters(config["motion_profile"], record["applied_parameters"])
            record["_clock"] = time.perf_counter()
            if alignment:
                run_steps(bot, {"steps": alignment}, record)
            run_steps(bot, plan, record)
            record.update(completed=True, final_pose=bot.pose(), final_suction=bot.suction_state(),
                          final_alarm_bytes=bot.alarms(), total_elapsed_s=time.perf_counter() - record["_clock"])
            record["grip_to_release_elapsed_s"] = record["release_finished_s"] - record["grip_started_s"]
            record["motion_and_handling_elapsed_s"] = record["verification_lift_finished_s"]
    except BaseException as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
        if "_clock" in record:
            record["total_elapsed_s"] = time.perf_counter() - record["_clock"]
        raise
    finally:
        record.pop("_clock", None)
        record["sampled_peak_tcp_velocity_mm_s"] = max((abs(s["tcp_velocity_mm_s"]) for s in record["samples"]), default=0)
        record["sampled_peak_tcp_acceleration_mm_s2"] = max((abs(s["tcp_acceleration_mm_s2"]) for s in record["samples"]), default=0)
        write_json(output / "record.json", record)
        write_json(RESULTS / "latest_run.json", record)
        print(f"运行记录：{output / 'record.json'}", flush=True)
    print(f"完成：总耗时 {record['total_elapsed_s']:.3f} s，吸取到释放 {record['grip_to_release_elapsed_s']:.3f} s")


def preview(plan):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
    a, b = plan["source"], plan["destination"]
    axes[0].plot([0, 0, plan["distance_xy_mm"], plan["distance_xy_mm"]],
                 [a["z"], plan["travel_z_mm"], plan["travel_z_mm"], plan["release_z_mm"]], "o-", color="#20546b")
    axes[0].plot([plan["distance_xy_mm"]] * 2, [plan["release_z_mm"], plan["verification_z_mm"]],
                 "o-", color="#bd6535", label="放币后抬起")
    top = b["z"] + plan["obstacle_height_mm"]
    axes[0].fill_between([plan["distance_xy_mm"] * .4, plan["distance_xy_mm"] * .6],
                         b["z"], top, color="#cccccc", label="约30mm障碍示意")
    axes[0].set(xlabel="沿 AB 方向 / mm", ylabel="机器人 Z / mm", title="单枚硬币门形搬运路径")
    axes[0].legend(fontsize=9);axes[0].grid(alpha=.2)
    points = [s["point"] for s in plan["steps"] if s["kind"] == "move"]
    axes[1].plot([p["x"] for p in points], [p["y"] for p in points], "o-", color="#20546b")
    axes[1].annotate("A：取币\n高位旋转+180°", (a["x"], a["y"]), xytext=(8, -18),
                     textcoords="offset points", fontsize=10, va="top")
    axes[1].annotate("B：放币\n抬起观察", (b["x"], b["y"]), xytext=(-10, 14),
                     textcoords="offset points", fontsize=10, ha="right")
    axes[1].set(xlabel="X / mm", ylabel="Y / mm", title="平面路径与旋转位置")
    axes[1].set_aspect("equal");axes[1].grid(alpha=.2)
    folder = RESULTS;folder.mkdir(parents=True, exist_ok=True)
    fig.savefig(folder / "transfer_path.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("plan", "run", "stop"), nargs="?", default="plan")
    parser.add_argument("--config", type=Path, default=ROOT / "demo_config.json")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--rotation", choices=("auto", "ccw", "cw"))
    args = parser.parse_args()
    if args.action == "stop":
        STOP_FILE.write_text(datetime.now(timezone.utc).isoformat(), encoding="ascii")
        print("已请求停止；运行中的脚本会在下次轮询时急停并清队列，吸盘保持当前状态。")
        return
    config = read_json(args.config)
    if args.rotation:
        config["rotation_direction"] = args.rotation
    plan = build_plan(read_json(ROOT / "source_A.json"), read_json(ROOT / "destination_B.json"), config)
    write_json(ROOT / "records/output/transfer_plan.json", plan)
    preview(plan)
    print(json.dumps({k: plan[k] for k in ("distance_xy_mm", "travel_z_mm", "release_z_mm", "verification_z_mm", "rotation")},
                     ensure_ascii=False, indent=2))
    if args.action == "run" and args.execute:
        execute(plan, config)
    else:
        print("计划已保存；未连接机械臂，未发送运动指令。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("已中断；运动停止，吸盘保留当前开关状态。", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        sys.exit(1)
