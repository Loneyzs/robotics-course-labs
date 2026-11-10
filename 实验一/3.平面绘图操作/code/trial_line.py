"""沿两个已采触纸点的连线，在中部绘制 10 mm 试线。

该段位于两点之间，Z 随对角线线性插值；不把两点扩展为完整纸面。
默认只保存计划，添加 --execute 才执行。
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from geometry import Point
from dobot_sdk import Dobot


ROOT = Path(__file__).resolve().parent
RECORDS = ROOT / "records"


def make_plan(data):
    a, b = data["lower_left"], data["upper_right"]
    diagonal = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
    if diagonal < 30:
        raise ValueError("两个采点间距不足")
    half = 5 / diagonal
    def at(t):
        return {key: a[key] + t * (b[key] - a[key]) for key in ("x", "y", "z")}
    return {"start": at(0.5 - half), "end": at(0.5 + half),
            "travel_z": max(a["z"], b["z"]) + 8,
            "length_xy_mm": 10, "draw_speed_mm_s": 4,
            "approach_speed_mm_s": 10,
            "note": "只使用两个采点之间的直线插值；横向纸面倾斜未标定"}


def save(data):
    RECORDS.mkdir(parents=True, exist_ok=True)
    (RECORDS / "trial_line_result.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    data = json.loads((ROOT / "paper_corners.json").read_text(encoding="utf-8"))
    plan = make_plan(data)
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    result = {"timestamp": datetime.now(timezone.utc).isoformat(), "plan": plan,
              "motion_sent": False, "completed": False, "events": []}
    save(result)
    if not args.execute:
        return
    with Dobot("COM7") as bot:
        bot.ensure_no_alarms()
        current = bot.pose()
        result["initial_pose"] = current
        offset = bot.offset()
        sampled_offset = json.loads((ROOT / "paper_lower_left.json").read_text(encoding="utf-8"))["tool_offset"]
        if any(abs(offset[k] - sampled_offset[k]) > 0.05 for k in ("x", "y", "z")):
            raise ValueError("工具偏移与采点时不一致")
        # 从已知触纸点处启动，避免从未知位置接近纸面。
        if min(math.dist([current[k] for k in ("x", "y", "z")],
                         [corner[k] for k in ("x", "y", "z")])
               for corner in (data["lower_left"], data["upper_right"])) > 2:
            raise ValueError("当前位置已离开采点，请先重新确认接近路径")
        r = current["r"]
        z = plan["travel_z"]
        start, end = Point(**plan["start"], r=r), Point(**plan["end"], r=r)
        bot.prepare_motion(10, 20)

        def move(label, target):
            result["motion_sent"] = True
            result["events"].append({"step": label, "target": vars(target), "state": "sending"})
            save(result)
            print(label, vars(target), flush=True)
            bot.move(target)
            result["events"][-1].update(state="completed", actual_pose=bot.pose())
            save(result)

        try:
            move("抬笔", Point(current["x"], current["y"], z, r))
            move("在纸面上方移到试线起点", Point(start.x, start.y, z, r))
            # 队列已完成后，以低速设置接近纸面和绘图。
            import ctypes as C
            from dobot_sdk import MotionParams
            settings = MotionParams(4, 10, 10, 20)
            index = C.c_uint64()
            bot.call("SetPTPCoordinateParams", C.byref(settings), False, C.byref(index))
            move("下降至插值触纸高度", start)
            move("绘制10mm直线", end)
            move("绘制后抬笔", Point(end.x, end.y, z, r))
            result["completed"] = True
            result["final_pose"] = bot.pose()
            result["alarm_bytes"] = bot.alarms()
            save(result)
        except BaseException as exc:
            result["error"] = str(exc)
            save(result)
            raise
    print("试线运动已完成，已抬笔、恢复速度参数并释放 COM7。", flush=True)


if __name__ == "__main__":
    main()
