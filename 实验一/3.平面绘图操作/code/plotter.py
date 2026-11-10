"""Dobot 平面绘图 CLI。默认 preview 不连接机械臂。

示例：python "3.平面绘图操作/code/plotter.py" preview
      python "3.平面绘图操作/code/plotter.py" status --port COM7
      python "3.平面绘图操作/code/plotter.py" calibrate --port COM7
      python "3.平面绘图操作/code/plotter.py" draw --pattern polygon --hover 10 --execute
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from geometry import Command, Drawing, Point, bounds, make_pattern, validate_bounds


ROOT = Path(__file__).resolve().parent
RESULTS = ROOT.parent / "results"


def save_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def render_preview(drawing, destination, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(8.5, 5), layout="constrained")
    for cmd in drawing.commands:
        if cmd.pen_down:
            points = cmd.samples()
            ax.plot([p.y for p in points], [p.x for p in points], color="#1c5673", linewidth=0.85)
    ax.set_aspect("equal")
    ax.set_xlabel("机器人 Y / mm")
    ax.set_ylabel("机器人 X / mm")
    ax.set_title(title + "（理论轨迹预览）")
    ax.grid(alpha=0.18)
    fig.savefig(destination, dpi=180)
    plt.close(fig)


def export_plan(drawing, folder, pattern):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    commands = []
    for cmd in drawing.commands:
        item = {"kind": cmd.kind, "pen_down": cmd.pen_down,
                "start": vars(cmd.start), "end": vars(cmd.end)}
        if cmd.middle:
            item.update(middle=vars(cmd.middle), center=cmd.center, sweep_deg=cmd.sweep)
        commands.append(item)
    save_json(ROOT / "records/trajectories" / f"{pattern}_trajectory.json", {"units": "mm, degrees",
              "bounds": bounds(drawing.commands), "commands": commands})
    labels = {"polygon": "多边形旋转花纹", "arc": "双圆弧花瓣", "circle": "圆形旋转花纹",
              "all": "多边形 / 圆弧 / 圆形"}
    render_preview(drawing, folder / f"{pattern}_preview.png", labels[pattern])


def load_config(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for name in ("origin", "paper_z", "r", "tool_offset", "limits", "lift"):
        if name not in data:
            raise ValueError(f"标定文件缺少 {name}，请先运行 calibrate")
    return data


def build(args, config=None):
    origin = tuple(config["origin"]) if config else (200.0, 0.0)
    z = config["paper_z"] + args.hover if config else 0.0
    return make_pattern(args.pattern, origin, z, config["r"] if config else 0.0,
                        args.size, args.repeats, config["lift"] if config else 10.0)


def status(args):
    from dobot_sdk import Dobot
    with Dobot(args.port) as bot:
        result = {"timestamp": datetime.now(timezone.utc).isoformat(), "port": args.port,
                  "firmware": bot.firmware, "pose": bot.pose(),
                  "tool_offset": bot.offset(), "alarm_bytes": bot.alarms(), "motion_sent": False}
    print(json.dumps(result, ensure_ascii=False, indent=2))
    save_json(ROOT / "records/status_latest.json", result)


def calibrate(args):
    """仅读取当前坐标；须先装笔并将笔尖轻贴图案中心处纸面。"""
    from dobot_sdk import Dobot
    with Dobot(args.port) as bot:
        bot.ensure_no_alarms()
        p, offset = bot.pose(), bot.offset()
    # 此范围是待用户确认的绘图盒，不能代替真实关节可达性检查。
    limits = {"x": [p["x"] - 30, p["x"] + 30],
              "y": [p["y"] - 80, p["y"] + 80],
              "z": [p["z"] - 0.5, p["z"] + args.lift + 20]}
    save_json(args.config, {"port": args.port, "origin": [p["x"], p["y"]],
              "paper_z": p["z"], "r": p["r"], "tool_offset": offset,
              "lift": args.lift, "limits": limits,
              "note": "原点须为装笔后笔尖轻贴纸面的中心位置；limits 须按纸张、障碍物和可达范围核对"})
    print(f"已保存当前纸面坐标：{args.config}；未发送运动指令。")
    print("执行前核对 limits，并先以 --hover 10 空中试画。")


def execute_plan(bot, drawing, config, speed, acceleration):
    """所有几何/起点检查通过后才准备队列，逐条执行并等待完成。"""
    if not math.isfinite(speed) or not 0 < speed <= 50:
        raise ValueError("绘图速度须在 0～50 mm/s 之间")
    if not math.isfinite(acceleration) or not 0 < acceleration <= 100:
        raise ValueError("加速度须在 0～100 mm/s² 之间")
    validate_bounds(drawing.commands, config["limits"])
    bot.ensure_no_alarms()
    offset = bot.offset()
    if any(abs(offset[n] - config["tool_offset"][n]) > 0.05 for n in ("x", "y", "z")):
        raise ValueError("工具偏移已变化，请重新标定纸面")
    pose = bot.pose()
    if abs(pose["r"] - config["r"]) > 0.5:
        raise ValueError("当前 R 与标定值不同，请恢复标定姿态或重新标定")
    if pose["z"] < config["paper_z"] - 0.2:
        raise ValueError("当前 Z 低于标定纸面，请先将笔尖抬离纸面")
    start = Point(pose["x"], pose["y"], pose["z"], pose["r"])
    target = drawing.commands[0].start
    safe_z = drawing.paper_z + drawing.lift
    raised = Point(start.x, start.y, safe_z, start.r)
    above = Point(target.x, target.y, safe_z, target.r)
    # 构图的第一点是纸面中心；先在上方接近它，再开始计划中的抬落笔动作。
    approach = [Command("line", start, raised, False),
                Command("line", raised, above, False)]
    validate_bounds(approach, config["limits"])
    bot.prepare_motion(speed, acceleration)
    bot.move(raised)
    bot.move(above)
    # 第一条计划命令为从理论纸面起点抬笔到 safe_z；实际已经抬笔，直接执行终点。
    for index, cmd in enumerate(drawing.commands):
        if cmd.kind == "arc":
            bot.arc(cmd.middle, cmd.end)
        else:
            bot.move(cmd.end)
        if (index + 1) % 20 == 0 or index == len(drawing.commands) - 1:
            print(f"已完成 {index + 1}/{len(drawing.commands)} 条指令", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    p = sub.add_parser("status", help="读取 COM7 位姿、报警和工具偏移，不运动")
    p.add_argument("--port", default="COM7")
    p = sub.add_parser("calibrate", help="读取当前笔尖轻贴纸面位置作为图案中心，不运动")
    p.add_argument("--port", default="COM7")
    p.add_argument("--config", type=Path, default=ROOT / "paper_config.json")
    p.add_argument("--lift", type=float, default=10)
    for action in ("preview", "draw"):
        p = sub.add_parser(action, help="离线生成预览" if action == "preview" else "预览或执行绘图")
        p.add_argument("--pattern", choices=("polygon", "arc", "circle", "all"), default="all")
        p.add_argument("--size", type=float, default=20, help="单组图案半径，mm")
        p.add_argument("--repeats", type=int, default=10, help="每组的旋转重复次数")
        p.add_argument("--output", type=Path, default=RESULTS)
        p.add_argument("--config", type=Path, default=ROOT / "paper_config.json")
        p.add_argument("--hover", type=float, default=0, help="轨迹相对纸面抬高距离，mm")
        if action == "draw":
            p.add_argument("--port", default="COM7")
            p.add_argument("--execute", action="store_true", help="向实物发送运动指令")
            p.add_argument("--speed", type=float, default=15, help="mm/s")
            p.add_argument("--acceleration", type=float, default=30, help="mm/s²")
    args = parser.parse_args(argv)
    if args.action == "status":
        status(args)
    elif args.action == "calibrate":
        if not math.isfinite(args.lift) or not 0 < args.lift <= 30:
            raise ValueError("抬笔高度须在 0～30 mm 之间")
        calibrate(args)
    else:
        if not math.isfinite(args.hover) or args.hover < 0:
            raise ValueError("空画高度须为非负有限数")
        config = load_config(args.config) if args.config.exists() else None
        drawing = build(args, config)
        if config:
            validate_bounds(drawing.commands, config["limits"])
        # 默认始终离线；--execute 才加载 SDK 并连接机械臂。
        if args.action == "draw" and args.execute and config is None:
            raise ValueError("缺少标定文件；装笔后先运行 calibrate")
        export_plan(drawing, args.output, args.pattern)
        print(f"已生成预览和轨迹：{args.output.resolve()}")
        print(f"指令数：{len(drawing.commands)}；范围：{bounds(drawing.commands)}")
        if args.action == "draw" and args.execute:
            from dobot_sdk import Dobot
            with Dobot(args.port) as bot:
                execute_plan(bot, drawing, config, args.speed, args.acceleration)
            print("绘图完成，已抬笔并断开连接。")
        else:
            print("离线模式，未连接机械臂。")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("已中断；已连接时会尝试急停并清空队列。", file=sys.stderr)
        sys.exit(130)
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        sys.exit(1)
