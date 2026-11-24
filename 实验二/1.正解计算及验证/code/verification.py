"""生成正解或逆解的对照记录、脚本输出图及程序流程图。"""

import argparse
from collections import Counter
import json
from math import dist, sqrt
import os
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from kinematics import CARTESIAN, JOINTS, angle_difference, forward, inverse

HERE = Path(__file__).resolve().parent
HISTORICAL = HERE / "records" / "实验一负X象限.json"
OCTANTS = {"+++", "++-", "+-+", "+--", "-++", "-+-", "--+", "---"}


def load_samples(live_path, historical_path):
    live = json.loads(live_path.read_text(encoding="utf-8"))
    historical = json.loads(historical_path.read_text(encoding="utf-8"))
    samples = []
    live_phase = "当次复采" if "sampling_joint1_max_abs_deg" in live else "当次首轮"
    for source, phase, data, sign in (
        ("LIVE", live_phase, live, "+"),
        ("HIST", "实验一点动", historical, "-"),
    ):
        for sample in data["samples"]:
            pose = sample["pose"]
            octant = "".join("+" if pose[axis] > 0 else "-" for axis in "xyz")
            if any(abs(pose[axis]) < 1e-5 for axis in "xyz") or octant != sample["octant"]:
                raise ValueError(f"象限与坐标不符 {sample}")
            if octant[0] == sign:
                samples.append({**sample, "group": source, "phase": phase})
    counter = Counter(sample["octant"] for sample in samples)
    if counter != Counter(OCTANTS):
        raise ValueError(f"所选数据不能恰好覆盖八个象限 {dict(counter)}")
    return sorted(samples, key=lambda sample: sample["octant"])


def compare(samples, inverse_mode):
    model, inputs, outputs = (
        (inverse, CARTESIAN, JOINTS) if inverse_mode else (forward, JOINTS, CARTESIAN)
    )
    rows = []
    for sample in samples:
        pose = sample["pose"]
        given = tuple(pose[name] for name in inputs)
        measured = tuple(pose[name] for name in outputs)
        calculated = model(*given)
        residual = tuple(
            angle_difference(computed, reported) if inverse_mode or i == 3 else computed - reported
            for i, (computed, reported) in enumerate(zip(calculated, measured))
        )
        error = max(map(abs, residual)) if inverse_mode else dist(calculated[:3], measured[:3])
        rows.append({**sample, "input": given, "measured": measured, "calculated": calculated,
                     "residual": residual, "error": error})
    return rows


def stats(rows, group):
    values = [row["error"] for row in rows if row["group"] == group]
    return max(values), sqrt(sum(value ** 2 for value in values) / len(values))


def font(filename, size):
    return ImageFont.truetype(str(Path(os.environ["WINDIR"]) / "Fonts" / filename), size)


def render_printout(lines, output):
    box_width = 46
    typeface = font("consola.ttf", 22)
    width = int(max(typeface.getlength(line) for line in lines) + 2 * box_width)
    height = 29 * len(lines) + 62
    image = Image.new("RGB", (width, height), "#17212d")
    draw = ImageDraw.Draw(image)
    for i, line in enumerate(lines):
        color = "#92d5f1" if i in (0, 3) else "#eef4fa"
        draw.text((box_width, 27 + i * 29), line, fill=color, font=typeface)
    image.save(output)


def render_flow(inverse_mode, output):
    title = "几何逆解验证" if inverse_mode else "D-H 正解验证"
    steps = [
        "从采样记录读取 GetPose 的 X、Y、Z、R 和 J1～J4",
        "按坐标符号核对八个象限，保留实测批次和来源",
        "用 X、Y、Z、R 求底座角、平面两连杆角和腕角" if inverse_mode
        else "用 J1～J4 构造四段变换，计算总齐次变换",
        "比较逆解关节角与控制器关节角，按周期计算残差" if inverse_mode
        else "提取 X、Y、Z、R，与控制器位姿逐点对比",
        "汇总各批次误差，打印并保存记录表",
    ]
    image = Image.new("RGB", (1040, 700), "#f5f8fb")
    draw = ImageDraw.Draw(image)
    header, body = font("msyh.ttc", 38), font("msyh.ttc", 27)
    draw.text((520, 58), title, fill="#152d42", font=header, anchor="mm")
    for i, step in enumerate(steps):
        y = 118 + i * 111
        draw.rounded_rectangle((106, y, 934, y + 73), 13, fill="#e4f0fa", outline="#8aa9c1", width=2)
        draw.text((520, y + 36), step, fill="#152d42", font=body, anchor="mm")
        if i < len(steps) - 1:
            draw.line((520, y + 73, 520, y + 101), fill="#326882", width=3)
            draw.polygon(((513, y + 98), (527, y + 98), (520, y + 107)), fill="#326882")
    image.save(output)


def write_outputs(rows, live, history, output, inverse_mode):
    unit, metric = ("deg", "max_abs_joint") if inverse_mode else ("mm", "xyz_norm")
    keys_in = "XYZR" if inverse_mode else "J1..J4"
    keys_out = "J1..J4" if inverse_mode else "XYZR"
    name = "Geometric inverse kinematics" if inverse_mode else "D-H forward kinematics"
    report = [
        f"{name} | Dobot Magician | {len(rows)} octants",
        "XYZ: mm | R and J1..J4: deg | residual = calculated - reported",
        "LIVE: experiment 2 SDK | HIST: experiment 1 jog records",
        f"{'octant':<7}{'src':<6}{('input ' + keys_in):<42}"
        f"{('reported ' + keys_out):<42}{('calculated ' + keys_out):<42} {metric} ({unit})",
    ]

    def line_values(values):
        return " ".join(f"{value:+9.4f}" for value in values)

    for row in rows:
        report.append(
            f"{row['octant']:<7}{row['group']:<6}{line_values(row['input']):<42}"
            f"{line_values(row['measured']):<42}{line_values(row['calculated']):<42} {row['error']:.8f}"
        )
    for group in ("LIVE", "HIST"):
        maximum, rms = stats(rows, group)
        report.append(f"{group}  {metric}: maximum={maximum:.8f} {unit}; RMS={rms:.8f} {unit}")
    if not inverse_mode:
        maximum_r = max(abs(row["residual"][-1]) for row in rows)
        report.append(f"R maximum absolute residual = {maximum_r:.8f} deg")
    content = "\n".join(report) + "\n"
    output.mkdir(exist_ok=True)
    (output / "脚本输出.txt").write_text(content, encoding="utf-8", newline="\n")
    render_printout(report, output / "脚本输出图.png")
    render_flow(inverse_mode, output / "程序流程图.png")

    def values_md(values, places=6):
        return "(" + ", ".join(f"{value:.{places}f}" for value in values) + ")"

    heading = "几何逆解验证" if inverse_mode else "D-H 正解验证"
    metric_zh = "最大单轴角度残差绝对值（°）" if inverse_mode else "XYZ 残差范数（mm）"
    table = [
        f"# {heading}记录",
        "",
        f"本次 SDK 日志为 `{live.name}`。X<0 四行来自 `{history.name}` 对应的实验一点动工作表。",
        "XYZ 使用 mm，R 与 J1～J4 使用度。残差按计算值减控制器值，角度残差归一到 [−180°, 180°)。",
        "",
        f"| 象限 | 来源 | 输入 {keys_in} | 控制器 {keys_out} | 计算 {keys_out} | 逐分量残差 | {metric_zh} |",
        "| --- | --- | --- | --- | --- | --- | ---: |",
    ]
    for row in rows:
        source = row["phase"] + " " + row.get("source_range", row.get("timestamp", ""))
        table.append(
            f"| {row['octant']} | {source} | {values_md(row['input'])} | {values_md(row['measured'])} | "
            f"{values_md(row['calculated'])} | {values_md(row['residual'], 8)} | {row['error']:.8f} |"
        )
    table += ["", "## 误差汇总", "", "| 批次 | 样本数 | 最大值 | RMS |", "| --- | ---: | ---: | ---: |"]
    for group in ("LIVE", "HIST"):
        maximum, rms = stats(rows, group)
        label = "当次实机 X>0" if group == "LIVE" else "实验一历史 X<0"
        table.append(f"| {label} | 4 | {maximum:.8f} | {rms:.8f} |")
    if not inverse_mode:
        table += ["", f"八点最大 R 残差绝对值为 {maximum_r:.8f}°。"]
    table += [
        "",
        "本表比较 Python 运动学模型和控制器位姿数据。实时帧和历史截图的显示精度不同。",
        "控制器的 XYZ/R 与关节值由同一控制器提供。机械零点偏移、碰限位后的丢步或绝对定位误差需要独立测量。",
    ]
    (output / "对照结果.md").write_text("\n".join(table) + "\n", encoding="utf-8", newline="\n")
    print(content, end="")


def main(inverse_mode, output):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("live", type=Path, help="当次实机 GetPose 日志")
    parser.add_argument("--history", type=Path, default=HISTORICAL)
    args = parser.parse_args()
    rows = compare(load_samples(args.live, args.history), inverse_mode)
    write_outputs(rows, args.live, args.history, output, inverse_mode)
