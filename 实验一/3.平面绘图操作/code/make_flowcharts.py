"""生成可插入实验报告的流程图；纯离线。"""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

ROOT = Path(__file__).resolve().parent.parent / "results"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["svg.fonttype"] = "none"


def node(ax, y, label, decision=False):
    if decision:
        shape = Polygon([(0.5, y + 0.044), (0.91, y), (0.5, y - 0.044), (0.09, y)],
                        facecolor="#e8f0f5", edgecolor="#28556c")
    else:
        shape = FancyBboxPatch((0.10, y - 0.026), 0.80, 0.052,
                               boxstyle="round,pad=0.007,rounding_size=0.008",
                               facecolor="#f3f7fa", edgecolor="#28556c")
    ax.add_patch(shape)
    ax.text(0.5, y, label, ha="center", va="center", fontsize=10)


def arrow(ax, y0, y1, label="", source_decision=False, target_decision=False):
    start = y0 - (0.044 if source_decision else 0.034)
    end = y1 + (0.044 if target_decision else 0.034)
    ax.annotate("", xy=(0.5, end), xytext=(0.5, start),
                arrowprops=dict(arrowstyle="->", color="#28556c", lw=1.2))
    if label:
        ax.text(0.52, (start + end) / 2, label, va="center", fontsize=9)


def chain(ax, labels, title, decisions=()):
    ax.set(xlim=(0, 1), ylim=(0, 1))
    ax.axis("off")
    ax.set_title(title, fontsize=13, pad=16)
    ys = [0.94 - i * 0.86 / (len(labels) - 1) for i in range(len(labels))]
    for i, (y, label) in enumerate(zip(ys, labels)):
        node(ax, y, label, i in decisions)
        if i:
            arrow(ax, ys[i - 1], y, "是" if i - 1 in decisions else "",
                  i - 1 in decisions, i in decisions)
    return ys


def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.5, 12), layout="constrained")
    ys = chain(ax, ["开始：选择图案、尺寸与重复次数", "生成各元素的旋转位置与绘图指令",
          "离线预览并导出轨迹 JSON", "指定实机执行 --execute？", "读取标定文件，连接 COM7，读取状态",
          "范围、工具偏移、R、报警检查通过？", "保存速度参数；停队列、清队列、设速度",
          "先抬笔，再在上方移动到起点", "依次发送 MOVL / ARC，并等待每条完成",
          "绘图结束抬笔；恢复参数；断开连接"], "平面绘图主程序流程图", decisions=(3, 5))
    for idx, text in ((3, "否：离线结束"), (5, "否：报错退出")):
        y = ys[idx]
        ax.annotate("", xy=(0.98, y), xytext=(0.91, y),
                    arrowprops=dict(arrowstyle="->", color="#28556c"))
        ax.text(0.965, y + 0.027, text, rotation=90, ha="center", va="bottom", fontsize=9)
    fig.savefig(ROOT / "main_flowchart.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 8), layout="constrained")
    chain(axes[0], ["输入：长度 L、方向角 θ", "读取规划器当前位置 (x, y, z, r)",
          "将 θ 从度转换为弧度", "x′ = x + L cos θ\ny′ = y + L sin θ",
          "追加 MOVL 终点，保持 z、r", "更新当前位置并返回"], "① draw_line：直线")
    ys = chain(axes[1], ["输入：圆心 (cx, cy)、圆弧角 α", "读取当前位置，计算半径与起始角",
          "校验半径；α=0 时直接返回", "分段数 n = ceil(|α| / 90°)",
          "求本段半角点及终点", "追加 ARC 指令，更新当前位置",
          "所有分段已处理？", "返回"], "② draw_arc：圆弧", decisions=(6,))
    ax = axes[1]
    ax.plot([0.09, 0.035, 0.035, 0.5], [ys[6], ys[6], ys[4] + 0.055, ys[4] + 0.055],
            color="#28556c", lw=1)
    ax.annotate("", xy=(0.5, ys[4] + 0.034), xytext=(0.5, ys[4] + 0.055),
                arrowprops=dict(arrowstyle="->", color="#28556c"))
    ax.text(0.04, (ys[4] + ys[6]) / 2, "否", fontsize=9)
    chain(axes[2], ["输入：圆心 (cx, cy)", "当前位置决定圆的半径与起点",
          "调用 draw_arc(cx, cy, 360°)", "生成四段 90° 圆弧", "闭合至起点，返回"],
          "③ draw_circle：圆形")
    fig.savefig(ROOT / "functions_flowchart.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
