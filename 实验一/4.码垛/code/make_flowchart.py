"""生成单枚码垛演示流程图，不连接机械臂。"""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

ROOT = Path(__file__).resolve().parent.parent / "results"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["svg.fonttype"] = "none"


def main():
    labels = ["用户手动拖回 A 附近：硬币放在原记录位置", "连接 COM7，读取位姿、报警和吸盘状态",
              "起点、工具偏移与旋转范围检查通过？",
              "设置并读取上限 200 / 倍率 100%；开始计时", "有偏差时：抬升 → 高位对准 A → 下降取币高度",
              "打开吸盘，等待 0.6 s 吸附",
              "携币竖直抬升：Z = −17.095 mm", "保持 XY，在高处逆时针旋转 +180°",
              "保持高位，MOVL 越过约 30 mm 障碍到 B", "下降至 B 桌面上方 5 mm",
              "关闭吸盘，等待 0.6 s 泄压释放", "竖直抬起 20 mm，停留 2 s 供观察拍摄",
              "空吸盘反向恢复初始腕角", "结束计时，记录参数、耗时和采样速度",
              "恢复原运动参数，断开 COM7；停在 B 上方"]
    fig, ax = plt.subplots(figsize=(8.5, 16), layout="constrained")
    ax.set(xlim=(0, 1), ylim=(0, 1));ax.axis("off")
    ax.set_title("实验四：单枚硬币越障、旋转与释放验证", fontsize=15, pad=18)
    positions = [0.95 - i * 0.9 / (len(labels) - 1) for i in range(len(labels))]
    for i, (y, label) in enumerate(zip(positions, labels)):
        if i == 2:
            shape = Polygon([(0.45, y + .029), (.85, y), (.45, y - .029), (.05, y)],
                            facecolor="#e8f0f5", edgecolor="#28556c")
        else:
            shape = FancyBboxPatch((.06, y - .019), .78, .038,
                                   boxstyle="round,pad=.005,rounding_size=.008",
                                   facecolor="#f3f7fa", edgecolor="#28556c")
        ax.add_patch(shape)
        ax.text(.45, y, label, ha="center", va="center", fontsize=10)
        if i:
            upper = positions[i - 1] - (.03 if i - 1 == 2 else .025)
            lower = y + (.03 if i == 2 else .025)
            ax.annotate("", xy=(.45, lower), xytext=(.45, upper),
                        arrowprops=dict(arrowstyle="->", lw=1.2, color="#28556c"))
            if i == 3:
                ax.text(.47, (upper + lower) / 2, "是", fontsize=9, va="center")
    y = positions[2]
    ax.annotate("", xy=(.98, y), xytext=(.85, y), arrowprops=dict(arrowstyle="->", color="#28556c"))
    ax.text(.92, y + .02, "否：退出", ha="center", fontsize=9)
    ax.text(.93, .38, "运行中持续检查停止请求和报警\n异常时停止运动、清空队列\n吸盘保持当前状态", rotation=90,
            ha="center", va="center", fontsize=10, color="#435b68")
    ROOT.mkdir(parents=True, exist_ok=True)
    fig.savefig(ROOT / "flowchart.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
