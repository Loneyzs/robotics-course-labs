"""生成当前连续码垛流程图，不连接机械臂。"""
from pathlib import Path
from coin_demo import build_plan, read_json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon

ROOT = Path(__file__).resolve().parent.parent / "results"
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["svg.fonttype"] = "none"


def main():
    code = Path(__file__).resolve().parent
    config = read_json(code / "demo_config.json")
    plan = build_plan(read_json(code / "source_A.json"), read_json(code / "destination_B.json"), config)
    count = plan["coin_count"]
    labels = [f"A叠好{count}枚、B清空；吸盘拖回A附近", f"连接{config['port']}，读取位姿、报警和吸盘状态",
              "起点、工具偏移与旋转范围检查通过？",
              "上限200 / 倍率100%；计时，i=0\n高处预转空吸盘，再下降对准A",
              "计算本层：取币Z=A0−i·t；放币Z=B0+(i+1)·t+1",
              "第2枚起，高位返回A，再竖直下降到本层取币Z",
              f"打开吸盘，等待{config['grip_wait_s']}s吸附",
              f"携币升至Z={plan['travel_z_mm']:.3f}mm；{plan['rotation']['direction_cn']}180°",
              "保持高位，MOVL越过约30mm障碍到B",
              f"下降至本层放币Z；关闭吸盘，等待{config['release_wait_s']}s",
              "抬回越障高度，停留观察；最后一枚停留2s",
              "空吸盘反向恢复腕角；记录本枚数据，i=i+1",
              f"i < {count}？",
              "结束计时，保存总耗时、逐枚耗时和速度采样",
              f"恢复原运动参数，断开{config['port']}；停在B上方"]
    decisions = (2, 12)
    fig, ax = plt.subplots(figsize=(8.5, 16), layout="constrained")
    ax.set(xlim=(0, 1), ylim=(0, 1));ax.axis("off")
    ax.set_title(f"实验四：{count}枚硬币连续取放、越障与旋转", fontsize=15, pad=18)
    positions = [0.95 - i * 0.9 / (len(labels) - 1) for i in range(len(labels))]
    for i, (y, label) in enumerate(zip(positions, labels)):
        if i in decisions:
            shape = Polygon([(0.45, y + .029), (.85, y), (.45, y - .029), (.05, y)],
                            facecolor="#e8f0f5", edgecolor="#28556c")
        else:
            shape = FancyBboxPatch((.06, y - .019), .78, .038,
                                   boxstyle="round,pad=.005,rounding_size=.008",
                                   facecolor="#f3f7fa", edgecolor="#28556c")
        ax.add_patch(shape)
        ax.text(.45, y, label, ha="center", va="center", fontsize=10)
        if i:
            upper = positions[i - 1] - (.03 if i - 1 in decisions else .025)
            lower = y + (.03 if i in decisions else .025)
            ax.annotate("", xy=(.45, lower), xytext=(.45, upper),
                        arrowprops=dict(arrowstyle="->", lw=1.2, color="#28556c"))
            if i == 3:
                ax.text(.47, (upper + lower) / 2, "是", fontsize=9, va="center")
            if i == 13:
                ax.text(.47, (upper + lower) / 2, "否", fontsize=9, va="center")
    y = positions[2]
    ax.annotate("", xy=(.98, y), xytext=(.85, y), arrowprops=dict(arrowstyle="->", color="#28556c"))
    ax.text(.92, y + .02, "否：退出", ha="center", fontsize=9)
    y_loop, y_compute = positions[12], positions[4]
    ax.plot([.05, .015, .015, .45], [y_loop, y_loop, y_compute + .035, y_compute + .035],
            color="#28556c", lw=1.2)
    ax.annotate("", xy=(.45, y_compute + .025), xytext=(.45, y_compute + .035),
                arrowprops=dict(arrowstyle="->", color="#28556c"))
    ax.text(.02, (y_loop + y_compute) / 2, "是：下一枚", rotation=90, fontsize=9, ha="left")
    ax.text(.93, .38, "运行中持续检查停止请求和报警\n异常时停止运动、清空队列\n吸盘保持当前状态", rotation=90,
            ha="center", va="center", fontsize=10, color="#435b68")
    ROOT.mkdir(parents=True, exist_ok=True)
    fig.savefig(ROOT / "flowchart.png", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
