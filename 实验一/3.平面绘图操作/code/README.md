# 平面绘图

在“实验一”目录运行：

```powershell
python "3.平面绘图操作/code/plotter.py" preview --size 10
python -m unittest discover -s "3.平面绘图操作/code" -p test_drawing.py -v
```

`geometry.py` 实现直线、圆弧和圆形函数；`plotter.py` 生成轨迹并通过 SDK 执行。默认离线预览。实机命令需加 `draw --execute`，使用 `paper_config.json` 的标定和显式速度参数。

图片统一平铺在 `../results/`：`*_preview.png` 为理论预览，`all_preview_实机计划.png` 为本次实机计划预览，另有原有程序流程图。轨迹及原始日志集中保存在 `records/`。实绘照片为 `../results/结果图形.png`，仅有多边形花纹；该次绘图日志记录中途停止。10 mm 试线的完成记录见 `records/trial_line_result.json`。
