# 码垛演示

在“实验一”目录运行：

```powershell
python "4.码垛/code/coin_demo.py" plan
python -m unittest discover -s "4.码垛/code" -p test_coin_demo.py -v
```

默认离线生成计划。`run --execute` 执行实机搬运，`stop` 请求停止。共享 SDK 位于 `../../3.平面绘图操作/code/`，请保持目录结构。

交付图片和表格平铺在 `../results/`，包括原有流程图和路径图。`latest_run.json` 是 results 中唯一保留的原始日志。`records/` 保存离线计划及历次运行记录。当前演示使用 1 枚硬币，最近一次完整日志的吸取至释放耗时为 6.9311614 s，全流程耗时为 14.3307074 s，障碍高度为估计值 30 mm。具体计时范围、速度回读和各阶段耗时见运行记录表。录像末尾仍显示硬币靠近吸盘，释放分离及文字倒立需补充清晰画面。
