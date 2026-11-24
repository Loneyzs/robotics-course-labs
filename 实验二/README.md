# 实验二

- `1.正解计算及验证/` 包含 SDK 实机采样、共享运动学模型、原始数据、正解推导和结果。
- `2.逆解计算及验证/` 使用同一批位姿记录，包含逆解推导、验证脚本和结果。
- 第三项单关节搬运在本轮范围之外。

## 采样与数据来源

2026-11-24 首次全向采样时，现场观察到底座 J1 碰限位并断电。首轮日志和重启后的限位报警读数保留在正解的 `code/records/`。现场通过小臂解锁退出限位后，控制器的报警读数恢复为全零。操作者确认目前 J1 的安全范围为 ±90°。重新采集的正 X 四点只使用 J1=±12°，以短行程取正、负 Z 各一点。其余四象限使用实验一 `点动坐标记录.xlsx` 的历史实测，原始表中 J1 的绝对值在 93.76°～113.16°。核对表按来源记录每一点。

采样后机械臂停在正前方，控制器回报 J1≈0°、Z≈8 mm，报警全零。关节 PTP 参数已恢复至采样前的读数，回读保存在 `code/records/post_scan_20261124_172432.json`。

本次模型对照覆盖八象限，其中四个是本次复采。若要求实验二同一批次现场重采全部八点，仍需有经确认可覆盖负 X 的机械行程。

## 复现

以下命令在 `实验二/` 执行。两次分析读取相同的 JSON 日志并生成各自的 `results/`。使用仓库的 64 位 Dobot SDK，分析脚本需要 NumPy 和 Pillow。

```powershell
$live = '1.正解计算及验证/code/records/live_20261124_172135_992814.json'
python -B '1.正解计算及验证/code/verify_forward.py' $live
python -B '2.逆解计算及验证/code/verify_inverse.py' $live
```

脚本输出图据实际打印的文本排版，原始终端文本同时保存。复采脚本无参数时打印运动计划。使用 `--execute` 会驱动机械臂。

厂商参考：[通信协议 V1.1.5](https://download.dobot.cc/product-manual/dobot-magician/pdf/en/Dobot-Communication-Protocol-V1.1.5.pdf)、[用户手册 V1.7.0](https://download.dobot.cc/product-manual/dobot-magician/pdf/V1.7.0/en/Dobot-Magician-User-Guide-V1.7.0.pdf)、[报警说明](https://download.dobot.cc/product-manual/dobot-magician/pdf/cn/Dobot-Magician-ALARM-Description.pdf)。
