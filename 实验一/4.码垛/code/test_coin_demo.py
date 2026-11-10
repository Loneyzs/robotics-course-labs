"""单枚硬币实验的离线验证，不连接串口。"""
import copy
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

import coin_demo as demo


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.a = demo.read_json(demo.ROOT / "source_A.json")
        self.b = demo.read_json(demo.ROOT / "destination_B.json")
        self.config = demo.read_json(demo.ROOT / "demo_config.json")
        self.plan = demo.build_plan(self.a, self.b, self.config)

    def test_correct_saved_destination(self):
        self.assertAlmostEqual(self.plan["distance_xy_mm"], 106.777979, places=5)
        self.assertAlmostEqual(self.plan["destination"]["y"], -263.819702, places=5)

    def test_start_at_A_directly_grips(self):
        self.assertEqual(self.plan["steps"][0]["kind"], "suction")
        self.assertTrue(self.plan["steps"][0]["on"])
        first_move = next(s for s in self.plan["steps"] if s["kind"] == "move")
        self.assertEqual(first_move["point"]["x"], self.a["pose"]["x"])
        self.assertGreater(first_move["point"]["z"], self.a["pose"]["z"])

    def test_reported_manual_position_is_accepted_and_aligned(self):
        current = dict(x=75.55317687988281, y=-170.6843719482422,
                       z=-64.07659149169922, r=-150.09161376953125)
        deviation, steps = demo.start_alignment(current, self.plan, self.config)
        self.assertLess(deviation["xy_mm"], 15)
        self.assertEqual(len(steps), 3)
        self.assertEqual(steps[0]["point"]["x"], current["x"])
        self.assertEqual(steps[0]["point"]["y"], current["y"])
        self.assertGreaterEqual(steps[0]["point"]["z"], self.plan["travel_z_mm"])
        for key in ("x", "y", "z", "r"):
            self.assertEqual(steps[-1]["point"][key], self.a["pose"][key])

    def test_exact_A_skips_unnecessary_alignment(self):
        _, steps = demo.start_alignment(self.a["pose"], self.plan, self.config)
        self.assertEqual(steps, [])

    def test_outside_start_region_rejected(self):
        for key, delta in (("x", 16), ("z", 11), ("r", 16)):
            current = self.a["pose"].copy();current[key] += delta
            with self.assertRaises(ValueError):
                demo.start_alignment(current, self.plan, self.config)

    def test_xy_tolerance_is_radial(self):
        current = self.a["pose"].copy();current["x"] += 12;current["y"] += 12
        with self.assertRaises(ValueError):
            demo.start_alignment(current, self.plan, self.config)

    def test_crosses_obstacle_at_high_z(self):
        step = next(s for s in self.plan["steps"] if "越过障碍" in s["label"])
        height_over_B_table = step["point"]["z"] - self.b["pose"]["z"]
        self.assertGreaterEqual(height_over_B_table, 50)
        self.assertEqual(step["point"]["z"], self.plan["travel_z_mm"])

    def test_clockwise_exceeds_current_wrist_range(self):
        rejected = next(c for c in self.plan["rotation_candidates"] if c["direction"] == "cw")
        self.assertFalse(rejected["feasible"])
        self.assertEqual(self.plan["rotation"]["delta_deg"], 180)
        cfg = copy.deepcopy(self.config);cfg["rotation_direction"] = "cw"
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, self.b, cfg)

    def test_selects_clockwise_for_another_pose(self):
        chosen, _ = demo.choose_rotation(60, (200, 0), (200, 10))
        self.assertEqual(chosen["direction"], "cw")

    def test_rotation_is_relative_to_A_not_recorded_B_r(self):
        self.assertAlmostEqual(self.plan["rotation"]["target_r_deg"] - self.a["pose"]["r"], 180)

    def test_release_clearance_and_verification_lift(self):
        self.assertAlmostEqual(self.plan["release_z_mm"] - self.b["pose"]["z"], 5)
        self.assertAlmostEqual(self.plan["verification_z_mm"] - self.plan["release_z_mm"], 20)
        labels = [s["label"] for s in self.plan["steps"]]
        self.assertLess(labels.index("关闭吸盘释放硬币"), labels.index("释放后竖直抬起验证"))
        self.assertLess(labels.index("等待泄压释放"), labels.index("释放后竖直抬起验证"))

    def test_reset_empty_wrist_for_next_manual_return(self):
        last = self.plan["steps"][-1]
        self.assertEqual(last["label"], "空吸盘反向恢复初始腕角")
        final_j4 = last["point"]["r"] - self.b["pose"]["j1"]
        self.assertAlmostEqual(final_j4, self.a["pose"]["j4"])

    def test_does_not_treat_table_as_coin_top_at_B(self):
        self.assertGreater(self.plan["release_z_mm"], self.b["pose"]["z"])

    def test_alarm_in_calibration_rejected(self):
        self.b["alarm_bytes"][10] = 1
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, self.b, self.config)

    def test_changed_tool_rejected(self):
        self.b["tool_offset"]["x"] += 10
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, self.b, self.config)

    def test_unrealistic_pose_relation_rejected(self):
        self.a["pose"]["r"] += 20
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, self.b, self.config)

    def test_full_ratio_and_upper_parameters(self):
        p = self.config["motion_profile"]
        self.assertEqual(p["velocity_ratio_percent"], 100)
        self.assertEqual(p["xyz_velocity_mm_s"], 200)
        self.assertEqual(p["xyz_acceleration_mm_s2"], 200)


class FakeRobot:
    def __init__(self):
        self.events = []
        self.on = False
        self.phase = ""

    def check_stop(self):
        pass

    def move(self, point):
        self.events.append(("move", self.phase, self.on, point))

    def suction(self, on):
        self.on = on
        self.events.append(("suction", self.phase, on))
        return {"enabled": True, "on": on}

    def pause(self, seconds):
        self.events.append(("wait", self.phase, seconds))


class RuntimeTests(unittest.TestCase):
    def test_aligns_with_suction_off_before_gripping(self):
        config = demo.read_json(demo.ROOT / "demo_config.json")
        plan = demo.build_plan(demo.read_json(demo.ROOT / "source_A.json"),
                               demo.read_json(demo.ROOT / "destination_B.json"), config)
        current = plan["source"].copy();current["x"] += 5;current["z"] += 3
        _, alignment = demo.start_alignment(current, plan, config)
        bot, record = FakeRobot(), {"events": [], "_clock": time.perf_counter()}
        demo.run_steps(bot, {"steps": alignment}, record)
        self.assertEqual(len(bot.events), 3)
        self.assertTrue(all(e[0] == "move" and e[2] is False for e in bot.events))
        demo.run_steps(bot, plan, record)
        self.assertEqual(bot.events[3][0], "suction")
        self.assertTrue(bot.events[3][2])

    def test_carries_with_suction_on_lifts_after_off_and_logs_times(self):
        plan = demo.build_plan(demo.read_json(demo.ROOT / "source_A.json"),
                               demo.read_json(demo.ROOT / "destination_B.json"),
                               demo.read_json(demo.ROOT / "demo_config.json"))
        bot, record = FakeRobot(), {"events": [], "_clock": time.perf_counter()}
        demo.run_steps(bot, plan, record)
        for event in bot.events:
            if event[0] == "move" and event[1] in ("携币竖直抬升越障高度", "在高处旋转180度", "保持高位越过障碍到B上方", "下降至B释放高度"):
                self.assertTrue(event[2])
            if event[0] == "move" and event[1] == "释放后竖直抬起验证":
                self.assertFalse(event[2])
        self.assertLessEqual(record["grip_started_s"], record["release_finished_s"])
        self.assertLessEqual(record["release_finished_s"], record["verification_lift_finished_s"])
        self.assertFalse(bot.on)

    def test_cooperative_stop_clears_queue_without_releasing_coin(self):
        events = []
        class API:
            def SetQueuedCmdForceStopExec(self):
                events.append("stop")
            def SetQueuedCmdClear(self):
                events.append("clear")
        bot = demo.CoinRobot.__new__(demo.CoinRobot)
        bot.api = API()
        with tempfile.TemporaryDirectory() as folder:
            flag = Path(folder) / "STOP_REQUESTED";flag.write_text("stop")
            with patch.object(demo, "STOP_FILE", flag):
                with self.assertRaises(demo.StopRequested):
                    bot.check_stop()
        self.assertEqual(events, ["stop", "clear"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
