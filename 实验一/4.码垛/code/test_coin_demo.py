"""十枚连续码垛的离线验证；不连接串口、不发送运动。"""
from contextlib import redirect_stdout
import copy
import io
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

    def test_current_ten_coin_calibration(self):
        self.assertEqual(self.config["port"], "COM8")
        self.assertEqual(self.plan["coin_count"], 10)
        self.assertAlmostEqual(self.plan["distance_xy_mm"], 83.569797008, places=6)
        self.assertAlmostEqual(self.plan["coin_thickness_mm"] * 10, 18.70079803466797)

    def test_pending_B_after_reset_blocks_plan(self):
        a = copy.deepcopy(self.a);a["destination_recapture_pending"] = True
        with self.assertRaises(ValueError):
            demo.build_plan(a, self.b, self.config)

    def test_source_descends_one_thickness_per_coin(self):
        levels = [c["pick_z_mm"] for c in self.plan["cycles"]]
        self.assertEqual(levels[0], self.a["pose"]["z"])
        for first, second in zip(levels, levels[1:]):
            self.assertAlmostEqual(first - second, self.plan["coin_thickness_mm"])
        self.assertAlmostEqual(levels[-1], self.plan["source_table_z_estimate_mm"] + self.plan["coin_thickness_mm"])

    def test_destination_rises_one_thickness_per_coin(self):
        cycles = self.plan["cycles"]
        for first, second in zip(cycles, cycles[1:]):
            self.assertAlmostEqual(second["release_z_mm"] - first["release_z_mm"], self.plan["coin_thickness_mm"])
        self.assertAlmostEqual(cycles[-1]["release_z_mm"],
                               self.plan["destination_full_stack_top_z_mm"] + self.config["release_offset_mm"])

    def test_held_coin_bottom_releases_above_existing_stack(self):
        for c in self.plan["cycles"]:
            bottom = c["release_z_mm"] - self.plan["coin_thickness_mm"]
            self.assertAlmostEqual(bottom - c["destination_existing_top_z_mm"], 1.0)

    def test_held_coin_clears_obstacle_and_full_stack(self):
        bottom = self.plan["travel_z_mm"] - self.plan["coin_thickness_mm"]
        for top in (self.b["pose"]["z"] + self.config["obstacle_height_mm"],
                    self.plan["destination_full_stack_top_z_mm"], self.a["pose"]["z"]):
            self.assertGreaterEqual(bottom + 1e-8, top + self.config["clearance_mm"])

    def test_each_verification_lift_allows_safe_return(self):
        for c in self.plan["cycles"]:
            self.assertGreaterEqual(c["verification_z_mm"] + 1e-8, self.plan["travel_z_mm"])
            self.assertGreaterEqual(c["verification_z_mm"] - c["release_z_mm"] + 1e-8,
                                    self.config["verification_lift_mm"])

    def test_new_pose_selects_clockwise_without_wrist_overflow(self):
        self.assertEqual(self.plan["rotation"]["delta_deg"], -180)
        self.assertTrue(self.plan["rotation"]["feasible"])
        self.assertFalse(next(c for c in self.plan["rotation_candidates"] if c["direction"] == "ccw")["feasible"])
        self.assertGreaterEqual(self.plan["rotation"]["j4_range_deg"][0], -150)
        self.assertLessEqual(self.plan["rotation"]["j4_range_deg"][1], 150)

    def test_rotation_does_not_accumulate_across_coins(self):
        rotations = [s for s in self.plan["steps"] if s["label"] == "在高处旋转180度"]
        self.assertEqual(len(rotations), 10)
        for s in rotations:
            self.assertAlmostEqual(s["point"]["r"] - self.plan["pickup_r_deg"], -180)

    def test_dead_zone_wrist_is_preoriented_before_grip(self):
        with self.assertRaises(ValueError):
            demo.choose_rotation(self.a["pose"]["r"],
                                 (self.a["pose"]["x"], self.a["pose"]["y"]),
                                 (self.b["pose"]["x"], self.b["pose"]["y"]))
        self.assertAlmostEqual(self.plan["pickup_j4_deg"], 90)
        self.assertGreater(abs(self.plan["pregrip_reorientation_deg"]), 1)
        _, steps = demo.start_alignment(self.a["pose"], self.plan, self.config)
        self.assertGreaterEqual(steps[1]["point"]["z"], self.plan["travel_z_mm"])
        self.assertEqual(steps[-1]["point"]["r"], self.plan["pickup_r_deg"])

    def test_empty_tool_restores_original_wrist_each_cycle(self):
        for cycle in self.plan["cycles"]:
            end = cycle["steps"][-1]["point"]
            expected = self.a["pose"]["j4"] if cycle["coin_index"] == self.plan["coin_count"] else self.plan["pickup_j4_deg"]
            self.assertAlmostEqual(end["r"] - self.b["pose"]["j1"], expected)

    def test_empty_returns_are_nine_and_last_stops_at_B(self):
        returns = [s for s in self.plan["steps"] if s["label"] == "空吸盘高位返回A"]
        self.assertEqual([s["coin_index"] for s in returns], list(range(2, 11)))
        last = self.plan["steps"][-1]["point"]
        self.assertEqual((last["x"], last["y"]), (self.b["pose"]["x"], self.b["pose"]["y"]))

    def test_exact_A_prepares_empty_tool_at_high_z_when_needed(self):
        _, steps = demo.start_alignment(self.a["pose"], self.plan, self.config)
        self.assertEqual(len(steps), 3)
        self.assertEqual(steps[0]["point"]["r"], self.a["pose"]["r"])
        self.assertEqual(steps[1]["point"]["r"], self.plan["pickup_r_deg"])

    def test_approximate_A_is_lifted_before_aligning(self):
        current = self.a["pose"].copy();current["x"] += 5;current["z"] += 3;current["r"] -= 2
        _, steps = demo.start_alignment(current, self.plan, self.config)
        self.assertEqual(len(steps), 3)
        self.assertEqual(steps[0]["point"]["x"], current["x"])
        self.assertEqual(steps[0]["point"]["y"], current["y"])
        self.assertGreaterEqual(steps[0]["point"]["z"], self.plan["travel_z_mm"])
        for key in ("x", "y", "z"):
            self.assertEqual(steps[-1]["point"][key], self.a["pose"][key])
        self.assertEqual(steps[-1]["point"]["r"], self.plan["pickup_r_deg"])

    def test_B_is_not_accepted_as_initial_A(self):
        with self.assertRaises(ValueError):
            demo.start_alignment(self.b["pose"], self.plan, self.config)

    def test_start_tolerance_is_radial(self):
        current = self.a["pose"].copy();current["x"] += 12;current["y"] += 12
        with self.assertRaises(ValueError):
            demo.start_alignment(current, self.plan, self.config)

    def test_invalid_coin_thickness_rejected(self):
        for thickness in (0, -1, float("nan"), 6):
            cfg = copy.deepcopy(self.config);cfg["coin_thickness_mm"] = thickness
            with self.assertRaises(ValueError):
                demo.build_plan(self.a, self.b, cfg)

    def test_invalid_count_rejected(self):
        for count in (0, 10.5, True, 101):
            cfg = copy.deepcopy(self.config);cfg["coin_count"] = count
            with self.assertRaises(ValueError):
                demo.build_plan(self.a, self.b, cfg)

    def test_alarm_or_changed_tool_rejected(self):
        bad_b = copy.deepcopy(self.b);bad_b["alarm_bytes"][10] = 1
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, bad_b, self.config)
        bad_b = copy.deepcopy(self.b);bad_b["tool_offset"]["x"] += 10
        with self.assertRaises(ValueError):
            demo.build_plan(self.a, bad_b, self.config)

    def test_explicit_CCW_can_use_opposite_empty_preorientation(self):
        cfg = copy.deepcopy(self.config);cfg["rotation_direction"] = "ccw"
        p = demo.build_plan(self.a, self.b, cfg)
        self.assertEqual(p["rotation"]["delta_deg"], 180)
        self.assertEqual(p["pickup_j4_deg"], -90)

    def test_parameters_readback_must_match(self):
        profile = self.config["motion_profile"]
        values = {k: v for k, v in profile.items() if k != "reference"}
        for k in ("joint_velocity_deg_s", "joint_acceleration_deg_s2"):
            values[k] = [profile[k]] * 4
        demo.verify_applied_parameters(profile, values)
        values["xyz_velocity_mm_s"] = 6
        with self.assertRaises(RuntimeError):
            demo.verify_applied_parameters(profile, values)


class FakeRobot:
    def __init__(self):
        self.events, self.on, self.phase = [], False, ""
    def check_stop(self):
        pass
    def move(self, point):
        self.events.append(("move", self.phase, self.on, point))
    def suction(self, on):
        self.on = on;self.events.append(("suction", self.phase, on))
        return {"enabled": True, "on": on}
    def pause(self, seconds):
        self.events.append(("wait", self.phase, seconds))


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.config = demo.read_json(demo.ROOT / "demo_config.json")
        self.plan = demo.build_plan(demo.read_json(demo.ROOT / "source_A.json"),
                                   demo.read_json(demo.ROOT / "destination_B.json"), self.config)
        self.bot = FakeRobot()
        self.record = {"events": [], "samples": [], "_clock": time.perf_counter()}
    def run_plan(self):
        with redirect_stdout(io.StringIO()):
            demo.run_steps(self.bot, self.plan, self.record)

    def test_all_ten_cycles_complete_and_timed(self):
        self.run_plan()
        self.assertEqual(self.record["coins_completed"], 10)
        self.assertEqual(len(self.record["coins"]), 10)
        for coin in self.record["coins"]:
            self.assertTrue(coin["completed"])
            self.assertLessEqual(coin["grip_started_s"], coin["release_finished_s"])
            self.assertLessEqual(coin["release_finished_s"], coin["verification_lift_finished_s"])
        self.assertLessEqual(self.record["grip_started_s"], self.record["release_finished_s"])

    def test_ten_grips_ten_releases_and_empty_final_tool(self):
        self.run_plan()
        switches = [e[2] for e in self.bot.events if e[0] == "suction"]
        self.assertEqual(switches, [True, False] * 10)
        self.assertFalse(self.bot.on)

    def test_all_horizontal_travel_is_above_obstacle(self):
        self.run_plan()
        prev = demo.Point(**{k: self.plan["source"][k] for k in ("x", "y", "z", "r")})
        for event in self.bot.events:
            if event[0] != "move":continue
            p = event[3]
            if abs(prev.x - p.x) + abs(prev.y - p.y) > 0.01:
                self.assertGreaterEqual(min(prev.z, p.z) + 1e-8, self.plan["travel_z_mm"])
            prev = p

    def test_carry_on_and_return_off_and_verify_after_release(self):
        self.run_plan()
        for e in self.bot.events:
            if e[0] != "move":continue
            if e[1] in ("携币竖直抬升越障高度", "在高处旋转180度", "保持高位越过障碍到B上方", "下降至本层放币高度"):
                self.assertTrue(e[2])
            if e[1] in ("释放后竖直抬起验证", "空吸盘高位返回A", "下降到下一层取币高度"):
                self.assertFalse(e[2])

    def test_partial_cycle_is_not_marked_complete_and_grip_preserved(self):
        class InterruptBot(FakeRobot):
            def pause(self, seconds):
                if self.phase == "等待吸附":raise demo.StopRequested("test stop")
                super().pause(seconds)
        bot = InterruptBot()
        with redirect_stdout(io.StringIO()), self.assertRaises(demo.StopRequested):
            demo.run_steps(bot, self.plan, self.record)
        self.assertTrue(bot.on)
        self.assertFalse(self.record["coins"][0]["completed"])
        self.assertEqual(self.record.get("coins_completed", 0), 0)

    def test_stop_flag_forces_stop_and_clear_without_pump_command(self):
        events = []
        class API:
            def SetQueuedCmdForceStopExec(self):events.append("stop")
            def SetQueuedCmdClear(self):events.append("clear")
        bot = demo.CoinRobot.__new__(demo.CoinRobot);bot.api = API()
        with tempfile.TemporaryDirectory() as folder:
            flag = Path(folder) / "STOP_REQUESTED";flag.write_text("stop")
            with patch.object(demo, "STOP_FILE", flag), self.assertRaises(demo.StopRequested):
                bot.check_stop()
        self.assertEqual(events, ["stop", "clear"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
