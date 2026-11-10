"""离线几何、SDK 内存布局和执行前检查；不会连接机械臂。"""

import ctypes
import math
import unittest

from geometry import Drawing, Point, bounds, make_pattern, validate_bounds
from dobot_sdk import ARC, PTP, Pose
from plotter import execute_plan


class GeometryTests(unittest.TestCase):
    def test_line_angle_in_degrees(self):
        d = Drawing(Point(10, 20, 5))
        d.draw_line(10, 90)
        self.assertAlmostEqual(d.point.x, 10)
        self.assertAlmostEqual(d.point.y, 30)
        self.assertEqual(d.point.z, 5)

    def test_line_negative_angle(self):
        d = Drawing(Point(0, 0, 5))
        d.draw_line(10, -90)
        self.assertAlmostEqual(d.point.y, -10)

    def test_counterclockwise_arc(self):
        d = Drawing(Point(10, 0, 0))
        d.draw_arc(0, 0, 90)
        self.assertAlmostEqual(d.point.x, 0)
        self.assertAlmostEqual(d.point.y, 10)
        self.assertAlmostEqual(d.commands[0].middle.x, math.sqrt(50))

    def test_clockwise_arc(self):
        d = Drawing(Point(10, 0, 0))
        d.draw_arc(0, 0, -90)
        self.assertAlmostEqual(d.point.y, -10)
        self.assertLess(d.commands[0].middle.y, 0)

    def test_full_circle_four_distinct_arcs(self):
        d = Drawing(Point(10, 0, 5, -30))
        d.draw_circle(0, 0)
        self.assertEqual(len(d.commands), 4)
        self.assertAlmostEqual(d.point.x, 10)
        self.assertAlmostEqual(d.point.y, 0)
        for c in d.commands:
            self.assertNotEqual(c.start, c.end)
            self.assertEqual(c.end.r, -30)
            for p in c.samples():
                self.assertAlmostEqual(math.hypot(p.x, p.y), 10)

    def test_arbitrary_arc_and_multiple_turns(self):
        d = Drawing(Point(10, 0, 0))
        d.draw_arc(0, 0, -450)
        self.assertEqual(len(d.commands), 5)
        self.assertAlmostEqual(d.point.y, -10)

    def test_zero_radius_rejected(self):
        d = Drawing(Point(1, 2, 3))
        with self.assertRaises(ValueError):
            d.draw_circle(1, 2)

    def test_nonfinite_and_negative_values_rejected(self):
        d = Drawing(Point(1, 2, 3))
        for value in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                d.draw_line(value, 0)

    def test_exact_arc_extrema_in_bounds(self):
        d = Drawing(Point(10 * math.cos(math.radians(7)), 10 * math.sin(math.radians(7)), 0))
        d.draw_arc(0, 0, 89)
        self.assertAlmostEqual(bounds(d.commands)["y"][1], 10)

    def test_all_pattern_stays_in_paper_box(self):
        d = make_pattern()
        box = validate_bounds(d.commands, {"x": [170, 230], "y": [-80, 80], "z": [0, 10]})
        self.assertGreaterEqual(box["x"][0], 180 - 1e-7)
        self.assertLessEqual(box["y"][1], 70 + 1e-7)
        self.assertFalse(d.commands[-1].pen_down)
        self.assertAlmostEqual(d.point.z, 10)

    def test_flower_petals_close(self):
        d = make_pattern("arc", origin=(200, 0), repeats=10)
        arcs = [c for c in d.commands if c.kind == "arc"]
        self.assertEqual(len(arcs), 40)
        for k in range(10):
            last = arcs[k * 4 + 3].end
            self.assertAlmostEqual(last.x, 200)
            self.assertAlmostEqual(last.y, 0)

    def test_polygon_returns_to_start(self):
        d = make_pattern("polygon", repeats=10)
        strokes = [c for c in d.commands if c.pen_down]
        self.assertEqual(len(strokes), 60)
        for k in range(10):
            self.assertAlmostEqual(strokes[k * 6 + 5].end.x, 200)
            self.assertAlmostEqual(strokes[k * 6 + 5].end.y, 0)

    def test_oversized_pattern_rejected(self):
        with self.assertRaises(ValueError):
            validate_bounds(make_pattern(size=100).commands,
                            {"x": [170, 230], "y": [-80, 80], "z": [0, 10]})

    def test_sdk_packed_structures(self):
        self.assertEqual(ctypes.sizeof(PTP), 17)
        self.assertEqual(PTP.x.offset, 1)
        self.assertEqual(ctypes.sizeof(ARC), 32)
        self.assertEqual(ctypes.sizeof(Pose), 32)


class FakeDobot:
    def __init__(self):
        self.events = []

    def ensure_no_alarms(self):
        pass

    def offset(self):
        return dict(x=0, y=0, z=0)

    def pose(self):
        return dict(x=200, y=0, z=5, r=0)

    def prepare_motion(self, speed, acceleration):
        self.events.append(("prepare", speed, acceleration))

    def move(self, point):
        self.events.append(("move", point))

    def arc(self, middle, end):
        self.events.append(("arc", middle, end))


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.bot = FakeDobot()
        self.config = {"paper_z": 0, "r": 0, "tool_offset": dict(x=0, y=0, z=0),
                       "limits": {"x": [170, 230], "y": [-80, 80], "z": [-0.5, 40]}}

    def test_lift_before_horizontal_move(self):
        d = make_pattern("circle", repeats=3)
        execute_plan(self.bot, d, self.config, 15, 30)
        self.assertEqual(self.bot.events[1], ("move", Point(200, 0, 10, 0)))
        self.assertEqual(sum(e[0] == "arc" for e in self.bot.events), 12)
        self.assertEqual(self.bot.events[-1][1].z, 10)

    def test_invalid_bounds_send_no_commands(self):
        with self.assertRaises(ValueError):
            execute_plan(self.bot, make_pattern(size=100), self.config, 15, 30)
        self.assertEqual(self.bot.events, [])

    def test_changed_tool_offset_sends_no_commands(self):
        self.config["tool_offset"]["x"] = 10
        with self.assertRaises(ValueError):
            execute_plan(self.bot, make_pattern(), self.config, 15, 30)
        self.assertEqual(self.bot.events, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
