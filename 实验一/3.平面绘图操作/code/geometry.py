"""实验一·四、3：平面绘图几何；角度单位为度，长度单位为 mm。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float
    r: float = 0.0


@dataclass(frozen=True)
class Command:
    kind: str
    start: Point
    end: Point
    pen_down: bool
    middle: Point | None = None
    center: tuple[float, float] | None = None
    sweep: float = 0.0

    def samples(self):
        """预览和范围检查共用轨迹采样，包含起点和终点。"""
        if self.kind != "arc":
            return [self.start, self.end]
        cx, cy = self.center
        radius = math.hypot(self.start.x - cx, self.start.y - cy)
        a0 = math.atan2(self.start.y - cy, self.start.x - cx)
        count = max(2, math.ceil(abs(self.sweep) / 2))
        points = [Point(cx + radius * math.cos(a0 + math.radians(self.sweep) * i / count),
                        cy + radius * math.sin(a0 + math.radians(self.sweep) * i / count),
                        self.start.z, self.start.r) for i in range(count + 1)]
        # 纳入精确极值，保证范围检查覆盖弧段的整个包围盒。
        angles = [a0 + math.radians(self.sweep) * i / count for i in range(count + 1)]
        low, high = min(angles), max(angles)
        for k in range(math.floor(low / (math.pi / 2)), math.ceil(high / (math.pi / 2)) + 1):
            a = k * math.pi / 2
            if low <= a <= high:
                points.append(Point(cx + radius * math.cos(a), cy + radius * math.sin(a),
                                    self.start.z, self.start.r))
        # 按沿弧段前进的角度排序，预览不会因极值点而产生折返。
        def progress(p):
            a = math.atan2(p.y - cy, p.x - cx)
            delta = (a - a0) % (2 * math.pi) if self.sweep > 0 else (a0 - a) % (2 * math.pi)
            return 0.0 if delta > 2 * math.pi - 1e-9 else delta
        points.sort(key=progress)
        return points


class Drawing:
    """用当前位置连续构建 MOVL / ARC 指令；构图阶段不连接机械臂。"""

    def __init__(self, start: Point, lift: float = 10):
        if not math.isfinite(lift) or lift <= 0:
            raise ValueError("抬笔高度必须为正数")
        if not all(math.isfinite(v) for v in (start.x, start.y, start.z, start.r)):
            raise ValueError("起点坐标必须是有限数")
        self.point = start
        self.paper_z = start.z
        self.lift = lift
        self.down = True
        self.commands: list[Command] = []

    def _line_to(self, point: Point):
        if math.dist((self.point.x, self.point.y, self.point.z),
                     (point.x, point.y, point.z)) > 1e-8:
            self.commands.append(Command("line", self.point, point, self.down))
        self.point = point

    def pen_up(self):
        self.down = False
        p = self.point
        self._line_to(Point(p.x, p.y, self.paper_z + self.lift, p.r))

    def pen_down(self):
        p = self.point
        self._line_to(Point(p.x, p.y, self.paper_z, p.r))
        self.down = True

    def place(self, x: float, y: float):
        """抬笔、水平移动、落笔到新图形的起点。"""
        self.pen_up()
        self._line_to(Point(x, y, self.paper_z + self.lift, self.point.r))
        self.pen_down()

    def draw_line(self, length: float, angle_deg: float):
        """从当前位置画指定长度的直线，方向相对 +X，逆时针为正。"""
        if not all(math.isfinite(v) for v in (length, angle_deg)) or length < 0:
            raise ValueError("直线长度须非负，长度和角度须为有限数")
        a = math.radians(angle_deg)
        p = self.point
        self._line_to(Point(p.x + length * math.cos(a),
                            p.y + length * math.sin(a), p.z, p.r))

    def draw_arc(self, center_x: float, center_y: float, angle_deg: float):
        """从当前位置绕指定圆心画弧，逆时针为正，顺时针为负。

        每段最多 90°，用起点、中间点、终点生成 ARC；整圆也可安全分段。
        """
        if not all(math.isfinite(v) for v in (center_x, center_y, angle_deg)):
            raise ValueError("圆心和圆弧角度须为有限数")
        p = self.point
        dx, dy = p.x - center_x, p.y - center_y
        if math.hypot(dx, dy) < 1e-6:
            raise ValueError("圆心与当前位置重合，无法确定半径")
        if abs(angle_deg) < 1e-9:
            return
        count = math.ceil(abs(angle_deg) / 90)
        sweep = angle_deg / count
        initial_angle = math.atan2(dy, dx)
        radius = math.hypot(dx, dy)

        def at(a):
            return Point(center_x + radius * math.cos(a),
                         center_y + radius * math.sin(a), p.z, p.r)

        for i in range(count):
            middle = at(initial_angle + math.radians(sweep) * (i + 0.5))
            end = at(initial_angle + math.radians(sweep) * (i + 1))
            self.commands.append(Command("arc", self.point, end, self.down, middle,
                                         (center_x, center_y), sweep))
            self.point = end

    def draw_circle(self, center_x: float, center_y: float):
        """以当前位置为起点，绕指定圆心逆时针画整圆。"""
        self.draw_arc(center_x, center_y, 360)


def make_pattern(kind="all", origin=(200.0, 0.0), z=0.0, r=0.0,
                 size=20.0, repeats=10, lift=10.0):
    """三个旋转重复图案；origin 为整组图案的中心。

    size 为单个花纹相对公共中心的最大伸出距离。
    all 时沿机器人 Y 轴排列三组，中心距为 2*size+10 mm。
    """
    if kind not in ("polygon", "arc", "circle", "all"):
        raise ValueError("未知图案类型")
    if not math.isfinite(size) or size <= 0 or not 3 <= repeats <= 100:
        raise ValueError("尺寸须为正数，重复次数须在 3～100 之间")
    d = Drawing(Point(*origin, z, r), lift)
    names = ["polygon", "arc", "circle"] if kind == "all" else [kind]
    gap = 2 * size + 10
    for index, name in enumerate(names):
        ox = origin[0]
        oy = origin[1] + (index - 1) * gap if kind == "all" else origin[1]
        for k in range(repeats):
            angle = 360 * k / repeats
            a = math.radians(angle)
            ux, uy = math.cos(a), math.sin(a)
            if name == "polygon":
                # 同一正六边形绕公共顶点旋转，呼应指导书图 5。
                d.place(ox, oy)
                side = size / 2
                for edge in range(6):
                    d.draw_line(side, angle - 60 + 60 * edge)
            elif name == "arc":
                # 两段 120° 圆弧围成花瓣：公共中心 -> 外端 -> 公共中心。
                tip = (ox + size * ux, oy + size * uy)
                mx, my = (ox + tip[0]) / 2, (oy + tip[1]) / 2
                h = size / (2 * math.sqrt(3))
                c1 = (mx - uy * h, my + ux * h)
                c2 = (mx + uy * h, my - ux * h)
                d.place(ox, oy)
                d.draw_arc(*c1, 120)
                d.draw_arc(*c2, 120)
            else:
                # 所有圆通过公共中心，每个圆的圆心沿半径 size/2 旋转。
                d.place(ox, oy)
                d.draw_circle(ox + size / 2 * ux, oy + size / 2 * uy)
    d.pen_up()
    return d


def bounds(commands):
    points = [p for c in commands for p in c.samples()]
    if not points:
        raise ValueError("轨迹为空")
    return {axis: (min(getattr(p, axis) for p in points),
                   max(getattr(p, axis) for p in points)) for axis in ("x", "y", "z")}


def validate_bounds(commands, limits):
    box = bounds(commands)
    for axis in ("x", "y", "z"):
        lo, hi = limits[axis]
        if not all(math.isfinite(v) for v in (lo, hi)) or lo >= hi:
            raise ValueError(f"{axis.upper()} 范围无效")
        if box[axis][0] < lo - 1e-6 or box[axis][1] > hi + 1e-6:
            raise ValueError(f"{axis.upper()} 轨迹范围 {box[axis]} 超出设定范围 {(lo, hi)}")
    return box
