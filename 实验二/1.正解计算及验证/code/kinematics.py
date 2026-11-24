"""Dobot Magician 标准末端配置的正解和几何逆解，长度 mm，输入输出角度 °。"""

from math import acos, atan2, cos, degrees, hypot, pi, radians, sin

import numpy as np

L1, L2, L3 = 135.0, 147.0, 59.7
JOINTS = ("j1", "j2", "j3", "j4")
CARTESIAN = ("x", "y", "z", "r")
JOINT_LIMITS = ((-125, 125), (-5, 90), (-15, 90), (-150, 150))


def angle_difference(angle, reference):
    return (angle - reference + 180) % 360 - 180


def dh(theta, a, alpha=0):
    """标准 D-H 变换，本实验各段 d 均为零。"""
    c, s, ca, sa = cos(theta), sin(theta), cos(alpha), sin(alpha)
    return np.array([
        [c, -s * ca, s * sa, a * c],
        [s, c * ca, -c * sa, a * s],
        [0, sa, ca, 0],
        [0, 0, 0, 1],
    ])


def transforms(j1, j2, j3, j4):
    q1, q2, q3, q4 = map(radians, (j1, j2, j3, j4))
    a1 = dh(q1, 0, pi / 2)
    a2 = dh(pi / 2 - q2, L1)
    a3 = dh(q2 - q3 - pi / 2, L2)
    a4 = dh(q3, L3, pi / 2) @ dh(-q4, 0)
    return a1, a2, a3, a4, a1 @ a2 @ a3 @ a4


def forward(j1, j2, j3, j4):
    transform = transforms(j1, j2, j3, j4)[-1]
    x, y, z = transform[:3, 3]
    r = degrees(atan2(transform[1, 0], transform[0, 0]))
    return float(x), float(y), float(z), r


def inverse(x, y, z, r):
    radius = hypot(x, y)
    if radius < 1e-9:
        raise ValueError("末端位于底座轴线上，J1 不唯一")
    j1 = degrees(atan2(y, x))
    p = radius - L3
    cosine = (p * p + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if not -1 - 1e-12 <= cosine <= 1 + 1e-12:
        raise ValueError("腕点超出平面两连杆工作范围")
    elbow = acos(max(-1, min(1, cosine)))
    for delta in (-elbow, elbow):
        shoulder = atan2(z, p) - atan2(L2 * sin(delta), L1 + L2 * cos(delta))
        solution = (
            j1,
            angle_difference(90 - degrees(shoulder), 0),
            angle_difference(-degrees(shoulder + delta), 0),
            angle_difference(r, j1),
        )
        if all(low - 1e-6 <= q <= high + 1e-6 for q, (low, high) in zip(solution, JOINT_LIMITS)):
            return solution
    raise ValueError("逆解候选均超出关节范围")
