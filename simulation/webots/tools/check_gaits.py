"""
Offline gait check (no Webots needed):  python3 tools/check_gaits.py
Reports, per gait: peak joint speed, joint-limit use, minimum stability margin (full-body centre of
mass incl. leg masses vs. support polygon) and peak static servo torque.
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "controllers", "spotmicro_gaits"))
import gaits
from gaits import *

M = dict(body=1.10, shoulder=0.06, thigh=0.09, shank=0.05)
LIM = dict(hip_abd=(-0.7, 0.7), hip_pitch=(-1.6, 1.6), knee=(-2.6, 0.2))
CEN = dict(hip_abd=0, hip_pitch=0, knee=-75)
TORQUE, VMAX = 1.2, 6.0
G = 9.81


def rot_x(t, v):
    return np.array([v[0], v[1] * math.cos(t) - v[2] * math.sin(t), v[1] * math.sin(t) + v[2] * math.cos(t)])


def leg_points(t1, t2, t3, side):
    hip = rot_x(t1, [0, side * L1, 0])
    knee = hip + rot_x(t1, [-L2 * math.sin(t2), 0, -L2 * math.cos(t2)])
    foot = np.array(leg_fk(t1, t2, t3, side))
    return hip, knee, foot


def body_frame(leg, p):
    return np.array([FRONT[leg] * BODY_L / 2, SIDE[leg] * BODY_W / 2, 0.0]) + p


def com_and_feet(pose):
    tot = M["body"]; com = np.zeros(3); feet = {}
    for l in LEGS:
        hip, knee, foot = leg_points(*pose[l], SIDE[l])
        for m, p in ((M["shoulder"], hip / 2), (M["thigh"], (hip + knee) / 2), (M["shank"], (knee + foot) / 2)):
            com += m * body_frame(l, p); tot += m
        feet[l] = body_frame(l, foot)
    return com / tot, feet, tot


def support_forces(com, feet, on, total_mass):
    pts = np.array([feet[l][:2] for l in on])
    A = np.vstack([np.ones(len(on)), pts[:, 0], pts[:, 1]])
    W = total_mass * G
    f = np.linalg.lstsq(A, W * np.array([1, com[0], com[1]]), rcond=None)[0]
    return dict(zip(on, f))


def joint_torques(leg, pose, fz):
    th = np.array(pose[leg]); side = SIDE[leg]; J = np.zeros((3, 3)); e = 1e-6
    for k in range(3):
        d = th.copy(); d[k] += e
        J[:, k] = (np.array(leg_fk(*d, side)) - np.array(leg_fk(*th, side))) / e
    return np.abs(J.T @ np.array([0, 0, fz]))


def margin(com, feet, on):
    """signed distance from COM to the support polygon edges (negative = outside); uses force positivity for >=3 feet."""
    pts = [feet[l][:2] for l in on]
    if len(pts) < 3:
        return -1.0
    c = np.mean(pts, axis=0)
    pts = sorted(pts, key=lambda p: math.atan2(p[1] - c[1], p[0] - c[0]))
    best = 1e9
    for i in range(len(pts)):
        a, b = np.array(pts[i]), np.array(pts[(i + 1) % len(pts)])
        e = b - a; n = np.array([-e[1], e[0]]) / (np.linalg.norm(e) + 1e-12)
        best = min(best, float(np.dot(com[:2] - a, n)))
    return best


def run(seq, label, verbose=True):
    e = GaitEngine(); dt = 0.008; prev = None
    vmax = 0; lim = 0; srv = 0; mg = 1e9; tq = np.zeros(3); steady = False
    for g, secs in seq:
        e.set_command(gait=g)
        for _ in range(int(secs / dt)):
            pose = e.step(dt)
            cur = {(l, j): a for l in LEGS for j, a in zip(JOINTS, pose[l])}
            for (l, j), a in cur.items():
                lim += not LIM[j][0] <= a <= LIM[j][1]
                srv += not 0 <= 90 + math.degrees(a) - CEN[j] <= 180
            if prev: vmax = max(vmax, max(abs(cur[k] - prev[k]) for k in cur) / dt)
            prev = cur
            if e.target == g == e.active and e.amp > 0.99:
                com, feet, tm = com_and_feet(pose)
                on = [l for l in LEGS if feet[l][2] < -e.height + 0.003]
                if len(on) >= 3:
                    mg = min(mg, margin(com, feet, on))
                    f = support_forces(com, feet, on, tm)
                    for l in on: tq = np.maximum(tq, joint_torques(l, pose, f[l]))
    if verbose:
        print(f"{label:6s} speed {vmax:4.2f} rad/s | limit viol {lim} | servo-range viol {srv} | COM margin {mg*1000 if mg<1e8 else float('nan'):5.1f} mm | "
              f"static torque abd/pitch/knee {tq[0]:.2f}/{tq[1]:.2f}/{tq[2]:.2f} N.m (servo {TORQUE})")
    return vmax, mg, tq


if __name__ == "__main__":
    for g in ("crawl", "walk", "trot"):
        run([(g, 16)], g)
    run([("walk", 8), ("crawl", 12), ("trot", 10), ("walk", 8), ("stand", 4), ("trot", 8), ("crawl", 10)], "mixed")
