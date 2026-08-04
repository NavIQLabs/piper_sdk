#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Gravity compensation calibration.

Moves the arm (in firmware position mode) through a random set of static
configurations within the joint limits, records the current-based torque
estimate at each pose, and fits a per-joint Fourier gravity model.

Usage:
    python calibrate_gravity.py --can can0 --poses 60 --order 2 \
        --out gravity_coeffs.json

NOTE: hardware required. The arm will move on its own during the sweep - keep
the workspace clear and the e-stop within reach.
'''

import argparse
import math
import random
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    fit_fourier_gravity, save_gravity_model,
    DEFAULT_JOINT_LIMITS,
)

DEG_TO_COUNT = 180.0 / math.pi / 0.001   # JointCtrl unit: 0.001 degrees


def sample_tau(piper, window=0.4):
    '''Average the torque estimate over `window` seconds at a static pose.'''
    sums = [0.0] * 6
    counts = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < window:
        motor = piper.GetArmHighSpdInfoMsgs()
        for i in range(1, 7):
            sums[i - 1] += getattr(motor, 'motor_%d' % i).effort
        counts += 1
        time.sleep(0.005)
    return [s / counts for s in sums]


def sample_q(piper):
    j = piper.GetArmJointMsgs().joint_state
    deg = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6]
    return [d * 0.001 * math.pi / 180.0 for d in deg]


def move_to(piper, q, spd=40, settle=1.2):
    counts = [int(v * DEG_TO_COUNT) for v in q]
    piper.MotionCtrl_2(0x01, 0x01, spd, 0x00)
    # stream the command for the whole settle window - some firmware builds
    # require continuous JointCtrl frames to hold the target.
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < settle:
        piper.JointCtrl(*counts)
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description='Fit Piper gravity compensation model')
    parser.add_argument('--can', default='can0', help='CAN interface name')
    parser.add_argument('--poses', type=int, default=60, help='number of static poses')
    parser.add_argument('--order', type=int, default=2, help='Fourier harmonics per joint')
    parser.add_argument('--out', default='gravity_coeffs.json', help='output JSON path')
    parser.add_argument('--margin', type=float, default=0.3,
                        help='radians of safety margin from each joint limit')
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()
    while not piper.EnablePiper():
        time.sleep(0.01)

    # randomized poses within the joint limits, ordered by nearest neighbour so
    # consecutive moves stay small and smooth (no large jerky sweeps)
    poses = []
    for _ in range(args.poses):
        q = []
        for lo, hi in DEFAULT_JOINT_LIMITS:
            q.append(random.uniform(lo + args.margin, hi - args.margin))
        poses.append(q)

    ordered = [poses.pop()]
    while poses:
        last = ordered[-1]
        i = min(range(len(poses)),
                key=lambda i: sum((poses[i][j] - last[j]) ** 2 for j in range(6)))
        ordered.append(poses.pop(i))
    poses = ordered

    # move to a neutral start pose and let it settle
    start = [0.0, 1.2, -1.0, 0.0, 0.0, 0.0]
    move_to(piper, start, settle=2.0)

    samples = []
    try:
        for i, q in enumerate(poses):
            move_to(piper, q)
            tau = sample_tau(piper)
            q_now = sample_q(piper)
            samples.append((q_now, tau))
            print("pose %2d/%d  q=%s  tau=%s" %
                  (i + 1, len(poses),
                   [round(v, 3) for v in q_now],
                   [round(v, 3) for v in tau]))
    finally:
        piper.DisablePiper()

    model = fit_fourier_gravity(samples, order=args.order)
    save_gravity_model(args.out, model)
    print("saved gravity model -> %s (order=%d)" % (args.out, args.order))


if __name__ == "__main__":
    main()
