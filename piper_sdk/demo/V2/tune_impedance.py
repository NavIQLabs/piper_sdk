#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space impedance step-response tuner.

Enables the joint impedance controller with gravity compensation (load the
model fitted by ``calibrate_gravity.py``), waits for it to settle at the
current pose, then steps joint 1 by a small angle and prints the tracked
response so you can tune K/D.

Usage:
    python tune_impedance.py --can can0 --K 25 --D 1.2 --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    JointImpedanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Tune joint impedance gains')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--K', type=float, default=25.0, help='stiffness N*m/rad')
    parser.add_argument('--D', type=float, default=1.2, help='damping N*m*s/rad')
    parser.add_argument('--gravity', default=None,
                        help='path to fitted gravity model JSON (optional)')
    parser.add_argument('--step', type=float, default=0.2,
                        help='joint-1 step amplitude [rad]')
    parser.add_argument('--rate', type=float, default=500.0)
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    model = ArmModel()
    if args.gravity:
        gm = FourierGravityModel()
        gm.load(args.gravity)
        model = ArmModel(gravity_model=gm)

    ctrl = JointImpedanceController(
        piper, rate_hz=args.rate, K=[args.K] * 6, D=[args.D] * 6,
        model=model)

    try:
        ctrl.enable()
        q0 = ctrl.state.q
        print("holding at q0 = %s ..." % [round(v, 3) for v in q0])
        time.sleep(2.0)

        target = list(q0)
        target[0] += args.step
        ctrl.set_target(target)
        print("stepping joint 1 +%.3f rad, recording response..." % args.step)

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < 3.0:
            q = ctrl.state.q
            print("t=%.3f  q1=%.4f (des %.4f)  qd1=%.4f  tau1=%.4f" %
                  (time.perf_counter() - t0, q[0], target[0],
                   ctrl.state.qd[0], ctrl.state.tau[0]))
            time.sleep(0.02)
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
