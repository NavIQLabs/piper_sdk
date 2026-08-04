#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space impedance control demo.

Holds the current pose with a tunable stiffness/damping, then steps a couple
of joints. Gravity compensation uses the fitted model if provided.

Usage:
    python demo_joint_impedance.py --can can0 --K 25 --D 1.2 \
        --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    JointImpedanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Joint impedance demo')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--K', type=float, default=25.0)
    parser.add_argument('--D', type=float, default=1.2)
    parser.add_argument('--gravity', default=None)
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
        piper, rate_hz=args.rate, K=[args.K] * 6, D=[args.D] * 6, model=model)

    try:
        ctrl.enable()
        q0 = list(ctrl.state.q)
        print("holding at %s ..." % [round(v, 3) for v in q0])
        time.sleep(2.0)

        target = list(q0)
        for step_idx, (j, amp) in enumerate([(0, 0.2), (1, 0.15), (3, 0.1)]):
            target[j] += amp
            ctrl.set_target(target)
            print("step %d -> joint %d += %.2f" % (step_idx + 1, j + 1, amp))
            time.sleep(2.0)
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
