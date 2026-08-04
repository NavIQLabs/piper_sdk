#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint torque control demo.

Commands a small open-loop torque profile per joint (kp = kd = 0). Gravity
compensation must be enabled via a fitted model or the arm will sag.

Usage:
    python demo_joint_torque.py --can can0 --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    JointTorqueController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Joint torque demo')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--gravity', default=None)
    parser.add_argument('--tau', type=float, default=0.5,
                        help='torque amplitude [N*m]')
    parser.add_argument('--rate', type=float, default=500.0)
    args = parser.parse_args()

    piper = C_PiperInterface_V2(args.can)
    piper.ConnectPort()

    model = ArmModel()
    if args.gravity:
        gm = FourierGravityModel()
        gm.load(args.gravity)
        model = ArmModel(gravity_model=gm)

    ctrl = JointTorqueController(piper, rate_hz=args.rate, model=model)

    try:
        ctrl.enable()
        time.sleep(1.0)
        print("applying +%.2f N*m to joints 1-3 for 2s ..." % args.tau)
        ctrl.set_torque([args.tau] * 3 + [0.0] * 3)
        time.sleep(2.0)
        print("zeroing torque ...")
        ctrl.set_torque([0.0] * 6)
        time.sleep(2.0)
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
