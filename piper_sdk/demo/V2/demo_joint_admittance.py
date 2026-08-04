#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint admittance control demo.

The arm behaves like a virtual mass-damper in joint space: push it and it
yields; it returns to the last set target when released. Requires a fitted
gravity model for sensible behavior.

Usage:
    python demo_joint_admittance.py --can can0 --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    JointAdmittanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Joint admittance demo')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--M', type=float, default=0.5)
    parser.add_argument('--D', type=float, default=2.0)
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

    ctrl = JointAdmittanceController(
        piper, rate_hz=args.rate,
        M=[args.M] * 6, D=[args.D] * 6, model=model)

    try:
        ctrl.enable()
        print("admittance active - guide the arm by hand. Ctrl-C to stop.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
