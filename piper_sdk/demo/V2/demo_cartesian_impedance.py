#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Cartesian impedance control demo.

Adds task-space stiffness/damping at the end-effector and offsets the target
pose, so the arm resists displacement from the virtual spring equilibrium.

Usage:
    python demo_cartesian_impedance.py --can can0 --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    CartesianImpedanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Cartesian impedance demo')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--K', nargs=6, type=float,
                        default=[300.0, 300.0, 300.0, 10.0, 10.0, 10.0])
    parser.add_argument('--D', nargs=6, type=float,
                        default=[30.0, 30.0, 30.0, 3.0, 3.0, 3.0])
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

    ctrl = CartesianImpedanceController(
        piper, rate_hz=args.rate, K=args.K, D=args.D, model=model)

    try:
        ctrl.enable()
        x0 = list(ctrl.state.pose)
        print("holding at x0 = %s ..." % [round(v, 4) for v in x0])
        time.sleep(2.0)

        target = list(x0)
        target[1] += 0.05   # +5 cm along y
        ctrl.set_target(target)
        print("offset target +5 cm along y ...")
        time.sleep(3.0)

        ctrl.set_target(x0)
        print("returning to x0 ...")
        time.sleep(3.0)
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
