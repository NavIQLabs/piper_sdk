#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Cartesian admittance control demo.

Task-space virtual mass-damper: push the end-effector and it yields in the
direction of the applied wrench. Requires a fitted gravity model.

Usage:
    python demo_cartesian_admittance.py --can can0 --gravity gravity_coeffs.json
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    CartesianAdmittanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Cartesian admittance demo')
    parser.add_argument('--can', default='can0')
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

    ctrl = CartesianAdmittanceController(piper, rate_hz=args.rate, model=model)

    try:
        ctrl.enable()
        print("cartesian admittance active - guide the end-effector by hand.")
        print("Ctrl-C to stop.")
        while True:
            x = ctrl.state.pose
            print("x=%s  F_ext=%s" % (
                [round(v, 4) for v in x],
                [round(v, 2) for v in ctrl._F_ext]))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
