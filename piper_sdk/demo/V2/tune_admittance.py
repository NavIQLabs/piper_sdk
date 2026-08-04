#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space admittance tuner.

Enables the joint admittance controller (gravity compensation active if the
fitted model is provided) and lets you drag the arm by hand while it prints
the external torque estimate and the admittance response. Tune M, D, and the
friction offset here.

Usage:
    python tune_admittance.py --can can0 --M 0.5 --D 2.0 \
        --gravity gravity_coeffs.json

NOTE: hardware required. With gravity compensation disabled the arm will sag.
'''

import argparse
import time

from piper_sdk import C_PiperInterface_V2
from piper_sdk.controller import (
    JointAdmittanceController, ArmModel, FourierGravityModel,
)

def main():
    parser = argparse.ArgumentParser(description='Tune joint admittance')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--M', type=float, default=0.5, help='virtual mass kg*m^2')
    parser.add_argument('--D', type=float, default=2.0, help='virtual damping')
    parser.add_argument('--K-inner', type=float, default=150.0)
    parser.add_argument('--D-inner', type=float, default=5.0)
    parser.add_argument('--gravity', default=None)
    parser.add_argument('--friction', nargs=6, type=float,
                        default=[0.0] * 6, help='coulomb friction N*m per joint')
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
        M=[args.M] * 6, D=[args.D] * 6,
        K_inner=args.K_inner, D_inner=args.D_inner,
        tau_friction=args.friction, model=model)

    try:
        ctrl.enable()
        print("admittance enabled. Drag the arm by hand. Ctrl-C to stop.")
        print("note: measured torque is current-based -> noisy by nature.")
        while True:
            print("tau_ext=%s  q=%s  qd=%s" % (
                [round(v, 3) for v in ctrl._tau_ext],
                [round(v, 3) for v in ctrl.state.q],
                [round(v, 3) for v in ctrl.state.qd]))
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ctrl.disable()

if __name__ == "__main__":
    main()
