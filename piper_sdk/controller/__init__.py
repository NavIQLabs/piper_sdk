#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Realtime force/compliance controllers for the Piper arm, built on the MIT
(per-joint PD + torque feedforward) primitive.

Implemented controller types:

* :class:`JointImpedanceController`     - task/joint stiffness + damping
* :class:`JointTorqueController`        - open-loop joint torque reference
* :class:`JointAdmittanceController`    - external-torque-driven virtual mass-damper
* :class:`CartesianImpedanceController` - task-space stiffness via Jacobian
* :class:`CartesianAdmittanceController`- external-wrench-driven task-space motion

Shared infrastructure:

* :class:`BaseController`   - MIT lifecycle + safety clamps + realtime loop
* :class:`RtLoop`           - fixed-rate control loop with watchdog
* :class:`ArmState`         - filtered joint/velocity/torque/pose estimator
* :class:`ArmModel`         - FK, numeric Jacobian, pseudo-inverse, limits
* :class:`FourierGravityModel` - fitted gravity compensation model

Calibration and tuning scripts live under ``demo/V2/`` (``calibrate_gravity.py``,
``tune_impedance.py``, ``tune_admittance.py``).
'''

from .mathx import (
    mat_mul,
    mat_vec,
    transpose,
    identity,
    diag,
    mat_add,
    vec_add,
    vec_sub,
    vec_scale,
    vec_dot,
    vec_norm,
    mat_inv,
    mat_pinv_damped,
    clip,
    clip_vec,
    svd_extreme,
    condition_number,
    saturate_torque_rate,
    quat_slerp,
)
from .model import (
    ArmModel,
    FourierGravityModel,
    SigmoidFrictionModel,
    FirstOrderLPF,
    DEFAULT_JOINT_LIMITS,
    DEFAULT_VELOCITY_LIMITS,
)
from .state import ArmState
from .rt_loop import RtLoop
from .base import BaseController
from .calibration import fit_fourier_gravity, save_gravity_model, load_gravity_model
from .joint_impedance import JointImpedanceController
from .joint_torque import JointTorqueController
from .joint_admittance import JointAdmittanceController
from .cartesian_impedance import CartesianImpedanceController
from .cartesian_admittance import CartesianAdmittanceController

__all__ = [
    'BaseController',
    'RtLoop',
    'ArmState',
    'ArmModel',
    'FourierGravityModel',
    'SigmoidFrictionModel',
    'FirstOrderLPF',
    'fit_fourier_gravity',
    'save_gravity_model',
    'load_gravity_model',
    'JointImpedanceController',
    'JointTorqueController',
    'JointAdmittanceController',
    'CartesianImpedanceController',
    'CartesianAdmittanceController',
    'DEFAULT_JOINT_LIMITS',
    'DEFAULT_VELOCITY_LIMITS',
    # mathx re-exports
    'mat_mul',
    'mat_vec',
    'mat_pinv_damped',
    'condition_number',
    'clip',
    'clip_vec',
    'saturate_torque_rate',
]
