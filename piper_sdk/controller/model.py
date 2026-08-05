#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Robot model helpers for the controller subpackage:
gravity compensation, numeric Jacobian, damped pseudo-inverse, joint limits,
and a small first-order low-pass filter.

The gravity model is intentionally pluggable:

* :class:`FourierGravityModel` - Fourier-series gravity model whose
  coefficients are fitted from static torque measurements by
  ``demo/V2/calibrate_gravity.py``. Recommended (no CAD mass data needed).
* ``None`` - gravity compensation can be turned off per-controller.

The numeric Jacobian is computed by finite differences of the SDK's
forward kinematics (``C_PiperForwardKinematics``), returning the geometric
Jacobian (translational + rotational) at the wrist/end-effector.
'''

import math
from ..kinematics.piper_fk import C_PiperForwardKinematics

# Joint angle limits [rad] (matches SDK param manager / angle-limit config).
# index 0 == joint 1.
DEFAULT_JOINT_LIMITS = [
    (-2.6179, 2.6179),
    (0.0, 3.14),
    (-2.967, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.09439, 2.09439),
]

# Velocity limits [rad/s] - conservative defaults used by safety clamping.
DEFAULT_VELOCITY_LIMITS = [2.0, 2.0, 2.0, 2.5, 2.5, 3.0]

class FirstOrderLPF:
    '''
    First-order low-pass filter: ``y += alpha * (x - y)``.

    :param alpha: filter coefficient in [0, 1]; ``alpha = 1`` disables filtering.
    '''
    def __init__(self, alpha=0.2):
        if not (0.0 <= alpha <= 1.0):
            raise ValueError(f"alpha must be in [0, 1], got {alpha}")
        self._alpha = alpha
        self._y = None

    def reset(self, value=0.0):
        self._y = value
        return self._y

    def update(self, x):
        if self._y is None:
            self._y = x
        else:
            self._y += self._alpha * (x - self._y)
        return self._y

    def value(self):
        return self._y

class FourierGravityModel:
    '''
    Gravity torque model ``g(q)`` as a sum of Fourier terms per joint.

    ``g_i(q) = sum_{k=1..N} a_{i,k} * sin(k*q_i) + b_{i,k} * cos(k*q_i)``

    Coefficients are fitted from static torque measurements by
    ``demo/V2/calibrate_gravity.py`` and stored per joint as
    ``[a1, b1, a2, b2, ..., aN, bN]`` (2*order entries per joint).
    '''
    def __init__(self, order=1, coeffs=None):
        self.order = int(order)
        self.coeffs = coeffs

    def set_coeffs(self, coeffs):
        '''Set fitted coefficients (list of 6 lists, each 2*order entries).'''
        self.coeffs = coeffs

    def load(self, path):
        '''
        Load coefficients from a JSON file produced by the calibration script.

        Expected layout: ``{"order": N, "coeffs": [[...x6...] per joint]}``.
        '''
        import json
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        self.order = int(data.get("order", self.order))
        self.coeffs = data["coeffs"]

    def gravity(self, q):
        '''
        Compute the gravity torque vector (length 6, N·m) at configuration q.

        If no coefficients are fitted a zero vector is returned (gravity
        compensation effectively disabled).
        '''
        if self.coeffs is None:
            return [0.0] * 6
        g = [0.0] * 6
        for i in range(6):
            c = self.coeffs[i]
            qi = q[i]
            total = 0.0
            for k in range(self.order):
                total += c[2 * k] * math.sin((k + 1) * qi)
                total += c[2 * k + 1] * math.cos((k + 1) * qi)
            g[i] = total
        return g

class SigmoidFrictionModel:
    '''
    Smooth friction model (adapted from the CRISP controllers).

    Per joint::

        f(qd) = fp1 / (1 + exp(-fp2 * (qd + fp3)))
              - fp1 / (1 + exp(-fp2 * fp3))

    ``fp1`` is the asymptotic friction magnitude [N·m], ``fp2`` the slope near
    zero velocity, ``fp3`` an offset. Unlike a hard ``sign(qd)`` model this is
    smooth at ``qd = 0`` and saturates for large speeds, which gives a
    stabler external-torque estimate in admittance control.
    '''
    def __init__(self, fp1=None, fp2=None, fp3=None):
        '''
        :param fp1: 6 friction magnitudes [N·m] (default 0.1 each).
        :param fp2: 6 slopes [s/rad] (default 100.0 each).
        :param fp3: 6 offsets [rad/s] (default 0.0 each).
        '''
        self.fp1 = [0.1] * 6 if fp1 is None else list(fp1)
        self.fp2 = [100.0] * 6 if fp2 is None else list(fp2)
        self.fp3 = [0.0] * 6 if fp3 is None else list(fp3)

    def friction(self, qd):
        '''Smooth friction torque vector (length 6, N·m) at joint speeds qd.'''
        out = []
        for i in range(6):
            f = self.fp1[i] / (1.0 + math.exp(-self.fp2[i] * (qd[i] + self.fp3[i])))
            f -= self.fp1[i] / (1.0 + math.exp(-self.fp2[i] * self.fp3[i]))
            out.append(f)
        return out


class ArmModel:
    '''
    Bundles FK-based numeric Jacobian, pseudo-inverse and safety tables.

    :param dh_is_offset: passed to :class:`C_PiperForwardKinematics` (0x00/0x01).
    :param joint_limits: list of 6 ``(min, max)`` tuples in rad.
    :param velocity_limits: list of 6 max speeds in rad/s.
    :param gravity_model: a gravity model exposing ``gravity(q) -> list[6]``
        (defaults to an unfitted :class:`FourierGravityModel`, i.e. zeros).
    '''
    def __init__(self, dh_is_offset=0x01,
                 joint_limits=None, velocity_limits=None,
                 gravity_model=None):
        self._fk = C_PiperForwardKinematics(dh_is_offset=dh_is_offset)
        self.joint_limits = joint_limits if joint_limits else DEFAULT_JOINT_LIMITS
        self.velocity_limits = velocity_limits if velocity_limits else DEFAULT_VELOCITY_LIMITS
        self.gravity_model = gravity_model if gravity_model is not None else FourierGravityModel()
        self._eps = 1e-4

    def gravity(self, q):
        '''Gravity compensation torque ``g(q)`` (length 6, N·m).'''
        return self.gravity_model.gravity(q)

    # ------------------------------------------------------------------ FK
    def fk(self, q):
        '''
        End-effector pose ``[x, y, z, rx, ry, rz]`` in (mm, deg).

        Index 5 of the SDK's per-joint FK output corresponds to link 6 /
        the end-effector frame.
        '''
        return list(self._fk.CalFK(list(q))[5])

    def fk_pose(self, q):
        '''
        Same as :meth:`fk` but returns ``[x, y, z]`` (m) and ``[r, p, y]`` (rad).
        '''
        pose = self.fk(q)
        xyz = [v / 1000.0 for v in pose[0:3]]
        rpy = [math.radians(v) for v in pose[3:6]]
        return xyz, rpy

    # ------------------------------------------------------------- Jacobian
    def jacobian(self, q):
        '''
        Numeric 6x6 geometric Jacobian at the end-effector (flat, row-major),
        in SI units:

        * rows 0-2: translational ``m / rad``
        * rows 3-5: rotational ``rad / rad`` (RPY parameterization)

        Computed by central finite differences of the SDK forward kinematics.
        '''
        base = self.fk(q)
        J = [0.0] * 36
        for j in range(6):
            qp = list(q)
            qm = list(q)
            qp[j] += self._eps
            qm[j] -= self._eps
            fp = self.fk(qp)
            fm = self.fk(qm)
            for i in range(6):
                d = (fp[i] - fm[i]) / (2.0 * self._eps)
                if i < 3:
                    d /= 1000.0          # mm -> m
                else:
                    d *= math.pi / 180.0  # deg -> rad
                J[6 * i + j] = d
        return J

    def jacobian_pinv(self, q, damp=1e-3):
        '''Damped least-squares pseudo-inverse of the 6x6 Jacobian.'''
        from .mathx import mat_pinv_damped
        return mat_pinv_damped(self.jacobian(q), 6, 6, damp=damp)

    def is_within_limits(self, q):
        '''True if q is strictly within all joint limits.'''
        for i in range(6):
            lo, hi = self.joint_limits[i]
            if not (lo <= q[i] <= hi):
                return False
        return True
