#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Cartesian-space impedance controller.

Implements a 6-DOF stiffness/damping in the task frame and maps it to joint
torques through the Jacobian transpose:

    F   = K_c * (x_d - x) + D_c * (0 - xdot) + F_feedforward
    tau = J^T * F + g(q)

The joint-level MIT primitive is used in pure-torque mode (``kp = kd = 0``);
``t_ref`` carries ``tau``. A small joint damping term can be added for extra
stability since there is no mass-matrix decoupling.

Singularities: near kinematic singularities the wrench is automatically scaled
down (via the Jacobian condition number) to avoid commanding huge joint
torques.
'''

import math
from .mathx import mat_vec, condition_number, clip
from .base import BaseController


def _wrap_angle(a):
    '''Wrap angle to [-pi, pi].'''
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class CartesianImpedanceController(BaseController):
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param K: 6-element stiffness diag ``[N/m, N/m, N/m, N·m/rad x3]``.
    :param D: 6-element damping diag (translational N·s/m, rotational N·m·s/rad).
    :param joint_damping: extra per-joint damping gain (default 0.5).
    :param gravity_comp: enable ``g(q)`` feedforward (default True).
    :param cond_thresh: Jacobian condition number above which the wrench is
        scaled down (default 50.0).
    :param kwargs: forwarded to :class:`BaseController`.
    '''
    def __init__(self, interface, K=None, D=None,
                 joint_damping=0.5, gravity_comp=True,
                 cond_thresh=50.0, **kwargs):
        kwargs.setdefault('name', 'cartesian_impedance')
        super().__init__(interface, **kwargs)
        self._K = [300.0, 300.0, 300.0, 10.0, 10.0, 10.0] if K is None else list(K)
        self._D = [30.0, 30.0, 30.0, 3.0, 3.0, 3.0] if D is None else list(D)
        self._joint_damping = joint_damping
        self._gravity_comp = gravity_comp
        self._cond_thresh = cond_thresh
        self._x_d = None
        self._F_ff = [0.0] * 6

    @property
    def stiffness(self):
        return self._K

    @stiffness.setter
    def stiffness(self, K):
        self._K = list(K)

    @property
    def damping(self):
        return self._D

    @damping.setter
    def damping(self, D):
        self._D = list(D)

    def set_stiffness_damping(self, K=None, D=None):
        '''Update task-space stiffness/damping.'''
        if K is not None:
            self._K = list(K)
        if D is not None:
            self._D = list(D)

    def set_target(self, x_d, F_ff=None):
        '''
        Set the desired end-effector pose.

        :param x_d: ``[x, y, z]`` (m) + ``[rx, ry, rz]`` (rad).
        :param F_ff: optional constant feedforward wrench ``[Fx..Fz, Tx..Tz]``.
        '''
        if len(x_d) != 6:
            raise ValueError("x_d must have length 6")
        self._x_d = list(x_d)
        if F_ff is not None:
            self._F_ff = list(F_ff)

    def _step(self, dt):
        q = self._state.q
        qd = self._state.qd
        x = self._state.pose

        if self._x_d is None:
            self._x_d = list(x)

        J = self._model.jacobian(q)
        xdot = mat_vec(J, qd, 6, 6)

        # ---- wrench from stiffness/damping + feedforward
        F = [0.0] * 6
        for i in range(6):
            err = self._x_d[i] - x[i]
            if i >= 3:
                err = _wrap_angle(err)
            F[i] = self._K[i] * err - self._D[i] * xdot[i] + self._F_ff[i]

        # ---- singularity scaling
        scale = 1.0
        cond = condition_number(J, 6, 6)
        if cond > self._cond_thresh:
            scale = clip(self._cond_thresh / cond, 0.0, 1.0)
        if scale < 1.0:
            F = [f * scale for f in F]

        # ---- map to joint torques + gravity + extra damping
        g = self._model.gravity(q) if self._gravity_comp else [0.0] * 6
        tau = _mat_vec_Jt(J, F)  # J^T F
        for i in range(6):
            tau[i] += g[i]
            if self._joint_damping > 0.0:
                tau[i] -= self._joint_damping * qd[i]

        # ---- joint limit safety
        if not self._model.is_within_limits(q):
            self._trip("joint limit reached during cartesian impedance")
            return

        for i in range(6):
            self._send_mit(i + 1, q[i], 0.0, 0.0, 0.0, tau[i])


def _mat_vec_Jt(J, v):
    '''
    Multiply ``J^T v`` where J is a 6x6 flat row-major matrix (J^T is 6x6).
    '''
    out = [0.0] * 6
    for j in range(6):
        s = 0.0
        for i in range(6):
            s += J[6 * i + j] * v[i]
        out[j] = s
    return out
