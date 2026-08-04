#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Cartesian-space admittance controller.

Estimates an external end-effector wrench from the joint torque estimate
(current-based), then drives a task-space virtual mass-damper:

    tau_ext = tau_est - g(q) - tau_friction(qd)
    F_ext   = (J^T)^+ * tau_ext
    M_c * xdd_d + D_c * xd_d = F_ext

``x_d`` integrates ``xd_d`` and is tracked by the inner Cartesian impedance
controller (inherited). The arm yields to contact forces in the task frame.

Caveats: the force estimate is only as good as the current-based torque
estimate (no F/T sensor), so filtering and friction calibration matter. The
runaway guard trips the safety system on sustained high external wrench.
'''

import time
from .mathx import mat_vec, mat_pinv_damped, transpose, clip
from .cartesian_impedance import CartesianImpedanceController, _wrap_angle


class CartesianAdmittanceController(CartesianImpedanceController):
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param M: 6 virtual masses ``[kg x3, kg·m² x3]`` (default 5, 1).
    :param D: 6 virtual damping ``[N·s/m x3, N·m·s/rad x3]`` (default 100, 10).
    :param tau_friction: 6-element Coulomb friction offset [N·m].
    :param tau_lpf_alpha: LPF coefficient for the external torque estimate.
    :param vel_linear_max: clamp for desired linear velocity [m/s].
    :param vel_angular_max: clamp for desired angular velocity [rad/s].
    :param runaway_force: external wrench magnitude trip threshold [N] (0 dis.).
    :param runaway_time: sustained exceed time before tripping [s].
    :param gravity_comp: enable ``g(q)`` subtraction (default True).
    :param kwargs: forwarded to :class:`CartesianImpedanceController`.
    '''
    def __init__(self, interface,
                 M=None, D=None,
                 tau_friction=None, tau_lpf_alpha=0.1,
                 vel_linear_max=0.3, vel_angular_max=1.0,
                 runaway_force=30.0, runaway_time=0.5,
                 gravity_comp=True, **kwargs):
        kwargs.setdefault('name', 'cartesian_admittance')
        super().__init__(interface, gravity_comp=gravity_comp, **kwargs)
        self._M = [5.0, 5.0, 5.0, 1.0, 1.0, 1.0] if M is None else list(M)
        self._D = [100.0, 100.0, 100.0, 10.0, 10.0, 10.0] if D is None else list(D)
        self._tau_friction = [0.0] * 6 if tau_friction is None else list(tau_friction)
        self._tau_lpf_alpha = tau_lpf_alpha
        self._vel_linear_max = vel_linear_max
        self._vel_angular_max = vel_angular_max
        self._runaway_force = runaway_force
        self._runaway_time = runaway_time
        self._gravity_comp = gravity_comp

        self._tau_ext = [0.0] * 6
        self._tau_lpf = _LPF(tau_lpf_alpha)
        self._F_ext = [0.0] * 6
        self._xd_d = [0.0] * 6
        self._t_high = time.perf_counter()

    def set_admittance(self, M=None, D=None):
        '''Update task-space virtual mass/damping.'''
        if M is not None:
            self._M = list(M)
        if D is not None:
            self._D = list(D)

    def set_target(self, x_d):
        '''Seed the desired pose (e.g. current pose on enable).'''
        if len(x_d) != 6:
            raise ValueError("x_d must have length 6")
        self._x_d = list(x_d)
        self._xd_d = [0.0] * 6

    def _step(self, dt):
        q = self._state.q
        qd = self._state.qd
        tau_est = self._state.tau

        if self._x_d is None:
            self._x_d = list(self._state.pose)

        g = self._model.gravity(q) if self._gravity_comp else [0.0] * 6

        # ---- external joint torque estimate (gravity/friction removed)
        for i in range(6):
            fric = self._tau_friction[i] * _sign(qd[i])
            raw = tau_est[i] - g[i] - fric
            self._tau_ext[i] = self._tau_lpf.update(i, raw)

        # ---- map to end-effector wrench
        J = self._model.jacobian(q)
        Jt_pinv = mat_pinv_damped(transpose(J, 6, 6), 6, 6, damp=1e-3)
        self._F_ext = mat_vec(Jt_pinv, self._tau_ext, 6, 6)

        # ---- runaway guard
        if self._runaway_force > 0.0:
            force_mag = max(abs(f) for f in self._F_ext[0:3])
            now = time.perf_counter()
            if force_mag > self._runaway_force:
                if now - self._t_high > self._runaway_time:
                    self._trip("cartesian admittance runaway force %.3f N" % force_mag)
                    return
            else:
                self._t_high = now

        # ---- task-space admittance integration
        for i in range(6):
            xdd_d = (self._F_ext[i] - self._D[i] * self._xd_d[i]) / self._M[i]
            self._xd_d[i] += xdd_d * dt
        for i in range(3):
            self._xd_d[i] = clip(self._xd_d[i], -self._vel_linear_max, self._vel_linear_max)
        for i in range(3, 6):
            self._xd_d[i] = clip(self._xd_d[i], -self._vel_angular_max, self._vel_angular_max)
        for i in range(6):
            self._x_d[i] += self._xd_d[i] * dt
        # wrap orientation reference
        for i in range(3, 6):
            self._x_d[i] = _wrap_angle(self._x_d[i])

        # ---- inner Cartesian impedance tracking
        super()._step(dt)


class _LPF:
    '''Per-channel first-order low-pass filter.'''
    def __init__(self, alpha):
        self._alpha = alpha
        self._y = {}

    def update(self, i, x):
        y = self._y.get(i, x)
        y += self._alpha * (x - y)
        self._y[i] = y
        return y


def _sign(x):
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)
