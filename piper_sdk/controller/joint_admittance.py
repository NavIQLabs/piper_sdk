#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space admittance controller.

The outer loop estimates the external joint torque (motor-current based,
gravity and friction subtracted) and drives a virtual spring-damper target:

    tau_ext   = tau_est - g(q) - tau_friction(qd)
    M_a * qdd_d + D_a * qd_d = tau_ext

``q_d`` integrates ``qd_d`` and is tracked by an inner stiff impedance loop.
The arm behaves like a mass-damper dragged by external forces (e.g. a human
guiding the arm).

Quality notes:

* The external torque estimate relies on ``cal_effort`` (current x
  coefficient) - noisy, so ``tau_lpf_alpha`` should stay low and the
  estimated friction should be calibrated.
* A runaway guard trips the safety system if ``|tau_ext|`` exceeds
  ``runaway_tau`` for ``runaway_time`` seconds.
'''

from .mathx import clip
from .base import BaseController

class JointAdmittanceController(BaseController):
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param M: 6 virtual masses (inertia) in kg·m² (default 0.5).
    :param D: 6 virtual damping in N·m·s/rad (default 2.0).
    :param K_inner: stiffness of the inner impedance loop (N·m/rad, default 150).
    :param D_inner: damping of the inner impedance loop (N·m·s/rad, default 5).
    :param tau_friction: 6-element Coulomb friction offset [N·m].
    :param tau_lpf_alpha: LPF coefficient for the external torque estimate.
    :param runaway_tau: external-torque runaway threshold [N·m] (0 disables).
    :param runaway_time: sustained-exceed time before tripping [s].
    :param gravity_comp: enable ``g(q)`` subtraction (default True).
    :param kwargs: forwarded to :class:`BaseController`.
    '''
    def __init__(self, interface,
                 M=None, D=None, K_inner=150.0, D_inner=5.0,
                 tau_friction=None, tau_lpf_alpha=0.1,
                 runaway_tau=6.0, runaway_time=0.5,
                 gravity_comp=True, **kwargs):
        kwargs.setdefault('name', 'joint_admittance')
        super().__init__(interface, **kwargs)
        self._M = [0.5] * 6 if M is None else list(M)
        self._D = [2.0] * 6 if D is None else list(D)
        self._K_inner = K_inner
        self._D_inner = D_inner
        self._tau_friction = [0.0] * 6 if tau_friction is None else list(tau_friction)
        self._gravity_comp = gravity_comp
        self._runaway_tau = runaway_tau
        self._runaway_time = runaway_time
        self._tau_ext = [0.0] * 6
        self._tau_ext_lpf = _LPF(tau_lpf_alpha)
        self._qd_d = [0.0] * 6
        self._q_d = None
        self._t_high = 0.0

    def set_admittance(self, M=None, D=None):
        '''Update virtual mass/damping.'''
        if M is not None:
            self._M = list(M)
        if D is not None:
            self._D = list(D)

    def set_target(self, q_d):
        '''Seed the integrated target (e.g. current pose on enable).'''
        if not self._check_limits(q_d):
            raise ValueError("target outside joint limits")
        self._q_d = list(q_d)
        self._qd_d = [0.0] * 6

    def _step(self, dt):
        q = self._state.q
        qd = self._state.qd
        tau_est = self._state.tau

        if self._q_d is None:
            self._q_d = list(q)

        g = self._model.gravity(q) if self._gravity_comp else [0.0] * 6

        # ---- external torque estimate
        for i in range(6):
            fric = self._tau_friction[i] * _sign(qd[i])
            raw = tau_est[i] - g[i] - fric
            self._tau_ext[i] = self._tau_ext_lpf.update(i, raw)

        # ---- runaway guard (accumulated control-loop time)
        if self._runaway_tau > 0.0:
            exceed = max(abs(t) for t in self._tau_ext) > self._runaway_tau
            if exceed:
                self._t_high += dt
                if self._t_high > self._runaway_time:
                    self._trip("admittance runaway torque %.3f N·m" %
                               max(abs(t) for t in self._tau_ext))
                    return
            else:
                self._t_high = 0.0

        # ---- admittance integration
        for i in range(6):
            qdd_d = (self._tau_ext[i] - self._D[i] * self._qd_d[i]) / self._M[i]
            self._qd_d[i] += qdd_d * dt
            lim = self._vel_limit[i]
            self._qd_d[i] = clip(self._qd_d[i], -lim, lim)
            self._q_d[i] += self._qd_d[i] * dt
            lo, hi = self._model.joint_limits[i]
            if self._q_d[i] < lo:
                self._q_d[i] = lo
                self._qd_d[i] = 0.0
            elif self._q_d[i] > hi:
                self._q_d[i] = hi
                self._qd_d[i] = 0.0

        # ---- inner stiff impedance tracking
        for i in range(6):
            self._send_mit(i + 1, self._q_d[i], self._qd_d[i],
                           self._K_inner, self._D_inner, g[i])


class _LPF:
    '''Per-joint first-order low-pass filter (indexed by channel).'''
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
