#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space impedance controller.

Control law (per joint, mapped directly onto the MIT primitive):

    tau = K_i * (q_d - q) + D_i * (qd_d - qd) + g(q)

with ``K_i`` sent as the MIT ``kp`` and ``D_i`` as the MIT ``kd``. This is the
same PD + feedforward structure the driver implements natively, so tracking
is as fast as the CAN loop allows.

Default gains are deliberately gentle; tune with
``demo/V2/tune_impedance.py``.
'''

from .base import BaseController


class JointImpedanceController(BaseController):
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param K: list of 6 stiffness gains ``kp`` in N·m/rad (default 20).
    :param D: list of 6 damping gains ``kd`` in N·m·s/rad (default 1.0).
    :param gravity_comp: enable ``g(q)`` feedforward (default True).
    :param kwargs: forwarded to :class:`BaseController`.
    '''
    def __init__(self, interface, K=None, D=None, gravity_comp=True, **kwargs):
        kwargs.setdefault('name', 'joint_impedance')
        super().__init__(interface, **kwargs)
        self._K = [20.0] * 6 if K is None else list(K)
        self._D = [1.0] * 6 if D is None else list(D)
        self._gravity_comp = gravity_comp
        self._q_d = None
        self._qd_d = [0.0] * 6

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
        '''Update stiffness/damping gains.'''
        if K is not None:
            self._K = list(K)
        if D is not None:
            self._D = list(D)

    def set_target(self, q_d, qd_d=None):
        '''
        Set the desired joint configuration.

        :param q_d: list of 6 desired joint angles [rad].
        :param qd_d: optional list of 6 desired joint velocities [rad/s].
        '''
        if not self._check_limits(q_d):
            raise ValueError("target outside joint limits")
        self._q_d = list(q_d)
        if qd_d is not None:
            self._qd_d = list(qd_d)
        else:
            self._qd_d = [0.0] * 6

    def _step(self, dt):
        if self._q_d is None:
            self._q_d = list(self._state.q)
        q = self._state.q
        qd = self._state.qd
        g = self._model.gravity(q) if self._gravity_comp else [0.0] * 6
        for i in range(6):
            self._send_mit(i + 1, self._q_d[i], self._qd_d[i],
                           self._K[i], self._D[i], g[i])
