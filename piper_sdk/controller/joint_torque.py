#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Joint-space torque controller.

Sends an open-loop torque reference per joint through the MIT primitive with
``kp = kd = 0``:

    tau_ref = tau_des + g(q)

where ``tau_des`` is set by the user (e.g. a wrench, a learned friction term,
or a computed task-space force projected back). Gravity compensation is
strongly recommended or the arm will sag.

Torque quality caveats:

* The SDK clamps ``t_ref`` to +/-``tau_limit`` (default 8 N·m).
* ``t_ref`` is a *feedforward* torque; with ``kp = kd = 0`` there is no
  restoring force. Keep a small joint impedance running if you need stiffness.
* Torque *feedback* is only estimated from motor current (no F/T sensor), so
  any closed-loop use is approximate.
'''

from .base import BaseController


class JointTorqueController(BaseController):
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param gravity_comp: enable ``g(q)`` feedforward (default True).
    :param tau_des: initial desired torque vector (default zeros).
    :param kwargs: forwarded to :class:`BaseController`.
    '''
    def __init__(self, interface, gravity_comp=True, tau_des=None, **kwargs):
        kwargs.setdefault('name', 'joint_torque')
        super().__init__(interface, **kwargs)
        self._gravity_comp = gravity_comp
        self._tau_des = [0.0] * 6 if tau_des is None else list(tau_des)

    @property
    def torque_ref(self):
        return self._tau_des

    def set_torque(self, tau_des):
        '''
        Set the desired joint torque vector (length 6, N·m).

        Values are clamped to ``+/-tau_limit`` before sending.
        '''
        if len(tau_des) != 6:
            raise ValueError("tau_des must have length 6")
        self._tau_des = list(tau_des)

    def set_torque_joint(self, motor_num, tau):
        '''Set the desired torque for a single joint (1-6).'''
        if not (1 <= motor_num <= 6):
            raise ValueError("motor_num must be in [1, 6]")
        self._tau_des[motor_num - 1] = tau

    def _step(self, dt):
        q = self._state.q
        g = self._model.gravity(q) if self._gravity_comp else [0.0] * 6
        for i in range(6):
            self._send_mit(i + 1, q[i], 0.0, 0.0, 0.0,
                           self._tau_des[i] + g[i])
