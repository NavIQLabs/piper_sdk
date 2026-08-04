#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Real-time arm state estimator for the controller subpackage.

Pulls the interface's *cached* feedback (populated by the SDK's CAN monitor
thread) and produces filtered, consistent units:

* ``q``   - joint positions [rad]
* ``qd``  - joint velocities [rad/s]
* ``tau`` - estimated joint torque [N·m] (from motor current x coefficient)
* ``x``   - end-effector position [m]
* ``rpy`` - end-effector orientation [rad]

Velocity is taken primarily from the high-speed driver feedback
(``motor_speed``), falling back to finite-differencing ``q`` when the value is
unavailable. Torque is low-pass filtered by default (the current-based
estimate is noisy).
'''

import math
import time
from .model import FirstOrderLPF

class ArmState:
    '''
    :param interface: a connected ``C_PiperInterface_V2`` instance.
    :param tau_alpha: LPF coefficient for the torque estimate (1.0 disables).
    :param qd_alpha: LPF coefficient for velocity (1.0 disables).
    '''
    def __init__(self, interface, tau_alpha=0.2, qd_alpha=0.3):
        self._intf = interface
        self._tau_lpf = [FirstOrderLPF(tau_alpha) for _ in range(6)]
        self._qd_lpf = [FirstOrderLPF(qd_alpha) for _ in range(6)]
        self._q = [0.0] * 6
        self._qd = [0.0] * 6
        self._tau = [0.0] * 6
        self._x = [0.0] * 3
        self._rpy = [0.0] * 3
        self._last_time = time.perf_counter()
        self._last_q = None

    # ------------------------------------------------------------- helpers
    @staticmethod
    def _joint_wrap(arm_joint_msgs):
        j = arm_joint_msgs.joint_state
        deg = [j.joint_1, j.joint_2, j.joint_3, j.joint_4, j.joint_5, j.joint_6]
        return [d * 0.001 * math.pi / 180.0 for d in deg]

    @staticmethod
    def _motor_wrap(motor_info_msgs):
        '''
        Returns ``(q_rad, qd_rad_s, effort_Nm)`` from the high-speed feedback.
        '''
        q = [0.0] * 6
        qd = [0.0] * 6
        tau = [0.0] * 6
        for i in range(1, 7):
            m = getattr(motor_info_msgs, 'motor_%d' % i)
            q[i - 1] = float(m.pos)
            qd[i - 1] = float(m.motor_speed) * 0.001
            tau[i - 1] = float(m.effort)
        return q, qd, tau

    # ------------------------------------------------------------- update
    def update(self):
        '''
        Refresh all state quantities from the interface. Call once per control
        tick (or at least every few ms). Cheap - only reads cached values.
        '''
        now = time.perf_counter()
        dt = now - self._last_time
        self._last_time = now

        mq, mqd, mtau = self._motor_wrap(self._intf.GetArmHighSpdInfoMsgs())
        jq = self._joint_wrap(self._intf.GetArmJointMsgs())

        # Joint angle: prefer joint-state feedback (stable), fallback to motor pos.
        self._q = [jq[i] if abs(jq[i]) > 1e-9 else mq[i] for i in range(6)]

        # Velocity: prefer high-speed motor_speed, else finite difference.
        for i in range(6):
            if abs(mqd[i]) > 1e-6:
                raw = mqd[i]
            elif self._last_q is not None and dt > 0:
                raw = (self._q[i] - self._last_q[i]) / dt
            else:
                raw = 0.0
            self._qd[i] = self._qd_lpf[i].update(raw)

        # Torque estimate (N·m), low-pass filtered.
        for i in range(6):
            self._tau[i] = self._tau_lpf[i].update(mtau[i])

        # End-effector pose via SDK FK (feedback mode).
        fk = self._intf.GetFK("feedback")
        pose = fk[5]
        self._x = [v / 1000.0 for v in pose[0:3]]
        self._rpy = [math.radians(v) for v in pose[3:6]]

        self._last_q = list(self._q)
        return self

    # ------------------------------------------------------------- access
    @property
    def q(self):
        return self._q

    @property
    def qd(self):
        return self._qd

    @property
    def tau(self):
        return self._tau

    @property
    def x(self):
        return self._x

    @property
    def rpy(self):
        return self._rpy

    @property
    def pose(self):
        return self._x + self._rpy
