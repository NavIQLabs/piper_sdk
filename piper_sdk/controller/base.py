#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Base class shared by all controllers in the controller subpackage.

Handles MIT-mode lifecycle, the realtime loop, and per-joint safety clamping.
The actual control law lives in ``_step(dt)`` implemented by subclasses.

MIT mode lifecycle
------------------
Controllers drive the arm through the per-joint MIT pass-through command
(``JointMitCtrl``, CAN 0x15A-0x15F). Entering/leaving it requires the
firmware mode command ``MotionCtrl_2``:

* enter: ``MotionCtrl_2(0x01, 0x04, 0, 0xAD)``   (CAN control + MOVE M + MIT)
* exit:  ``MotionCtrl_2(0x01, 0x01, 0, 0x00)``   (CAN control + MOVE J + position)

Safety clamping
---------------
Per-joint limits are enforced *before* encoding:

* ``pos_ref`` clamped to SDK range [-12.5, 12.5] rad.
* ``vel_ref`` clamped to SDK range [-45, 45] rad/s.
* ``kp``      clamped to [0, 500].
* ``kd``      clamped to [-5, 5].
* ``t_ref``   clamped to ``tau_limit`` (default 8 N·m, matching the SDK's own
  clamp inside ``__JointMitCtrl``).

The realtime-loop watchdog wires to ``_safety_disable`` which exits MIT mode
and disables the motors.
'''

import time
from .rt_loop import RtLoop
from .state import ArmState
from .model import ArmModel, DEFAULT_VELOCITY_LIMITS

# SDK MIT argument ranges (see __JointMitCtrl).
POS_MIN, POS_MAX = -12.5, 12.5
VEL_MIN, VEL_MAX = -45.0, 45.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = -5.0, 5.0

class BaseController:
    '''
    :param interface: connected ``C_PiperInterface_V2``.
    :param rate_hz: control rate (default 500 Hz).
    :param name: controller name (used by loop/FPS and mode bookkeeping).
    :param model: :class:`ArmModel` or None (a default is created).
    :param tau_limit: per-joint torque feedforward clamp in N·m (default 8.0).
    :param vel_limit: per-joint velocity clamp in rad/s (default conservative).
    :param enable_on_start: enter MIT mode automatically in ``enable()``.
    :param disable_motors_on_stop: call ``DisablePiper()`` on ``disable()``.
    '''
    def __init__(self, interface, rate_hz=500.0, name="base",
                 model=None, tau_limit=8.0, vel_limit=None,
                 enable_on_start=True, disable_motors_on_stop=True):
        self._intf = interface
        self._rate_hz = rate_hz
        self._name = name
        self._model = model if model is not None else ArmModel()
        self._state = ArmState(interface)
        self._tau_limit = float(tau_limit)
        self._vel_limit = vel_limit if vel_limit else DEFAULT_VELOCITY_LIMITS
        self._enable_on_start = enable_on_start
        self._disable_motors_on_stop = disable_motors_on_stop
        self._in_mit = False
        self._safety_ok = True
        self._last_disable = 0.0

        self._loop = RtLoop(rate_hz=rate_hz, callback=self._on_tick, name=name,
                            max_step_s=0.05, max_late_s=0.5,
                            on_timeout=self._safety_disable)
        self._enabled = False

    # ------------------------------------------------------------- public
    @property
    def state(self):
        return self._state

    @property
    def model(self):
        return self._model

    @property
    def name(self):
        return self._name

    @property
    def rate_hz(self):
        return self._rate_hz

    @property
    def achieved_hz(self):
        return self._loop.achieved_hz

    @property
    def safety_ok(self):
        return self._safety_ok

    @property
    def in_mit(self):
        return self._in_mit

    @property
    def enabled(self):
        return self._enabled

    def enable(self):
        '''Enable the arm, enter MIT mode and start the control loop.'''
        if self._enabled:
            return
        self._safety_ok = True
        if not self._intf.get_connect_status():
            self._intf.ConnectPort()
        if self._enable_on_start:
            self._intf.EnablePiper()
            self._enter_mit()
        self._loop.start()
        self._enabled = True

    def disable(self, disable_motors=True):
        '''
        Stop the control loop, exit MIT mode and (optionally) disable motors.
        '''
        self._loop.stop()
        if self._in_mit:
            try:
                self._exit_mit()
            except Exception:  # noqa: BLE001 - best-effort on shutdown
                pass
        if disable_motors and self._disable_motors_on_stop:
            try:
                self._intf.DisablePiper()
            except Exception:  # noqa: BLE001
                pass
        self._in_mit = False
        self._enabled = False

    def __enter__(self):
        self.enable()
        return self

    def __exit__(self, *exc):
        self.disable()
        return False

    # ------------------------------------------------------ MIT lifecycle
    def _enter_mit(self):
        self._intf.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
        self._in_mit = True

    def _exit_mit(self):
        self._intf.MotionCtrl_2(0x01, 0x01, 0, 0x00)
        self._in_mit = False

    # ------------------------------------------------------------ helpers
    def _send_mit(self, motor_num, pos_ref, vel_ref, kp, kd, t_ref):
        '''Clamp to SDK/MIT ranges and send a single-joint MIT command.'''
        from .mathx import clip
        pos_ref = clip(pos_ref, POS_MIN, POS_MAX)
        vel_ref = clip(vel_ref, VEL_MIN, VEL_MAX)
        kp = clip(kp, KP_MIN, KP_MAX)
        kd = clip(kd, KD_MIN, KD_MAX)
        t_ref = clip(t_ref, -self._tau_limit, self._tau_limit)
        self._intf.JointMitCtrl(motor_num, pos_ref, vel_ref, kp, kd, t_ref)

    def _clamp_joint_velocity(self, qd_des):
        '''Clamp desired joint velocity to the per-joint velocity limits.'''
        out = []
        for i, v in enumerate(qd_des):
            lim = self._vel_limit[i]
            out.append(max(-lim, min(lim, v)))
        return out

    def _check_limits(self, q_des, margin=0.02):
        '''
        True if q_des is within joint limits (with a small safety margin) and
        velocity is within the configured limit.
        '''
        for i in range(6):
            lo, hi = self._model.joint_limits[i]
            if not (lo + margin <= q_des[i] <= hi - margin):
                return False
        return True

    # ------------------------------------------------------------- safety
    def _safety_disable(self, reason):
        '''
        Watchdog / safety callback: record the reason, exit MIT and disable
        motors as fast as possible.
        '''
        self._safety_ok = False
        try:
            if self._in_mit:
                self._exit_mit()
            self._intf.DisablePiper()
        except Exception:  # noqa: BLE001
            pass
        self._last_disable = time.perf_counter()
        self._enabled = False

    def _trip(self, reason):
        '''Manually trip the safety system (e.g. limit violation in a step).'''
        if self._safety_ok:
            self._loop.trip(reason)

    # ------------------------------------------------------------- loop
    def _on_tick(self, dt):
        self._state.update()
        if self._safety_ok:
            self._step(dt)

    def _step(self, dt):
        '''Control law, implemented by subclasses. Must send MIT commands.'''
        raise NotImplementedError
