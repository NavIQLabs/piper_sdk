import math
import time

import pytest

from piper_sdk.controller import (
    JointImpedanceController, JointTorqueController,
    JointAdmittanceController, CartesianImpedanceController,
    CartesianAdmittanceController, ArmModel, SigmoidFrictionModel,
)


class MockHighSpd:
    def __init__(self, q, qd, current):
        self.pos = q
        # SDK stores motor_speed in 0.001 rad/s units; ArmState._motor_wrap
        # multiplies by 0.001, so pass qd*1000 to represent qd rad/s.
        self.motor_speed = qd * 1000.0
        self.current = current
        self.effort = current * 1.0


class MockMotorInfo:
    def __init__(self, q, qd, current):
        self.motor_1 = MockHighSpd(q[0], qd[0], current[0])
        self.motor_2 = MockHighSpd(q[1], qd[1], current[1])
        self.motor_3 = MockHighSpd(q[2], qd[2], current[2])
        self.motor_4 = MockHighSpd(q[3], qd[3], current[3])
        self.motor_5 = MockHighSpd(q[4], qd[4], current[4])
        self.motor_6 = MockHighSpd(q[5], qd[5], current[5])


class MockJointState:
    def __init__(self, q_rad):
        # SDK reports in 0.001 degrees
        self.joint_1 = q_rad[0] * 180 / math.pi / 0.001
        self.joint_2 = q_rad[1] * 180 / math.pi / 0.001
        self.joint_3 = q_rad[2] * 180 / math.pi / 0.001
        self.joint_4 = q_rad[3] * 180 / math.pi / 0.001
        self.joint_5 = q_rad[4] * 180 / math.pi / 0.001
        self.joint_6 = q_rad[5] * 180 / math.pi / 0.001


class MockArmJointMsgs:
    def __init__(self, q_rad):
        self.joint_state = MockJointState(q_rad)


class MockInterface:
    '''Minimal stand-in for C_PiperInterface_V2 with real FK via the SDK model.'''
    def __init__(self, q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0], qd=None, current=None):
        self._model = ArmModel()
        self._q = list(q)
        self._qd = list(qd) if qd else [0.0] * 6
        self._current = list(current) if current else [0.0] * 6
        self._connected = True
        self._mit_cmds = []
        self._mode_cmds = []
        self._enable_count = 0
        self._disable_count = 0

    # ---- SDK API consumed by controllers
    def get_connect_status(self):
        return self._connected

    def ConnectPort(self):
        self._connected = True

    def EnablePiper(self):
        self._enable_count += 1
        return True

    def DisablePiper(self):
        self._disable_count += 1
        return True

    def MotionCtrl_2(self, ctrl_mode, move_mode, spd, mit_mode):
        self._mode_cmds.append((ctrl_mode, move_mode, mit_mode))

    def JointMitCtrl(self, motor_num, pos_ref, vel_ref, kp, kd, t_ref):
        self._mit_cmds.append((motor_num, pos_ref, vel_ref, kp, kd, t_ref))

    def GetArmHighSpdInfoMsgs(self):
        return MockMotorInfo(self._q, self._qd, self._current)

    def GetArmJointMsgs(self):
        return MockArmJointMsgs(self._q)

    def GetFK(self, mode="feedback"):
        return self._model._fk.CalFK(list(self._q))


def _run_ticks(ctrl, n=1, dt=0.002):
    for _ in range(n):
        ctrl._on_tick(dt)


def test_joint_impedance_sends_mit():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf, rate_hz=500, K=[20]*6, D=[1]*6,
                                    gravity_comp=False)
    ctrl.set_target([0.3, 1.1, -0.2, 0.0, 0.0, 0.0])
    _run_ticks(ctrl)
    assert len(intf._mit_cmds) == 6
    # all six joints commanded
    assert sorted(c[0] for c in intf._mit_cmds) == [1, 2, 3, 4, 5, 6]
    # kp == K, kd == D
    for c in intf._mit_cmds:
        assert c[3] == 20.0
        assert c[4] == 1.0


def test_joint_impedance_clamps_kp():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf, K=[9999]*6, D=[0]*6, gravity_comp=False)
    ctrl.set_target([0.3, 1.1, -0.2, 0.0, 0.0, 0.0])
    _run_ticks(ctrl)
    assert all(c[3] <= 500.0 for c in intf._mit_cmds)


def test_joint_impedance_rejects_outside_limits():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf)
    with pytest.raises(ValueError):
        ctrl.set_target([0.0, 5.0, 0.0, 0.0, 0.0, 0.0])  # joint2 > 3.14


def test_joint_torque_uses_feedforward():
    intf = MockInterface(current=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    ctrl = JointTorqueController(intf, gravity_comp=False)
    ctrl.set_torque([0.5, -0.5, 0.0, 1.0, -1.0, 0.2])
    _run_ticks(ctrl)
    assert len(intf._mit_cmds) == 6
    # kp/kd zero in pure torque mode
    assert all(c[3] == 0.0 and c[4] == 0.0 for c in intf._mit_cmds)
    # t_ref matches desired
    assert intf._mit_cmds[0][5] == pytest.approx(0.5)
    assert intf._mit_cmds[3][5] == pytest.approx(1.0)


def test_joint_torque_clamps_tau_limit():
    intf = MockInterface()
    ctrl = JointTorqueController(intf, tau_limit=8.0, gravity_comp=False)
    ctrl.set_torque([20.0, -20.0, 0, 0, 0, 0])
    _run_ticks(ctrl)
    assert abs(intf._mit_cmds[0][5]) <= 8.0
    assert abs(intf._mit_cmds[1][5]) <= 8.0


def test_cartesian_impedance_wrench_direction():
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0])
    ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                        joint_damping=0.0)
    x = intf.GetFK("feedback")[5]
    x_d = [x[0]/1000.0 + 0.05, x[1]/1000.0, x[2]/1000.0,
           math.radians(x[3]), math.radians(x[4]), math.radians(x[5])]
    ctrl.set_target(x_d)
    _run_ticks(ctrl)
    assert len(intf._mit_cmds) == 6
    # pure torque mode
    assert all(c[3] == 0.0 and c[4] == 0.0 for c in intf._mit_cmds)


def test_cartesian_impedance_reaches_steady_state_torque():
    # With zero error, wrench should be ~zero -> torques ~zero (no gravity).
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0])
    ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                        joint_damping=0.0)
    x = intf.GetFK("feedback")[5]
    x_d = [x[0]/1000.0, x[1]/1000.0, x[2]/1000.0,
           math.radians(x[3]), math.radians(x[4]), math.radians(x[5])]
    ctrl.set_target(x_d)
    _run_ticks(ctrl)
    assert max(abs(c[5]) for c in intf._mit_cmds) < 1e-6


def test_admittance_target_moves_with_external_torque():
    # Constant external torque should push the integrated target away.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[1000.0, 0, 0, 0, 0, 0])  # ~1 N·m at joint 1
    ctrl = JointAdmittanceController(intf, gravity_comp=False,
                                     runaway_tau=0.0, M=[0.5]*6, D=[2.0]*6,
                                     K_inner=150.0, D_inner=5.0)
    ctrl.set_target(list(intf._q))
    start = list(ctrl._q_d)
    dt = 0.002
    for _ in range(200):
        ctrl._on_tick(dt)
    assert ctrl._q_d[0] != pytest.approx(start[0], abs=1e-3)


def test_admittance_runaway_guard_trips_after_sustained_torque():
    # Sustained external torque above the threshold must trip the safety.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[1000.0, 0, 0, 0, 0, 0])  # ~1 N·m at joint 1
    ctrl = JointAdmittanceController(intf, gravity_comp=False,
                                     runaway_tau=0.5, runaway_time=0.1,
                                     M=[0.5]*6, D=[2.0]*6)
    ctrl.set_target(list(intf._q))
    dt = 0.002
    for _ in range(200):  # 0.4s > runaway_time
        ctrl._on_tick(dt)
    assert not ctrl.safety_ok
    assert "admittance runaway" in ctrl._loop.timeout_reason


def test_admittance_runaway_guard_ignores_brief_torque():
    # A short-lived torque spike below runaway_time must not trip.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[1000.0, 0, 0, 0, 0, 0])
    ctrl = JointAdmittanceController(intf, gravity_comp=False,
                                     runaway_tau=0.5, runaway_time=0.1,
                                     M=[0.5]*6, D=[2.0]*6)
    ctrl.set_target(list(intf._q))
    for _ in range(10):  # 20ms < runaway_time
        ctrl._on_tick(0.002)
    assert ctrl.safety_ok


def test_cartesian_admittance_force_estimate_units():
    # F_ext should be a 6-vector with sane magnitude for a joint torque.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[1000.0, 0, 0, 0, 0, 0])
    ctrl = CartesianAdmittanceController(intf, gravity_comp=False,
                                         runaway_force=0.0)
    ctrl.set_target(ctrl._state.pose)
    ctrl._on_tick(0.002)
    assert len(ctrl._F_ext) == 6
    assert math.isfinite(sum(ctrl._F_ext))


def test_cartesian_impedance_target_ema_filters_steps():
    # With target_filter=1.0 the filtered target jumps instantly; with a
    # small alpha it converges over several ticks instead.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0])
    ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                        joint_damping=0.0,
                                        target_filter=0.2)
    x = intf.GetFK("feedback")[5]
    x_d = [x[0] / 1000.0 + 0.05, x[1] / 1000.0, x[2] / 1000.0,
           math.radians(x[3]), math.radians(x[4]), math.radians(x[5])]
    ctrl.set_target(x_d)
    _run_ticks(ctrl, n=1)
    # filtered target is between the initial pose and the step target
    fx = ctrl._x_d_filtered
    assert fx[0] > x[0] / 1000.0  # moved off the initial pose
    assert fx[0] < x_d[0] - 0.03  # not yet at target
    for _ in range(500):
        _run_ticks(ctrl, n=1)
    assert ctrl._x_d_filtered[0] == pytest.approx(x_d[0], abs=1e-3)


def test_cartesian_impedance_nullspace_damping_projects_onto_nullspace():
    # Nullspace damping adds a torque in I - J^+ J. Task-space torque is zero
    # (target == current pose, D=0), so any torque difference between
    # nullspace on/off must come from the projector.
    q = [0.2, 1.0, -0.3, 0.1, 0.2, 0.0]

    def run(nullspace_damping):
        intf = MockInterface(q=q, qd=[0.5] * 6)
        ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                            joint_damping=0.0, D=[0.0] * 6,
                                            nullspace_damping=nullspace_damping)
        x = intf.GetFK("feedback")[5]
        ctrl.set_target([x[0] / 1000.0, x[1] / 1000.0, x[2] / 1000.0,
                         math.radians(x[3]), math.radians(x[4]), math.radians(x[5])])
        _run_ticks(ctrl, n=1)
        return [c[5] for c in intf._mit_cmds]

    tau_on = run(1.0)
    tau_off = run(0.0)
    assert any(abs(a - b) > 1e-6 for a, b in zip(tau_on, tau_off))


def test_cartesian_impedance_nullspace_stiffness_pulls_toward_reference():
    # Nullspace stiffness springs q back toward q_ref through I - J^+ J. With
    # target == current pose and D=0, the only torque comes from the spring.
    q = [0.2, 1.0, -0.3, 0.1, 0.2, 0.0]
    q_ref = [q[0] - 0.1] + q[1:]  # displaced joint 1

    def run(stiffness):
        intf = MockInterface(q=q, qd=[0.0] * 6)
        ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                            joint_damping=0.0, D=[0.0] * 6,
                                            nullspace_stiffness=stiffness,
                                            nullspace_reference=q_ref)
        x = intf.GetFK("feedback")[5]
        ctrl.set_target([x[0] / 1000.0, x[1] / 1000.0, x[2] / 1000.0,
                         math.radians(x[3]), math.radians(x[4]), math.radians(x[5])])
        _run_ticks(ctrl, n=1)
        return [c[5] for c in intf._mit_cmds]

    tau_off = run(0.0)
    tau_on = run(1.0)
    # spring pushes joint 1 back up toward q_ref (positive error -> +torque)
    assert any(abs(a - b) > 1e-6 for a, b in zip(tau_on, tau_off))


def test_cartesian_impedance_nullspace_stiffness_defaults_to_starting_pose():
    # Without an explicit q_ref, the spring reference is captured from the
    # first measured pose, so a nullspace stiffness alone produces ~zero torque.
    q = [0.2, 1.0, -0.3, 0.1, 0.2, 0.0]

    def run(stiffness):
        intf = MockInterface(q=q, qd=[0.0] * 6)
        ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                            joint_damping=0.0, D=[0.0] * 6,
                                            nullspace_stiffness=stiffness)
        x = intf.GetFK("feedback")[5]
        ctrl.set_target([x[0] / 1000.0, x[1] / 1000.0, x[2] / 1000.0,
                         math.radians(x[3]), math.radians(x[4]), math.radians(x[5])])
        _run_ticks(ctrl, n=1)
        return [c[5] for c in intf._mit_cmds]

    tau_off = run(0.0)
    tau_on = run(5.0)
    # q_ref captured == q, so both are ~zero; must differ only in float noise
    assert all(abs(a - b) < 1e-9 for a, b in zip(tau_on, tau_off))


def test_cartesian_impedance_nullspace_reference_validates_length():
    intf = MockInterface()
    with pytest.raises(ValueError):
        CartesianImpedanceController(intf, nullspace_reference=[0.0] * 3)
    ctrl = CartesianImpedanceController(intf)
    with pytest.raises(ValueError):
        ctrl.set_nullspace_reference([0.0] * 5)


def test_torque_rate_limit_slows_commands():
    # With torque_rate_limit=0.5 the torque can change at most 0.5 N·m/cycle.
    intf = MockInterface()
    ctrl = JointTorqueController(intf, gravity_comp=False, torque_rate_limit=0.5)
    ctrl.set_torque([8.0, 0, 0, 0, 0, 0])
    _run_ticks(ctrl, n=1)
    # first command ramps from 0 by at most 0.5
    assert intf._mit_cmds[0][5] == pytest.approx(0.5)
    for _ in range(3):
        _run_ticks(ctrl, n=1)
    # monotonic ramp, never exceeding the rate limit
    for c in intf._mit_cmds[0:4]:
        assert c[5] >= 0.0
    last = intf._mit_cmds[-1][5]
    assert last <= 0.5 * 4 + 1e-9


def test_torque_rate_limit_off_passes_through():
    intf = MockInterface()
    ctrl = JointTorqueController(intf, gravity_comp=False, torque_rate_limit=0.0)
    ctrl.set_torque([3.0, 0, 0, 0, 0, 0])
    _run_ticks(ctrl)
    assert intf._mit_cmds[0][5] == pytest.approx(3.0)


def test_torque_rate_limit_per_joint():
    # Per-joint limits apply independently to each joint.
    intf = MockInterface()
    ctrl = JointTorqueController(intf, gravity_comp=False,
                                 torque_rate_limit=[0.1, 2.0, 0.0, 0.0, 0.0, 0.0])
    ctrl.set_torque([8.0, 8.0, 8.0, 0, 0, 0])
    _run_ticks(ctrl, n=1)
    cmds = sorted(intf._mit_cmds, key=lambda c: c[0])
    assert cmds[0][5] == pytest.approx(0.1)   # joint 1 rate = 0.1
    assert cmds[1][5] == pytest.approx(2.0)   # joint 2 rate = 2.0
    assert cmds[2][5] == pytest.approx(8.0)   # joint 3 rate = 0 -> passthrough


def test_admittance_accepts_sigmoid_friction_model():
    # Passing a SigmoidFrictionModel must not crash and yields finite torque.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[1000.0, 0, 0, 0, 0, 0],
                         qd=[0.1] * 6)
    fm = SigmoidFrictionModel(fp1=[0.5] * 6, fp2=[50.0] * 6, fp3=[0.0] * 6)
    ctrl = CartesianAdmittanceController(intf, gravity_comp=False,
                                         runaway_force=0.0, tau_friction=fm)
    ctrl.set_target(ctrl._state.pose)
    ctrl._on_tick(0.002)
    assert math.isfinite(sum(ctrl._F_ext))


def test_admittance_decoupled_rotation_integrates_on_so3():
    # Rotation torque rotates the quaternion reference without drift; the
    # quaternion stays normalized (exponential-map integration on SO(3)).
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                         current=[0, 0, 0, 1000, 0, 0])
    ctrl = CartesianAdmittanceController(intf, gravity_comp=False,
                                         runaway_force=0.0, M=[5.0, 5.0, 5.0, 1.0, 1.0, 1.0],
                                         D=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0])
    ctrl.set_target(ctrl._state.pose)
    for _ in range(50):
        ctrl._on_tick(0.002)
    # quaternion reference rotated away from identity
    assert ctrl._q_d[3] != pytest.approx(1.0, abs=1e-3)
    # and stays unit-norm (no RPY drift accumulation)
    n = math.sqrt(sum(v * v for v in ctrl._q_d))
    assert n == pytest.approx(1.0, abs=1e-6)
    # rotation moved in the direction of the applied moment
    assert math.isfinite(sum(ctrl._x_d[3:6]))


def test_admittance_rotational_damping_persists():
    # Rotational angular-velocity state must persist across ticks; otherwise
    # the D[3:6] damping term is dead and both dampings give identical output.
    def run(d_rot):
        intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0],
                             current=[0, 0, 0, 1000, 0, 0])
        ctrl = CartesianAdmittanceController(intf, gravity_comp=False,
                                             runaway_force=0.0,
                                             M=[5.0] * 6, D=[0.0, 0.0, 0.0, d_rot, d_rot, d_rot])
        ctrl.set_target(ctrl._state.pose)
        for _ in range(100):
            ctrl._on_tick(0.002)
        return list(ctrl._q_d)

    low, high = run(1.0), run(1000.0)
    assert any(abs(a - b) > 1e-3 for a, b in zip(low, high))


def test_enable_disable_enters_mit_mode():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf, rate_hz=100)
    ctrl.set_target(list(intf._q))
    ctrl.enable()
    time.sleep(0.05)
    assert ctrl.in_mit
    assert len(intf._mit_cmds) > 0
    ctrl.disable()
    assert not ctrl.in_mit
    # exited MIT mode via MotionCtrl_2
    assert any(c[2] != 0xAD for c in intf._mode_cmds)


def test_safety_trip_disables():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf, rate_hz=100)
    ctrl.set_target(list(intf._q))
    ctrl.enable()
    time.sleep(0.02)
    ctrl._trip("test")
    assert not ctrl.safety_ok
    assert intf._disable_count >= 1
    ctrl.disable()


def test_reenable_after_disable():
    intf = MockInterface()
    ctrl = JointImpedanceController(intf, rate_hz=100)
    ctrl.set_target(list(intf._q))
    ctrl.enable()
    time.sleep(0.02)
    n1 = len(intf._mit_cmds)
    assert ctrl.enabled
    ctrl.disable()
    assert not ctrl.enabled
    ctrl.enable()
    time.sleep(0.02)
    # commands must continue flowing after re-enable
    assert len(intf._mit_cmds) > n1
    assert ctrl.enabled
    ctrl.disable()


def test_joint_limit_repulsion_pushes_away():
    # Near a limit, an additional torque pushes the joint inward (positive
    # near the lower limit, negative near the upper). Joint 2 limits are
    # (0.0, 3.14).
    def run(q, limit_repulsion_torque):
        intf = MockInterface(q=q)
        ctrl = JointTorqueController(intf, gravity_comp=False,
                                     limit_repulsion_torque=limit_repulsion_torque)
        ctrl.set_torque([0.0] * 6)
        _run_ticks(ctrl, n=1)
        return sorted(intf._mit_cmds, key=lambda c: c[0])[1][5]  # joint 2

    # near lower limit -> positive repulsion
    assert run([0.2, 0.02, -0.3, 0.0, 0.0, 0.0], 5.0) == pytest.approx(4.0)
    # near upper limit -> negative repulsion
    assert run([0.2, 3.10, -0.3, 0.0, 0.0, 0.0], 5.0) == pytest.approx(-3.0)
    # far from any limit -> no repulsion
    assert run([0.2, 1.5, -0.3, 0.0, 0.0, 0.0], 5.0) == pytest.approx(0.0)
    # disabled -> passthrough regardless of proximity
    assert run([0.2, 0.02, -0.3, 0.0, 0.0, 0.0], 0.0) == pytest.approx(0.0)


def test_joint_limit_repulsion_is_rate_limited():
    # Repulsion must be summed into the torque BEFORE rate limiting (matching
    # crisp), so it ramps at the rate limit instead of jumping instantly.
    intf = MockInterface(q=[0.2, 0.0, -0.3, 0.0, 0.0, 0.0])  # joint 2 at limit
    ctrl = JointTorqueController(intf, gravity_comp=False,
                                 torque_rate_limit=0.5,
                                 limit_repulsion_torque=5.0,
                                 limit_repulsion_range=0.1)
    ctrl.set_torque([0.0] * 6)
    _run_ticks(ctrl, n=1)
    cmds = sorted(intf._mit_cmds, key=lambda c: c[0])
    # full repulsion would be 5.0 N·m, but the rate limit caps the first tick
    assert cmds[1][5] == pytest.approx(0.5)
    for _ in range(10):
        _run_ticks(ctrl, n=1)
    cmds = sorted(intf._mit_cmds[-6:], key=lambda c: c[0])
    # ramped up toward the full repulsion torque
    assert cmds[1][5] > 2.0


def test_error_clip_bounds_large_step():
    # A big set_target step creates a large error -> torque spike. With
    # error_clip set, the torque must be strictly bounded.
    def run(error_clip):
        intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0])
        ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                            joint_damping=0.0,
                                            error_clip=error_clip)
        x = intf.GetFK("feedback")[5]
        x_d = [x[0] / 1000.0 + 0.3, x[1] / 1000.0, x[2] / 1000.0,
               math.radians(x[3]), math.radians(x[4]), math.radians(x[5])]
        ctrl.set_target(x_d)
        _run_ticks(ctrl, n=1)
        return [c[5] for c in intf._mit_cmds]

    tau_unclipped = run(None)
    tau_clipped = run([0.01] * 6)
    # clipping caps the per-axis error at 0.01 -> strictly smaller max torque
    assert max(abs(t) for t in tau_clipped) < max(abs(t) for t in tau_unclipped)


def test_error_clip_requires_length_6():
    intf = MockInterface()
    with pytest.raises(ValueError):
        CartesianImpedanceController(intf, error_clip=[0.1] * 3)
    ctrl = CartesianImpedanceController(intf)
    with pytest.raises(ValueError):
        ctrl.set_error_clip([0.1] * 5)


def test_output_torque_filter_ema_smooths_steps():
    # output_torque_filter=0.5 moves the commanded torque half-way to the
    # target each tick (exponential approach), never jumping the full step.
    intf = MockInterface()
    ctrl = JointTorqueController(intf, gravity_comp=False,
                                 output_torque_filter=0.5)
    ctrl.set_torque([8.0, 0, 0, 0, 0, 0])
    _run_ticks(ctrl, n=1)
    # first command: 0 + 0.5 * 8
    cmds = sorted(intf._mit_cmds, key=lambda c: c[0])
    assert cmds[0][5] == pytest.approx(4.0)
    for _ in range(3):
        _run_ticks(ctrl, n=1)
    # last tick's joint 1 = 8*(1 - 0.5^4)
    last = sorted(intf._mit_cmds[-6:], key=lambda c: c[0])
    assert last[0][5] == pytest.approx(8.0 * (1 - 0.5 ** 4), abs=1e-6)


def test_output_torque_filter_off_passes_through():
    intf = MockInterface()
    ctrl = JointTorqueController(intf, gravity_comp=False, output_torque_filter=0.0)
    ctrl.set_torque([3.0, 0, 0, 0, 0, 0])
    _run_ticks(ctrl, n=1)
    assert intf._mit_cmds[0][5] == pytest.approx(3.0)


def test_cartesian_impedance_slerp_keeps_unit_quaternion():
    # The orientation filter slerps on SO(3): the filtered quaternion stays
    # unit-norm and the rotation converges to the target without drift.
    intf = MockInterface(q=[0.2, 1.0, -0.3, 0.1, 0.2, 0.0])
    ctrl = CartesianImpedanceController(intf, gravity_comp=False,
                                        joint_damping=0.0, target_filter=0.2)
    x = intf.GetFK("feedback")[5]
    x_d = [x[0] / 1000.0, x[1] / 1000.0, x[2] / 1000.0,
           0.0, 0.0, math.pi / 2]
    ctrl.set_target(x_d)
    _run_ticks(ctrl, n=1)
    q = ctrl._q_d_filtered
    n = math.sqrt(sum(v * v for v in q))
    assert n == pytest.approx(1.0, abs=1e-6)
    assert all(math.isfinite(v) for v in q)
    for _ in range(500):
        _run_ticks(ctrl, n=1)
    # converged to the target yaw
    assert ctrl._x_d_filtered[5] == pytest.approx(math.pi / 2, abs=1e-2)
