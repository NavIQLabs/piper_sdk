import math

import pytest

from piper_sdk.controller import (
    ArmModel, FourierGravityModel, FirstOrderLPF, SigmoidFrictionModel,
    DEFAULT_JOINT_LIMITS,
)


Q_ZERO = [0.0] * 6
Q_HOME = [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0]


def test_fk_returns_6d():
    m = ArmModel()
    pose = m.fk(Q_ZERO)
    assert len(pose) == 6
    # base-ish configuration should keep z above zero (mm)
    assert pose[2] > 0


def test_fk_pose_units():
    m = ArmModel()
    xyz, rpy = m.fk_pose(Q_ZERO)
    assert len(xyz) == 3 and len(rpy) == 3
    assert xyz[0] == pytest.approx(m.fk(Q_ZERO)[0] / 1000.0)


def test_jacobian_shape_and_finite():
    m = ArmModel()
    J = m.jacobian(Q_ZERO)
    assert len(J) == 36
    for v in J:
        assert math.isfinite(v)


def test_jacobian_translation_sensitivity():
    # Moving joint 2 (shoulder) changes x/y of the end-effector in home pose.
    m = ArmModel()
    J = m.jacobian(Q_HOME)
    # translational rows, joint 2 column -> non-trivial value
    assert abs(J[1]) > 1e-4 or abs(J[0]) > 1e-4


def test_jacobian_pinv_identity_property():
    m = ArmModel()
    q = [0.2, 1.0, -0.3, 0.1, 0.2, 0.0]  # non-singular config
    J = m.jacobian(q)
    pinv = m.jacobian_pinv(q, damp=1e-9)
    # J * J^+ should be approx identity (full rank square)
    JJp = [0.0] * 36
    for i in range(6):
        for k in range(6):
            for j in range(6):
                JJp[6 * i + k] += J[6 * i + j] * pinv[6 * j + k]
    I = [0.0] * 36
    for i in range(6):
        I[6 * i + i] = 1.0
    assert JJp == pytest.approx(I, abs=1e-4)


def test_joint_limits_defaults():
    assert len(DEFAULT_JOINT_LIMITS) == 6
    assert DEFAULT_JOINT_LIMITS[1] == (0.0, 3.14)


def test_is_within_limits():
    m = ArmModel()
    assert m.is_within_limits([0.0, 1.5, -1.0, 0.0, 0.0, 0.0])
    assert not m.is_within_limits([0.0, 4.0, 0.0, 0.0, 0.0, 0.0])


def test_joint_limit_torque_ramps_toward_limits():
    # Joint 2 limits (0.0, 3.14); torque rises linearly inside safe_range and
    # pushes inward: + near lower, - near upper, 0 far away.
    m = ArmModel()
    t = m.joint_limit_torque([0.0, 0.05, 0.0, 0.0, 0.0, 0.0], safe_range=0.1, max_torque=5.0)
    assert t[1] == pytest.approx(5.0 * 0.5)  # halfway inside the ramp
    t = m.joint_limit_torque([0.0, 3.09, 0.0, 0.0, 0.0, 0.0], safe_range=0.1, max_torque=5.0)
    assert t[1] == pytest.approx(-5.0 * 0.5)
    t = m.joint_limit_torque([0.0, 1.5, 0.0, 0.0, 0.0, 0.0], safe_range=0.1, max_torque=5.0)
    assert t[1] == pytest.approx(0.0)
    # at the limit -> full max_torque
    t = m.joint_limit_torque([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], safe_range=0.1, max_torque=5.0)
    assert t[1] == pytest.approx(5.0)


def test_fourier_gravity_zeros_when_unfitted():
    m = FourierGravityModel(order=1)
    assert m.gravity(Q_ZERO) == [0.0] * 6


def test_fourier_gravity_value():
    # sin only: g_i = a*sin(q_i) + b*cos(q_i)
    coeffs = [[1.0, 0.0] for _ in range(6)]
    m = FourierGravityModel(order=1, coeffs=coeffs)
    g = m.gravity([math.pi / 2, 0, 0, 0, 0, 0])
    assert g[0] == pytest.approx(1.0)
    assert g[1] == pytest.approx(0.0)


def test_first_order_lpf():
    f = FirstOrderLPF(alpha=0.5)
    assert f.update(10.0) == pytest.approx(10.0)
    assert f.update(0.0) == pytest.approx(5.0)
    assert f.update(0.0) == pytest.approx(2.5)


def test_sigmoid_friction_zero_at_rest():
    fm = SigmoidFrictionModel(fp1=[1.0] * 6, fp2=[100.0] * 6, fp3=[0.0] * 6)
    f = fm.friction([0.0] * 6)
    assert all(abs(v) < 1e-3 for v in f)


def test_sigmoid_friction_saturates_and_antisymmetric():
    fm = SigmoidFrictionModel(fp1=[1.0] * 6, fp2=[100.0] * 6, fp3=[0.0] * 6)
    pos = fm.friction([1.0] * 6)
    neg = fm.friction([-1.0] * 6)
    # saturates toward fp1/2 = 0.5 for this parameterization
    assert pos[0] == pytest.approx(0.5, abs=1e-3)
    assert neg[0] == pytest.approx(-0.5, abs=1e-3)


def test_sigmoid_friction_smooth_around_zero():
    fm = SigmoidFrictionModel(fp1=[1.0] * 6, fp2=[100.0] * 6, fp3=[0.0] * 6)
    # small positive velocity -> small positive friction, strictly increasing
    f_small = fm.friction([0.05] * 6)
    f_zero = fm.friction([0.0] * 6)
    assert f_small[0] > f_zero[0]
    assert f_small[0] < 0.5


def test_sigmoid_friction_offset_shifts_curve():
    # fp3 shifts the sigmoid transition point. Rest friction stays zero by
    # construction, but the model saturates earlier on the negative side.
    fm = SigmoidFrictionModel(fp1=[1.0] * 6, fp2=[100.0] * 6, fp3=[0.5] * 6)
    assert abs(fm.friction([0.0] * 6)[0]) < 1e-6  # rest zero by construction
    # transition midpoint sits at qd = -fp3 with friction = -fp1/2
    assert fm.friction([-0.5] * 6)[0] == pytest.approx(-0.5, abs=1e-3)
    # already saturated to 0 at qd = -0.3 where the unshifted model is -0.5
    assert fm.friction([-0.3] * 6)[0] == pytest.approx(0.0, abs=1e-3)
    fm0 = SigmoidFrictionModel(fp1=[1.0] * 6, fp2=[100.0] * 6, fp3=[0.0] * 6)
    assert fm0.friction([-0.3] * 6)[0] == pytest.approx(-0.5, abs=1e-3)
