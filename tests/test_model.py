import math

import pytest

from piper_sdk.controller import (
    ArmModel, FourierGravityModel, FirstOrderLPF,
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
