import pytest

from piper_sdk.controller import (
    mat_mul, mat_vec, mat_inv, mat_pinv_damped, transpose,
    clip, clip_vec, svd_extreme, condition_number,
    saturate_torque_rate, quat_slerp,
)


def test_quat_slerp_halfway_yaw():
    # Slerping from identity to a 90 deg yaw by t=0.5 must land on a 45 deg
    # yaw (half the rotation), not a linear blend of the components.
    import math
    q0 = [0.0, 0.0, 0.0, 1.0]
    q1 = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    q = quat_slerp(q0, q1, 0.5)
    # yaw = 2*atan2(z, w) = 45 deg
    yaw = 2 * math.atan2(q[2], q[3])
    assert yaw == pytest.approx(math.pi / 4, abs=1e-6)
    # unit norm preserved
    assert math.sqrt(sum(x * x for x in q)) == pytest.approx(1.0, abs=1e-6)


def test_quat_slerp_shortest_arc():
    # q and -q represent the same rotation; slerp must take the short way.
    import math
    q0 = [0.0, 0.0, 0.0, 1.0]
    q1 = [0.0, 0.0, -0.7, 0.7]
    q = quat_slerp(q0, q1, 0.5)
    assert q[3] >= 0.0  # flipped so the arc stays on the near side


def test_quat_slerp_endpoints():
    # t=0 -> q0, t=1 -> q1 (modulo sign, i.e. same rotation).
    import math
    q0 = [0.1 / math.sqrt(0.95), 0.2 / math.sqrt(0.95),
          0.3 / math.sqrt(0.95), 0.9 / math.sqrt(0.95)]
    q1 = [0.4, 0.5, 0.6, 0.3]
    n1 = math.sqrt(sum(x * x for x in q1))
    q1 = [x / n1 for x in q1]
    qa = quat_slerp(q0, q1, 0.0)
    qb = quat_slerp(q0, q1, 1.0)
    assert qa == pytest.approx(q0, abs=1e-6)
    assert abs(abs(sum(a * b for a, b in zip(qb, q1))) - 1.0) < 1e-6


def test_quat_slerp_near_identical():
    # dot close to 1 falls back to linear interpolation without dividing by 0.
    import math
    q0 = [0.0, 0.0, 0.0, 1.0]
    q1 = [1e-6, 0.0, 0.0, 1.0]
    q = quat_slerp(q0, q1, 0.5)
    assert math.isfinite(q[0])
    assert abs(q[0]) < 1e-6
    assert q[3] == pytest.approx(1.0, abs=1e-6)


def test_mat_mul_identity():
    A = [1.0, 2.0, 3.0, 4.0]
    I = [1.0, 0.0, 0.0, 1.0]
    assert mat_mul(A, I, 2, 2, 2) == pytest.approx(A)


def test_mat_mul_dimensions():
    A = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]   # 2x3
    B = [1.0, 0.0, 0.0, 1.0, 1.0, 1.0]   # 3x2
    C = mat_mul(A, B, 2, 3, 2)
    assert len(C) == 4


def test_mat_vec():
    A = [1.0, 0.0, 0.0, 1.0]
    v = [3.0, 4.0]
    assert mat_vec(A, v, 2, 2) == pytest.approx([3.0, 4.0])


def test_mat_inv_roundtrip():
    A = [4.0, 7.0, 2.0, 6.0]
    Ainv = mat_inv(A, 2)
    I = mat_mul(A, Ainv, 2, 2, 2)
    assert I == pytest.approx([1.0, 0.0, 0.0, 1.0], abs=1e-9)


def test_mat_inv_singular_raises():
    with pytest.raises(ValueError):
        mat_inv([1.0, 2.0, 2.0, 4.0], 2)


def test_transpose():
    A = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]  # 2x3
    At = transpose(A, 2, 3)
    assert len(At) == 6
    assert At[0] == 1.0 and At[2] == 2.0 and At[4] == 3.0


def test_mat_pinv_damped():
    # pseudo-inverse of identity is identity
    I = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    pinv = mat_pinv_damped(I, 3, 3, damp=0.0)
    for i in range(9):
        assert pinv[i] == pytest.approx(I[i], abs=1e-9)


def test_pinv_full_rank_property():
    # For full-rank square A with zero damping: A^+ = A^{-1}
    A = [1.0, 2.0, 3.0, 4.0]
    pinv = mat_pinv_damped(A, 2, 2, damp=0.0)
    assert pinv == pytest.approx(mat_inv(A, 2), abs=1e-9)


def test_clip():
    assert clip(5, 0, 3) == 3
    assert clip(-1, 0, 3) == 0
    assert clip(2, 0, 3) == 2


def test_clip_vec_scalar_and_list():
    assert clip_vec([-5.0, 2.0, 9.0], 0, 3) == [0.0, 2.0, 3.0]
    assert clip_vec([-5.0, 9.0], [0.0, 0.0], [3.0, 6.0]) == [0.0, 6.0]


def test_svd_extreme_identity():
    I = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    smax, smin = svd_extreme(I, 3, 3)
    assert smax == pytest.approx(1.0, abs=1e-6)
    assert smin == pytest.approx(1.0, abs=1e-6)


def test_condition_number():
    diag_mat = [3.0, 0.0, 0.0, 1.0]
    assert condition_number(diag_mat, 2, 2) == pytest.approx(3.0, rel=1e-3)


def test_saturate_torque_rate_limits_change_per_cycle():
    out = saturate_torque_rate([1.0] * 6, [0.0] * 6, 0.5)
    assert out == pytest.approx([0.5] * 6)
    # steady state passes through unchanged
    out = saturate_torque_rate([2.0] * 6, [2.0] * 6, 0.5)
    assert out == pytest.approx([2.0] * 6)


def test_saturate_torque_rate_per_joint_limits():
    out = saturate_torque_rate([1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                               [0.0] * 6,
                               [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert out == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])


def test_saturate_torque_rate_symmetric_and_negative():
    out = saturate_torque_rate([-1.0] * 6, [0.0] * 6, 0.5)
    assert out == pytest.approx([-0.5] * 6)
