#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Pure-python small matrix/vector helpers used by the controller subpackage.

Kept dependency-free (matching the SDK's hand-rolled math style in
``kinematics/piper_fk.py``) and sized for 6-DOF robotics math.

Matrices are represented as flat ``list[float]`` in row-major order; vectors
are ``list[float]`` of length n.
'''

import math

def mat_mul(A, B, m, l, n):
    '''
    Multiply row-major flat matrices.

    :param A: m x l matrix (flat, row-major)
    :param B: l x n matrix (flat, row-major)
    :param m: rows of A
    :param l: cols of A / rows of B
    :param n: cols of B
    :return: m x n matrix (flat, row-major)
    '''
    out = [0.0] * (m * n)
    for i in range(m):
        ai = l * i
        oi = n * i
        for k in range(l):
            av = A[ai + k]
            if av == 0.0:
                continue
            bj = n * k
            for j in range(n):
                out[oi + j] += av * B[bj + j]
    return out

def mat_vec(A, v, m, l):
    '''
    Multiply an m x l matrix (flat, row-major) by a length-l vector.

    :return: length-m vector
    '''
    out = [0.0] * m
    for i in range(m):
        s = 0.0
        ai = l * i
        for k in range(l):
            s += A[ai + k] * v[k]
        out[i] = s
    return out

def transpose(A, m, l):
    '''
    Transpose an m x l matrix (flat, row-major).

    :return: l x m matrix (flat, row-major)
    '''
    out = [0.0] * (m * l)
    for i in range(m):
        ai = l * i
        for j in range(l):
            out[m * j + i] = A[ai + j]
    return out

def identity(n):
    '''Return the n x n identity matrix (flat, row-major).'''
    out = [0.0] * (n * n)
    for i in range(n):
        out[n * i + i] = 1.0
    return out

def diag(v):
    '''Return a diagonal matrix (flat, row-major) from a vector.'''
    n = len(v)
    out = [0.0] * (n * n)
    for i in range(n):
        out[n * i + i] = v[i]
    return out

def mat_add(A, B, m, n):
    '''Element-wise sum of two m x n matrices (flat, row-major).'''
    return [a + b for a, b in zip(A, B)]

def vec_add(a, b):
    '''Element-wise sum of two vectors.'''
    return [x + y for x, y in zip(a, b)]

def vec_sub(a, b):
    '''Element-wise difference of two vectors.'''
    return [x - y for x, y in zip(a, b)]

def vec_scale(a, s):
    '''Scale a vector by scalar s.'''
    return [x * s for x in a]

def vec_dot(a, b):
    '''Dot product of two vectors.'''
    return sum(x * y for x, y in zip(a, b))

def vec_norm(a):
    '''Euclidean norm of a vector.'''
    return math.sqrt(vec_dot(a, a))

def mat_inv(A, n):
    '''
    Invert an n x n matrix (flat, row-major) via Gauss-Jordan elimination.

    :raises ValueError: if the matrix is singular (within tolerance).
    '''
    aug = [0.0] * (n * n * 2)
    for i in range(n):
        ai = 2 * n * i
        for j in range(n):
            aug[ai + j] = A[n * i + j]
        aug[ai + n + i] = 1.0
    for col in range(n):
        pivot = col
        best = abs(aug[2 * n * col + col])
        for r in range(col + 1, n):
            v = abs(aug[2 * n * r + col])
            if v > best:
                best = v
                pivot = r
        if best < 1e-12:
            raise ValueError("singular matrix in mat_inv")
        if pivot != col:
            pi = 2 * n * pivot
            ci = 2 * n * col
            for j in range(2 * n):
                aug[pi + j], aug[ci + j] = aug[ci + j], aug[pi + j]
        ci = 2 * n * col
        piv = aug[ci + col]
        for j in range(2 * n):
            aug[ci + j] /= piv
        for r in range(n):
            if r == col:
                continue
            ri = 2 * n * r
            f = aug[ri + col]
            if f == 0.0:
                continue
            for j in range(2 * n):
                aug[ri + j] -= f * aug[ci + j]
    out = [0.0] * (n * n)
    for i in range(n):
        ai = 2 * n * i
        for j in range(n):
            out[n * i + j] = aug[ai + n + j]
    return out

def mat_pinv_damped(A, m, l, damp=1e-3):
    '''
    Damped least-squares pseudo-inverse of an m x l matrix.

    ``A^+ = A^T (A A^T + damp^2 I)^{-1}`` (when m <= l this is the usual
    over-constrained form; both handled via (m x m) inverse).

    :return: l x m matrix (flat, row-major)
    '''
    AAT = mat_mul(A, transpose(A, m, l), m, l, m)
    for i in range(m):
        AAT[m * i + i] += damp * damp
    AAT_inv = mat_inv(AAT, m)
    return mat_mul(transpose(A, m, l), AAT_inv, l, m, m)

def clip(v, lo, hi):
    '''Clamp scalar v into [lo, hi].'''
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def saturate_torque_rate(tau_desired, tau_prev, delta_tau_max):
    '''
    Clamp the per-cycle torque-rate limit (rate limiting).

    ``tau_out = tau_prev + clip(tau_desired - tau_prev, -d, +d)``

    :param tau_desired: commanded torque vector.
    :param tau_prev: torque vector sent on the previous cycle.
    :param delta_tau_max: max allowed change per cycle, either a scalar applied
        to every joint or a per-joint sequence of the same length.
    :return: rate-limited torque vector.
    :raises ValueError: if ``delta_tau_max`` is per-joint and shorter than
        ``tau_desired``.
    '''
    out = []
    per_joint = isinstance(delta_tau_max, (list, tuple))
    if per_joint and len(delta_tau_max) < len(tau_desired):
        raise ValueError(
            "per-joint delta_tau_max length %d < tau_desired length %d"
            % (len(delta_tau_max), len(tau_desired)))
    for i, t in enumerate(tau_desired):
        d = delta_tau_max[i] if per_joint else delta_tau_max
        out.append(tau_prev[i] + clip(t - tau_prev[i], -d, d))
    return out

def svd_extreme(A, m, l, max_iter=200, tol=1e-12):
    '''
    Approximate largest and smallest singular values of an m x l matrix via
    power / inverse iteration on A^T A.

    :return: ``(sigma_max, sigma_min)``
    '''
    # A^T A (l x l)
    At = transpose(A, m, l)
    ATA = mat_mul(At, A, l, m, l)

    # ---- largest singular value via power iteration
    v = [1.0] * l
    v = vec_scale(v, 1.0 / (vec_norm(v) or 1.0))
    lam = 0.0
    for _ in range(max_iter):
        w = mat_vec(ATA, v, l, l)
        n = vec_norm(w)
        if n < tol:
            break
        v = vec_scale(w, 1.0 / n)
        lam = n
    sigma_max = math.sqrt(max(lam, 0.0))

    # ---- smallest singular value via inverse iteration on ATA
    try:
        ATA_inv = mat_inv(ATA, l)
    except ValueError:
        return sigma_max, 0.0
    v = [1.0] * l
    v = vec_scale(v, 1.0 / (vec_norm(v) or 1.0))
    lam = 0.0
    for _ in range(max_iter):
        w = mat_vec(ATA_inv, v, l, l)
        n = vec_norm(w)
        if n < tol:
            break
        v = vec_scale(w, 1.0 / n)
        lam = n
    sigma_min = math.sqrt(1.0 / lam) if lam > tol else 0.0
    return sigma_max, sigma_min

def condition_number(A, m, l, max_iter=200, tol=1e-12):
    '''Condition number ``sigma_max / sigma_min`` of an m x l matrix.'''
    smax, smin = svd_extreme(A, m, l, max_iter=max_iter, tol=tol)
    if smin < 1e-12:
        return float('inf')
    return smax / smin

def clip_vec(v, lo, hi):
    '''
    Clamp each element of vector v into [lo, hi].

    ``lo``/``hi`` may be scalars (broadcast) or iterables of the same length.
    '''
    if hasattr(lo, '__iter__') or hasattr(hi, '__iter__'):
        lo_ = list(lo) if hasattr(lo, '__iter__') else None
        hi_ = list(hi) if hasattr(hi, '__iter__') else None
        out = []
        for i, x in enumerate(v):
            l = lo_[i] if lo_ is not None else lo
            h = hi_[i] if hi_ is not None else hi
            out.append(clip(x, l, h))
        return out
    return [clip(x, lo, hi) for x in v]
