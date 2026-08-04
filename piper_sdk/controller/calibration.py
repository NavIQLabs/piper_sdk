#!/usr/bin/env python3
# -*-coding:utf8-*-
'''
Offline (non-realtime) utilities for controller calibration and tuning.

:func:`fit_fourier_gravity` - least-squares fit of the per-joint Fourier
gravity model from ``(q, tau_est)`` samples gathered at static poses by
``demo/V2/calibrate_gravity.py``.
'''

import math
import json

from .mathx import mat_mul, mat_vec, mat_inv, transpose


def _design_row(qi, order):
    '''Fourier features ``[sin(q), cos(q), sin(2q), cos(2q), ...]`` for one joint.'''
    row = []
    for k in range(1, order + 1):
        row.append(math.sin(k * qi))
        row.append(math.cos(k * qi))
    return row


def fit_fourier_gravity(samples, order=2, damp=1e-6):
    '''
    Fit a per-joint Fourier gravity model from static ``(q, tau_est)`` samples.

    :param samples: list of ``(q_list, tau_list)`` where each ``q_list`` is a
        6-element joint config [rad] and ``tau_list`` its 6 torque estimates.
    :param order: number of Fourier harmonics per joint.
    :param damp: ridge term for the least-squares solve.
    :return: dict ``{"order": order, "coeffs": [[...x6...]]}`` matching the
        layout expected by :class:`FourierGravityModel`.
    '''
    if not samples:
        raise ValueError("no samples provided")
    n_params = 2 * order
    coeffs = []
    for j in range(6):
        rows = []
        b = []
        for q, tau in samples:
            rows.append(_design_row(q[j], order))
            b.append(tau[j])
        A = [v for row in rows for v in row]           # N x n_params flat
        N = len(b)
        # normal equations: (A^T A + damp I) c = A^T b
        ATA = mat_mul(transpose(A, N, n_params), A, n_params, N, n_params)
        for i in range(n_params):
            ATA[n_params * i + i] += damp
        ATA_inv = mat_inv(ATA, n_params)
        Atb = mat_vec(transpose(A, N, n_params), b, n_params, N)
        c = mat_vec(ATA_inv, Atb, n_params, n_params)
        coeffs.append(c)
    return {"order": order, "coeffs": coeffs}


def save_gravity_model(path, model_data):
    '''Save the fitted gravity model dict to a JSON file.'''
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(model_data, f, indent=2)


def load_gravity_model(path):
    '''Load a fitted gravity model dict from a JSON file.'''
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)
