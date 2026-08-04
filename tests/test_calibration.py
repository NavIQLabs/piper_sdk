import random

import pytest

from piper_sdk.controller import fit_fourier_gravity, FourierGravityModel

random.seed(42)


def _fake_samples(coeffs_true, order, n=400):
    '''Generate (q, tau_est) samples from a known Fourier gravity model.'''
    model = FourierGravityModel(order=order, coeffs=coeffs_true)
    samples = []
    for _ in range(n):
        # spread joints across most of [-pi, pi] so the Fourier basis is
        # well conditioned
        q = [random.uniform(-3.0, 3.0) for _ in range(6)]
        samples.append((q, model.gravity(q)))
    return samples


def test_fit_recovers_coefficients():
    true_coeffs = [
        [0.8, 0.2, 0.1, 0.0],
        [0.5, 0.3, 0.0, 0.1],
        [1.2, -0.4, 0.2, 0.0],
        [0.3, 0.1, 0.0, 0.0],
        [0.6, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ]
    order = 2
    samples = _fake_samples(true_coeffs, order)
    fitted = fit_fourier_gravity(samples, order=order)
    assert fitted["order"] == order
    assert len(fitted["coeffs"]) == 6
    for j in range(6):
        assert fitted["coeffs"][j] == pytest.approx(true_coeffs[j], abs=1e-6)


def test_fit_zero_gravity():
    zero = [[0.0, 0.0] for _ in range(6)]
    samples = _fake_samples(zero, order=1, n=50)
    fitted = fit_fourier_gravity(samples, order=1)
    for j in range(6):
        assert max(abs(v) for v in fitted["coeffs"][j]) < 1e-8


def test_fit_requires_samples():
    with pytest.raises(ValueError):
        fit_fourier_gravity([])
