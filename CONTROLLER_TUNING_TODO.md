# Controller tuning TODO (Piper-specific)

Values below are placeholders / pre-port defaults. **None of them are
Piper-calibrated.** Everything marked "TUNE" needs real hardware measurement
or hand-tuning before the controllers are used for anything serious.

How to read this file:

- **Status**: `TUNE` = must measure/tune on the actual arm. `VERIFY` =
  looks plausible but should be confirmed against Piper specs / SDK.
  `OK` = known-good from SDK or geometry.
- **Find with**: the experiment or source that produces the real value.

---

## 1. Motor current → torque conversion (foundation for everything)

Every controller's external-torque estimate starts here. If these constants
are wrong, gravity comp, friction comp, and admittance are all wrong.

| Constant | Value | Where | Status |
|---|---|---|---|
| `_COEFFICIENT_1_3` | `1.18125` | `piper_msgs/msg_v2/feedback/arm_feedback_high_spd.py:76` | VERIFY |
| `_COEFFICIENT_4_6` | `0.95844` | same file `:77` | VERIFY |

**Find with**: joint-by-joint static hold test — command a known torque in
MIT mode, measure steady-state current, torque/current = real constant.
Cross-check against Piper datasheet motor torque constants (Kt × gearbox
ratio). These two coefficients are hardcoded by Agilex; confirm they match
the actual hardware revision.

---

## 2. Gravity model (calibrate per arm)

| Param | Value | Where | Status |
|---|---|---|---|
| Fourier model order | `1` | `controller/model.py:73` | TUNE |
| coefficients | `{}` (empty = zero gravity) | `FourierGravityModel.coeffs` | TUNE |

**Find with**: `demo/V2/calibrate_gravity.py --poses 60 --order 2` → saves
JSON, pass via `gravity=...` in the demos. Higher order = better fit but
more overfit risk. Run on the *specific* arm — each unit differs.

---

## 3. Friction model (new — crisp port, placeholders only)

| Param | Default | Where | Status |
|---|---|---|---|
| `fp1` (asymptotic magnitude, N·m) | `[0.1]*6` | `controller/model.py:133` | TUNE |
| `fp2` (slope, s/rad) | `[100.0]*6` | `controller/model.py:134` | TUNE |
| `fp3` (offset, rad/s) | `[0.0]*6` | `controller/model.py:135` | TUNE |

crisp's real values (for FROG, not Piper): fp1 ≈ `[0.55, 0.87, 0.64, 1.28, 0.84, 0.30, 0.56]`,
fp2 ≈ `[5.1, 9.1, 10.1, 5.6, 8.3, 17.1, 10.3]`, fp3 ≈ `[0.04, 0.03, -0.05, 0.04, 0.03, -0.02, 0.004]`.

**Find with**: constant-velocity sweep per joint (drive qd at several fixed
speeds, record `tau - gravity(q)`), fit the sigmoid per joint. fp2 is very
joint-speed-scale dependent. Tune at realistic speeds, not extremes.

---

## 4. Cartesian impedance (inner loop)

| Param | Default | Where | Status |
|---|---|---|---|
| `K` trans (N/m) | `[300, 300, 300]` | `cartesian_impedance.py:79` | TUNE |
| `K` rot (N·m/rad) | `[10, 10, 10]` | same | TUNE |
| `D` trans (N·s/m) | `[30, 30, 30]` | `:80` | TUNE |
| `D` rot (N·m·s/rad) | `[3, 3, 3]` | same | TUNE |
| `joint_damping` (N·m·s/rad) | `0.5` | `:81` | TUNE |
| `cond_thresh` | `50.0` | `:83` | VERIFY |
| `target_filter` (EMA α) | `0.0` (off) | `:84` | TUNE |
| `error_clip` (max err) | `None` (off) | `:95` | TUNE |
| `nullspace_damping` | `0.0` (off) | `:85` | TUNE |
| `nullspace_stiffness` | `0.0` (off) | `:86` | TUNE |
| `nullspace_max_tau` (N·m) | `0.0` (off) | `:87` | TUNE |

**Find with**: `demo/V2/tune_impedance.py` (joint) and
`demo_cartesian_impedance.py`. Start K low, raise until stiff but stable.
D ~ `2*sqrt(K·J_eff)` for critical damping. `target_filter` ≈ 0.2–0.5 smooths
`set_target` steps (rotation is slerped on SO(3), so no wrap artifacts).
crisp defaults: K 500/30, D auto `2*sqrt(K)`.
Nullspace: `nullspace_stiffness` springs q back toward `q_ref` (defaults to the
starting pose) through `I - J^+ J`; pair with `nullspace_damping` ~ `2*sqrt(K_ns)`
and clamp with `nullspace_max_tau` so it never fights the main task
(crisp default max_tau 5.0).

---

## 5. Cartesian admittance (outer loop, on top of impedance)

| Param | Default | Where | Status |
|---|---|---|---|
| `M` trans (kg) | `[5, 5, 5]` | `cartesian_admittance.py:56` | TUNE |
| `M` rot (kg·m²) | `[1, 1, 1]` | same | TUNE |
| `D` trans (N·s/m) | `[100, 100, 100]` | `:57` | TUNE |
| `D` rot (N·m·s/rad) | `[10, 10, 10]` | same | TUNE |
| `tau_lpf_alpha` | `0.1` | `:59` | TUNE |
| `vel_linear_max` (m/s) | `0.3` | `:60` | VERIFY |
| `vel_angular_max` (rad/s) | `1.0` | `:61` | VERIFY |
| `runaway_force` (N) | `30.0` | `:62` | TUNE |
| `runaway_time` (s) | `0.5` | `:63` | TUNE |

**Find with**: `demo/V2/tune_admittance.py` and `demo_cartesian_admittance.py`.
Lower M = more responsive, higher = sluggish. D sets max follow speed
(`v_max ≈ F_ext / D`). `runaway_force/time` trip the safety system — set to
just above the max legitimate contact force. crisp defaults: M 1.0/0.1,
D 50/5.

---

## 6. Joint-space controllers

| Param | Default | Where | Status |
|---|---|---|---|
| Joint impedance `K` (N·m/rad) | `[20]*6` | `joint_impedance.py:32` | TUNE |
| Joint impedance `D` (N·m·s/rad) | `[1]*6` | `:33` | TUNE |
| Joint admittance `D` | `[2.0]*6` | `joint_admittance.py:50` | TUNE |

**Find with**: `demo/V2/tune_impedance.py`.

---

## 7. Safety / limits in BaseController

| Param | Default | Where | Status |
|---|---|---|---|
| `tau_limit` (N·m) | `8.0` | `base.py:56` | VERIFY |
| `torque_rate_limit` (N·m/tick) | `0.0` (off) | `base.py:82` | TUNE |
| `output_torque_filter` (EMA α) | `0.0` (off) | `base.py:83` | TUNE |
| `limit_repulsion_torque` (N·m) | `0.0` (off) | `base.py:84` | TUNE |
| `limit_repulsion_range` (rad) | `0.1` | `base.py:85` | TUNE |
| `error_clip` (6-vec, units) | `None` (off) | `cartesian_impedance.py:76` | TUNE |
| `DEFAULT_VELOCITY_LIMITS` (rad/s) | `[2, 2, 2, 2.5, 2.5, 3]` | `model.py:35` | VERIFY |

**Find with**:
- `tau_limit`: per-joint max continuous torque from datasheet; must be below
  the MIT protocol's own clamp.
- `torque_rate_limit`: measure the largest step torque the arm handles
  without overshoot/ringing; crisp uses 0.5 N·m/cycle. Start ~0.2–0.5.
- `output_torque_filter`: smooths residual torque chatter after rate limiting;
  crisp default 0.5. Too high (close to 1) removes the smoothing; too low
  delays response.
- `limit_repulsion_torque`/`range`: ramp torque near limits so the arm keeps
  being controllable instead of tripping. crisp uses range 0.1 rad / max 5 N·m;
  verify against `tau_limit` (repulsion must stay below it).
- `error_clip`: cap task-space error entering the stiffness law to bound
  torque spikes on `set_target` steps. crisp default 0.1 (lin) / 0.5 (rot);
  start symmetric per-axis.
- velocity limits: query actual max via
  `demo/V2/V2_piper_ctrl_motor_max_spd.py`; the constants may be optimistic.

---

## 8. State estimation filters

| Param | Default | Where | Status |
|---|---|---|---|
| `tau_alpha` (torque LPF) | `0.2` | `state.py:31` | TUNE |
| `qd_alpha` (velocity LPF) | `0.3` | `state.py:31` | TUNE |

**Find with**: static hold + impulse — enough smoothing to kill noise,
little enough to not add lag. Current-based torque is noisy; if admittance
is jittery, lower `tau_alpha`.

---

## 9. Kinematics

| Param | Default | Where | Status |
|---|---|---|---|
| Jacobian FD `_eps` | `1e-4` | `model.py:164` | OK |
| `DEFAULT_JOINT_LIMITS` | — | `model.py:25` | VERIFY |

`_eps` is a finite-difference step; only matters if the Jacobian shows
noise. Joint limits come from the SDK param manager but the static table
should be confirmed per arm.

---

## General tuning rules

1. **Order matters**: torque constant → gravity → friction → impedance →
   admittance. Fix each layer before tuning the next.
2. Tune with `gravity_comp=True` first (no friction), then add friction.
3. Start every new feature **disabled** (default 0.0), raise slowly.
4. Keep the e-stop within reach; MIT mode can damage the arm.
5. Record working values per-arm — they do not transfer between units.
