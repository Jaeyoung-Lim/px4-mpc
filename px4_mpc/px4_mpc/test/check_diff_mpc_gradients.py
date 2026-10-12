#!/usr/bin/env python3
"""
Finite-difference check of the differentiable fixed-wing NMPC (controllers.fixedwing_diff_mpc):
compares acados solution sensitivities du0/dtheta and du0/dx0 against central differences of
re-solved OCPs, and checks that the torch layer's backward() matches the same Jacobians.

Two cases: a nominal one (constraints inactive) and one with a low airspeed reference in a
tight turn with wind, where the soft AoA limit is active - the regime the path-following demo
is meant to exercise, and where sensitivities are most likely to go wrong.

Usage:
    python3 check_diff_mpc_gradients.py [--no-build]
"""
import argparse
import os
import sys

import numpy as np
import torch

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from controllers.fixedwing_diff_mpc import (  # noqa: E402
    FixedWingDiffMPC, THETA_NAMES, IDX_MODEL, diff_mpc, level_state, nominal_theta,
    path_stage_params)
from paths.dubins import dubins_path  # noqa: E402


def finite_difference(mpc, x0, theta, sp, iterate, rel_step=1e-5):
    def u0_at(x, th):
        u, info = mpc.solve(x, th, sp, warm_iterate=iterate)
        assert info['status'] == 0, f"FD solve failed with status {info['status']}"
        return u

    J_theta = np.zeros((mpc.nu, mpc.ntheta))
    for i in range(mpc.ntheta):
        h = rel_step * max(1.0, abs(theta[i]))
        tp, tm = theta.copy(), theta.copy()
        tp[i] += h
        tm[i] -= h
        J_theta[:, i] = (u0_at(x0, tp) - u0_at(x0, tm)) / (2 * h)

    J_x0 = np.zeros((mpc.nu, mpc.nx))
    for i in range(mpc.nx):
        h = rel_step * max(1.0, abs(x0[i]))
        xp, xm = x0.copy(), x0.copy()
        xp[i] += h
        xm[i] -= h
        J_x0[:, i] = (u0_at(xp, theta) - u0_at(xm, theta)) / (2 * h)
    return J_theta, J_x0


def set_tolerances(solver, nlp_tol, qp_tol):
    for field in ['tol_stat', 'tol_eq', 'tol_ineq', 'tol_comp']:
        solver.options_set(field, nlp_tol)
    for field in ['qp_tol_stat', 'qp_tol_eq', 'qp_tol_ineq', 'qp_tol_comp']:
        solver.options_set(field, qp_tol)


def column_errors(J, J_ref):
    scale = max(np.abs(J_ref).max(), 1e-8)
    return np.abs(J - J_ref).max(axis=0) / scale


def run_case(mpc, name, x0, theta, path):
    print(f'\n=== {name} ===')
    sp, _ = path_stage_params(path, x0[:3], s_hint=0.0, spacing=theta[11] * mpc.Ts, N=mpc.N)
    mpc.reset()
    u0, info = mpc.solve(x0, theta, sp)
    assert info['status'] == 0, f"nominal solve failed with status {info['status']}"
    X = info['X']
    alpha_max = max(np.arctan2(x[5], x[3]) for x in X)  # still-air proxy for logging
    print(f'u0 = {np.round(u0, 4)}, max AoA along plan ~ {np.degrees(alpha_max):.1f} deg')

    J_theta, J_x0 = mpc.jacobians(x0, theta, sp, info['iterate'])
    J_theta_fd, J_x0_fd = finite_difference(mpc, x0, theta, sp, info['iterate'])

    err_theta = column_errors(J_theta, J_theta_fd)
    err_x0 = column_errors(J_x0, J_x0_fd)
    print(f'{"param":>18s} {"|du0/dp| acados":>16s} {"|du0/dp| FD":>12s} {"rel err":>9s}')
    for i, n in enumerate(THETA_NAMES):
        print(f'{n:>18s} {np.linalg.norm(J_theta[:, i]):16.4e} '
              f'{np.linalg.norm(J_theta_fd[:, i]):12.4e} {err_theta[i]:9.2e}')
    print(f'du0/dtheta max rel err: {err_theta.max():.2e}')
    print(f'du0/dx0    max rel err: {err_x0.max():.2e}')

    # torch layer: backward of a random linear functional must equal seed^T J
    seed = np.random.default_rng(1).normal(size=mpc.nu)
    x0_t = torch.tensor(x0[None], requires_grad=True)
    th_t = torch.tensor(theta[None], requires_grad=True)
    mpc.reset()
    u0_t = diff_mpc(mpc, x0_t, th_t, sp[None])
    (u0_t[0] @ torch.tensor(seed)).backward()
    torch_err = max(np.abs(th_t.grad.numpy()[0] - seed @ J_theta).max(),
                    np.abs(x0_t.grad.numpy()[0] - seed @ J_x0).max())
    print(f'torch backward vs Jacobians: max abs err {torch_err:.2e}')
    return err_theta.max(), err_x0.max(), torch_err


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-build', action='store_true', help='reuse generated solvers')
    args = parser.parse_args()

    mpc = FixedWingDiffMPC(N=20, Ts=0.1, build=not args.no_build)
    # FD needs solutions far more accurate than the step: at the default tol a 1e-5 change
    # of a weight can leave the warm-started solve below tolerance, so it returns unchanged
    set_tolerances(mpc.solver, nlp_tol=1e-10, qp_tol=1e-12)
    path = dubins_path([0, 0, -50], 0.0, [40, 60, -50], np.pi, radius=20.0)

    # nominal: 4 m right of the path start, 15 deg course error, cruise airspeed, no wind
    theta = nominal_theta(airspeed_ref=12.0)
    x0 = level_state(np.array([0.0, 4.0, -50.0]), np.radians(15.0), 12.0)
    results = [run_case(mpc, 'nominal', x0, theta, path)]

    # low airspeed in a tight turn with crosswind and mismatched lift slope: AoA limit active
    theta = nominal_theta(airspeed_ref=8.0)
    theta[IDX_MODEL + 3] = 4.0                   # c_l_a
    theta[IDX_MODEL + 4:IDX_MODEL + 6] = [0.0, 4.0]  # wind from the west, 4 m/s
    tight = dubins_path([0, 0, -50], 0.0, [10, 25, -50], np.pi, radius=10.0)
    x0 = level_state(np.array([0.0, -3.0, -50.0]), 0.0, 9.0, phys=theta[IDX_MODEL:])
    results.append(run_case(mpc, 'low airspeed, tight turn, wind', x0, theta, tight))

    print(f"\nsolver stats: {mpc.stats}")
    ok = all(r[0] < 1e-3 and r[1] < 1e-3 and r[2] < 1e-8 for r in results)
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
