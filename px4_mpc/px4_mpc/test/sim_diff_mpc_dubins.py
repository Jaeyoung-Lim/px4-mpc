#!/usr/bin/env python3
"""
Closed-loop Dubins path following with the differentiable fixed-wing NMPC at fixed theta (the
untrained baseline), against a plant with model mismatch and wind.

Like PX4 (EKF2 estimates horizontal wind), the baseline MPC is given the true horizontal wind
by default; vertical wind and the mass / lift-slope mismatch stay unknown to it.

The circuit is two Dubins paths A -> B -> A joined into a closed loop, flown for --laps laps.
Prints cross-track / AoA / solver statistics and optionally saves a plot.

Usage:
    python3 sim_diff_mpc_dubins.py [--airspeed 12] [--wind 0 4 0] [--radius 20]
                                   [--laps 2] [--plot out.png] [--no-build]
                                   [--no-wind-estimate | --oracle]
"""
import argparse
import os
import sys
import time

import casadi as cs
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from controllers.fixedwing_diff_mpc import (  # noqa: E402
    FixedWingDiffMPC, NOMINAL_MODEL, air_data, level_state, make_discrete_dynamics,
    nominal_theta, path_stage_params)
from paths.dubins import Path, dubins_path  # noqa: E402


def dubins_circuit(radius, altitude=-50.0):
    a, b = np.array([0.0, 0.0, altitude]), np.array([60.0, 120.0, altitude])
    out = dubins_path(a, 0.0, b, np.pi / 2, radius)
    back = dubins_path(b, np.pi / 2, a, 0.0, radius)
    return Path(out.segments + back.segments, loop=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--airspeed', type=float, default=12.0)
    parser.add_argument('--radius', type=float, default=20.0)
    parser.add_argument('--wind', type=float, nargs=3, default=[0.0, 4.0, 0.0])
    parser.add_argument('--c-l-a', type=float, default=4.5, help='true plant lift slope')
    parser.add_argument('--mass', type=float, default=1.1, help='true plant mass')
    parser.add_argument('--laps', type=float, default=2.0)
    parser.add_argument('--oracle', action='store_true',
                        help='give the MPC the true plant parameters and wind (upper bound)')
    parser.add_argument('--no-wind-estimate', action='store_true',
                        help='do not give the MPC the horizontal wind estimate')
    parser.add_argument('--plot', type=str, default=None)
    parser.add_argument('--no-build', action='store_true')
    args = parser.parse_args()

    mpc = FixedWingDiffMPC(build=not args.no_build)
    true_phys = NOMINAL_MODEL.copy()
    true_phys[0], true_phys[3] = args.mass, args.c_l_a
    true_phys[4:7] = args.wind
    if args.oracle:
        mpc_phys = true_phys
    else:
        mpc_phys = NOMINAL_MODEL.copy()
        if not args.no_wind_estimate:
            mpc_phys[4:6] = true_phys[4:6]  # horizontal wind, as estimated by PX4
    theta = nominal_theta(args.airspeed, model_params=mpc_phys)
    plant = make_discrete_dynamics(mpc.Ts, n_sub=4)

    xs = cs.SX.sym('x', 10)
    ph = cs.SX.sym('ph', 7)
    _, _, V_e, alpha_e = air_data(xs, ph)
    air = cs.Function('air', [xs, ph], [V_e, alpha_e])

    path = dubins_circuit(args.radius)
    x = level_state(path.pose(0.0)[0], 0.0, args.airspeed, phys=true_phys)
    s = 0.0
    steps = int(args.laps * path.length / (args.airspeed * mpc.Ts))
    log = {k: [] for k in ['pos', 'ref', 'xtrack', 'alt_err', 'alpha', 'V', 'u', 'status',
                           't_solve']}

    for _ in range(steps):
        # ground speed for reference spacing: |R v| horizontal
        v_body = x[3:6]
        qw, qx, qy, qz = x[6:10]
        R = np.array([[1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                      [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
                      [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)]])
        gs = max(np.linalg.norm((R @ v_body)[:2]), 3.0)
        sp, s = path_stage_params(path, x[:3], s, gs * mpc.Ts, mpc.N)

        t0 = time.perf_counter()
        u, info = mpc.solve(x, theta, sp, slot=0)
        log['t_solve'].append(time.perf_counter() - t0)
        log['status'].append(info['status'])

        ref = path.pose(s)[0]
        V, alpha = (float(v) for v in air(x, true_phys))
        log['pos'].append(x[:3].copy())
        log['ref'].append(ref)
        log['xtrack'].append(np.linalg.norm(x[:2] - ref[:2]))
        log['alt_err'].append(x[2] - ref[2])
        log['alpha'].append(alpha)
        log['V'].append(V)
        log['u'].append(u)
        x = np.array(plant(x, u, true_phys)).ravel()

    xt = np.array(log['xtrack'])
    status = np.array(log['status'])
    t_solve = np.array(log['t_solve'])
    alpha = np.degrees(np.array(log['alpha']))
    V = np.array(log['V'])
    # skip the first half lap for steady-state statistics
    ss = slice(len(xt) // 4, None)
    print(f'steps {len(xt)}, path length {path.length:.0f} m, wind {args.wind}, '
          f'c_l_a {args.c_l_a}, mass {args.mass}')
    alt = np.array(log['alt_err'])
    print(f'cross-track RMS {np.sqrt(np.mean(xt[ss]**2)):.2f} m, max {xt[ss].max():.2f} m; '
          f'altitude error RMS {np.sqrt(np.mean(alt[ss]**2)):.2f} m')
    print(f'airspeed {V[ss].min():.1f}..{V[ss].max():.1f} m/s, AoA max {alpha.max():.1f} deg '
          f'(MPC limit {np.degrees(mpc.h_ub[0]):.1f})')
    print(f'solver: {np.mean(status != 0) * 100:.1f}% non-zero status, '
          f'solve time median {np.median(t_solve) * 1e3:.1f} ms, '
          f'p95 {np.percentile(t_solve, 95) * 1e3:.1f} ms')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        pos = np.array(log['pos'])
        ref_s = np.linspace(0, path.length, 400)
        ref_pts = path.sample(ref_s)[0]
        t = np.arange(len(xt)) * mpc.Ts
        fig, ax = plt.subplots(1, 2, figsize=(12, 5))
        ax[0].plot(ref_pts[:, 1], ref_pts[:, 0], 'k--', lw=1, label='Dubins path')
        ax[0].plot(pos[:, 1], pos[:, 0], lw=1.5, label='flown')
        ax[0].set_xlabel('East [m]')
        ax[0].set_ylabel('North [m]')
        ax[0].set_aspect('equal')
        ax[0].legend()
        ax[1].plot(t, xt, label='cross-track [m]')
        ax[1].plot(t, alpha, label='AoA [deg]')
        ax[1].axhline(np.degrees(mpc.h_ub[0]), color='gray', ls=':', lw=1)
        ax[1].set_xlabel('time [s]')
        ax[1].legend()
        if args.oracle:
            label = 'Oracle'
        else:
            label = 'Baseline (no wind est.)' if args.no_wind_estimate else 'Baseline'
        fig.suptitle(f'{label} NMPC, V_ref {args.airspeed} m/s, wind {args.wind} m/s')
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f'saved {args.plot}')


if __name__ == '__main__':
    main()
