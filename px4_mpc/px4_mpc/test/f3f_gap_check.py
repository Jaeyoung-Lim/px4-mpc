#!/usr/bin/env python3
"""
F3F horizon-gap check: how much race time does a short-horizon MPC lose against the offline
minimum-time race for the same model?

1. Fly a 10-leg race with controllers.f3f_mpc in closed loop (plant = same model, finer RK4).
2. Solve the full-race minimum-time OCP (f3f.race_optimal), warm-started from that race.
3. Report per-leg times, energy, constraint margins, and optionally plot both trajectories.

If the gap is large, a learned terminal value (cost-to-go) has something to recover; if it is
small, short-horizon MPC is already near-optimal for this setting.

Usage:
    python3 f3f_gap_check.py [--no-build] [--N 40] [--w-dist-e 1] [--w-energy-e 1]
                             [--wind 10] [--h-min 20] [--keep-energy] [--N-leg 40]
                             [--plot out.png] [--save out.npz]
"""
import argparse
import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from controllers.f3f_mpc import F3FMPC, default_theta  # noqa: E402
from f3f.course import BASE_A, BASE_B  # noqa: E402
from f3f.glider import Glider, Ridge, make_diagnostics  # noqa: E402
from f3f.race_optimal import node_times, resample_race, solve_race_optimal  # noqa: E402
from f3f.simulate import default_start, run_mpc_race  # noqa: E402


def summarize(name, X, diag):
    h = -X[:, 2]
    print(f'{name:>8s}: height min {h.min():5.1f} m, {h[0]:5.1f} -> {h[-1]:5.1f} m, '
          f'V {diag[:, 0].min():5.1f}..{diag[:, 0].max():5.1f} m/s, '
          f'AoA max {np.degrees(diag[:, 1].max()):5.1f} deg, '
          f'load {diag[:, 3].min():5.1f}..{diag[:, 3].max():5.1f} g, '
          f'min clearance {diag[:, 4].min():5.1f} m, '
          f'energy height {diag[0, 5]:6.1f} -> {diag[-1, 5]:6.1f} m')


def plot(path, ridge, mpc_log, opt, opt_diag, t_opt, mpc_time):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    xm, xo = mpc_log['x'], opt['X'].T
    fig, ax = plt.subplots(2, 2, figsize=(14, 9))

    a = ax[0, 0]
    a.plot(xm[:, 1], xm[:, 0], lw=1, label=f'MPC ({mpc_time:.2f} s)')
    a.plot(xo[:, 1], xo[:, 0], lw=1, label=f'offline optimum ({opt["time"]:.2f} s)')
    for b in (BASE_A, BASE_B):
        a.axhline(b, color='gray', ls='--', lw=0.8)
    a.axvline(ridge.safety_e, color='r', ls=':', lw=1, label='safety line')
    a.set_xlabel('East [m]')
    a.set_ylabel('North [m]')
    a.set_title('Top view (ridge crest at East = 0, wind from the left)')
    a.legend(loc='upper left')

    a = ax[0, 1]
    th = np.linspace(0, np.pi, 200)
    a.fill_between(ridge.radius * np.cos(th), 0, ridge.radius * np.sin(th), color='0.85')
    ee, hh = np.meshgrid(np.linspace(-160, 20, 120), np.linspace(1, 160, 100))
    r4 = (ee**2 + hh**2)**2
    w_up = -2 * ridge.wind_speed * ridge.radius**2 * ee * hh / r4
    w_up[ee**2 + hh**2 < ridge.radius**2] = np.nan
    cs_ = a.contourf(ee, hh, w_up, levels=np.linspace(-2, 12, 15), cmap='Blues', alpha=0.6)
    fig.colorbar(cs_, ax=a, label='updraft [m/s]')
    a.plot(xm[:, 1], -xm[:, 2], lw=0.8)
    a.plot(xo[:, 1], -xo[:, 2], lw=0.8)
    a.set_xlabel('East [m]')
    a.set_ylabel('height [m]')
    a.set_aspect('equal')
    a.set_title('Cross-section view')

    a = ax[1, 0]
    a.plot(mpc_log['t'][:-1], mpc_log['diag'][:, 5], label='MPC')
    a.plot(t_opt, opt_diag[:, 5], label='offline optimum')
    a.set_xlabel('race time [s]')
    a.set_ylabel('energy height [m]')
    a.legend()

    a = ax[1, 1]
    a.plot(mpc_log['t'][:-1], mpc_log['diag'][:, 0], label='MPC airspeed')
    a.plot(t_opt, opt_diag[:, 0], label='optimum airspeed')
    a.plot(mpc_log['t'][:-1], mpc_log['diag'][:, 3], lw=0.8, label='MPC load [g]')
    a.plot(t_opt, opt_diag[:, 3], lw=0.8, label='optimum load [g]')
    a.set_xlabel('race time [s]')
    a.legend()

    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'saved {path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-build', action='store_true')
    parser.add_argument('--N', type=int, default=40, help='MPC horizon (stages of 0.05 s)')
    parser.add_argument('--w-dist-e', type=float, default=1.0)
    parser.add_argument('--w-energy-e', type=float, default=1.0)
    parser.add_argument('--wind', type=float, default=10.0)
    parser.add_argument('--h-min', type=float, default=Ridge.h_min,
                        help='minimum height above the valley floor [m]')
    parser.add_argument('--keep-energy', action='store_true',
                        help='require the offline optimum to finish with its starting energy')
    parser.add_argument('--N-leg', type=int, default=40, help='offline OCP intervals per leg')
    parser.add_argument('--plot', type=str, default=None)
    parser.add_argument('--save', type=str, default=None)
    args = parser.parse_args()

    glider, ridge = Glider(), Ridge(wind_speed=args.wind, h_min=args.h_min)
    name = 'f3f_mpc' if args.N == 40 else f'f3f_mpc_N{args.N}'
    mpc = F3FMPC(glider, ridge, N=args.N, build=not args.no_build, name=name)
    theta = default_theta(w_dist_e=args.w_dist_e, w_energy_e=args.w_energy_e)
    x0 = default_start(glider, ridge)

    res = run_mpc_race(mpc, theta, x0, glider, ridge)
    race, log = res['race'], res['log']
    print(f'MPC race: {race.time:.2f} s, legs {race.leg}, crashed {race.crashed}, '
          f'safety violations {race.safety_violations}, '
          f'solve median {1e3 * np.median(log["t_solve"]):.0f} ms')
    if race.leg < race.n_legs:
        print('MPC did not finish the race; no warm start for the offline OCP')
        return 1

    X0, U0, T0 = resample_race(log['t'], log['x'], log['u'], race.crossings, args.N_leg)
    diag = make_diagnostics(glider, ridge)
    e_min = float(diag(x0)[5]) if args.keep_energy else None
    opt = solve_race_optimal(glider, ridge, x0, X0, U0, T0, N_leg=args.N_leg,
                             min_final_energy=e_min, verbose=False)
    print(f'offline optimum: {opt["time"]:.2f} s ({opt["status"]}, {opt["iter"]} iterations, '
          f'integration defect {opt["defect"]:.1e})')
    if opt['defect'] > 0.1:
        print('WARNING: large integration defect - the optimum exploits integration error; '
              'increase --N-leg or n_sub before trusting it')

    print(f'\n{"leg":>4s} {"MPC [s]":>8s} {"opt [s]":>8s}')
    for i, (tm, to) in enumerate(zip(race.leg_times(), opt['T'])):
        print(f'{i + 1:4d} {tm:8.2f} {to:8.2f}')
    gap = race.time - opt['time']
    print(f'{"sum":>4s} {race.time:8.2f} {opt["time"]:8.2f}   gap {gap:.2f} s '
          f'({100 * gap / opt["time"]:.1f}% of the optimum)\n')

    opt_diag = np.array(diag.map(opt['X'].shape[1])(opt['X'])).T
    summarize('MPC', log['x'][:len(log['diag'])], log['diag'])
    summarize('optimum', opt['X'].T, opt_diag)
    t_opt = node_times(opt['T'], args.N_leg)

    if args.save:
        np.savez(args.save, mpc_x=log['x'], mpc_u=log['u'], mpc_t=log['t'],
                 mpc_crossings=race.crossings, opt_X=opt['X'], opt_U=opt['U'], opt_T=opt['T'])
    if args.plot:
        plot(args.plot, ridge, log, opt, opt_diag, t_opt, race.time)
    return 0


if __name__ == '__main__':
    sys.exit(main())
