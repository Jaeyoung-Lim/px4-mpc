"""Closed-loop F3F race simulation."""
import time

import numpy as np

from f3f.course import Race
from f3f.glider import Glider, Ridge, make_diagnostics, make_step


def default_start(glider: Glider, ridge: Ridge, ground_speed=35.0, standoff=20.0,
                  elevation_deg=45.0):
    """Start on base A heading for B, `standoff` m out from the slope at the given elevation."""
    from f3f.glider import level_state
    r = ridge.radius + standoff
    el = np.radians(elevation_deg)
    pos = np.array([0.0, -r * np.cos(el), -r * np.sin(el)])
    return level_state(pos, 0.0, ground_speed, glider)


def run_mpc_race(mpc, theta, x0, glider: Glider, ridge: Ridge, t_max=120.0, n_sub_plant=4,
                 verbose=False):
    """
    Fly a race with the MPC in the loop against the same model integrated more finely.
    Returns a dict with the race result and logs.
    """
    F_plant = make_step(mpc.Ts, glider, ridge, n_sub=n_sub_plant)
    F_guess = make_step(mpc.Ts, glider, ridge, n_sub=1)
    diag = make_diagnostics(glider, ridge)
    race = Race()
    mpc.reset()
    x = np.asarray(x0, dtype=float)
    log = {k: [] for k in ['t', 'x', 'u', 'diag', 'status', 't_solve', 'leg', 'residual',
                           'sqp_iter']}

    while not race.finished and race.t < t_max:
        t0 = time.perf_counter()
        u, info = mpc.solve(x, race.target, theta, F_guess)
        log['t_solve'].append(time.perf_counter() - t0)
        log['status'].append(info['status'])
        log['residual'].append(max(mpc.solver.get_stats('residuals')))
        log['sqp_iter'].append(mpc.solver.get_stats('sqp_iter'))
        log['t'].append(race.t)
        log['x'].append(x)
        log['u'].append(u)
        log['leg'].append(race.leg)
        d = np.array(diag(x)).ravel()
        log['diag'].append(d)
        x_next = np.array(F_plant(x, u)).ravel()
        race.update(x, x_next, mpc.Ts, ridge, float(diag(x_next)[4]))
        if verbose and len(log['t']) % 100 == 0:
            print(f"t {race.t:5.1f} leg {race.leg} n {x[0]:6.1f} e {x[1]:6.1f} h {-x[2]:5.1f} "
                  f"V {d[0]:4.1f} load {d[3]:4.1f} E {d[5]:5.1f} status {info['status']}")
        x = x_next

    log['x'].append(x)
    log['t'].append(race.t)
    return {'race': race, 'log': {k: np.array(v) for k, v in log.items()}}
