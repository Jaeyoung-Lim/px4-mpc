"""
Offline minimum-time F3F race: one OCP over all legs, the upper bound a short-horizon MPC is
compared against.

Direct multiple shooting with N_leg intervals per leg and a free duration T_i per leg
(interval length T_i / N_leg). Leg i ends exactly on its base line (n = B, A, B, ...), which
is where the race clock for that leg stops; turns happen inside the following leg. Path
constraints are those of the MPC (f3f.glider.path_constraints), imposed as hard constraints at
every node. Objective: total race time, plus a tiny control regularization to remove
chattering in directions the race time is insensitive to.

The problem is nonconvex, so IPOPT returns a local optimum near its initial guess. Warm
starting from a closed-loop MPC race gives a solution at least as good as that race (when the
race satisfies the hard constraints), so T_MPC - T_opt is a lower bound on what the MPC loses
by being myopic.
"""
import casadi as cs
import numpy as np

from f3f.course import leg_target
from f3f.glider import NU, NX, Glider, Ridge, energy_height, path_constraints, rk4


def resample_race(log_t, log_x, log_u, crossings, N_leg):
    """Initial guess per leg from a logged race: (X (NX, n_legs*N_leg+1), U, T)."""
    bounds = np.concatenate([[0.0], crossings])
    T = np.diff(bounds)
    t_nodes = np.concatenate([np.linspace(bounds[i], bounds[i + 1], N_leg + 1)[:-1]
                              for i in range(len(T))] + [[bounds[-1]]])
    X = np.array([np.interp(t_nodes, log_t, log_x[:, j]) for j in range(NX)])
    t_u = log_t[:len(log_u)]
    U = np.array([np.interp(t_nodes[:-1], t_u, log_u[:, j]) for j in range(NU)])
    return X, U, T


def solve_race_optimal(glider: Glider, ridge: Ridge, x0, X_guess, U_guess, T_guess, N_leg=40,
                       n_sub=4, max_iter=3000, w_reg=1e-4, min_final_energy=None,
                       verbose=True):
    n_legs = len(T_guess)
    K = n_legs * N_leg

    xs = cs.SX.sym('x', NX)
    us = cs.SX.sym('u', NU)
    dts = cs.SX.sym('dt')
    F = cs.Function('F', [xs, us, dts], [rk4(xs, us, dts, n_sub, glider, ridge)])
    g_expr, lb, ub = path_constraints(xs, glider, ridge)
    G = cs.Function('G', [xs], [g_expr])

    opti = cs.Opti()
    X = opti.variable(NX, K + 1)
    U = opti.variable(NU, K)
    T = opti.variable(n_legs)

    dt_row = cs.kron(T.T, cs.DM.ones(1, N_leg)) / N_leg
    opti.subject_to(X[:, 0] == x0)
    opti.subject_to(X[:, 1:] == F.map(K)(X[:, :-1], U, dt_row))
    for i in range(n_legs):
        opti.subject_to(X[0, (i + 1) * N_leg] == leg_target(i))

    opti.subject_to(opti.bounded(np.tile(lb[:, None], (1, K)), G.map(K)(X[:, 1:]),
                                 np.tile(ub[:, None], (1, K))))
    max_rates = np.array(glider.max_rates)[:, None]
    opti.subject_to(opti.bounded(-np.tile(max_rates, (1, K)), U, np.tile(max_rates, (1, K))))
    opti.subject_to(opti.bounded(0.3, T, 30.0))
    if min_final_energy is not None:
        # e.g. the starting energy: stops the optimum from winning by spending its reserve
        opti.subject_to(energy_height(X[:, -1]) >= min_final_energy)

    opti.minimize(cs.sum1(T) + w_reg * cs.sumsqr(U) / K)

    opti.set_initial(X, X_guess)
    opti.set_initial(U, U_guess)
    opti.set_initial(T, T_guess)
    opti.solver('ipopt', {'print_time': False},
                {'max_iter': max_iter, 'tol': 1e-6, 'acceptable_tol': 1e-4,
                 'acceptable_iter': 20, 'mu_strategy': 'adaptive',
                 'print_level': 5 if verbose else 0, 'print_frequency_iter': 50})
    try:
        sol = opti.solve()
        stats, success = sol.stats(), True
        X_opt, U_opt, T_opt = sol.value(X), sol.value(U), np.atleast_1d(sol.value(T))
    except RuntimeError:
        stats, success = opti.stats(), False
        X_opt, U_opt = opti.debug.value(X), opti.debug.value(U)
        T_opt = np.atleast_1d(opti.debug.value(T))

    # integration defect check: re-integrate each interval with a finer RK4. A large defect
    # means the optimizer is exploiting integration error (seen with n_sub=2 at ~0.14 s
    # intervals, where the substep approaches the rate-loop time constant)
    F_fine = cs.Function('Ff', [xs, us, dts], [rk4(xs, us, dts, 8 * n_sub, glider, ridge)])
    dt_nodes = np.repeat(T_opt / N_leg, N_leg)
    X_fine = np.array(F_fine.map(K)(X_opt[:, :-1], U_opt, dt_nodes))
    defect = np.abs(X_fine - X_opt[:, 1:]).max()

    return {'success': success, 'status': stats.get('return_status'), 'T': T_opt,
            'time': float(np.sum(T_opt)), 'X': X_opt, 'U': U_opt, 'N_leg': N_leg,
            'defect': defect, 'iter': stats.get('iter_count')}


def node_times(T, N_leg):
    return np.concatenate([[0.0], np.cumsum(np.repeat(T / N_leg, N_leg))])
