"""
Short-horizon minimum-time NMPC for F3F racing (see f3f.glider for the model).

Stage cost: distance to the current target base line, |n - n_target| (smoothed), integrated
over the horizon - getting to the base sooner lowers it, and overshooting it raises it again,
so the plan turns around at the base. The target switches when the real glider crosses the
base, not inside the prediction.

Terminal cost: w_dist_e * |n_N - n_target| - w_energy_e * energy_height(x_N). This is the
hand-designed stand-in for the true cost-to-go (remaining race time), which depends on the
legs beyond the horizon; it is the piece a learned terminal value would replace.

theta (p_global) = [w_dist, w_dist_e, w_energy_e, w_rate] so that the weights can later be
differentiated through (same structure as controllers.fixedwing_diff_mpc).
"""
import os

import casadi as cs
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from f3f.glider import NU, NX, Glider, Ridge, energy_height, path_constraints, rk4

THETA_NAMES = ['w_dist', 'w_dist_e', 'w_energy_e', 'w_rate']
NTHETA = len(THETA_NAMES)


def default_theta(w_dist=1.0, w_dist_e=1.0, w_energy_e=1.0, w_rate=1e-3):
    return np.array([w_dist, w_dist_e, w_energy_e, w_rate])


class F3FMPC:
    def __init__(self, glider: Glider, ridge: Ridge, N=40, Ts=0.05, n_sub=1, max_iter=50,
                 hessian_approx='EXACT', regularize_method='MIRROR',
                 nlp_solver_type='SQP', tol=1e-4, build=True, codegen_root=None,
                 name='f3f_mpc'):
        self.glider, self.ridge = glider, ridge
        self.N, self.Ts = N, Ts
        self.name = name
        self.solver_opts = dict(hessian_approx=hessian_approx,
                                regularize_method=regularize_method,
                                nlp_solver_type=nlp_solver_type, tol=tol)
        if codegen_root is None:
            codegen_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                                        'mpc_codegen'))
        self.solver = self._build(codegen_root, n_sub, max_iter, build)
        self._warm = None
        self.stats = {'solves': 0, 'failures': 0}

    def _build(self, codegen_root, n_sub, max_iter, build):
        x = cs.SX.sym('x', NX)
        u = cs.SX.sym('u', NU)
        theta = cs.SX.sym('theta', NTHETA)
        n_target = cs.SX.sym('n_target', 1)

        model = AcadosModel()
        model.name = self.name
        model.x, model.u, model.p, model.p_global = x, u, n_target, theta
        model.disc_dyn_expr = rk4(x, u, self.Ts, n_sub, self.glider, self.ridge)

        dist = cs.sqrt((x[0] - n_target)**2 + 1.0)
        model.cost_expr_ext_cost = self.Ts * (theta[0] * dist + theta[3] * cs.sumsqr(u))
        model.cost_expr_ext_cost_e = theta[1] * dist - theta[2] * energy_height(x)

        g, lb, ub = path_constraints(x, self.glider, self.ridge)
        model.con_h_expr = g

        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.N * self.Ts
        ocp.p_global_values = default_theta()
        ocp.parameter_values = np.zeros(1)
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        max_rates = np.array(self.glider.max_rates)
        ocp.constraints.idxbu = np.arange(NU)
        ocp.constraints.lbu, ocp.constraints.ubu = -max_rates, max_rates
        ocp.constraints.x0 = np.zeros(NX)

        ocp.constraints.lh, ocp.constraints.uh = lb, ub
        ocp.constraints.idxsh = np.arange(len(lb))
        # [alpha, beta, load, clearance, height, safety]: L1 / L2 slack penalties per unit
        ocp.cost.zl = ocp.cost.zu = np.array([1e3, 1e3, 1e2, 1e2, 1e2, 1e2])
        ocp.cost.Zl = ocp.cost.Zu = np.array([1e4, 1e4, 1e3, 1e3, 1e3, 1e3])

        opts = ocp.solver_options
        opts.integrator_type = 'DISCRETE'
        opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        opts.qp_solver_cond_N = self.N // 4
        opts.qp_solver_mu0 = 1e3
        so = self.solver_opts
        opts.hessian_approx = so['hessian_approx']
        opts.regularize_method = so['regularize_method']
        opts.reg_epsilon = 1e-6
        opts.nlp_solver_type = so['nlp_solver_type']
        if so['nlp_solver_type'] == 'SQP':
            opts.globalization = 'MERIT_BACKTRACKING'
            opts.nlp_solver_max_iter = max_iter
            opts.tol = so['tol']
        opts.qp_solver_iter_max = 100

        codegen_dir = os.path.join(codegen_root, self.name)
        os.makedirs(codegen_dir, exist_ok=True)
        ocp.code_export_directory = codegen_dir
        return AcadosOcpSolver(ocp, json_file=os.path.join(codegen_dir, f'{self.name}.json'),
                               build=build, generate=build, verbose=False)

    def reset(self):
        self._warm = None

    def solve(self, x0, n_target, theta, F_guess):
        """
        One MPC step. F_guess(x, u) -> x+ is used to roll out an initial guess on a cold start.
        Returns (u0, info) with the predicted trajectory.
        """
        S = self.solver
        S.set_p_global_and_precompute_dependencies(np.asarray(theta, dtype=float))
        for k in range(self.N + 1):
            S.set(k, 'p', np.array([n_target]))
        S.set(0, 'lbx', x0)
        S.set(0, 'ubx', x0)

        if self._warm is not None:
            S.set_iterate(self._warm)
            for k in range(self.N):
                S.set(k, 'x', S.get(k + 1, 'x'))
                if k < self.N - 1:
                    S.set(k, 'u', S.get(k + 1, 'u'))
            S.set(0, 'x', x0)
        else:
            S.reset()
            x = np.asarray(x0, dtype=float)
            u = np.zeros(NU)
            for k in range(self.N):
                S.set(k, 'x', x)
                S.set(k, 'u', u)
                x = np.array(F_guess(x, u)).ravel()
            S.set(self.N, 'x', x)

        status = S.solve()
        self.stats['solves'] += 1
        if status not in (0, 2):
            self.stats['failures'] += 1
        # keep warm-starting from max-iter exits too (the plan is usually still good), but not
        # from QP failures
        self._warm = S.get_iterate() if status in (0, 2) else None
        X = np.array([S.get(k, 'x') for k in range(self.N + 1)])
        U = np.array([S.get(k, 'u') for k in range(self.N)])
        return U[0].copy(), {'status': status, 'X': X, 'U': U}
