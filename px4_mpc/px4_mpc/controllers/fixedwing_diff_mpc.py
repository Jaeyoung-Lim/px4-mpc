"""
Differentiable body-rate NMPC for fixed-wing path following.

The OCP is written so that acados can differentiate its solution with respect to a global
parameter vector theta (`p_global`), which makes the MPC usable as a layer in a PyTorch model:
    u0 = MPC(x0; theta, reference)
    dL/dtheta, dL/dx0  via acados adjoint / forward solution sensitivities

theta = [log cost-weight multipliers (COST_NAMES), airspeed reference, model params (MODEL_NAMES)].
theta = nominal_theta() is a hand-tuned baseline (all log multipliers 0, nominal model), so a
learner starts from a working controller and only has to improve on it.

The path reference enters as per-stage parameters p_k = [r_n, r_e, r_d, t_n, t_e] (reference
point and horizontal unit tangent of the path, see paths.dubins.Path.sample). These are NOT
differentiated - they come from the path, not from the learner.

Requirements acados imposes on differentiable OCPs, and how they are met here:
    - parameters may only enter through DISCRETE dynamics, EXTERNAL/LINEAR_LS cost and h
      constraints -> RK4 discrete dynamics, EXTERNAL cost, AoA/airspeed limits as soft h.
    - sensitivities need the exact, unregularized Lagrangian Hessian -> a second "sensitivity"
      solver with EXACT Hessian and NO_REGULARIZE is loaded with the converged iterate of the
      (regularized) solve solver, and only factorizes; it never iterates itself.
    - sensitivities are those of the converged NLP solution -> full SQP, not SQP_RTI.

Model: point-mass with body-rate inputs (same Beard & McLain formulation as
models.fixedwing_bodyrate_model) with constant wind, NED / FRD / Hamilton (w,x,y,z) body->NED.
State x = [p (NED), v (inertial velocity in body frame), q], control u = [T (N), p, q, r].
"""
import os

import casadi as cs
import numpy as np
import torch
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from models.fixedwing_model import skew_symmetric, q_to_rot_mat

GRAVITY = 9.81
RHO = 1.2041
WING_AREA = 0.35
ASPECT_RATIO = 6.5
OSWALD_EFF = 0.97

NX = 10
NU = 4
NP_STAGE = 5  # [r_n, r_e, r_d, t_n, t_e]

COST_NAMES = ['lateral', 'altitude', 'along', 'course', 'airspeed', 'sideslip',
              'thrust', 'p_rate', 'q_rate', 'r_rate', 'terminal']
# weights at theta = 0; chosen so typical errors (5 m, 20 deg, 2 m/s, 0.5 rad/s) are comparable
COST_SCALE = np.array([1.0, 2.0, 0.05, 50.0, 2.0, 10.0, 0.01, 1.0, 2.0, 2.0, 5.0])
MODEL_NAMES = ['mass', 'c_d_0', 'c_l_0', 'c_l_a', 'wind_n', 'wind_e', 'wind_d']
NOMINAL_MODEL = np.array([1.0, 0.029, 0.15188, 5.015, 0.0, 0.0, 0.0])
THETA_NAMES = ['log_w_' + n for n in COST_NAMES] + ['airspeed_ref'] + MODEL_NAMES
IDX_AIRSPEED_REF = len(COST_NAMES)
IDX_MODEL = IDX_AIRSPEED_REF + 1
NTHETA = len(THETA_NAMES)


def nominal_theta(airspeed_ref=12.0, model_params=None):
    model_params = NOMINAL_MODEL if model_params is None else np.asarray(model_params, dtype=float)
    return np.concatenate([np.zeros(len(COST_NAMES)), [airspeed_ref], model_params])


def air_data(x, phys):
    """Air-relative velocity (body), airspeed and angle of attack."""
    q = x[6:10] / cs.sqrt(cs.sumsqr(x[6:10]) + 1e-9)
    R = q_to_rot_mat(q)
    v_air = x[3:6] - cs.mtimes(R.T, phys[4:7])
    V = cs.sqrt(cs.sumsqr(v_air) + 1e-6)
    alpha = cs.atan2(v_air[2], v_air[0] + 1e-6)
    return R, v_air, V, alpha


def ode(x, u, phys):
    mass, c_d_0, c_l_0, c_l_a = phys[0], phys[1], phys[2], phys[3]
    R, v_air, V, alpha = air_data(x, phys)
    v = x[3:6]
    omega = u[1:4]

    c_l = c_l_0 + c_l_a * alpha
    c_d = c_d_0 + c_l**2 / (cs.pi * OSWALD_EFF * ASPECT_RATIO)
    q_bar = 0.5 * RHO * V**2
    L = q_bar * WING_AREA * c_l
    D = q_bar * WING_AREA * c_d
    cos_a, sin_a = cs.cos(alpha), cs.sin(alpha)
    f_aero = cs.vertcat(-D * cos_a + L * sin_a, 0, -D * sin_a - L * cos_a) / mass
    f_gravity = cs.mtimes(R.T, cs.vertcat(0, 0, GRAVITY))
    f_thrust = cs.vertcat(u[0] / mass, 0, 0)

    v_dot = f_aero + f_gravity + f_thrust - cs.cross(omega, v)
    p_dot = cs.mtimes(R, v)
    q_dot = 0.5 * cs.mtimes(skew_symmetric(omega), x[6:10])
    return cs.vertcat(p_dot, v_dot, q_dot)


def rk4(x, u, phys, dt, n_sub):
    h = dt / n_sub
    for _ in range(n_sub):
        k1 = ode(x, u, phys)
        k2 = ode(x + h / 2 * k1, u, phys)
        k3 = ode(x + h / 2 * k2, u, phys)
        k4 = ode(x + h * k3, u, phys)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def make_discrete_dynamics(dt, n_sub=2):
    """CasADi function (x, u, phys) -> x+, with quaternion renormalization; usable as a plant."""
    x = cs.SX.sym('x', NX)
    u = cs.SX.sym('u', NU)
    phys = cs.SX.sym('phys', len(MODEL_NAMES))
    xn = rk4(x, u, phys, dt, n_sub)
    xn = cs.vertcat(xn[:6], xn[6:10] / cs.norm_2(xn[6:10]))
    return cs.Function('F', [x, u, phys], [xn])


def trim_level(airspeed, phys=NOMINAL_MODEL):
    """Approximate wings-level trim (alpha, thrust) in still air."""
    mass, c_d_0, c_l_0, c_l_a = phys[:4]
    q_bar_s = 0.5 * RHO * airspeed**2 * WING_AREA
    c_l = mass * GRAVITY / q_bar_s
    alpha = (c_l - c_l_0) / c_l_a
    c_d = c_d_0 + c_l**2 / (np.pi * OSWALD_EFF * ASPECT_RATIO)
    return alpha, q_bar_s * c_d


def level_state(pos, course, airspeed, phys=NOMINAL_MODEL):
    """Wings-level trimmed state at the given NED position and course."""
    alpha, _ = trim_level(airspeed, phys)
    # yaw = course, pitch = alpha (zero flight path angle), roll = 0
    cy, sy = np.cos(course / 2), np.sin(course / 2)
    cp, sp = np.cos(alpha / 2), np.sin(alpha / 2)
    q = np.array([cy * cp, -sy * sp, cy * sp, sy * cp])
    v = airspeed * np.array([np.cos(alpha), 0.0, np.sin(alpha)])
    return np.concatenate([pos, v, q])


class FixedWingDiffMPC:
    def __init__(self, N=20, Ts=0.1, n_sub=2, alpha_max=0.25, alpha_min=-0.15, airspeed_min=7.0,
                 airspeed_max=25.0, max_thrust=10.0, max_rates=(3.0, 2.0, 1.0),
                 regularize_method='CONVEXIFY', reg_epsilon=1e-6, max_iter=100,
                 build=True, codegen_root=None):
        self.N, self.Ts, self.n_sub = N, Ts, n_sub
        self.nx, self.nu, self.ntheta = NX, NU, NTHETA
        self.u_lb = np.array([0.0, -max_rates[0], -max_rates[1], -max_rates[2]])
        self.u_ub = np.array([max_thrust, max_rates[0], max_rates[1], max_rates[2]])
        self.h_lb = np.array([alpha_min, airspeed_min])
        self.h_ub = np.array([alpha_max, airspeed_max])
        self.regularize_method, self.reg_epsilon, self.max_iter = (regularize_method, reg_epsilon,
                                                                   max_iter)

        if codegen_root is None:
            codegen_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                                        'mpc_codegen'))
        self.solver = self._build(codegen_root, sensitivity=False, build=build)
        self.sens_solver = self._build(codegen_root, sensitivity=True, build=build)
        self.F = make_discrete_dynamics(Ts, n_sub)
        self._warm = {}  # slot -> iterate, so independent rollouts don't share warm starts
        self.stats = {'solves': 0, 'failures': 0}

    # ------------------------------------------------------------------ OCP definition
    def _model(self, name):
        x = cs.SX.sym('x', NX)
        u = cs.SX.sym('u', NU)
        theta = cs.SX.sym('theta', NTHETA)
        p = cs.SX.sym('p', NP_STAGE)
        phys = theta[IDX_MODEL:]

        model = AcadosModel()
        model.name = name
        model.x, model.u, model.p, model.p_global = x, u, p, theta
        model.disc_dyn_expr = rk4(x, u, phys, self.Ts, self.n_sub)

        w = COST_SCALE * cs.exp(theta[:len(COST_NAMES)])
        w = [w[i] for i in range(len(COST_NAMES))]
        airspeed_ref = theta[IDX_AIRSPEED_REF]
        R, v_air, V, alpha = air_data(x, phys)

        ref, t_h = p[0:3], p[3:5]
        e_h = x[0:2] - ref[0:2]
        e_along = t_h[0] * e_h[0] + t_h[1] * e_h[1]
        e_lat = t_h[0] * e_h[1] - t_h[1] * e_h[0]  # > 0: right of the path
        e_alt = x[2] - ref[2]
        v_ground = cs.mtimes(R, x[3:6])
        ground_speed = cs.sqrt(cs.sumsqr(v_ground[0:2]) + 1e-3)
        course_err = 1.0 - (t_h[0] * v_ground[0] + t_h[1] * v_ground[1]) / ground_speed

        tracking = (w[0] * e_lat**2 + w[1] * e_alt**2 + w[3] * course_err
                    + w[4] * (V - airspeed_ref)**2)
        model.cost_expr_ext_cost = (tracking + w[2] * e_along**2 + w[5] * v_air[1]**2
                                    + w[6] * u[0]**2 + w[7] * u[1]**2 + w[8] * u[2]**2
                                    + w[9] * u[3]**2)
        model.cost_expr_ext_cost_e = w[10] * tracking

        # stage-wise only: a p_global-dependent terminal h hits a codegen bug in acados 0.5.1
        # (acados_solver.in.h declares nl_constr_h_hess_ux_pdiff_adj_pdiff twice instead of the
        # nl_constr_h_e_ variant), and stages 1..N-1 already enforce the limits
        model.con_h_expr = cs.vertcat(alpha, V)
        return model

    def _build(self, codegen_root, sensitivity, build):
        name = 'fw_diff_mpc_sens' if sensitivity else 'fw_diff_mpc'
        ocp = AcadosOcp()
        ocp.model = self._model(name)
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.tf = self.N * self.Ts
        ocp.p_global_values = nominal_theta()
        ocp.parameter_values = np.zeros(NP_STAGE)

        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        ocp.constraints.x0 = level_state(np.zeros(3), 0.0, 12.0)
        ocp.constraints.idxbu = np.arange(NU)
        ocp.constraints.lbu, ocp.constraints.ubu = self.u_lb, self.u_ub

        # AoA and airspeed limits as soft constraints: hard ones make the OCP infeasible after
        # a gust, and an active hard bound has zero gradient w.r.t. everything upstream
        ocp.constraints.lh, ocp.constraints.uh = self.h_lb, self.h_ub
        ocp.constraints.idxsh = np.arange(2)
        ocp.cost.zl = ocp.cost.zu = np.array([1e3, 1e2])
        ocp.cost.Zl = ocp.cost.Zu = np.array([1e4, 1e3])

        opts = ocp.solver_options
        opts.integrator_type = 'DISCRETE'
        opts.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        opts.qp_solver_cond_N = self.N
        opts.qp_solver_ric_alg = 0
        opts.qp_solver_cond_ric_alg = 0
        opts.qp_solver_mu0 = 1e3
        opts.hessian_approx = 'EXACT'
        opts.nlp_solver_type = 'SQP'
        if sensitivity:
            opts.regularize_method = 'NO_REGULARIZE'
            ocp.code_gen_options.with_solution_sens_wrt_params = True
        else:
            opts.regularize_method = self.regularize_method
            opts.reg_epsilon = self.reg_epsilon
            opts.globalization = 'MERIT_BACKTRACKING'
            opts.nlp_solver_max_iter = self.max_iter
            opts.qp_solver_iter_max = 200
            opts.tol = 1e-6

        codegen_dir = os.path.join(codegen_root, name)
        os.makedirs(codegen_dir, exist_ok=True)
        ocp.code_export_directory = codegen_dir
        return AcadosOcpSolver(ocp, json_file=os.path.join(codegen_dir, f'{name}.json'),
                               build=build, generate=build, verbose=False)

    # ------------------------------------------------------------------ solving
    def reset(self, slot=None):
        if slot is None:
            self._warm.clear()
        else:
            self._warm.pop(slot, None)

    def _set_problem(self, solver, x0, theta, stage_params):
        solver.set_p_global_and_precompute_dependencies(np.asarray(theta, dtype=float))
        for k in range(self.N + 1):
            solver.set(k, 'p', stage_params[k])
        solver.set(0, 'lbx', x0)
        solver.set(0, 'ubx', x0)

    def _initial_guess(self, x0, theta):
        phys = np.asarray(theta[IDX_MODEL:], dtype=float)
        _, thrust = trim_level(theta[IDX_AIRSPEED_REF], phys)
        u = np.array([thrust, 0.0, 0.0, 0.0])
        x = np.asarray(x0, dtype=float)
        # zero multipliers/slacks left over from previous solves (keeps parameters and x0)
        self.solver.reset()
        for k in range(self.N):
            self.solver.set(k, 'x', x)
            self.solver.set(k, 'u', u)
            x = np.array(self.F(x, u, phys)).ravel()
        self.solver.set(self.N, 'x', x)

    def solve(self, x0, theta, stage_params, slot=0, warm_iterate=None):
        """
        Solve the OCP. Returns (u0, info) with info = {status, X, U, iterate}; the iterate is
        what backward() needs to evaluate sensitivities at this solution later.
        warm_iterate: start from this iterate as-is (no time shift), e.g. for finite differences.
        """
        x0 = np.asarray(x0, dtype=float)
        self._set_problem(self.solver, x0, theta, stage_params)
        if warm_iterate is not None:
            self.solver.set_iterate(warm_iterate)
            self.solver.set(0, 'x', x0)
        elif slot in self._warm:
            self.solver.set_iterate(self._warm[slot])
            # shift the warm start one stage forward in time
            for k in range(self.N):
                self.solver.set(k, 'x', self.solver.get(min(k + 1, self.N), 'x'))
                if k < self.N - 1:
                    self.solver.set(k, 'u', self.solver.get(k + 1, 'u'))
            self.solver.set(0, 'x', x0)
        else:
            self._initial_guess(x0, theta)

        status = self.solver.solve()
        self.stats['solves'] += 1
        if status != 0:
            self.stats['failures'] += 1
            # do not warm start from a failed solve
            self._warm.pop(slot, None)
        iterate = self.solver.get_iterate()
        if status == 0:
            self._warm[slot] = iterate

        X = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)])
        U = np.array([self.solver.get(k, 'u') for k in range(self.N)])
        return U[0].copy(), {'status': status, 'X': X, 'U': U, 'iterate': iterate}

    # ------------------------------------------------------------------ sensitivities
    def _load_sensitivity(self, x0, theta, stage_params, iterate):
        self._set_problem(self.sens_solver, x0, theta, stage_params)
        self.sens_solver.set_iterate(iterate)
        self.sens_solver.setup_qp_matrices_and_factorize()

    def vjp(self, x0, theta, stage_params, iterate, grad_u0, wrt_x0=True):
        """Vector-Jacobian products (dL/dtheta, dL/dx0) given dL/du0 at a stored solution."""
        self._load_sensitivity(x0, theta, stage_params, iterate)
        seed = np.asarray(grad_u0, dtype=float).reshape(NU, 1)
        g_theta = self.sens_solver.eval_adjoint_solution_sensitivity(
            seed_x=[], seed_u=[(0, seed)]).ravel()
        g_x0 = None
        if wrt_x0:
            sens_u0 = self.sens_solver.eval_solution_sensitivity(
                0, 'initial_state', return_sens_x=False, return_sens_u=True)['sens_u']
            g_x0 = sens_u0.T @ seed.ravel()
        return g_theta, g_x0

    def jacobians(self, x0, theta, stage_params, iterate):
        """Full du0/dtheta (nu x ntheta) and du0/dx0 (nu x nx) at a stored solution."""
        self._load_sensitivity(x0, theta, stage_params, iterate)
        du_dtheta = self.sens_solver.eval_adjoint_solution_sensitivity(
            seed_x=[], seed_u=[(0, np.eye(NU))])
        du_dx0 = self.sens_solver.eval_solution_sensitivity(
            0, 'initial_state', return_sens_x=False, return_sens_u=True)['sens_u']
        return du_dtheta, du_dx0


class _DiffMPCFunction(torch.autograd.Function):
    @staticmethod
    def forward(ctx, x0, theta, mpc, stage_params, slots):
        x0_np = x0.detach().cpu().double().numpy()
        theta_np = theta.detach().cpu().double().numpy()
        u0, infos = [], []
        for b in range(x0_np.shape[0]):
            u, info = mpc.solve(x0_np[b], theta_np[b], stage_params[b], slot=slots[b])
            u0.append(u)
            infos.append(info)
        ctx.mpc, ctx.stage_params, ctx.infos = mpc, stage_params, infos
        ctx.x0_np, ctx.theta_np = x0_np, theta_np
        mpc.last_infos = infos
        return torch.as_tensor(np.array(u0), dtype=x0.dtype, device=x0.device)

    @staticmethod
    def backward(ctx, grad_u0):
        mpc = ctx.mpc
        g = grad_u0.detach().cpu().double().numpy()
        B = g.shape[0]
        g_x0 = np.zeros((B, NX))
        g_theta = np.zeros((B, NTHETA))
        want_x0 = ctx.needs_input_grad[0]
        for b in range(B):
            # sensitivities of an unconverged solve are meaningless: drop that sample's gradient
            if ctx.infos[b]['status'] != 0 or not np.any(g[b]):
                continue
            gt, gx = mpc.vjp(ctx.x0_np[b], ctx.theta_np[b], ctx.stage_params[b],
                             ctx.infos[b]['iterate'], g[b], wrt_x0=want_x0)
            g_theta[b] = gt
            if want_x0:
                g_x0[b] = gx
        to = dict(dtype=grad_u0.dtype, device=grad_u0.device)
        return (torch.as_tensor(g_x0, **to) if want_x0 else None,
                torch.as_tensor(g_theta, **to), None, None, None)


def diff_mpc(mpc: FixedWingDiffMPC, x0, theta, stage_params, slots=None):
    """
    Differentiable MPC layer. x0: (B, nx) tensor, theta: (B, ntheta) tensor, stage_params:
    (B, N+1, 5) array of path references. Returns u0: (B, nu). Gradients flow to x0 and theta.
    """
    if slots is None:
        slots = list(range(x0.shape[0]))
    return _DiffMPCFunction.apply(x0, theta, mpc, stage_params, slots)


def path_stage_params(path, pos, s_hint, spacing, N):
    """
    Stage parameters for the MPC from a paths.dubins.Path: project pos onto the path, then
    sample N+1 references spaced by `spacing` (e.g. current ground speed * Ts).
    Returns (stage_params (N+1, 5), s_closest).
    """
    s0 = path.project(pos, s_hint=s_hint, window=max(10 * spacing, 20.0))
    refs, tangents, _ = path.sample(s0 + spacing * np.arange(N + 1))
    t_h = tangents[:, :2] / np.linalg.norm(tangents[:, :2], axis=1, keepdims=True)
    return np.hstack([refs, t_h]), s0
