import numpy as np
import casadi as cs
import os

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from models.fixedwing_energy_model import FixedWingEnergyModel


class FixedWingEnergyMPC:
    """
    Zero-throttle, minimum-sink glide controller.

    Stage cost = -specific energy rate (edot = g*hdot + V*Vdot), i.e. minimizing
    -edot == maximizing edot == minimizing sink rate. Throttle (u[0], specific thrust)
    is hard-locked to zero.

    edot alone is NOT enough: it only measures energy dissipated to drag, so a bare
    -edot cost is indifferent to *how* that energy is dissipated and happily dives to
    trade altitude for airspeed (cashing in potential energy "for free", since gravity
    does no net work on edot) instead of settling into a sustained glide. The classical
    "minimum sink airspeed" is specifically a property of the TRIM manifold (steady,
    non-accelerating flight: accel=0 and pitch_rate=0, i.e. lift balances weight along
    the flight path). Penalizing deviation from that manifold is what turns -edot's
    minimizer into the minimum-sink trim rather than a permanent dive; the penalty
    vanishes identically ON the trim manifold, so it does not bias *where* the minimum
    sits, only rules out non-trimmed shortcuts.
    """

    def __init__(self, model: FixedWingEnergyModel, x0_init=None, N=20, Ts=.05, w_steady=1.0):
        self.N = N
        self.Ts = Ts
        self.Tf = N * Ts
        self.model = model  # instance of FixedWingEnergyModel, not the AcadosModel
        self.acados_model = self.model.get_acados_model()
        self.nx = self.acados_model.x.size()[0]  # 8
        self.nu = self.acados_model.u.size()[0]  # 3
        self.w_steady = w_steady

        if x0_init is None:
            print("Warning: No initial state provided, using default initialization.")
            x0_init = np.zeros(self.nx)
            x0_init[3] = 10.0  # Initialize speed to 10 m/s
            x0_init[4] = 1.0  # Initialize quaternion w=1

        self.ocp_solver, self.integrator = self.setup(x0_init)

        for i in range(self.N + 1):
            self.ocp_solver.set(i, "x", x0_init)
            if i < self.N:
                # zero throttle, trim-ish lift guess (L=W), wings level
                self.ocp_solver.set(i, "u", np.array([0.0, -self.model.gravity, 0.0]))

    def setup(self, x0):
        ocp = AcadosOcp()
        ocp.model = self.acados_model

        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.abspath(os.path.join(this_file_dir, '..'))
        codegen_dir = os.path.join(package_root, 'mpc_codegen')
        json_path = os.path.join(codegen_dir, 'acados_ocp_energy.json')
        os.makedirs(codegen_dir, exist_ok=True)
        ocp.code_export_directory = codegen_dir

        # COST FORMULATION
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'
        steady_penalty = self.w_steady * (self.model.accel_expr**2 + self.model.pitch_rate_expr**2)
        reg = 1e-3 * ocp.model.u[2]**2  # tiny roll-rate reg, just breaks the wings-level tie
        # Unlike NONLINEAR_LS, acados' EXTERNAL cost type is a raw discrete-stage cost - it is
        # NOT scaled by Ts automatically. Multiplying by Ts here matches the continuous-time
        # cost this was validated against (an IPOPT rollout that explicitly integrated
        # stage_cost*dt); leaving it out doesn't change the abstract minimizer, but it changes
        # the absolute cost/Hessian magnitude SQP_RTI's single Newton step and levenberg_marquardt
        # regularization see each iteration, which in practice was the difference between this
        # converging to the correct minimum-sink trim and converging ~10-100% off from it.
        ocp.model.cost_expr_ext_cost = self.Ts * (-self.model.specific_energy_rate_expr + steady_penalty + reg)
        ocp.model.cost_expr_ext_cost_e = cs.MX(0)

        # ctrl constraints
        ocp.constraints.lbu = np.array([0.0, self.model.min_fzw, -self.model.max_roll_rate])
        ocp.constraints.ubu = np.array([0.0, self.model.max_fzw, self.model.max_roll_rate])
        ocp.constraints.idxbu = np.array([0, 1, 2])

        ocp.constraints.idxbx = np.array([3])  # Index 3 is speed
        ocp.constraints.lbx = np.array([0.1])  # Minimum airspeed (stall limit)
        ocp.constraints.ubx = np.array([45.0])  # Max structural airspeed

        ocp.constraints.x0 = x0
        ocp.constraints.lbu_0 = ocp.constraints.lbu
        ocp.constraints.ubu_0 = ocp.constraints.ubu
        ocp.constraints.idxbu_0 = ocp.constraints.idxbu

        # SOLVER OPTIONS
        ocp.solver_options.tf = self.Tf
        ocp.solver_options.N_horizon = self.N
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        # GAUSS_NEWTON is not well-defined for an EXTERNAL cost (acados falls back to the exact
        # Hessian, which is indefinite far from the trim manifold for this nonconvex cost); this
        # value was tuned empirically against the Ts-scaled cost above - too small (~1e-4) lets
        # SQP_RTI's single Newton step diverge or badly overshoot from an off-trim start, too
        # large (>=1) converges cleanly but to the wrong fixed point.
        ocp.solver_options.levenberg_marquardt = 0.01
        ocp.solver_options.qp_solver_cond_N = self.N
        ocp_solver = AcadosOcpSolver(ocp, json_file=json_path)

        ocp.solver_options.qp_solver_warm_start = 2
        acados_integrator = AcadosSimSolver(ocp, json_file=json_path)

        return ocp_solver, acados_integrator

    def solve(self, x0, x_warm_start=None, u_warm_start=None, verbose=True):
        """
        Solves one MPC step from the current state x0. No reference trajectory is
        needed - the cost is the specific energy rate itself, not a tracking error.
        """
        ocp_solver = self.ocp_solver
        ocp_solver.set(0, "lbx", x0)
        ocp_solver.set(0, "ubx", x0)

        if u_warm_start is not None and x_warm_start is not None:
            for i in range(self.N):
                ocp_solver.set(i, "u", u_warm_start[i, :])
                ocp_solver.set(i, "x", x_warm_start[i, :])

        status = ocp_solver.solve()

        if status != 0:
            print(f"\n[!] ACADOS SOLVER FAILED (Status {status})")
            ocp_solver.print_statistics()
            if verbose:
                print(f"-> Initial State (x0): {np.round(x0, 3)}")

        simX = np.zeros((self.N + 1, self.nx))
        simU = np.zeros((self.N, self.nu))

        for i in range(self.N):
            simX[i, :] = ocp_solver.get(i, "x")
            simU[i, :] = ocp_solver.get(i, "u")
        simX[self.N, :] = ocp_solver.get(self.N, "x")

        return simU, simX, status
