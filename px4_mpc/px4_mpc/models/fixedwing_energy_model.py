import casadi as cs
from acados_template import AcadosModel

from px4_mpc.models.fixedwing_model import FixedWingModel, skew_symmetric, q_to_rot_mat


class FixedWingEnergyModel(FixedWingModel):
    """
    Same wind-frame kinematics as FixedWingModel, but f_xw is reinterpreted strictly
    as SPECIFIC THRUST (T/m >= 0) instead of the combined (thrust-drag)/m pseudo-control
    used by the tracking model. Drag is computed explicitly from the drag polar so that
    f_zw (specific lift) and airspeed jointly determine the induced+parasitic drag
    deceleration. This coupling is what makes a minimum-sink airspeed exist at all -
    in the base model f_xw and f_zw are independent, so there is no drag penalty on
    lift and no such optimum exists.

    Also exposes `specific_energy_rate_expr`, the mass-specific total energy rate
    edot = g*hdot + V*Vdot, for use as an MPC cost (see FixedWingEnergyMPC).
    """

    def __init__(self):
        super().__init__()
        self.name = 'fixedwing_energy_model'
        self.mass = 1.0                    # kg
        self.wing_area = 0.35              # m^2
        self.aspect_ratio = 6.5
        self.rho = 1.2041                  # kg/m^3, sea level
        self.c_d_0 = 0.029
        self.oswald_eff = 0.97
        self.max_thrust = 8.54858e-06 * 3500.0**2  # N, from PX4 config.sdf motor model
        self.min_fxw = 0.0
        self.max_fxw = self.max_thrust / self.mass

    def get_acados_model(self) -> AcadosModel:
        model = AcadosModel()

        # states
        p      = cs.MX.sym('p', 3)
        speed  = cs.MX.sym('speed', 1)
        q      = cs.MX.sym('q', 4)
        # controls [f_xw (specific thrust), f_zw (specific lift), roll_rate]
        f_xw = cs.MX.sym('f_xw', 1)
        f_zw = cs.MX.sym('f_zw', 1)
        roll_r = cs.MX.sym('roll_rate', 1)

        x = cs.vertcat(p, speed, q)
        u = cs.vertcat(f_xw, f_zw, roll_r)

        xdot = cs.MX.sym('xdot', 8) # p_dot(3), s_dot(1), q_dot(4)
        q_safe = q/cs.sqrt(cs.sumsqr(q)+1e-6)
        R = q_to_rot_mat(q_safe)
        velocity_i = cs.mtimes(R, cs.vertcat(speed, 0, 0))

        speed_safe = cs.sqrt(speed**2 + 1e-6)

        g_inertial = cs.vertcat(0, 0, self.gravity)
        g_wind = cs.mtimes(R.T, g_inertial)

        q_wind = -(f_zw + g_wind[2]) / speed_safe
        r_wind = g_wind[1] / speed_safe

        # drag polar: C_L is inferred from the specific lift force f_zw = L/m,
        # then C_D = C_D0 + C_L^2/(pi*e*AR) gives the induced+parasitic specific drag.
        k1 = 0.5 * self.rho * self.wing_area / self.mass  # so f_zw = k1 * V^2 * C_L
        c_l = f_zw / (k1 * speed_safe**2)
        c_d = self.c_d_0 + (c_l**2) / (cs.pi * self.oswald_eff * self.aspect_ratio)
        specific_drag = k1 * speed_safe**2 * c_d
        accel = f_xw - specific_drag + g_wind[0]

        omega = cs.vertcat(roll_r, q_wind, r_wind)
        q_dot = 0.5 * cs.mtimes(skew_symmetric(omega), q)
        f_expl = cs.vertcat(velocity_i, accel, q_dot)

        # mass-specific total energy rate: e = g*h + V^2/2  =>  edot = g*hdot + V*Vdot.
        # Since g_wind[0] = -g*hdot/V identically (gravity does no net work), this
        # collapses to edot = V*(f_xw - specific_drag), i.e. purely the thrust/drag power.
        self.specific_energy_rate_expr = speed * accel + (-self.gravity) * velocity_i[2]
        # exposed so a "steady, non-accelerating flight" penalty/constraint can be built
        # around the trim conditions accel=0, pitch_rate_expr=0 (see FixedWingEnergyMPC).
        self.accel_expr = accel
        self.pitch_rate_expr = q_wind

        model.f_impl_expr = xdot - f_expl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = self.name

        return model
