"""
F3F slope-racing glider in a 2D ridge wind field.

Frames: NED world (x-North, y-East, z-Down), FRD body, Hamilton quaternion (w,x,y,z) body->NED,
the same conventions as models.fixedwing_bodyrate_model.

Ridge: an infinitely long ridge along North, modeled as a half cylinder of radius R whose axis
lies on the valley floor at e = 0, h = 0 (h = -d is height above the valley floor). Wind blows
from the west (towards +e) with far-field speed U, so the windward slope faces west (e < 0).
The wind field is 2D potential flow around the cylinder:
    w_e  = U (1 - R^2 (e^2 - h^2) / r^4)
    w_up = -2 U R^2 e h / r^4                      (r^2 = e^2 + h^2)
which gives the classic slope-lift band in front of the ridge (w_up = U at 45 deg on the
surface, decaying with 1/r^2) and speed-up over the crest.

Potential flow is inviscid, so on its own it puts the strongest lift right at the surface and
dead air at the foot of the ridge, which makes flying low and close to the face artificially
cheap. Both wind components are therefore scaled by a near-ground profile in the height above
terrain z (the smaller of the distance to the ridge surface and the height above the valley
floor):
    f(z) = (min(z, z_ref) / z_ref)^alpha       (power law, alpha ~ 1/7 for open terrain)
smoothed so the MPC's exact Hessian stays well defined. shear_exponent = 0 disables it.

Glider: point mass with body-rate inputs (Beard & McLain), no thrust, plus a first-order lag
between commanded and actual body rates standing in for the autopilot rate loop:
    x = [p (NED), v (inertial velocity in body frame), q, omega (actual body rates)]   (13)
    u = omega_cmd                                                                       (3)
Aerodynamics use the local air-relative velocity v - R^T w(p), so energy extraction from the
updraft (and from wind gradients) falls out of the dynamics without extra terms.
"""
from dataclasses import dataclass

import casadi as cs
import numpy as np

from models.fixedwing_model import skew_symmetric, q_to_rot_mat

GRAVITY = 9.81
RHO = 1.225

NX = 13
NU = 3


@dataclass
class Glider:
    # ballasted ~2.5 m span F3F racer
    mass: float = 3.0
    wing_area: float = 0.55
    aspect_ratio: float = 15.0
    oswald_eff: float = 0.9
    c_d_0: float = 0.010
    c_l_0: float = 0.05
    c_l_a: float = 5.5
    alpha_max: float = 0.20     # rad, C_L max ~1.15; stall is not modeled, so keep below it
    alpha_min: float = -0.10
    beta_max: float = 0.10      # rad, no side force is modeled: keep turns coordinated
    load_max: float = 15.0      # g, structural
    tau_rate: float = 0.08      # s, rate-loop time constant
    max_rates: tuple = (6.0, 5.0, 2.0)  # rad/s, roll / pitch / yaw commands

    @property
    def k_induced(self):
        return 1.0 / (np.pi * self.oswald_eff * self.aspect_ratio)


@dataclass
class Ridge:
    wind_speed: float = 10.0  # far-field U, m/s
    radius: float = 60.0      # ridge height = cylinder radius, m
    clearance: float = 5.0    # min distance from the ridge surface, m
    h_min: float = 5.0        # min height above the valley floor, m
    safety_e: float = 0.0     # safety line: must stay at e <= safety_e (in front of the crest)
    shear_exponent: float = 0.14     # near-ground wind profile exponent (0: pure potential flow)
    shear_ref_height: float = 50.0   # height above terrain where the profile reaches 1, m


def _smooth_min(a, b, eps):
    return 0.5 * (a + b - cs.sqrt((a - b)**2 + eps**2))


def height_above_terrain(p, ridge: Ridge):
    """Smoothed min of the distance to the ridge surface and the height above the floor."""
    return _smooth_min(ridge_distance(p, ridge), -p[2], 1.0)


def wind_profile(p, ridge: Ridge):
    """Near-ground wind reduction factor in (0, 1]."""
    if ridge.shear_exponent == 0.0:
        return 1.0
    z = height_above_terrain(p, ridge)
    z = 0.5 * (z + cs.sqrt(z**2 + 1.0))  # smooth max(z, 0), stays > 0
    z = _smooth_min(z, ridge.shear_ref_height, 0.1)
    return (z / ridge.shear_ref_height)**ridge.shear_exponent


def ridge_wind(p, ridge: Ridge):
    """NED wind vector at NED position p (potential flow times the near-ground profile)."""
    e, h = p[1], -p[2]
    r2 = e**2 + h**2 + 1e-6
    r4 = r2**2
    U, R = ridge.wind_speed, ridge.radius
    w_e = U * (1 - R**2 * (e**2 - h**2) / r4)
    w_up = -2 * U * R**2 * e * h / r4
    return wind_profile(p, ridge) * cs.vertcat(0, w_e, -w_up)


def ridge_distance(p, ridge: Ridge):
    """Distance from the ridge surface (negative inside the ridge)."""
    return cs.sqrt(p[1]**2 + p[2]**2 + 1e-6) - ridge.radius


def aero(x, glider: Glider, ridge: Ridge):
    """Air data and aerodynamic forces (body frame, per unit mass)."""
    q = x[6:10] / cs.sqrt(cs.sumsqr(x[6:10]) + 1e-9)
    R = q_to_rot_mat(q)
    v_air = x[3:6] - cs.mtimes(R.T, ridge_wind(x[0:3], ridge))
    V = cs.sqrt(cs.sumsqr(v_air) + 1e-6)
    alpha = cs.atan2(v_air[2], v_air[0] + 1e-6)
    beta = cs.asin(v_air[1] / V)
    c_l = glider.c_l_0 + glider.c_l_a * alpha
    c_d = glider.c_d_0 + glider.k_induced * c_l**2
    q_bar_s = 0.5 * RHO * V**2 * glider.wing_area
    L, D = q_bar_s * c_l, q_bar_s * c_d
    cos_a, sin_a = cs.cos(alpha), cs.sin(alpha)
    f_aero = cs.vertcat(-D * cos_a + L * sin_a, 0, -D * sin_a - L * cos_a) / glider.mass
    return {'R': R, 'v_air': v_air, 'V': V, 'alpha': alpha, 'beta': beta,
            'load': L / (glider.mass * GRAVITY), 'f_aero': f_aero}


def ode(x, u, glider: Glider, ridge: Ridge):
    a = aero(x, glider, ridge)
    v, omega = x[3:6], x[10:13]
    v_dot = a['f_aero'] + cs.mtimes(a['R'].T, cs.vertcat(0, 0, GRAVITY)) - cs.cross(omega, v)
    p_dot = cs.mtimes(a['R'], v)
    q_dot = 0.5 * cs.mtimes(skew_symmetric(omega), x[6:10])
    omega_dot = (u - omega) / glider.tau_rate
    return cs.vertcat(p_dot, v_dot, q_dot, omega_dot)


def rk4(x, u, dt, n_sub, glider, ridge):
    h = dt / n_sub
    for _ in range(n_sub):
        k1 = ode(x, u, glider, ridge)
        k2 = ode(x + h / 2 * k1, u, glider, ridge)
        k3 = ode(x + h / 2 * k2, u, glider, ridge)
        k4 = ode(x + h * k3, u, glider, ridge)
        x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def make_step(dt, glider: Glider, ridge: Ridge, n_sub=4):
    """CasADi function (x, u) -> x+ with quaternion renormalization (the simulation plant)."""
    x = cs.SX.sym('x', NX)
    u = cs.SX.sym('u', NU)
    xn = rk4(x, u, dt, n_sub, glider, ridge)
    xn = cs.vertcat(xn[:6], xn[6:10] / cs.norm_2(xn[6:10]), xn[10:])
    return cs.Function('F', [x, u], [xn])


def energy_height(x):
    """Specific total energy as an equivalent height, h + |v|^2 / 2g (inertial speed)."""
    return -x[2] + cs.sumsqr(x[3:6]) / (2 * GRAVITY)


def path_constraints(x, glider: Glider, ridge: Ridge):
    """
    Constraint expressions g(x) with bounds lb <= g <= ub, shared by the MPC and the offline
    optimum: [alpha, beta, load factor, ridge clearance, height, safety line].
    """
    a = aero(x, glider, ridge)
    g = cs.vertcat(a['alpha'], a['beta'], a['load'], ridge_distance(x[0:3], ridge),
                   -x[2], x[1])
    lb = np.array([glider.alpha_min, -glider.beta_max, -0.5 * glider.load_max,
                   ridge.clearance, ridge.h_min, -1e4])
    ub = np.array([glider.alpha_max, glider.beta_max, glider.load_max, 1e4, 1e4,
                   ridge.safety_e])
    return g, lb, ub


def make_diagnostics(glider: Glider, ridge: Ridge):
    """CasADi function x -> [V_air, alpha, beta, load, ridge distance, energy height]."""
    x = cs.SX.sym('x', NX)
    a = aero(x, glider, ridge)
    return cs.Function('diag', [x], [cs.vertcat(a['V'], a['alpha'], a['beta'], a['load'],
                                                ridge_distance(x[0:3], ridge),
                                                energy_height(x))])


def level_state(pos, course, ground_speed, glider: Glider):
    """Wings-level state flying at course with the given ground speed, pitched to 1 g trim."""
    q_bar_s = 0.5 * RHO * ground_speed**2 * glider.wing_area
    alpha = (glider.mass * GRAVITY / q_bar_s - glider.c_l_0) / glider.c_l_a
    cy, sy = np.cos(course / 2), np.sin(course / 2)
    cp, sp = np.cos(alpha / 2), np.sin(alpha / 2)
    q = np.array([cy * cp, -sy * sp, cy * sp, sy * cp])
    v = ground_speed * np.array([np.cos(alpha), 0.0, np.sin(alpha)])
    return np.concatenate([np.asarray(pos, dtype=float), v, q, np.zeros(3)])
