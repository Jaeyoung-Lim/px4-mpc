#!/usr/bin/env python3
"""
Plots the zero-throttle sink polar for FixedWingEnergyModel, and runs
FixedWingEnergyMPC in closed loop (real acados NMPC, SQP_RTI) from two
arbitrary (banked, off-trim) initial states to show it converges to the
minimum-sink trim airspeed.

Usage:
    python3 plot_min_sink_polar.py [--steps 500] [--out sink_polar.png]

Shows the plot in a window by default; pass --out to save a PNG instead (or as well).
"""
import argparse
import os
import sys
import time

import numpy as np
from scipy.optimize import brentq, minimize_scalar
from scipy.spatial.transform import Rotation as Rot
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, to_rgb
from matplotlib.lines import Line2D

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from models.fixedwing_energy_model import FixedWingEnergyModel  # noqa: E402
from controllers.fixedwing_energy_mpc import FixedWingEnergyMPC  # noqa: E402

C_POLAR = '#2a78d6'
C_A = '#eb6834'
C_B = '#1baf7a'
MUTED = '#898781'


def specific_drag(model, V, fzw):
    """Induced + parasitic specific drag from the model's own drag polar."""
    k1 = 0.5 * model.rho * model.wing_area / model.mass
    k = 1.0 / (np.pi * model.oswald_eff * model.aspect_ratio)
    c_l = fzw / (k1 * V**2)
    c_d = model.c_d_0 + k * c_l**2
    return k1 * V**2 * c_d


def trim_gamma(model, g, V):
    """Steady, zero-throttle trim flight-path angle at airspeed V: L=Wcos(gamma), D=Wsin(gamma)."""
    return brentq(lambda gam: specific_drag(model, V, g * np.cos(gam)) - g * np.sin(gam),
                  1e-6, np.radians(80))


def sink_rate_at(model, g, V):
    return V * np.sin(trim_gamma(model, g, V))


def compute_sink_polar(model, V_min=2.5, V_max=30.0, n=400):
    g = -model.gravity
    V_sweep = np.linspace(V_min, V_max, n)
    sink_sweep = np.array([sink_rate_at(model, g, v) for v in V_sweep])

    res_ms = minimize_scalar(lambda v: sink_rate_at(model, g, v),
                              bounds=(V_min, V_max), method='bounded', options={'xatol': 1e-10})
    res_bg = minimize_scalar(lambda v: np.tan(trim_gamma(model, g, v)),
                              bounds=(V_min, V_max), method='bounded', options={'xatol': 1e-10})
    V_ms, sink_ms = res_ms.x, res_ms.fun
    V_bg = res_bg.x
    sink_bg = sink_rate_at(model, g, V_bg)
    LD_max = 1.0 / np.tan(trim_gamma(model, g, V_bg))
    return dict(V_sweep=V_sweep, sink_sweep=sink_sweep, V_ms=V_ms, sink_ms=sink_ms,
                V_bg=V_bg, sink_bg=sink_bg, LD_max=LD_max)


def arbitrary_quaternion(roll_deg, pitch_deg, yaw_deg):
    x, y, z, w = Rot.from_euler('xyz', np.radians([roll_deg, pitch_deg, yaw_deg])).as_quat()
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rollout_guess(mpc: FixedWingEnergyMPC, x0: np.ndarray, u_const: np.ndarray):
    """Dynamically-consistent initial guess: physically integrate x0 forward N steps with
    a constant control, rather than repeating x0. SQP_RTI does exactly one Newton step per
    solve() call (unlike IPOPT, which iterates a single call to convergence), so it needs a
    guess that already respects the dynamics, or the first linearization can be badly off."""
    N = mpc.N
    x_guess = np.zeros((N + 1, mpc.nx))
    x_guess[0, :] = x0
    x = x0.copy()
    for i in range(N):
        mpc.integrator.set("x", x)
        mpc.integrator.set("u", u_const)
        mpc.integrator.solve()
        x = mpc.integrator.get("x")
        x_guess[i + 1, :] = x
    u_guess = np.tile(u_const, (N, 1))
    return x_guess, u_guess


def run_closed_loop(mpc: FixedWingEnergyMPC, x0: np.ndarray, n_steps: int, Ts: float, n_prep=10):
    """Closed-loop receding-horizon rollout: mpc.solve() then mpc.integrator to step the
    real nonlinear plant, mirroring the pattern used in test_fw_solver.py.

    Two robustness measures beyond a naive port of that pattern, both needed because SQP_RTI
    (one Newton/SQP step per call) is far less forgiving of a bad starting point than IPOPT
    (which iterates a single solve() to convergence):
      - the initial guess is a real dynamic rollout, not x0 repeated at every stage;
      - a few "prep" solves are run at the fixed initial state (RTI's standard trick for
        approximating several SQP iterations) before the plant is allowed to move.
    Both diagnose and recover from - rather than silently propagate - a QP failure by holding
    the previous applied control instead of trusting a possibly-garbage new one.
    """
    nx = mpc.nx
    x_curr = x0.copy()
    x_hist = np.zeros((n_steps + 1, nx))
    x_hist[0, :] = x_curr

    u_trim = np.array([0.0, -mpc.model.gravity, 0.0])
    x_guess, u_guess = rollout_guess(mpc, x0, u_trim)
    u_last_good = u_trim.copy()

    for _ in range(n_prep):
        simU, simX, status = mpc.solve(x_curr, x_guess, u_guess, verbose=False)
        if status == 0:
            x_guess[:-1, :] = simX[1:, :]
            x_guess[-1, :] = simX[-1, :]
            u_guess[:-1, :] = simU[1:, :]
            u_guess[-1, :] = simU[-1, :]
            u_last_good = simU[0, :]

    for k in range(n_steps):
        simU, simX, status = mpc.solve(x_curr, x_guess, u_guess, verbose=False)
        if status != 0 or not np.all(np.isfinite(simU)):
            print(f"  [!] solver status {status} at step {k}, holding last good control")
            u_apply = u_last_good
        else:
            u_apply = simU[0, :]
            u_last_good = u_apply
            x_guess[:-1, :] = simX[1:, :]
            x_guess[-1, :] = simX[-1, :]
            u_guess[:-1, :] = simU[1:, :]
            u_guess[-1, :] = simU[-1, :]

        mpc.integrator.set("x", x_curr)
        mpc.integrator.set("u", u_apply)
        if mpc.integrator.solve() != 0 or not np.all(np.isfinite(mpc.integrator.get("x"))):
            print(f"  [!] plant integration diverged at step {k}, stopping rollout early")
            return x_hist[: k + 1], k * Ts
        x_curr = mpc.integrator.get("x")
        x_hist[k + 1, :] = x_curr

    return x_hist, n_steps * Ts


def mono_cmap(hexcolor):
    rgb = to_rgb(hexcolor)
    light = tuple(1 - 0.35 * (1 - c) for c in rgb)
    return LinearSegmentedColormap.from_list('mono', [light, rgb])


def plot_traj(ax, V, sink, color, n_markers=45):
    ax.plot(V, sink, color=color, lw=1.4, alpha=0.55, zorder=2)
    n = len(V)
    idx = np.linspace(0, n - 1, min(n, n_markers)).astype(int)
    ax.scatter(np.asarray(V)[idx], np.asarray(sink)[idx], c=np.arange(len(idx)),
               cmap=mono_cmap(color), s=14, zorder=4, edgecolors='none')
    ax.scatter([V[0]], [sink[0]], s=90, facecolors='none', edgecolors=color, lw=1.6, zorder=5)
    ax.annotate('start', (V[0], sink[0]), textcoords='offset points', xytext=(6, 8),
                fontsize=8.5, color=color)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steps', type=int, default=500, help='closed-loop steps per run (Ts=0.05s each)')
    parser.add_argument('--out', type=str, default=None, help='save a PNG to this path instead of showing a window')
    args = parser.parse_args()

    model = FixedWingEnergyModel()
    Ts = 0.05
    N = 20

    print("Computing zero-throttle sink polar from the drag polar...")
    polar = compute_sink_polar(model)
    print(f"  V_min_sink = {polar['V_ms']:.3f} m/s   sink = {polar['sink_ms']:.4f} m/s")
    print(f"  V_best_glide = {polar['V_bg']:.3f} m/s   L/D_max = {polar['LD_max']:.2f}")

    scenarios = {
        'A (from above trim, banked climb)': dict(
            p=[50, -30, 120], V0=18.0, q=arbitrary_quaternion(35, -25, 60), color=C_A),
        'B (from below trim, banked dive)': dict(
            p=[-80, 10, 60], V0=3.5, q=arbitrary_quaternion(-15, 40, -100), color=C_B),
    }

    results = {}
    for name, sc in scenarios.items():
        print(f"Running closed-loop MPC: {name} ...")
        x0 = np.array(sc['p'] + [sc['V0']] + list(sc['q']))
        mpc = FixedWingEnergyMPC(model, x0_init=x0, N=N, Ts=Ts)
        t0 = time.time()
        x_hist, t_final = run_closed_loop(mpc, x0, args.steps, Ts)
        print(f"  done in {time.time() - t0:.1f}s wall, V: {x_hist[0, 3]:.2f} -> {x_hist[-1, 3]:.3f} m/s")
        sink_hist = -np.diff(x_hist[:, 2]) / Ts  # altitude is state index 2
        # sink is one shorter than V (computed between consecutive states); V_full keeps
        # every state for the time-series panel, V is trimmed to pair with sink for the polar
        results[name] = dict(V_full=x_hist[:, 3], V=x_hist[:-1, 3], sink=sink_hist, dt=Ts, color=sc['color'])

    # ---------------- plot ----------------
    fig = plt.figure(figsize=(10.5, 8.2))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.4, 1], hspace=0.32)
    ax = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])

    ax.plot(polar['V_sweep'], polar['sink_sweep'], color=C_POLAR, lw=2.2, zorder=3)
    x_end = polar['V_sweep'][-1]
    ax.plot([0, x_end], [0, (polar['sink_bg'] / polar['V_bg']) * x_end],
            color=MUTED, lw=1.2, ls=(0, (4, 3)), zorder=1)

    legend_lines = [Line2D([0], [0], color=C_POLAR, lw=2.2)]
    legend_labels = ['Zero-throttle sink polar']
    for name, r in results.items():
        plot_traj(ax, r['V'], r['sink'], r['color'])
        legend_lines.append(Line2D([0], [0], color=r['color'], lw=1.4, alpha=0.8))
        legend_labels.append(f"{name}, V0={r['V'][0]:.1f} m/s")
    legend_lines.append(Line2D([0], [0], color=MUTED, lw=1.2, ls=(0, (4, 3))))
    legend_labels.append(f"Best-glide tangent (L/D_max={polar['LD_max']:.1f})")

    for V0, s0, label, dx, dy in [
        (polar['V_ms'], polar['sink_ms'], f"Min sink\n{polar['V_ms']:.2f} m/s", 10, 6),
        (polar['V_bg'], polar['sink_bg'], f"Best glide\n{polar['V_bg']:.2f} m/s", 10, -28),
    ]:
        ax.scatter([V0], [s0], s=60, color=C_POLAR, zorder=6, edgecolors='white', linewidths=1.2)
        ax.annotate(label, (V0, s0), textcoords='offset points', xytext=(dx, dy),
                    fontsize=9.5, fontweight='bold', color='#0b0b0b')

    ax.set_xlim(0, polar['V_sweep'][-1])
    ax.set_ylim(0, 4)
    ax.set_xlabel('Airspeed V  [m/s]')
    ax.set_ylabel(r'Sink rate  $-\dot{h}$  [m/s]')
    ax.set_title('Zero-throttle sink polar: MPC converges to the minimum-sink trim\n'
                  'from arbitrary (banked, off-trim) initial states', fontsize=12.5)
    ax.grid(True, color='#e1e0d9', lw=0.8, zorder=0)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(legend_lines, legend_labels, loc='upper left', frameon=False, fontsize=9.5)

    for name, r in results.items():
        t = np.arange(len(r['V_full'])) * r['dt']
        ax2.plot(t, r['V_full'], color=r['color'], lw=1.8, label=f"{name.split('(')[0].strip()} ({r['V_full'][0]:.1f} m/s)")
    ax2.axhline(polar['V_ms'], color=C_POLAR, lw=1.2, ls='--', label=f"V_min_sink={polar['V_ms']:.2f} m/s")
    ax2.set_xlabel('time [s]')
    ax2.set_ylabel('airspeed V [m/s]')
    ax2.set_title('Airspeed vs. time, receding-horizon closed loop (thrust locked to 0)', fontsize=11)
    ax2.grid(True, color='#e1e0d9', lw=0.8)
    ax2.spines[['top', 'right']].set_visible(False)
    ax2.legend(loc='upper right', frameon=False, fontsize=9)

    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=160, facecolor='white')
        print(f"Saved {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
