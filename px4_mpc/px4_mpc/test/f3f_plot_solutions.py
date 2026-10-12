#!/usr/bin/env python3
"""
Plot F3F race trajectories saved by f3f_gap_check.py --save: the path colored by ground speed,
with the glider's pose drawn every --stamp seconds (scaled up so it is visible), in a 3D view
and a top view, for the closed-loop MPC race and the offline optimum.

Usage:
    python3 f3f_plot_solutions.py gap.npz [--out races.png] [--stamp 1.0] [--scale 6]
"""
import argparse
import os
import sys

import matplotlib
import numpy as np

matplotlib.use('Agg')
# a distro mpl_toolkits (/usr/lib/python3/dist-packages) can shadow the one shipped with a pip
# matplotlib and fail to import; prefer the one next to the matplotlib actually in use
import mpl_toolkits  # noqa: E402
mpl_toolkits.__path__.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    matplotlib.__file__)), 'mpl_toolkits'))
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection  # noqa: E402

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from f3f.course import BASE_A, BASE_B  # noqa: E402
from f3f.glider import Ridge  # noqa: E402
from f3f.race_optimal import node_times  # noqa: E402


def rot(q):
    qw, qx, qy, qz = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)]])


def ned_to_plot(p):
    """NED -> plot axes (East, North, Up)."""
    p = np.atleast_2d(p)
    return np.stack([p[:, 1], p[:, 0], -p[:, 2]], axis=1)


# glider outline in body FRD coordinates (~2.5 m span F3F glider)
WING = np.array([[0.15, -1.25, 0], [0.15, 1.25, 0], [-0.1, 1.15, 0], [-0.1, -1.15, 0]])
TAIL = np.array([[-0.85, -0.35, 0], [-0.85, 0.35, 0], [-1.0, 0.3, 0], [-1.0, -0.3, 0]])
FIN = np.array([[-0.8, 0, 0], [-1.0, 0, 0], [-1.05, 0, -0.35], [-0.95, 0, -0.35]])
FUSELAGE = np.array([[0.7, 0, 0], [-1.0, 0, 0]])


def glider_polys(x, scale):
    R, p = rot(x[6:10]), x[0:3]
    out = []
    for shape in (WING, TAIL, FIN):
        out.append(ned_to_plot(p + (R @ (scale * shape).T).T))
    fus = ned_to_plot(p + (R @ (scale * FUSELAGE).T).T)
    return out, fus


def ground_speed(X):
    return np.array([np.linalg.norm(rot(x[6:10]) @ x[3:6]) for x in X])


def interp_states(t, X, t_query):
    Xq = np.array([np.interp(t_query, t, X[:, j]) for j in range(X.shape[1])]).T
    Xq[:, 6:10] /= np.linalg.norm(Xq[:, 6:10], axis=1, keepdims=True)
    return Xq


def draw_ridge_3d(ax, ridge, n_range):
    th = np.linspace(0, np.pi, 40)
    n = np.linspace(*n_range, 2)
    TH, NN = np.meshgrid(th, n)
    ax.plot_surface(ridge.radius * np.cos(TH), NN, ridge.radius * np.sin(TH),
                    color='0.7', alpha=0.25, linewidth=0, shade=False)


def draw_race(ax3, ax2, t, X, title, norm, cmap, stamp, scale, ridge):
    P = ned_to_plot(X[:, 0:3])
    gs = ground_speed(X)
    seg = np.stack([P[:-1], P[1:]], axis=1)
    c = 0.5 * (gs[:-1] + gs[1:])

    lc3 = Line3DCollection(seg, cmap=cmap, norm=norm, linewidths=1.6)
    lc3.set_array(c)
    ax3.add_collection3d(lc3)
    lc2 = LineCollection(seg[:, :, :2], cmap=cmap, norm=norm, linewidths=1.6)
    lc2.set_array(c)
    ax2.add_collection(lc2)

    t_stamp = np.arange(0.0, t[-1] + 1e-9, stamp)
    for xs, gss in zip(interp_states(t, X, t_stamp), np.interp(t_stamp, t, gs)):
        polys, fus = glider_polys(xs, scale)
        col = cmap(norm(gss))
        ax3.add_collection3d(Poly3DCollection(polys, facecolors=[col] * 3, edgecolors='k',
                                              linewidths=0.4, alpha=0.95))
        ax3.plot(fus[:, 0], fus[:, 1], fus[:, 2], 'k-', lw=0.8)
        for poly in polys:
            ax2.fill(poly[:, 0], poly[:, 1], facecolor=col, edgecolor='k', lw=0.4, zorder=3)
        ax2.plot(fus[:, 0], fus[:, 1], 'k-', lw=0.8, zorder=3)

    n_range = (P[:, 1].min() - 10, P[:, 1].max() + 10)
    draw_ridge_3d(ax3, ridge, n_range)
    for b in (BASE_A, BASE_B):
        ax3.plot([-150, 0], [b, b], [0, 0], color='gray', ls='--', lw=0.8)
        ax2.axhline(b, color='gray', ls='--', lw=0.8)
    ax2.axvline(ridge.safety_e, color='r', ls=':', lw=1)
    ax2.axvspan(-ridge.radius, ridge.radius, color='0.85', zorder=0)

    lo, hi = P.min(axis=0), P.max(axis=0)
    lo[2] = 0.0
    ax3.set_xlim(lo[0] - 10, max(hi[0], 0) + 10)
    ax3.set_ylim(lo[1] - 10, hi[1] + 10)
    ax3.set_zlim(0, hi[2] + 10)
    ax3.set_box_aspect(np.maximum(hi - lo + 20, 1.0))
    ax3.set_xlabel('East [m]')
    ax3.set_ylabel('North [m]')
    ax3.set_zlabel('height [m]')
    ax3.view_init(elev=22, azim=-150)
    ax3.set_title(title)

    ax2.set_xlim(lo[0] - 10, ridge.radius * 0.3)
    ax2.set_ylim(lo[1] - 10, hi[1] + 10)
    ax2.set_aspect('equal')
    ax2.set_xlabel('East [m]  (crest at 0, wind from the left)')
    ax2.set_ylabel('North [m]')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('npz')
    parser.add_argument('--out', default='f3f_races.png')
    parser.add_argument('--stamp', type=float, default=1.0, help='pose stamp interval [s]')
    parser.add_argument('--scale', type=float, default=6.0, help='glider glyph scale')
    parser.add_argument('--wind', type=float, default=10.0)
    args = parser.parse_args()

    d = np.load(args.npz)
    ridge = Ridge(wind_speed=args.wind)
    mpc_X, mpc_t = d['mpc_x'], d['mpc_t']
    T = d['opt_T']
    N_leg = (d['opt_X'].shape[1] - 1) // len(T)
    opt_X, opt_t = d['opt_X'].T, node_times(T, N_leg)
    t_mpc = d['mpc_crossings'][-1]

    gs_all = np.concatenate([ground_speed(mpc_X), ground_speed(opt_X)])
    norm = matplotlib.colors.Normalize(vmin=np.floor(gs_all.min()), vmax=np.ceil(gs_all.max()))
    cmap = plt.get_cmap('viridis')

    fig = plt.figure(figsize=(16, 13))
    axes = [(fig.add_subplot(2, 2, 1, projection='3d'), fig.add_subplot(2, 2, 2)),
            (fig.add_subplot(2, 2, 3, projection='3d'), fig.add_subplot(2, 2, 4))]
    draw_race(*axes[0], mpc_t, mpc_X, f'Short-horizon MPC: {t_mpc:.2f} s', norm, cmap,
              args.stamp, args.scale, ridge)
    draw_race(*axes[1], opt_t, opt_X, f'Offline optimum: {T.sum():.2f} s', norm, cmap,
              args.stamp, args.scale, ridge)
    sm = matplotlib.cm.ScalarMappable(norm=norm, cmap=cmap)
    fig.colorbar(sm, ax=[a for pair in axes for a in pair], shrink=0.6,
                 label='ground speed [m/s]')
    fig.suptitle(f'F3F, {len(T)} legs, wind {args.wind:.0f} m/s from the west - glider pose '
                 f'every {args.stamp:g} s (drawn {args.scale:g}x size)')
    fig.savefig(args.out, dpi=110, bbox_inches='tight')
    print(f'saved {args.out}')


if __name__ == '__main__':
    main()
