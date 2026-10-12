import os
import sys

import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from paths.dubins import dubins_path, loiter_circle  # noqa: E402


def _angle_diff(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def test_dubins_reaches_goal():
    rng = np.random.default_rng(0)
    for _ in range(200):
        start = np.append(rng.uniform(-100, 100, 2), -50.0)
        goal = np.append(rng.uniform(-100, 100, 2), rng.uniform(-80, -20))
        chi0, chi1 = rng.uniform(-np.pi, np.pi, 2)
        path = dubins_path(start, chi0, goal, chi1, radius=25.0)
        end_pos, end_tangent, _ = path.pose(path.length)
        assert np.allclose(end_pos, goal, atol=1e-6)
        assert abs(_angle_diff(np.arctan2(end_tangent[1], end_tangent[0]), chi1)) < 1e-6
        # the straight-line distance is a lower bound on the Dubins length
        assert path.length >= np.linalg.norm(goal[:2] - start[:2]) - 1e-9


def test_dubins_is_shortest_word():
    path = dubins_path([0, 0, 0], 0.0, [100, 0, 0], 0.0, radius=20.0)
    assert np.isclose(path.length, 100.0)


def test_turn_sign_ned():
    # heading north, goal to the east-north: first turn should be to the right (kappa > 0)
    path = dubins_path([0, 0, 0], 0.0, [100, 100, 0], np.pi / 2, radius=20.0)
    assert path.segments[0].kappa > 0


def test_projection_on_path():
    path = dubins_path([0, 0, -50], 0.0, [60, 80, -50], np.pi, radius=20.0)
    for s in np.linspace(0, path.length, 37):
        pos, tangent, _ = path.pose(s)
        normal = np.array([-tangent[1], tangent[0], 0.0])
        s_proj = path.project(pos + 2.0 * normal, s_hint=s, window=20.0)
        assert abs(s_proj - s) < 1e-6 or np.allclose(path.pose(s_proj)[0], pos, atol=1e-6)


def test_loiter_circle():
    path = loiter_circle([0, 0, -50], 30.0, clockwise=True)
    for s in np.linspace(0, 3 * path.length, 50):
        pos, _, kappa = path.pose(s)
        assert np.isclose(np.linalg.norm(pos[:2]), 30.0)
        assert kappa > 0
    # projection keeps progress continuous across laps
    pos, _, _ = path.pose(2.2 * path.length)
    assert np.isclose(path.project(pos, s_hint=2.19 * path.length), 2.2 * path.length)
