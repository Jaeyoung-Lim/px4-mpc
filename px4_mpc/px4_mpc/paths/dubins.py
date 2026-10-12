"""
Dubins paths for fixed-wing path following, in the NED frame used by the body-rate models.

Conventions:
    position (n, e, d), course chi measured from north towards east (so chi increasing is a
    RIGHT turn seen from above). A segment's signed curvature kappa = d chi / ds, i.e. kappa > 0
    is a right turn and kappa < 0 a left turn. Note this is the opposite sign of
    terrain_navigation's PathSegment (ENU, positive = left turn) - flip the sign when importing
    paths from the terrain planner.

The shortest-path computation follows the standard closed-form solution (Shkel & Lumelsky,
"Classification of the Dubins set", 2001), written in the (n, e, chi) plane, which is a
right-handed (x, y, theta) plane, so the textbook "L" (theta increasing) is a right turn here.
"""
import numpy as np

TWO_PI = 2.0 * np.pi


def _mod2pi(a):
    return a - TWO_PI * np.floor(a / TWO_PI)


class Segment:
    """Constant-curvature horizontal segment with an optional constant flight-path slope."""

    def __init__(self, start, chi0, kappa, length, d_slope=0.0):
        self.start = np.asarray(start, dtype=float)  # (n, e, d)
        self.chi0 = float(chi0)
        self.kappa = float(kappa)
        self.length = float(length)
        self.d_slope = float(d_slope)  # d(down)/ds, i.e. -tan(flight path angle)

    def pose(self, s):
        """Position, unit tangent (3D) and curvature at arc length s from the segment start."""
        k = self.kappa
        if abs(k) < 1e-9:
            dn, de = s * np.cos(self.chi0), s * np.sin(self.chi0)
            chi = self.chi0
        else:
            chi = self.chi0 + k * s
            dn = (np.sin(chi) - np.sin(self.chi0)) / k
            de = (np.cos(self.chi0) - np.cos(chi)) / k
        pos = self.start + np.array([dn, de, self.d_slope * s])
        tangent = np.array([np.cos(chi), np.sin(chi), self.d_slope])
        return pos, tangent / np.linalg.norm(tangent), k

    def end(self):
        pos, tangent, _ = self.pose(self.length)
        return pos, self.chi0 + self.kappa * self.length

    def closest(self, pos):
        """Arc length (clamped to the segment) of the horizontally closest point to pos."""
        p = np.asarray(pos, dtype=float)[:2]
        if abs(self.kappa) < 1e-9:
            t = np.array([np.cos(self.chi0), np.sin(self.chi0)])
            s = float(np.dot(p - self.start[:2], t))
        else:
            r = 1.0 / self.kappa  # signed radius
            # center lies to the right of the course for kappa > 0
            center = self.start[:2] + r * np.array([-np.sin(self.chi0), np.cos(self.chi0)])
            rel = p - center
            # course at the point on the circle in direction rel from the center
            chi = np.arctan2(rel[1], rel[0]) + (np.pi / 2 if self.kappa > 0 else -np.pi / 2)
            dchi = chi - self.chi0 if self.kappa > 0 else self.chi0 - chi
            dchi = _mod2pi(dchi)
            s = dchi / abs(self.kappa)
            # past the end of the arc: snap to whichever end is closer in angle
            if s > self.length:
                overshoot = s - self.length
                undershoot = TWO_PI / abs(self.kappa) - s
                s = self.length if overshoot < undershoot else 0.0
        return float(np.clip(s, 0.0, self.length))


class Path:
    """
    Concatenation of segments, parametrized by total arc length s.

    loop=True wraps s modulo the path length (e.g. a loiter circle); otherwise the path is
    extended past its end by a straight line along the final tangent, so the MPC horizon can
    always be filled with references.
    """

    def __init__(self, segments, loop=False):
        self.segments = list(segments)
        self.loop = loop
        self.offsets = np.concatenate([[0.0], np.cumsum([sg.length for sg in self.segments])])
        self.length = float(self.offsets[-1])

    def pose(self, s):
        if self.loop:
            s = _mod2pi(s * TWO_PI / self.length) * self.length / TWO_PI
        if s < 0.0:
            pos, tangent, _ = self.segments[0].pose(0.0)
            return pos + s * tangent, tangent, 0.0
        if s > self.length:
            pos, tangent, _ = self.segments[-1].pose(self.segments[-1].length)
            return pos + (s - self.length) * tangent, tangent, 0.0
        i = int(np.clip(np.searchsorted(self.offsets, s, side='right') - 1, 0,
                        len(self.segments) - 1))
        return self.segments[i].pose(s - self.offsets[i])

    def project(self, pos, s_hint=None, window=None):
        """
        Arc length of the closest point on the path. With s_hint and window, only segments
        overlapping [s_hint - window/4, s_hint + window] are considered, which keeps the
        projection from jumping between parts of a self-intersecting path.
        """
        best_s, best_d = 0.0, np.inf
        for i, sg in enumerate(self.segments):
            if s_hint is not None and window is not None and not self.loop:
                behind = self.offsets[i + 1] < s_hint - 0.25 * window
                if behind or self.offsets[i] > s_hint + window:
                    continue
            s = self.offsets[i] + sg.closest(pos)
            ref, _, _ = self.pose(s)
            d = np.linalg.norm(ref[:2] - np.asarray(pos)[:2])
            if d < best_d:
                best_s, best_d = s, d
        if self.loop and s_hint is not None:
            # unwrap so progress is continuous across laps
            lap = np.round((s_hint - best_s) / self.length)
            best_s += lap * self.length
        return best_s

    def sample(self, s_list):
        """Stacked (positions, tangents, curvatures) at the given arc lengths."""
        poses = [self.pose(s) for s in s_list]
        return (np.array([p[0] for p in poses]), np.array([p[1] for p in poses]),
                np.array([p[2] for p in poses]))


def _dubins_words(d, a, b):
    """Normalized (t, p, q) for each word; None if infeasible. 'L' = theta increasing."""
    sa, sb, ca, cb = np.sin(a), np.sin(b), np.cos(a), np.cos(b)
    c_ab = np.cos(a - b)
    words = {}

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq >= 0:
        tmp = np.arctan2(cb - ca, d + sa - sb)
        words['LSL'] = (_mod2pi(tmp - a), np.sqrt(p_sq), _mod2pi(b - tmp))

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq >= 0:
        tmp = np.arctan2(ca - cb, d - sa + sb)
        words['RSR'] = (_mod2pi(a - tmp), np.sqrt(p_sq), _mod2pi(tmp - b))

    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq >= 0:
        p = np.sqrt(p_sq)
        tmp = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        words['LSR'] = (_mod2pi(tmp - a), p, _mod2pi(tmp - b))

    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq >= 0:
        p = np.sqrt(p_sq)
        tmp = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        words['RSL'] = (_mod2pi(a - tmp), p, _mod2pi(b - tmp))

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(TWO_PI - np.arccos(tmp))
        t = _mod2pi(a - np.arctan2(ca - cb, d - sa + sb) + p / 2.0)
        words['RLR'] = (t, p, _mod2pi(a - b - t + p))

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1:
        p = _mod2pi(TWO_PI - np.arccos(tmp))
        t = _mod2pi(-a - np.arctan2(ca - cb, d + sa - sb) + p / 2.0)
        words['LRL'] = (t, p, _mod2pi(b - a - t + p))

    return words


def dubins_path(start, chi_start, goal, chi_goal, radius, word=None):
    """
    Shortest Dubins path between two (n, e, d) / course poses with minimum turn radius.
    Altitude is interpolated linearly in arc length between start and goal. Pass word
    (e.g. 'RSL') to force a particular word instead of the shortest.
    """
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    delta = goal[:2] - start[:2]
    theta = _mod2pi(np.arctan2(delta[1], delta[0]))
    d = np.linalg.norm(delta) / radius
    a, b = _mod2pi(chi_start - theta), _mod2pi(chi_goal - theta)

    words = _dubins_words(d, a, b)
    if word is not None:
        if word not in words:
            raise ValueError(f'Dubins word {word} infeasible for this pose pair')
        name, (t, p, q) = word, words[word]
    else:
        name, (t, p, q) = min(words.items(), key=lambda kv: sum(kv[1]))

    lengths = np.array([t, p, q]) * radius
    total = lengths.sum()
    d_slope = (goal[2] - start[2]) / total if total > 1e-9 else 0.0
    kappa_of = {'L': 1.0 / radius, 'R': -1.0 / radius, 'S': 0.0}

    segments, pos, chi = [], start.copy(), chi_start
    for letter, length in zip(name, lengths):
        if length < 1e-9:
            continue
        sg = Segment(pos, chi, kappa_of[letter], length, d_slope)
        segments.append(sg)
        pos, chi = sg.end()
    return Path(segments)


def loiter_circle(center, radius, clockwise=True, chi_start=0.0):
    """Closed loiter circle (clockwise seen from above = right turns = kappa > 0 in NED)."""
    center = np.asarray(center, dtype=float)
    kappa = (1.0 if clockwise else -1.0) / radius
    # start point: center is to the right of the course for a right turn
    r = 1.0 / kappa
    start = center - r * np.array([-np.sin(chi_start), np.cos(chi_start), 0.0])
    return Path([Segment(start, chi_start, kappa, TWO_PI * radius)], loop=True)
