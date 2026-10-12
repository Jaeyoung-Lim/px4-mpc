"""
F3F course: two bases 100 m apart along the ridge (North axis), raced for 10 legs.

Simplified rules used here (check the current FAI sporting code for the real ones):
    - timing starts when the glider crosses base A heading towards base B,
    - each leg ends when the glider crosses the next base line (B, A, B, ...),
    - the race ends at the end of the 10th leg,
    - the glider must stay in front of the safety line (e <= safety_e) and clear of terrain.
The entry phase (launch and dive before the first crossing) is not modeled: races start from a
given state on base A.
"""
import numpy as np

BASE_A = 0.0
BASE_B = 100.0
LEG_LENGTH = BASE_B - BASE_A
N_LEGS = 10


def leg_target(leg):
    """Base line that ends leg `leg` (0-indexed): B, A, B, ..."""
    return BASE_B if leg % 2 == 0 else BASE_A


def leg_direction(leg):
    return 1.0 if leg % 2 == 0 else -1.0


class Race:
    def __init__(self, n_legs=N_LEGS):
        self.n_legs = n_legs
        self.leg = 0
        self.t = 0.0
        self.crossings = []           # time at the end of each leg
        self.safety_violations = 0    # steps spent behind the safety line
        self.crashed = False

    @property
    def finished(self):
        return self.leg >= self.n_legs or self.crashed

    @property
    def target(self):
        return leg_target(self.leg)

    def update(self, x_prev, x_next, dt, ridge, ridge_distance):
        """Advance the race clock by dt along the straight segment x_prev -> x_next."""
        n0, n1 = x_prev[0], x_next[0]
        tgt, d = leg_target(self.leg), leg_direction(self.leg)
        if d * (n0 - tgt) < 0 <= d * (n1 - tgt):
            frac = (tgt - n0) / (n1 - n0)
            self.crossings.append(self.t + frac * dt)
            self.leg += 1
        self.t += dt
        if x_next[1] > ridge.safety_e:
            self.safety_violations += 1
        if ridge_distance < 0.0 or -x_next[2] < 0.0:
            self.crashed = True

    @property
    def time(self):
        return self.crossings[-1] if self.leg >= self.n_legs else np.inf

    def leg_times(self):
        return np.diff(np.concatenate([[0.0], self.crossings]))
