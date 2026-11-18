"""
Kinematics + gait generator for a 12-DoF Spot-Micro style quadruped.

Pure Python (no Webots import) so it can be unit-tested on its own.

Frame convention (matches the Webots ENU world):  x = forward, y = left, z = up.
Leg joints, per leg:
    hip_abd   : rotation about +x  (swings the leg sideways)
    hip_pitch : rotation about +y  (positive -> foot moves backwards)
    knee      : rotation about +y  (negative -> knee points backwards, like Spot)
"""
import math

# ---------------------------------------------------------------- geometry [m]
BODY_L = 0.186          # shoulder-to-shoulder, front <-> rear
BODY_W = 0.078          # shoulder-to-shoulder, left <-> right
L1 = 0.055              # lateral offset: abduction axis -> hip-pitch axis
L2 = 0.1075             # thigh length
L3 = 0.130              # shank length
FOOT_R = 0.012          # foot radius

LEGS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}      # +1 = left
FRONT = {"FL": +1, "FR": +1, "RL": -1, "RR": -1}     # +1 = front
JOINTS = ("hip_abd", "hip_pitch", "knee")


# ------------------------------------------------------------------ kinematics
def leg_ik(x, y, z, side):
    """Foot position (x, y, z) relative to the shoulder  ->  (abd, pitch, knee) [rad]."""
    d = math.sqrt(max(y * y + z * z - L1 * L1, 1e-9))     # leg-plane vertical reach
    t1 = math.atan2(z, y) - math.atan2(-d, side * L1)
    c = (x * x + d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    c = max(-1.0, min(1.0, c))
    t3 = -math.acos(c)                                    # knee points backwards
    gamma = math.atan2(-x, d)
    t2 = gamma - math.atan2(L3 * math.sin(t3), L2 + L3 * math.cos(t3))
    return t1, t2, t3


def leg_fk(t1, t2, t3, side):
    """Inverse of leg_ik (used by the self-test)."""
    x = -L2 * math.sin(t2) - L3 * math.sin(t2 + t3)
    d = L2 * math.cos(t2) + L3 * math.cos(t2 + t3)
    a, b = side * L1, -d
    y = a * math.cos(t1) - b * math.sin(t1)
    z = a * math.sin(t1) + b * math.cos(t1)
    return x, y, z


# ------------------------------------------------------------------ gait table
# offsets: phase offset of each leg.  A leg swings while (u + offset) % 1 is in [duty, 1).
_CREEP = {"FL": 0.00, "RR": 0.75, "FR": 0.50, "RL": 0.25}   # FL -> RR -> FR -> RL
_TROT = {"FL": 0.00, "RR": 0.00, "FR": 0.50, "RL": 0.50}    # diagonal pairs together

GAITS = {
    # height: body height [m]   period: cycle time [s]   duty: fraction of the cycle a foot is on the ground
    # stride/lift: foot travel / foot clearance [m]
    # shift: weight shift toward the centre of the 3 supporting feet (0 = none, 1 = full)
    # lead/tail: how early (before lift-off) the shift is complete / how long it is held after touch-down [phase]
    "stand": dict(height=0.180, period=1.00, duty=0.75, stride=0.00, lift=0.000, shift=0.00, lead=0.10, tail=0.05, offsets=_CREEP),
    "crawl": dict(height=0.160, period=2.00, duty=0.85, stride=0.065, lift=0.030, shift=1.00, lead=0.12, tail=0.12, offsets=_CREEP),
    "walk":  dict(height=0.180, period=2.00, duty=0.80, stride=0.080, lift=0.035, shift=0.80, lead=0.16, tail=0.04, offsets=_CREEP),
    "trot":  dict(height=0.180, period=0.80, duty=0.60, stride=0.070, lift=0.035, shift=0.00, lead=0.10, tail=0.05, offsets=_TROT),
}
GAIT_KEYS = {"W": "walk", "C": "crawl", "T": "trot", "S": "stand"}


def _smoothstep(s):
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def _bump(phi, duty, lead, tail):
    """Weight shift envelope: rises to 1 by the moment the foot lifts (phi=duty), stays 1 during
    the swing, then falls back to 0 over `tail` after touch-down (phi=1)."""
    if phi >= duty:                                   # swing: fully shifted
        return 1.0
    if phi > duty - lead:                             # lead-in, before lift-off
        return _smoothstep((phi - (duty - lead)) / lead)
    if phi < tail:                                    # tail-out, after touch-down
        return 1.0 - _smoothstep(phi / tail)
    return 0.0


def foot_target(phi, duty, stride, lift):
    """Foot x offset and lift for one leg at cycle phase phi (cycloid swing, linear stance)."""
    if phi < duty:                                   # stance: foot slides backwards
        s = phi / duty
        return 0.5 * stride - stride * s, 0.0
    s = (phi - duty) / (1.0 - duty)                  # swing: foot returns forwards
    x = -0.5 * stride + stride * (s - math.sin(2.0 * math.pi * s) / (2.0 * math.pi))
    return x, lift * 0.5 * (1.0 - math.cos(2.0 * math.pi * s))


class GaitEngine:
    """Open-loop gait generator. Call set_command(), then step(dt) every control tick."""

    AMP_RATE = 1.0 / 0.8      # stride ramps 0<->1 in 0.8 s
    HEIGHT_RATE = 0.06        # m/s body height change
    TURN_GAIN = 0.6           # stride difference between inner / outer side at full turn

    def __init__(self):
        self.active = "stand"
        self.target = "stand"
        self.turn = 0.0       # -1 (right) .. +1 (left)
        self.reverse = False
        self.u = 0.0          # cycle phase
        self.amp = 0.0
        self.height = GAITS["stand"]["height"]

    # ------------------------------------------------------------ commands
    def set_command(self, gait=None, turn=None, reverse=None):
        if gait is not None:
            self.target = gait
        if turn is not None:
            self.turn = max(-1.0, min(1.0, turn))
        if reverse is not None:
            self.reverse = reverse

    @property
    def moving(self):
        return self.active != "stand" and self.amp > 0.01

    # ------------------------------------------------------------ update
    def step(self, dt):
        # --- gait switching: wind stride down, switch at zero, wind up again
        if self.target != self.active:
            if self.amp <= 1e-3:
                self.active = self.target
                self.amp = 0.0
            amp_goal = 0.0
        else:
            amp_goal = 0.0 if self.active == "stand" else 1.0
        g = GAITS[self.active]

        # --- body height follows the active gait
        dh = g["height"] - self.height
        self.height += max(-self.HEIGHT_RATE * dt, min(self.HEIGHT_RATE * dt, dh))
        if abs(dh) > 0.005:                       # finish lowering / raising before stepping
            amp_goal = 0.0

        da = self.AMP_RATE * dt
        self.amp += max(-da, min(da, amp_goal - self.amp))
        amp = _smoothstep(self.amp)

        self.u = (self.u + dt / g["period"]) % 1.0

        direction = -1.0 if self.reverse else 1.0
        stride = g["stride"] * amp * direction
        lift = g["lift"] * amp
        duty = g["duty"]

        phis = {leg: (self.u + g["offsets"][leg]) % 1.0 for leg in LEGS}
        weights = {leg: _bump(phis[leg], duty, g["lead"], g["tail"]) for leg in LEGS}
        # weight shift: while a leg is (about to be) in the air the body moves toward the centroid of the
        # other three feet, i.e. by -p_leg/3.  In the body frame every foot therefore moves by +p_leg/3.
        gain = g["shift"] * amp / 3.0
        shift_x = gain * (BODY_L / 2) * sum(FRONT[l] * weights[l] for l in LEGS)
        shift_y = gain * (BODY_W / 2 + L1) * sum(SIDE[l] * weights[l] for l in LEGS)

        out = {}
        for leg in LEGS:
            side = SIDE[leg]
            # turning: shorten the stride on the inside of the turn
            k = 1.0 - self.TURN_GAIN * self.turn * side
            fx, fl = foot_target(phis[leg], duty, stride * k, lift)
            x = fx + shift_x                          # foot below its own shoulder + gait + shift
            y = side * L1 + shift_y
            z = -self.height + fl
            out[leg] = leg_ik(x, y, z, side)
        return out


# ------------------------------------------------------------------ self-test
if __name__ == "__main__":
    import itertools
    worst = 0.0
    for side, x, y, z in itertools.product((1, -1), (-0.06, 0, 0.06), (0.03, 0.055, 0.09), (-0.12, -0.18, -0.22)):
        y *= side
        fx = leg_fk(*leg_ik(x, y, z, side), side)
        worst = max(worst, max(abs(a - b) for a, b in zip((x, y, z), fx)))
    print("IK/FK round-trip worst error: %.2e m" % worst)
