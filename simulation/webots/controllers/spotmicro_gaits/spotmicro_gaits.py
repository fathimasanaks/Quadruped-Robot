"""
Spot-Micro style 12-DoF quadruped - keyboard-driven gait demo (Webots).

The robot has NO sensors: the controller is purely open-loop and only talks to
12 servo motors through a PCA9685-style servo-driver layer.

Keys (click the 3D view first so it has keyboard focus):
    W  walk      C  crawl      T  trot      S  stand / stop
    Q  turn left (latched)     E  turn right (latched)     A  go straight
    R  toggle reverse
"""
import math
from controller import Supervisor
from gaits import GaitEngine, GAIT_KEYS, LEGS, JOINTS

STAND_UP_TIME = 2.0      # s, ramp from the straight-leg spawn pose into the stand pose
LABEL_COLOUR = 0x202020


# --------------------------------------------------------------------------
class ServoDriver:
    """
    Emulates a PCA9685 16-channel PWM driver driving hobby servos.

    angle [rad] -> servo angle [deg] -> pulse [us] -> 12-bit tick count -> back to an
    angle, so the simulated joint sees the same quantisation and 0-180 deg travel limit
    as the real hardware.  Replace `_apply()` with real I2C writes to drive hardware.
    """
    PWM_HZ = 50
    TICKS = 4096
    PULSE_MIN_US = 500.0
    PULSE_MAX_US = 2500.0

    # PCA9685 channel numbers (edit to match your wiring)
    CHANNEL = {}
    for _i, _leg in enumerate(LEGS):
        for _j, _joint in enumerate(JOINTS):
            CHANNEL["%s_%s" % (_leg.lower(), _joint)] = _i * 3 + _j

    # Servo horn centre: the joint angle (deg) that sits at 90 deg on the servo.
    # The knee works around -75 deg so its travel fits inside the servo's 180 deg.
    CENTRE_DEG = {"hip_abd": 0.0, "hip_pitch": 0.0, "knee": -75.0}
    # +1 / -1 per channel; flip for servos that are mounted mirrored on the real robot.
    SIGN = {name: 1.0 for name in CHANNEL}
    TRIM_DEG = {name: 0.0 for name in CHANNEL}

    def __init__(self, robot):
        self.motors = {}
        for name in self.CHANNEL:
            m = robot.getDevice(name)
            self.motors[name] = (m, m.getMinPosition(), m.getMaxPosition())

    def write(self, name, angle_rad):
        joint = name.split("_", 1)[1]
        sign, trim = self.SIGN[name], self.TRIM_DEG[name]
        centre = self.CENTRE_DEG[joint]
        deg = 90.0 + sign * (math.degrees(angle_rad) - centre) + trim
        deg = max(0.0, min(180.0, deg))
        pulse = self.PULSE_MIN_US + deg / 180.0 * (self.PULSE_MAX_US - self.PULSE_MIN_US)
        tick_us = 1e6 / self.PWM_HZ / self.TICKS
        pulse_q = round(pulse / tick_us) * tick_us                      # 12-bit quantisation
        deg_q = (pulse_q - self.PULSE_MIN_US) / (self.PULSE_MAX_US - self.PULSE_MIN_US) * 180.0
        angle_q = math.radians((deg_q - 90.0 - trim) / sign + centre)
        self._apply(name, angle_q)

    def _apply(self, name, angle):
        motor, lo, hi = self.motors[name]
        motor.setPosition(max(lo, min(hi, angle)))


# --------------------------------------------------------------------------
def smoothstep(s):
    s = max(0.0, min(1.0, s))
    return s * s * (3.0 - 2.0 * s)


def main():
    robot = Supervisor()                 # Supervisor is used ONLY for the on-screen text label
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    keyboard = robot.getKeyboard()
    keyboard.enable(dt_ms)

    driver = ServoDriver(robot)
    engine = GaitEngine()
    zero_pose = {leg: (0.0, 0.0, 0.0) for leg in LEGS}      # the robot spawns with straight legs

    print(__doc__)
    t = 0.0
    while robot.step(dt_ms) != -1:
        t += dt

        # ---- keyboard (drain the queue; all commands are latched)
        key = keyboard.getKey()
        while key != -1:
            ch = chr(key & 0xFF).upper() if 0 < (key & 0xFFFF) < 128 else ""
            if ch in GAIT_KEYS:
                engine.set_command(gait=GAIT_KEYS[ch], turn=0.0)
            elif ch == "Q":
                engine.set_command(turn=+1.0)
            elif ch == "E":
                engine.set_command(turn=-1.0)
            elif ch == "A":
                engine.set_command(turn=0.0)
            elif ch == "R":
                engine.set_command(reverse=not engine.reverse)
            key = keyboard.getKey()

        # ---- gait -> joint angles
        pose = engine.step(dt)
        k = smoothstep(t / STAND_UP_TIME)            # first seconds: rise from straight legs
        for leg in LEGS:
            for joint, a_stand, a_zero in zip(JOINTS, pose[leg], zero_pose[leg]):
                driver.write("%s_%s" % (leg.lower(), joint), a_zero + k * (a_stand - a_zero))

        # ---- HUD
        state = "STOPPING" if (engine.target == "stand" and engine.moving) else engine.target.upper()
        steer = "LEFT" if engine.turn > 0 else "RIGHT" if engine.turn < 0 else "STRAIGHT"
        robot.setLabel(0, "GAIT: %s   (%s%s)" % (state, steer, ", REVERSE" if engine.reverse else ""),
                       0.02, 0.02, 0.07, LABEL_COLOUR, 0.0, "Arial")
        robot.setLabel(1, "W walk   C crawl   T trot   S stand   Q/E turn   A straight   R reverse",
                       0.02, 0.95, 0.045, LABEL_COLOUR, 0.0, "Arial")


main()
