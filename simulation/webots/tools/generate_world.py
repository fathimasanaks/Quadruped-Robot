"""
Regenerates worlds/gait_arena.wbt and worlds/textures/arena_floor.png.
Edit the dimensions below, run `python3 tools/generate_world.py`, reload the world in Webots.
"""
import math, os
from PIL import Image, ImageDraw

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# ------------------------------------------------------------- robot dimensions [m]
BODY = (0.186, 0.078, 0.050)       # body box (x, y, z)
SHOULDER_X, SHOULDER_Y = 0.093, 0.039
L1, L2, L3, FOOT_R = 0.055, 0.1075, 0.130, 0.012
MASS = dict(body=1.10, shoulder=0.06, thigh=0.09, shank=0.05)     # kg  (~1.9 kg total)

# Hobby servo (MG995 / MG996R class, 6 V): ~1.2 N.m stall, ~6 rad/s
MAX_TORQUE, MAX_VEL, PID_P = 1.2, 6.0, 30
LIMITS = dict(hip_abd=(-0.7, 0.7), hip_pitch=(-1.6, 1.6), knee=(-2.6, 0.2))

ARENA = 16.0           # arena size [m] (square, keep it an even number of metres)
WALL_H, WALL_T = 0.12, 0.06
YELLOW, BLACK, GREY = "0.98 0.76 0.08", "0.08 0.08 0.08", "0.75 0.77 0.80"


def fmt(v):
    return ("%.5f" % v).rstrip("0").rstrip(".") if v else "0"


def shape(geom, colour, rough=0.6):
    return ("Shape { appearance PBRAppearance { baseColor %s roughness %s metalness 0 } geometry %s }"
            % (colour, rough, geom))


def box(x, y, z):
    return "Box { size %s %s %s }" % (fmt(x), fmt(y), fmt(z))


def at(shape_txt, tx=0, ty=0, tz=0):
    return "Transform { translation %s %s %s children [ %s ] }" % (fmt(tx), fmt(ty), fmt(tz), shape_txt)


def motor(name, joint):
    lo, hi = LIMITS[joint]
    return ("RotationalMotor { name \"%s\" minPosition %s maxPosition %s maxTorque %s maxVelocity %s "
            "controlPID %s 0 0 }" % (name, lo, hi, MAX_TORQUE, MAX_VEL, PID_P))


def leg(prefix, sx, sy):
    side = 1 if sy > 0 else -1
    ax, ay = sx * SHOULDER_X, sy * SHOULDER_Y
    off = side * L1
    shank_geom = [at(shape(box(0.016, 0.014, L3), BLACK), 0, 0, -L3 / 2),
                  at(shape("Sphere { radius %s subdivision 2 }" % FOOT_R, GREY), 0, 0, -L3)]
    shank_bound = ("Group { children [ %s %s ] }" % (
        at("Box { size 0.016 0.014 %s }" % fmt(L3), 0, 0, -L3 / 2),
        at("Sphere { radius %s }" % FOOT_R, 0, 0, -L3)))
    return f"""
    HingeJoint {{
      jointParameters HingeJointParameters {{ axis 1 0 0 anchor {fmt(ax)} {fmt(ay)} 0 }}
      device [ {motor(prefix + "_hip_abd", "hip_abd")} ]
      endPoint Solid {{
        translation {fmt(ax)} {fmt(ay)} 0
        name "{prefix}_shoulder"
        children [
          {at(shape(box(0.034, abs(off) + 0.034, 0.034), YELLOW), 0, off / 2, 0)}
          HingeJoint {{
            jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 {fmt(off)} 0 }}
            device [ {motor(prefix + "_hip_pitch", "hip_pitch")} ]
            endPoint Solid {{
              translation 0 {fmt(off)} 0
              name "{prefix}_thigh"
              children [
                {at(shape(box(0.022, 0.020, L2 + 0.02), YELLOW), 0, 0, -L2 / 2)}
                HingeJoint {{
                  jointParameters HingeJointParameters {{ axis 0 1 0 anchor 0 0 {fmt(-L2)} }}
                  device [ {motor(prefix + "_knee", "knee")} ]
                  endPoint Solid {{
                    translation 0 0 {fmt(-L2)}
                    name "{prefix}_shank"
                    contactMaterial "rubber"
                    children [ {" ".join(shank_geom)} ]
                    boundingObject {shank_bound}
                    physics Physics {{ density -1 mass {MASS["shank"]} }}
                  }}
                }}
              ]
              boundingObject {at("Box { size 0.022 0.020 %s }" % fmt(L2 + 0.02), 0, 0, -L2 / 2)}
              physics Physics {{ density -1 mass {MASS["thigh"]} }}
            }}
          }}
        ]
        boundingObject {at("Box { size 0.034 %s 0.034 }" % fmt(abs(off) + 0.034), 0, off / 2, 0)}
        physics Physics {{ density -1 mass {MASS["shoulder"]} }}
      }}
    }}"""


def robot():
    bx, by, bz = BODY
    legs = "".join(leg(n, sx, sy) for n, sx, sy in
                   (("fl", 1, 1), ("fr", 1, -1), ("rl", -1, 1), ("rr", -1, -1)))
    spawn_z = L2 + L3 + FOOT_R + 0.004
    return f"""Robot {{
  translation 0 0 {fmt(spawn_z)}
  name "spotmicro"
  controller "spotmicro_gaits"
  supervisor TRUE
  children [
    {at(shape(box(bx, by, bz), YELLOW))}
    {at(shape(box(0.10, 0.052, 0.012), BLACK), -0.01, 0, bz / 2 + 0.006)}
    {at(shape(box(0.012, 0.060, 0.030), BLACK), bx / 2 + 0.002, 0, 0.004)}
    {legs}
  ]
  boundingObject {box(bx, by, bz)}
  physics Physics {{ density -1 mass {MASS["body"]} }}
}}"""


def arena():
    half = ARENA / 2
    walls = []
    for n, (x, y, sx, sy) in dict(N=(0, half + WALL_T / 2, ARENA + 2 * WALL_T, WALL_T),
                                  S=(0, -half - WALL_T / 2, ARENA + 2 * WALL_T, WALL_T),
                                  E=(half + WALL_T / 2, 0, WALL_T, ARENA),
                                  W=(-half - WALL_T / 2, 0, WALL_T, ARENA)).items():
        walls.append(f"""Solid {{
  translation {fmt(x)} {fmt(y)} {fmt(WALL_H / 2)}
  name "wall_{n}"
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.45 0.28 0.16 roughness 0.8 metalness 0 }} geometry Box {{ size {fmt(sx)} {fmt(sy)} {WALL_H} }} }} ]
  boundingObject Box {{ size {fmt(sx)} {fmt(sy)} {WALL_H} }}
}}""")
    floor = f"""Solid {{
  translation 0 0 -0.05
  name "arena_floor"
  contactMaterial "arena"
  children [ Shape {{
    appearance PBRAppearance {{ baseColorMap ImageTexture {{ url [ "textures/arena_floor.png" ] }} textureTransform TextureTransform {{ scale {int(ARENA)} {int(ARENA)} }} roughness 0.9 metalness 0 }}
    geometry Box {{ size {ARENA} {ARENA} 0.1 }}
  }} ]
  boundingObject Box {{ size {ARENA} {ARENA} 0.1 }}
}}"""
    ring = """Solid {
  translation 0 0 0.0005
  name "start_marker"
  children [ Shape { appearance PBRAppearance { baseColor 0.8 0.15 0.15 roughness 0.9 metalness 0 } geometry Cylinder { height 0.001 radius 0.16 subdivision 32 } } ]
}"""
    return floor + "\n" + ring + "\n" + "\n".join(walls)


def camera_orientation(pos, target):
    """axis-angle for a Webots viewpoint looking from pos to target, z up (camera looks along local -z)."""
    f = [t - p for p, t in zip(pos, target)]
    n = math.sqrt(sum(v * v for v in f)); f = [v / n for v in f]
    r = [f[1] * 1 - f[2] * 0, f[2] * 0 - f[0] * 1, 0.0]          # f x up(0,0,1)
    n = math.hypot(r[0], r[1]); r = [v / n for v in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    m = [[r[0], u[0], -f[0]], [r[1], u[1], -f[1]], [r[2], u[2], -f[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    ang = math.acos(max(-1, min(1, (tr - 1) / 2)))
    ax = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    n = math.sqrt(sum(v * v for v in ax))
    return [v / n for v in ax] + [ang]


def make_texture(path, px=512):
    """One 1 m x 1 m tile (4 x 4 squares of 25 cm, bold line on two edges); the world repeats it ARENA x ARENA times."""
    img = Image.new("RGB", (px, px))
    d = ImageDraw.Draw(img)
    sq = px // 4
    for i in range(4):
        for j in range(4):
            c = (176, 184, 196) if (i + j) % 2 == 0 else (150, 160, 175)
            d.rectangle([i * sq, j * sq, (i + 1) * sq - 1, (j + 1) * sq - 1], fill=c)
    d.rectangle([0, 0, px - 1, 3], fill=(60, 70, 90))
    d.rectangle([0, 0, 3, px - 1], fill=(60, 70, 90))
    img.save(path)


def main():
    pos = (-0.95, -0.85, 0.62)
    ori = camera_orientation(pos, (0.15, 0.0, 0.08))
    world = f"""#VRML_SIM R2023b utf8

WorldInfo {{
  title "Spot Micro Gait Arena"
  info [ "12-DoF servo-driven quadruped, no sensors. Keys: W walk, C crawl, T trot, S stand, Q/E turn, R reverse" ]
  basicTimeStep 8
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {{ material1 "rubber" material2 "arena" coulombFriction [ 1.2 ] bounce 0 }}
  ]
}}
Viewpoint {{
  orientation {" ".join(fmt(v) for v in ori)}
  position {" ".join(fmt(v) for v in pos)}
  follow "spotmicro"
  followType "Tracking Shot"
  followSmoothness 0.3
}}
Background {{ skyColor [ 0.67 0.80 0.95 ] }}
DirectionalLight {{ direction -0.4 -0.3 -1 intensity 1.4 ambientIntensity 0.5 castShadows TRUE }}
{arena()}
{robot()}
"""
    os.makedirs(os.path.join(ROOT, "worlds", "textures"), exist_ok=True)
    open(os.path.join(ROOT, "worlds", "gait_arena.wbt"), "w").write(world)
    make_texture(os.path.join(ROOT, "worlds", "textures", "arena_floor.png"))
    print("wrote worlds/gait_arena.wbt and worlds/textures/arena_floor.png")


if __name__ == "__main__":
    main()
