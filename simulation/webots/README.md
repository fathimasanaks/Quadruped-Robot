# Spot Micro Gait Arena (Webots)

12-DoF Spot-Micro style quadruped standing in a flat test arena. Press a key and it
stands, walks, crawls or trots. **The robot has no sensors** - only 12 servos driven
through a PCA9685-style servo-driver layer, so all gaits are open-loop.

## Run
1. Webots **R2023b or newer** (older versions: change the first line of the `.wbt` to your version).
2. `File > Open World...` -> `worlds/gait_arena.wbt`. Python 3 must be set in Webots *Preferences > Python command*.
3. Click inside the 3D view (keyboard focus), then:

| Key | Action |
|---|---|
| W | Walk (careful 4-beat walk, ~0.05 m/s, 80 % duty) |
| C | Crawl (lower body, slow but steady, ~0.04 m/s, full weight shift, 85 % duty) |
| T | Trot (diagonal pairs together, 60 % duty, ~0.15 m/s) |
| S | Stand / stop |
| Q / E | Turn left / right (latched) |
| A | Go straight again |
| R | Toggle reverse |

Switching gait first winds the stride down to a stand, changes body height, then winds up the new gait.

## Files
- `worlds/gait_arena.wbt` - 16 x 16 m arena (checker squares = 25 cm, bold lines = 1 m, red ring = start), low wooden rim, pale-blue sky, tracking camera. Robot is defined inline.
- `controllers/spotmicro_gaits/gaits.py` - IK/FK and gait generator (no Webots import; `python3 gaits.py` runs a self-test).
- `controllers/spotmicro_gaits/spotmicro_gaits.py` - Webots controller: keyboard, servo-driver layer, on-screen label.
- `tools/generate_world.py` - regenerates the world + floor texture (edit dimensions, masses, servo torque/speed at the top).

## Robot model
Body 186 x 78 mm, hip offset 55 mm, thigh 107.5 mm, shank 130 mm, ~1.9 kg. Joints per leg: `*_hip_abd` (x axis),
`*_hip_pitch` (y), `*_knee` (y); motors named `fl_/fr_/rl_/rr_` + joint. Servos modelled as MG995/MG996R class:
1.2 N.m, 6 rad/s. **No PositionSensor, IMU, camera or other devices are in the model.** (The robot is flagged
`supervisor` only so the controller can draw the on-screen text; it does not read any state back.)

## Servo driver layer
`ServoDriver` converts each joint angle to servo degrees (90 deg = horn centre, 0-180 travel), to a 500-2500 us pulse,
quantises it to the PCA9685's 12-bit / 50 Hz tick, then sends it to the Webots motor. Channels are 0-11
(FL 0-2, FR 3-5, RL 6-8, RR 9-11 as abd/pitch/knee). Edit `CHANNEL`, `SIGN`, `TRIM_DEG`, `CENTRE_DEG` to match your build.
For real hardware keep `gaits.py` and replace `ServoDriver._apply()` with I2C writes (e.g. Adafruit ServoKit).

## Tuning
All gait parameters live in the `GAITS` table in `gaits.py` (body height, cycle time, duty, stride, foot lift, weight shift).
If the robot tips: lower `stride`, raise `period`, or increase `sway` for walk/crawl.

## How the crawl/walk stay upright
While one leg is in the air the body is shifted toward the centre of the other three feet (forward/back *and* sideways),
and the shift is finished *before* the foot lifts. `python3 tools/check_gaits.py` reports the stability margin using the
full-body centre of mass (legs included), peak servo speed and static servo torque for every gait.

## Verification status
Checked offline with `tools/check_gaits.py`: all joints inside motor limits and the servo 0-180 deg range; peak joint speed
below the 6 rad/s servo limit (crawl 4.2, walk 4.9, trot 4.4); centre-of-mass margin inside the support polygon of
33 mm (crawl) and 24 mm (walk); peak static servo torque 0.83 N.m of 1.2. The controller also runs against a mock Webots API.
**It has not been run inside Webots itself.** Trot is a dynamic gait (only two feet on the ground part of the time) with no
balance feedback, so it is the most likely to need tuning (`GAITS["trot"]` in `gaits.py`: lower `stride`, raise `period` or `duty`).
