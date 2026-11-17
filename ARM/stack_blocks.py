"""
Block-stacking lab: UFACTORY 850 + Bio Gripper G2

Flow:
  1. Measure every location (horizontal gripper, jaws on the block's ENDS).
  2. Report each height and which location is empty.
  3. Split the blocks into 3 towers of equal total height (tallest block at the bottom).
  4. Stack them: grab each block horizontally by its SIDES from its end (the
     GRIP poses), lift it, swing the gripper to point down so the block hangs
     upright, magnet end down, then set it on its tower.

Layout rule: blocks lie on their sides with the MAGNET end pointing toward -y
(the location 8/9 side), i.e. away from where the gripper comes in.

Every move toward or away from a block is straight up or down, so the script
never has to know which way "backwards" is for a given wrist orientation.
Orientation changes only happen at TRAVEL_Z, well above everything.

Run with DRY_RUN = True first (no arm needed) to check the solver, then
STEP = True on the real arm so it waits for Enter before every move.
"""
import itertools
import math
import time

ARM_IP = '192.168.1.232'

# ----------------------------------------------------------------------------
# Recorded poses (from txtv2). Index = location number.
# ----------------------------------------------------------------------------
ABOVE = [  # gripper pointing down, just above each block (kept for reference)
    {'x': 244.103989, 'y': 296.435425, 'z': 29.022118, 'roll': 179.349999, 'pitch': 4.280854, 'yaw': -179.98879},
    {'x': 554.347107, 'y': 335.588013, 'z': 30.814697, 'roll': 176.279117, 'pitch': -1.44288, 'yaw': 52.932763},
    {'x': 355.88559, 'y': 181.480865, 'z': 31.436768, 'roll': -175.875984, 'pitch': 1.238907, 'yaw': 100.433314},
    {'x': 589.181396, 'y': 181.40274, 'z': 30.069012, 'roll': 177.738842, 'pitch': 0.998723, 'yaw': 23.886954},
    {'x': 352.312103, 'y': 13.442041, 'z': 25.767178, 'roll': 178.743122, 'pitch': 3.487881, 'yaw': 133.362057},
    {'x': 597.891174, 'y': 14.704798, 'z': 31.761097, 'roll': 177.67341, 'pitch': 0.986404, 'yaw': 37.674897},
    {'x': 345.024231, 'y': -125.77063, 'z': 24.047682, 'roll': 177.65519, 'pitch': 6.647227, 'yaw': 128.884851},
    {'x': 583.71405, 'y': -156.157761, 'z': 28.085361, 'roll': 175.318554, 'pitch': 0.280463, 'yaw': 33.706076},
    {'x': 269.121704, 'y': -279.997101, 'z': 27.511454, 'roll': -179.302902, 'pitch': 7.545911, 'yaw': 139.855159},
    {'x': 536.27301, 'y': -318.084076, 'z': 27.262878, 'roll': 175.630472, 'pitch': 0.917764, 'yaw': 8.035389},
]
GRIP = [  # horizontal, jaws on the block's sides (pick-up)
    {'x': 268.498383, 'y': 287.58667, 'z': -5.343417, 'roll': -3.788282, 'pitch': 88.514703, 'yaw': -95.576064},
    {'x': 533.685852, 'y': 299.291138, 'z': -5.413782, 'roll': -25.592019, 'pitch': 89.37116, 'yaw': -115.224964},
    {'x': 365.113129, 'y': 146.431122, 'z': -4.685304, 'roll': -54.454482, 'pitch': 87.978414, 'yaw': -145.234201},
    {'x': 569.272644, 'y': 155.848984, 'z': -2.905553, 'roll': -85.492236, 'pitch': 88.074385, 'yaw': -171.157161},
    {'x': 377.068359, 'y': -5.97753, 'z': -8.350518, 'roll': 91.886731, 'pitch': 89.999981, 'yaw': 0.0},
    {'x': 586.231323, 'y': -8.459813, 'z': -5.72499, 'roll': 22.207959, 'pitch': 89.081129, 'yaw': -64.476316},
    {'x': 363.081848, 'y': -160.339447, 'z': -6.900063, 'roll': 40.023566, 'pitch': 88.7919, 'yaw': -49.459837},
    {'x': 564.310181, 'y': -187.499405, 'z': -5.817453, 'roll': -23.88873, 'pitch': 87.992624, 'yaw': -112.874863},
    {'x': 282.51712, 'y': -307.488007, 'z': -6.113029, 'roll': -24.082046, 'pitch': 86.640787, 'yaw': -112.880936},
    {'x': 512.033386, 'y': -338.101807, 'z': -5.070799, 'roll': -15.788024, 'pitch': 85.689964, 'yaw': -104.936246},
]
MEASURE = [  # horizontal, jaws on the block's ends
    {'x': 255.569687, 'y': 297.574402, 'z': -5.450772, 'roll': -51.402106, 'pitch': 86.746784, 'yaw': 123.23331},
    {'x': 556.852234, 'y': 312.647583, 'z': -2.044532, 'roll': 16.698855, 'pitch': 86.685019, 'yaw': 12.343287},
    {'x': 379.869873, 'y': 155.716705, 'z': -3.440981, 'roll': -98.10573, 'pitch': 87.885996, 'yaw': -98.459932},
    {'x': 582.500549, 'y': 165.750153, 'z': -6.927601, 'roll': -107.175365, 'pitch': 87.658475, 'yaw': -106.498072},
    {'x': 388.08667, 'y': 9.309186, 'z': -5.664007, 'roll': -97.071942, 'pitch': 88.327861, 'yaw': -98.354852},
    {'x': 596.509216, 'y': -0.152637, 'z': -1.130583, 'roll': 65.544366, 'pitch': 89.559205, 'yaw': 69.006006},
    {'x': 374.344116, 'y': -152.616516, 'z': -5.04246, 'roll': -131.945133, 'pitch': 87.165158, 'yaw': -132.481307},
    {'x': 579.455139, 'y': -173.752335, 'z': -5.835606, 'roll': -39.783439, 'pitch': 87.649193, 'yaw': -38.655686},
    {'x': 297.779144, 'y': -293.968445, 'z': -3.17562, 'roll': -5.949995, 'pitch': 89.185293, 'yaw': -8.619176},
    {'x': 523.827148, 'y': -325.128418, 'z': -5.95574, 'roll': -39.850704, 'pitch': 87.092793, 'yaw': -41.178534},
]

# ============================================================================
# >>> FILL IN: TOWER BASE POSITIONS <<<
# ============================================================================
# One pose per tower: where its bottom block goes (e.g. on the bank of
# close-spaced discs). Record them with Army.py: point the gripper straight
# down over each spot (any height), press the button, and paste the printed
# dicts in below. Only x and y are used; the script sets height and orientation.
#
# Placement rules (the script warns if these are broken):
#   - at least MIN_GAP_TO_BLOCKS mm from every one of the 10 block locations
#   - at least MIN_GAP_BETWEEN_TOWERS mm from each other: the open jaws are
#     ~83 mm wide, so the gripper needs room beside each tower
STACK_SPOTS = [
    None,   # tower 0  (tallest block of each tower goes on the bottom)
    None,   # tower 1
    None,   # tower 2
]

MIN_GAP_TO_BLOCKS = 70.0
MIN_GAP_BETWEEN_TOWERS = 80.0
# ============================================================================

# ----------------------------------------------------------------------------
# Settings: check these in the lab
# ----------------------------------------------------------------------------
DRY_RUN = False       # True = no arm, uses FAKE_HEIGHTS to test the solver
STEP = True           # True = press Enter before every arm move
SPEED = 60            # mm/s for set_position
GRIPPER_SPEED = 2000

GRIP_OPEN = 150
GRIP_CLOSE = 0
GRIPPER_OFFSET = 67.0  # subtract from get_bio_gripper_g2_position() (lab says ~67: confirm with a ruler)
EMPTY_BELOW_MM = 12.0  # a closed width under this means there's no block

BLOCK_SIDE = 25.4      # footprint edge of the blocks (mm). Lying block's centre = table + BLOCK_SIDE/2
TABLE_Z = sum(p['z'] for p in MEASURE) / len(MEASURE) - BLOCK_SIDE / 2

HOVER = 60             # approach/retreat height above a measure/grip pose
LIFT = 120             # lift after gripping, before swinging upright (must exceed longest block, ~76 mm)
TRAVEL_Z = TABLE_Z + 200   # orientation changes and moves between spots happen up here

# MAGNET: blocks must lie with the magnet end pointing the way the gripper points
# in the GRIP poses (-y, i.e. toward locations 8/9). That far end is what swings
# to the bottom when the block is turned upright.
#
# Hang below the TCP once upright: blocks sit centred on the discs (MEASURE x/y is
# the block centre, since the jaws close on both ends), and the fixed GRIP pose
# sits PAST_CENTRE[loc] mm beyond that centre toward the magnet end. So the
# magnet face ends up height/2 + PAST_CENTRE[loc] + HANG_ADJUST below the TCP.
# Calibrate HANG_ADJUST once with STEP on: swing a known block upright, measure
# from its bottom face up to the TCP z, compare with the printed prediction.
HANG_ADJUST = 0.0
PLACE_GAP = 3.0        # release this many mm above the tower top (the magnet pulls it down)

FAKE_HEIGHTS = [50.8, 76.2, None, 25.4, 63.5, 38.1, 50.8, 50.8, 38.1, 63.5]  # mm, DRY_RUN only


# ----------------------------------------------------------------------------
# Rotation helpers (xArm uses R = Rz(yaw) * Ry(pitch) * Rx(roll), tool z = approach)
# ----------------------------------------------------------------------------
def rpy_to_R(roll, pitch, yaw):
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def R_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def past_centre(loc):
    """How far the GRIP TCP sits beyond the block centre, along the approach direction."""
    g, m = GRIP[loc], MEASURE[loc]
    R = rpy_to_R(g['roll'], g['pitch'], g['yaw'])
    ax, ay = R[0][2], R[1][2]
    n = math.hypot(ax, ay)
    return ((g['x'] - m['x']) * ax + (g['y'] - m['y']) * ay) / n


def hang(loc, height):
    """Distance from TCP down to the block's magnet face once it's upright."""
    return height / 2 + past_centre(loc) + HANG_ADJUST


def upright_orientation(pose):
    """Orientation that swings the gripper (and the block it's holding) from
    horizontal to pointing straight down, rotating about the jaw axis only."""
    R = rpy_to_R(pose['roll'], pose['pitch'], pose['yaw'])
    a = [R[0][2], R[1][2], R[2][2]]              # current approach direction
    t = [0.0, 0.0, -1.0]                          # want: straight down
    k = [a[1] * t[2] - a[2] * t[1], a[2] * t[0] - a[0] * t[2], a[0] * t[1] - a[1] * t[0]]
    s = math.sqrt(sum(v * v for v in k))
    c = sum(a[i] * t[i] for i in range(3))
    if s < 1e-9:
        return pose['roll'], pose['pitch'], pose['yaw']
    k = [v / s for v in k]
    K = [[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]
    K2 = matmul(K, K)
    Rrot = [[(1 if i == j else 0) + s * K[i][j] + (1 - c) * K2[i][j] for j in range(3)] for i in range(3)]
    return R_to_rpy(matmul(Rrot, R))


# ----------------------------------------------------------------------------
# Solver
# ----------------------------------------------------------------------------
def solve(heights):
    """heights: {location: mm}. Returns 3 lists of locations (bottom first) and the spread.
    Tries every assignment (3^9 = 19683), picks the smallest max-min tower
    difference, then the most even block counts."""
    locs = list(heights)
    best = None
    for assign in itertools.product(range(3), repeat=len(locs)):
        if 0 not in assign or 1 not in assign or 2 not in assign:
            continue
        sums, counts = [0.0] * 3, [0] * 3
        for loc, t in zip(locs, assign):
            sums[t] += heights[loc]
            counts[t] += 1
        key = (round(max(sums) - min(sums), 1), max(counts) - min(counts))
        if best is None or key < best[0]:
            best = (key, assign)
    towers = [[], [], []]
    for loc, t in zip(locs, best[1]):
        towers[t].append(loc)
    for tw in towers:
        tw.sort(key=lambda l: heights[l], reverse=True)   # tallest at the bottom
    return towers, best[0][0]


# ----------------------------------------------------------------------------
# Arm wrapper
# ----------------------------------------------------------------------------
class Robot:
    def __init__(self):
        from xarm.wrapper import XArmAPI
        self.arm = XArmAPI(ARM_IP)
        while self.arm.get_err_warn_code()[1][0]:
            print("Error flagged: is the emergency stop released?")
            time.sleep(0.5)
            self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)      # position control
        self.arm.set_state(0)
        time.sleep(0.5)

    def pos(self):
        return self.arm.get_position()[1]

    def move(self, why, **kw):
        if STEP:
            input("  [Enter] -> %s %s" % (why, {k: round(v, 1) for k, v in kw.items()}))
        code = self.arm.set_position(speed=SPEED, wait=True, **kw)
        if code != 0:
            raise RuntimeError("set_position failed (code %s) during: %s" % (code, why))

    def grip(self, value):
        self.arm.set_bio_gripper_g2_position(value, speed=GRIPPER_SPEED, wait=True)
        time.sleep(0.4)

    def width(self):
        readings = []
        for _ in range(3):
            readings.append(self.arm.get_bio_gripper_g2_position()[1])
            time.sleep(0.1)
        return sum(readings) / len(readings) - GRIPPER_OFFSET

    def go_over(self, pose, z):
        """Rise to TRAVEL_Z, rotate to pose's orientation, move over it, drop to z."""
        self.move("rise", z=max(self.pos()[2], TRAVEL_Z))
        self.move("travel", x=pose['x'], y=pose['y'], z=TRAVEL_Z,
                  roll=pose['roll'], pitch=pose['pitch'], yaw=pose['yaw'])
        self.move("descend to hover", z=z)

    def measure(self, loc):
        p = MEASURE[loc]
        self.grip(GRIP_OPEN)
        self.go_over(p, p['z'] + HOVER)
        self.move("lower onto block ends", z=p['z'])
        self.grip(GRIP_CLOSE)
        w = self.width()
        self.grip(GRIP_OPEN)
        self.move("back up", z=p['z'] + HOVER)
        return None if w < EMPTY_BELOW_MM else w

    def pick_and_place(self, loc, height, spot, tower_top_z, clear_z):
        p = GRIP[loc]
        self.grip(GRIP_OPEN)
        self.go_over(p, p['z'] + HOVER)
        self.move("lower to grip sides", z=p['z'])
        self.grip(GRIP_CLOSE)
        self.move("lift", z=p['z'] + LIFT)
        r, pi, y = upright_orientation(p)
        self.move("swing block upright", roll=r, pitch=pi, yaw=y)
        h = hang(loc, height)                          # magnet face = tcp_z - h
        print("    predicted: magnet face %.1f mm below TCP" % h)
        self.move("rise clear", z=clear_z + h)
        self.move("over tower", x=spot['x'], y=spot['y'])
        place_z = tower_top_z + h + PLACE_GAP
        self.move("approach tower", z=place_z + 25)
        self.move("set down", z=place_z)
        self.grip(GRIP_OPEN)
        self.move("retreat", z=clear_z + h)


# ----------------------------------------------------------------------------
# Tower spots: sanity checks
# ----------------------------------------------------------------------------
def check_stack_spots(spots):
    """Print warnings; return True if the spots look usable."""
    if any(s is None for s in spots):
        print("STACK_SPOTS isn't filled in yet: stopping after measuring.")
        return False
    ok = True
    for t, s in enumerate(spots):
        for loc in range(10):
            d = math.hypot(s['x'] - MEASURE[loc]['x'], s['y'] - MEASURE[loc]['y'])
            if d < MIN_GAP_TO_BLOCKS:
                print("WARNING: tower %d is only %.0f mm from block location %d" % (t, d, loc))
                ok = False
        for u in range(t + 1, len(spots)):
            d = math.hypot(s['x'] - spots[u]['x'], s['y'] - spots[u]['y'])
            if d < MIN_GAP_BETWEEN_TOWERS:
                print("WARNING: towers %d and %d are only %.0f mm apart" % (t, u, d))
                ok = False
    if ok:
        print("Tower spots look OK.")
    return ok


# ----------------------------------------------------------------------------
def main():

    print("Check: every block's MAGNET end points toward -y (the location 8/9 side).")
    if not DRY_RUN:
        input("[Enter] when the blocks are laid out ")
    robot = None if DRY_RUN else Robot()

    # 1. Measure
    heights = {}
    print("Measuring...")
    for loc in range(10):
        h = FAKE_HEIGHTS[loc] if DRY_RUN else robot.measure(loc)
        if h is None:
            print("  location %d: EMPTY" % loc)
        else:
            heights[loc] = h
            print("  location %d: %.1f mm (%.2f in)" % (loc, h, h / 25.4))
    empty = [l for l in range(10) if l not in heights]
    print("Empty location(s):", empty)

    # 2. Solve
    towers, spread = solve(heights)
    print("\nPlan (bottom -> top):")
    for i, tw in enumerate(towers):
        print("  tower %d: %s  = %.1f mm" % (i, ["L%d (%.1f)" % (l, heights[l]) for l in tw],
                                              sum(heights[l] for l in tw)))
    print("Spread between tallest and shortest tower: %.1f mm" % spread)
    if spread > 5:
        print("  WARNING: no near-exact split; check the measurements / GRIPPER_OFFSET.")

    print()
    spots_ok = check_stack_spots(STACK_SPOTS)
    if DRY_RUN or any(s is None for s in STACK_SPOTS):
        return
    if not spots_ok and input("Stack anyway? [y/N] ").strip().lower() != 'y':
        return

    # 3. Stack, one tower at a time, bottom up
    tops = [TABLE_Z] * 3
    for t, tw in enumerate(towers):
        for loc in tw:
            clear_z = max(tops) + 40          # tallest thing on the table + margin
            print("\nTower %d: placing L%d (%.1f mm)" % (t, loc, heights[loc]))
            robot.pick_and_place(loc, heights[loc], STACK_SPOTS[t], tops[t], clear_z)
            tops[t] += heights[loc]

    robot.move("park", z=TRAVEL_Z + 100)
    print("\nDone.")


if __name__ == '__main__':
    main()
