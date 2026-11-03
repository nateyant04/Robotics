"""
Pen plotter driver: SVG -> strokes -> inverse kinematics -> servo pulses -> serial.

  python Kinematic.py monitor                 manual mode: print the rig's pulse readouts
  python Kinematic.py draw house.svg          draw an SVG (any file, no code changes)
  python Kinematic.py draw house.svg --dry    simulate only, no rig needed (needs matplotlib)
  python Kinematic.py goto 50 110             move the pen (up) to x, y in mm
  python Kinematic.py angles 90 20            move to shoulder/elbow angles, for checking calibration
  python Kinematic.py check                   test which parts of DRAW_BOX are reachable

Coordinates are mm with the origin at the shoulder pivot, +x to the right, +y up the page.
"""
import math
import sys
import time

from svgelements import SVG, Shape, Path, Move, Line, Close, QuadraticBezier, CubicBezier  # pip install svgelements
import serial  # pip install pyserial

# ---------------------------------------------------------------- rig setup

PORT = 'COM5'  # Remember to change COM number

SHOULDER_LEN = 116.0  # mm, shoulder pivot -> elbow pivot (11.6 cm)
ELBOW_LEN = 100.0     # mm, elbow pivot -> pen tip (10 cm)

# Angle limits in degrees, measured from +x, counter-clockwise positive.
SHOULDER_LIMITS = (50, 180)
ELBOW_LIMITS = (-50, 130)
# True: the elbow angle is the forearm's direction measured from +x, the same axis as the
#       shoulder (this is what your original convert_angles returned, as y + b).
# False: the elbow angle is the bend relative to the upper arm (0 = straight).
ELBOW_IS_ABSOLUTE = True

# Two (angle_deg, pulse) pairs per servo; everything in between is linear.
# ASSUMED: your angle limits sit at the firmware's pulse limits (75/444 and 75/470).
# Check with `angles` and replace these with measured pairs if they're off,
# or swap the pulses if a joint turns the wrong way.
SHOULDER_CAL = ((50, 75), (180, 444))
ELBOW_CAL = ((-50, 75), (130, 470))
PULSE_LIMITS = {'shoulder': (75, 444), 'elbow': (75, 470)}  # firmware clamps

PEN_UP = 1    # firmware: 1 -> servo 130, anything else -> 280; 0 is rejected
PEN_DOWN = 2  # swap these if the pen goes the wrong way

# Box the drawing is scaled into: x_min, y_min, x_max, y_max (mm).
# Everything in it is reachable with the limits above (run `check` after changing them).
DRAW_BOX = (-20, 55, 120, 160)
PARK = (0, 150)       # where the pen rests when finished

MM_PER_STEP = 1.0     # straight lines are split into steps this long
MOVE_DELAY = 0.02     # s per small step (servos update at 50 Hz anyway)
TRAVEL_DELAY = 0.4    # s after a pen-up jump to a new stroke
PEN_DELAY = 0.25      # s after raising/lowering the pen


# ---------------------------------------------------------------- SVG -> strokes

def bezier_points(p0, p1, p2, p3, steps=20):
    # Points along a cubic bezier curve, not including its start point p0
    points = []
    for i in range(1, steps + 1):
        t = i / steps
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points.append((x, y))
    return points


def svg_strokes(filename, x_min, y_min, x_max, y_max, mm_per_step=1):
    # Read an SVG file and return its outlines as strokes (lists of (x, y) points), scaled to fit
    # inside the box x_min..x_max, y_min..y_max (mm), keeping the aspect ratio and centred in the box.
    # Curves are split into straight moves roughly mm_per_step long.
    paths = []
    for element in SVG.parse(filename).elements():
        if isinstance(element, Shape):  # rect, circle, ellipse, line, polyline, polygon and path
            path = Path(element)
            path.reify()  # bake in any transforms (translate, rotate, scale) from the file
            path.approximate_arcs_with_cubics()
            if path.bbox() is not None:
                paths.append(path)
    if not paths:
        raise ValueError(f"No drawable shapes found in {filename}")

    boxes = [path.bbox() for path in paths]
    left, top = min(b[0] for b in boxes), min(b[1] for b in boxes)
    right, bottom = max(b[2] for b in boxes), max(b[3] for b in boxes)
    width, height = max(right - left, 1e-9), max(bottom - top, 1e-9)
    scale = min((x_max - x_min) / width, (y_max - y_min) / height)
    offset_x = x_min + ((x_max - x_min) - width * scale) / 2
    offset_y = y_min + ((y_max - y_min) - height * scale) / 2

    def to_mm(point):
        # SVG's y axis points down the page, the plotter's points up, so flip it
        return (offset_x + (point.x - left) * scale, offset_y + (bottom - point.y) * scale)

    strokes = []
    for path in paths:
        stroke = []
        for segment in path:
            if isinstance(segment, Move):  # pen lifts here: start a new stroke
                if len(stroke) > 1:
                    strokes.append(stroke)
                stroke = [to_mm(segment.end)]
                continue
            if not stroke:
                stroke = [to_mm(segment.start)]
            if isinstance(segment, (Line, Close)):  # Close is a straight line back to the stroke's start
                stroke.append(to_mm(segment.end))
            elif isinstance(segment, (QuadraticBezier, CubicBezier)):
                p0, p3 = to_mm(segment.start), to_mm(segment.end)
                if isinstance(segment, QuadraticBezier):  # turn it into the equivalent cubic curve
                    q = to_mm(segment.control)
                    p1 = (p0[0] + 2 / 3 * (q[0] - p0[0]), p0[1] + 2 / 3 * (q[1] - p0[1]))
                    p2 = (p3[0] + 2 / 3 * (q[0] - p3[0]), p3[1] + 2 / 3 * (q[1] - p3[1]))
                else:
                    p1, p2 = to_mm(segment.control1), to_mm(segment.control2)
                # the control points' path is a bit longer than the curve, so this errs on the smooth side
                length = math.dist(p0, p1) + math.dist(p1, p2) + math.dist(p2, p3)
                stroke += bezier_points(p0, p1, p2, p3, steps=max(2, math.ceil(length / mm_per_step)))
        if len(stroke) > 1:
            strokes.append(stroke)
    return strokes


# ---------------------------------------------------------------- kinematics

def wrap(angle):
    # into -180..180 degrees
    return (angle + 180) % 360 - 180


def in_limits(angle, limits):
    lo, hi = limits
    # also try the angle one full turn up, so a shoulder of 180 isn't lost as -180
    return any(lo <= a <= hi for a in (angle, angle + 360))


def convert_angles(point):
    # Inverse kinematics: (x, y) in mm -> (shoulder, elbow) in degrees, or None if unreachable.
    # There are two ways to bend the elbow for any point; try both and keep the first that
    # fits the joint limits (elbow-clockwise first, as in the original version).
    x, y = point
    c = (x * x + y * y - SHOULDER_LEN**2 - ELBOW_LEN**2) / (2 * SHOULDER_LEN * ELBOW_LEN)
    if not -1 <= c <= 1:
        return None  # too far away, or too close to the shoulder
    for bend in (-math.acos(c), math.acos(c)):  # law of cosines
        shoulder = math.atan2(y, x) - math.atan2(ELBOW_LEN * math.sin(bend),
                                                 SHOULDER_LEN + ELBOW_LEN * math.cos(bend))
        shoulder = wrap(math.degrees(shoulder))
        elbow = wrap(shoulder + math.degrees(bend)) if ELBOW_IS_ABSOLUTE else math.degrees(bend)
        if in_limits(shoulder, SHOULDER_LIMITS) and in_limits(elbow, ELBOW_LIMITS):
            if shoulder < SHOULDER_LIMITS[0]:
                shoulder += 360
            if elbow < ELBOW_LIMITS[0]:
                elbow += 360
            return shoulder, elbow
    return None


def forward(shoulder, elbow):
    # Forward kinematics: angles in degrees -> pen position in mm
    forearm = elbow if ELBOW_IS_ABSOLUTE else shoulder + elbow
    s, f = math.radians(shoulder), math.radians(forearm)
    return (SHOULDER_LEN * math.cos(s) + ELBOW_LEN * math.cos(f),
            SHOULDER_LEN * math.sin(s) + ELBOW_LEN * math.sin(f))


def angle_to_pulse(angle, cal):
    (a0, p0), (a1, p1) = cal
    return round(p0 + (angle - a0) * (p1 - p0) / (a1 - a0))


def pulse_to_angle(pulse, cal):
    (a0, p0), (a1, p1) = cal
    return a0 + (pulse - p0) * (a1 - a0) / (p1 - p0)


def to_pulses(point):
    # (x, y) mm -> (shoulder pulse, elbow pulse), or None if the rig can't get there
    angles = convert_angles(point)
    if angles is None:
        return None
    s = angle_to_pulse(angles[0], SHOULDER_CAL)
    e = angle_to_pulse(angles[1], ELBOW_CAL)
    if not (PULSE_LIMITS['shoulder'][0] <= s <= PULSE_LIMITS['shoulder'][1]
            and PULSE_LIMITS['elbow'][0] <= e <= PULSE_LIMITS['elbow'][1]):
        return None
    return s, e


# ---------------------------------------------------------------- serial

def open_rig(port=PORT):
    ser = serial.Serial(port, 115200, timeout=1)
    # Some boards reset when the port opens and greet ~2 s later; wait up to 3 s for "Hi."
    end = time.time() + 3
    while time.time() < end:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print('rig:', line)
            if line.lower().startswith('hi'):
                break
    ser.reset_input_buffer()
    return ser


def send(ser, s, e, pen):
    # Send one position and wait for the rig to answer '>' (ok) or '!' (rejected)
    # Only '\n' at the end: the firmware needs '}' to be the last byte before it.
    ser.write(f'{{{s},{e},{pen}}}\n'.encode('utf-8'))
    end = time.time() + 2.5
    while time.time() < end:
        reply = ser.readline().decode('utf-8', errors='replace').strip()
        if reply == '>':
            return True
        if reply == '!':
            print(f'\nrig rejected {{{s},{e},{pen}}}')
            return False
    raise RuntimeError('No reply from the rig. Is the toggle switch up (drawing mode)?')


# ---------------------------------------------------------------- drawing

class Plotter:
    # Keeps track of where the pen is, so it only sends what changed.
    # ser=None is a dry run: commands are only recorded in self.log.

    def __init__(self, ser=None):
        self.ser = ser
        self.pulses = None
        self.pen = None
        self.skipped = 0
        self.log = []

    def command(self, s, e, pen, delay):
        if (s, e) == self.pulses and pen == self.pen:
            return  # rounding made this step a no-op
        self.log.append((s, e, pen))
        if self.ser:
            send(self.ser, s, e, pen)
            time.sleep(delay)  # servos have no feedback, so give them time to get there
        self.pulses, self.pen = (s, e), pen

    def lift(self):
        if self.pulses and self.pen != PEN_UP:
            self.command(*self.pulses, PEN_UP, PEN_DELAY)

    def visit(self, point, down):
        pulses = to_pulses(point)
        if pulses is None:
            self.skipped += 1
            self.lift()  # never drag ink across a gap the arm can't follow
            return
        if down and self.pen != PEN_DOWN:
            self.command(*pulses, PEN_UP, TRAVEL_DELAY)  # get there with the pen up,
            self.command(*pulses, PEN_DOWN, PEN_DELAY)   # then lower it
        else:
            self.command(*pulses, PEN_DOWN if down else PEN_UP, MOVE_DELAY)

    def stroke(self, points):
        self.lift()
        self.visit(points[0], True)
        # A straight line on paper is a curve for the joints, so walk it in small steps
        for p0, p1 in zip(points, points[1:]):
            steps = max(1, math.ceil(math.dist(p0, p1) / MM_PER_STEP))
            for k in range(1, steps + 1):
                t = k / steps
                self.visit((p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t), True)

    def draw(self, strokes):
        for i, points in enumerate(strokes, 1):
            print(f'\rstroke {i}/{len(strokes)}', end='', flush=True)
            self.stroke(points)
        self.lift()
        self.visit(PARK, False)
        print(f'\n{len(self.log)} commands sent, {self.skipped} unreachable points skipped')


def draw_svg(filename, box=DRAW_BOX, dry=False):
    strokes = svg_strokes(filename, *box, mm_per_step=MM_PER_STEP)
    print(f'{filename}: {len(strokes)} strokes')
    ser = None if dry else open_rig()
    plotter = Plotter(ser)
    try:
        plotter.draw(strokes)
    except KeyboardInterrupt:
        print('\nstopped, lifting pen')
        plotter.lift()
    finally:
        if ser:
            ser.close()
    if dry:
        preview(plotter.log, filename)


def preview(log, title=''):
    # Plot what the rig would actually draw (integer pulses -> angles -> forward kinematics)
    import matplotlib.pyplot as plt  # pip install matplotlib
    prev = None
    for s, e, pen in log:
        x, y = forward(pulse_to_angle(s, SHOULDER_CAL), pulse_to_angle(e, ELBOW_CAL))
        if prev and pen == prev[2] == PEN_DOWN:
            plt.plot([prev[0], x], [prev[1], y], 'k-', lw=0.9)
        elif prev:
            plt.plot([prev[0], x], [prev[1], y], ':', color='orange', lw=0.6)
        prev = (x, y, pen)
    x0, y0, x1, y1 = DRAW_BOX
    plt.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0], 'b--', lw=0.7)
    plt.plot(0, 0, 'ro')
    plt.gca().set_aspect('equal')
    plt.title(f'simulated: {title}')
    plt.show()


# ---------------------------------------------------------------- other modes

def monitor(port=PORT):
    # Manual mode: turn the knobs and watch the pulse readouts (and where the maths thinks the pen is)
    ser = serial.Serial(port, 115200, timeout=1)
    print('Toggle down for manual mode. Ctrl+C to quit.')
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line.startswith('[') and line.endswith(']'):
                s, e, pen = (int(v) for v in line[1:-1].split('|'))
                a_s, a_e = pulse_to_angle(s, SHOULDER_CAL), pulse_to_angle(e, ELBOW_CAL)
                x, y = forward(a_s, a_e)
                print(f'{line:18} shoulder {a_s:6.1f}°  elbow {a_e:6.1f}°  ->  x {x:6.1f}  y {y:6.1f} mm')
            elif line:
                print(line)
    except KeyboardInterrupt:
        ser.close()


def check(box=DRAW_BOX, n=20):
    x0, y0, x1, y1 = box
    bad = [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * j / n)
           for i in range(n + 1) for j in range(n + 1)
           if to_pulses((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * j / n)) is None]
    print(f'{(n + 1)**2 - len(bad)}/{(n + 1)**2} grid points in {box} reachable')
    for p in bad[:10]:
        print(f'  unreachable: ({p[0]:.1f}, {p[1]:.1f})')


def main(args):
    if not args:
        print(__doc__)
    elif args[0] == 'draw':
        draw_svg(args[1], dry='--dry' in args)
    elif args[0] == 'monitor':
        monitor()
    elif args[0] == 'check':
        check()
    elif args[0] == 'goto':
        point = (float(args[1]), float(args[2]))
        pulses = to_pulses(point)
        if pulses is None:
            sys.exit(f'{point} is unreachable')
        print(f'angles {convert_angles(point)} -> pulses {pulses}')
        ser = open_rig()
        send(ser, *pulses, PEN_UP)
        ser.close()
    elif args[0] == 'angles':
        s, e = angle_to_pulse(float(args[1]), SHOULDER_CAL), angle_to_pulse(float(args[2]), ELBOW_CAL)
        print(f'pulses {s}, {e}  (the maths expects x, y = {forward(float(args[1]), float(args[2]))})')
        ser = open_rig()
        send(ser, s, e, PEN_UP)
        ser.close()
    else:
        print(__doc__)


def run_all(svg_files, dry=False):
    # Check the box, then draw each SVG in turn (pausing in between to change paper)
    check()
    for i, filename in enumerate(svg_files):
        if not dry:
            input(f'\nReady to draw {filename}? Toggle UP, uncap the pen, then press Enter...')
        draw_svg(filename, dry=dry)
    print('\nDone. Remember to cap the pen!')


if __name__ == '__main__':
    if len(sys.argv) > 1:          # command line still works the same as before
        main(sys.argv[1:])
    else:                          # pressing Run in Thonny lands here
        run_all(['house.svg', 'star.svg'], dry=True)   # change to dry=False on the rig
