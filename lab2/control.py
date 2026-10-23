#Sample movement to ensure communication with the 3D printer
import math
import os
import serial
import time
from svgelements import SVG, Shape, Path, Move, Line, Close, QuadraticBezier, CubicBezier  # pip install svgelements

# maybe adjust these
pen_up_z = 10
pen_down_z = 0

def send(ser, command, timeout=30):
    # Send one G-code line and wait for the printer's "ok" before returning.
    # "ok" means the command was accepted into the printer's queue (not that the move finished),
    # so moves sent back to back blend into one smooth motion.
    ser.write((command + '\n').encode())
    deadline = time.time() + timeout
    while True:
        reply = ser.readline().decode(errors='ignore').strip()
        if reply.startswith('ok'):
            return
        if reply:
            if reply.startswith('Error') or 'Unknown command' in reply:
                print(f"Printer: {reply}  (after '{command}')")
            deadline = time.time() + timeout  # any reply (e.g. "busy: processing" while homing) means it's alive
        elif time.time() > deadline:
            raise TimeoutError(f"No 'ok' from printer after '{command}'")

def wait_for_startup(ser):
    # Opening the port usually resets the printer, which then prints a startup banner.
    # Read until it goes quiet so those lines aren't mistaken for replies to our commands.
    while ser.readline():
        pass

def finish_moves(ser):
    send(ser, 'M400', timeout=300)  # M400's "ok" only comes once every queued move has finished

def home_printer(ser):
    send(ser, 'G28', timeout=120)  # homing can take a while

def coord_mode_absolute(ser, mode=True):
    if mode:
        send(ser, 'G90')  # Set to absolute positioning
    else:
        send(ser, 'G91')  # Set to relative positioning

def move_to(ser, x=None, y=None, z=None, feedrate=1500):
    command = 'G1'
    if x is not None:
        command += f' X{x}'
    if y is not None:
        command += f' Y{y}'
    if z is not None:
        command += f' Z{z}'
    command += f' F{feedrate}'
    send(ser, command)

def pen_up(ser):
    move_to(ser, z=pen_up_z)  # Move the pen up (adjust Z value as needed)

def pen_down(ser):
    move_to(ser, z=pen_down_z)  # Move the pen down (adjust Z value as needed)

def draw_polygon(ser, points):
    pen_up(ser)
    move_to(ser, x=points[0][0], y=points[0][1])
    pen_down(ser)
    for point in points[1:]:
        move_to(ser, x=point[0], y=point[1])
    pen_up(ser)

def bezier_points(p0, p1, p2, p3, steps=20):
    # Points along a cubic bezier curve, not including its start point p0
    points = []
    for i in range(1, steps + 1):
        t = i / steps
        x = (1 - t)**3 * p0[0] + 3 * (1 - t)**2 * t * p1[0] + 3 * (1 - t) * t**2 * p2[0] + t**3 * p3[0]
        y = (1 - t)**3 * p0[1] + 3 * (1 - t)**2 * t * p1[1] + 3 * (1 - t) * t**2 * p2[1] + t**3 * p3[1]
        points.append((x, y))
    return points

def draw_bezier(ser, p0, p1, p2, p3, steps=20):
    draw_polygon(ser, [p0] + bezier_points(p0, p1, p2, p3, steps))

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

def draw_svg(ser, filename, x_min, y_min, x_max, y_max):
    for stroke in svg_strokes(filename, x_min, y_min, x_max, y_max):
        draw_polygon(ser, stroke)

def main():
    port = 'COM3' #Set this to your actual COM port!
    baudrate = 115200  # Common baudrate for 3D printers, adjust if needed

    try:
        # Open the serial connection
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to 3D printer on {port}")

        # Allow the printer to initialize
        wait_for_startup(ser)

        coord_mode_absolute(ser)
        home_printer(ser)
        coord_mode_absolute(ser)

        pen_up(ser)
        move_to(ser, x=50, y=50)

        # draw square
        draw_polygon(ser, [(50, 50), (100, 50), (100, 100), (50, 100), (50, 50)])


        move_to(ser, x=100, y=150)

        # draw bezier example
        draw_bezier(ser, (100, 125), (125, 150), (150, 100), (200, 125))

        # draw an SVG file, fitted inside the box X10..90, Y120..200
        draw_svg(ser, os.path.join(os.path.dirname(__file__), 'examples', 'test.svg'), 10, 120, 90, 200)


        # wait for the printer to actually finish moving before closing the port
        finish_moves(ser)
        print("Commands sent successfully.")

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        # Close the connection
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Connection closed.")

if __name__ == "__main__":
    main()
