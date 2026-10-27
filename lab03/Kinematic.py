import serial
from svgelements import SVG, Shape, Path, Move, Line, Close, QuadraticBezier, CubicBezier  # pip install svgelements
import math
import os
import numpy as np
#11.6cm shoulder
#10cm elbow
#elbow min -50 max 130
#shoulder min 50 max 180

#ser = serial.Serial('COM5', 115200, timeout=1) #Remember to change COM number
#print( ser.readline().decode('utf-8').strip() ) #Should say 'hi.'
#while True: #Might time out as blank lines if you don't move the knobs
    #print( ser.readline().decode('utf-8').strip() )
    
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

def draw_svg(filename, x_min, y_min, x_max, y_max):
    for stroke in svg_strokes(filename, x_min, y_min, x_max, y_max):
        for i in stroke:
            print(i)
        
#11.6cm shoulder
#10cm elbow
#elbow min -50 max 130
#shoulder min 50 max 180        
def convert_angles(point):
    point = np.asarray(point)
    point = tuple(point / 20)
    a = np.arccos(0.5 * np.linalg.norm(point))
    print(np.linalg.norm(point))
    b = -2 * a
    y = a + np.arctan(point[1] / point[0])
    print(y)
    print(b)
    return (np.clip(y, 0.87, 3.1415), np.clip(y+b, -0.87, 2.268))
    
#draw_svg("drawing.svg", 0, 0, 10, 10)
print(convert_angles((10, 10)))
