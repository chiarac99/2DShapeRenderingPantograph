#!/usr/bin/env python3
"""
Normalize the three SVG-derived shape CSVs to a common workspace (~8 cm
bounding box, centered at origin) and emit C arrays for both the Arduino
sketch and the Processing sketch.

Each input CSV: 200 (x, y) rows, raw SVG units, y points DOWNWARD.
Each output: 200 (x, y) rows, meters, y points UPWARD, centered at origin.

Also verifies CCW winding (positive signed area). SVG default is CW
(because y is flipped), so after flipping y the order naturally becomes
CCW.  We double-check with the shoelace formula.
"""

import csv
from pathlib import Path

INPUT_DIR  = Path('/mnt/user-data/uploads')
SHAPES = {
    'bunny':  '1778956239955_bunny_outline_sampled.csv',
    'hammer': '1778956239955_hammer_outline_sampled.csv',
    'pear':   '1778956239955_pear_outline_sampled.csv',
}

# Target bounding box: total 8 cm wide (or tall, whichever is larger)
# so the shape fits in the ~ +/-0.04 m workspace.
TARGET_SIZE = 0.07   # m, slight margin from 8 cm

def signed_area(points):
    """Shoelace formula. Positive = CCW, negative = CW."""
    a = 0.0
    n = len(points)
    for i in range(n):
        j = (i + 1) % n
        a += points[i][0] * points[j][1] - points[j][0] * points[i][1]
    return 0.5 * a

def normalize(points):
    """Center at origin, flip y (SVG y-down -> math y-up), scale to TARGET_SIZE."""
    # Flip y first (negate y in SVG coords)
    pts = [(x, -y) for x, y in points]

    # Bounding box of flipped coords
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = (min(xs) + max(xs)) / 2
    cy = (min(ys) + max(ys)) / 2
    w  = max(xs) - min(xs)
    h  = max(ys) - min(ys)

    # Scale so the larger of w, h matches TARGET_SIZE
    scale = TARGET_SIZE / max(w, h)
    pts = [((x - cx) * scale, (y - cy) * scale) for x, y in pts]
    return pts

def emit_arduino_c(name, points):
    """C array for Arduino .ino"""
    lines = [f'  // ----- {name}, {len(points)} pts, CCW -----',
             f'  const int N_POINTS = {len(points)};',
             f'  const float shape[N_POINTS][2] = {{']
    for i, (x, y) in enumerate(points):
        sep = ',' if i < len(points) - 1 else ''
        lines.append(f'    {{ {x:8.5f}f, {y:8.5f}f}}{sep}')
    lines.append('  };')
    return '\n'.join(lines)

def emit_processing_java(name, points):
    """Java array literal for Processing .pde"""
    lines = [f'// ----- {name}, {len(points)} pts -----',
             f'int N_POINTS = {len(points)};',
             f'float[][] shape = {{']
    for i, (x, y) in enumerate(points):
        sep = ',' if i < len(points) - 1 else ''
        lines.append(f'  {{ {x:8.5f}f, {y:8.5f}f}}{sep}')
    lines.append('};')
    return '\n'.join(lines)

def main():
    for name, fname in SHAPES.items():
        path = INPUT_DIR / fname
        with open(path) as f:
            reader = csv.reader(f)
            raw = [(float(row[0]), float(row[1])) for row in reader if row]

        pts = normalize(raw)
        area = signed_area(pts)
        winding = 'CCW' if area > 0 else 'CW'

        # Verify CCW; if CW, reverse so all output is consistent.
        if area < 0:
            pts = list(reversed(pts))
            area = signed_area(pts)
            winding = 'CCW (reversed)'

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        print(f'=== {name} ===')
        print(f'  n_points: {len(pts)}')
        print(f'  area: {area:.6f} m^2 ({winding})')
        print(f'  x range: {min(xs):.4f} to {max(xs):.4f}  (width {max(xs)-min(xs):.4f})')
        print(f'  y range: {min(ys):.4f} to {max(ys):.4f}  (height {max(ys)-min(ys):.4f})')
        print()

        # Write Arduino fragment
        out_ino = INPUT_DIR.parent / 'outputs' / f'{name}_arduino.h.txt'
        out_ino.write_text(emit_arduino_c(name, pts))

        # Write Processing fragment
        out_pde = INPUT_DIR.parent / 'outputs' / f'{name}_processing.txt'
        out_pde.write_text(emit_processing_java(name, pts))

        print(f'  wrote {out_ino}')
        print(f'  wrote {out_pde}')
        print()

if __name__ == '__main__':
    main()
