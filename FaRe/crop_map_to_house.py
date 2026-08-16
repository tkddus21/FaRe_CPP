#!/usr/bin/env python3
"""Marks everything outside the house as unmapped, in a saved map_server map.

The turtlebot3_house has doorways that open onto the garden - the east wall is
split between y = -0.40 and y = 0.50, and the south wall has a gap around
x = 5.0..5.8. The laser sees straight through them, so gmapping records the
garden as free space and the map bleeds outdoors with nothing to bound it.

That is not a mapping error, it is what the sensor saw. It is still wrong for
FaRe, which would place waypoints out there and then report coverage over a
patrol area that includes the lawn.

So bound the map by the thing that actually defines the patrol area: the rooms in
model.sdf. Cells outside them become 205, the same unknown value the rest of the
map uses for unseen space, which FaRe already treats as a barrier.

This is not the image-editor touch-up the README warns about. That warning is
about values: an editor exports 255s and anti-aliased greys, and FaRe matches free
space by exact equality with 254, so a map that looks right becomes half-invisible
to the planner. Here nothing is resampled, no cell changes to a value the map did
not already use, and the origin is untouched, so the map still lines up with
Gazebo world coordinates.

    python3 FaRe/crop_map_to_house.py maps/turtlebot3_house/map.pgm

Writes in place after keeping a .orig copy, and prints the before/after areas.
"""

import argparse
import os
import shutil
import sys

import numpy as np

# Same room boxes as tour_house_for_mapping.py, each widened per side before the
# crop - a room clipped exactly at its boundary loses the wall that bounds it, and
# an unbounded free edge is a frontier all over again.
#
# 0.60 m is the default and it is deliberately generous: on nine of the twelve
# sides there is nothing but unknown space behind the boundary, so the widening
# costs nothing and no smaller value changes the result by a single cell.
#
# The three exceptions are the sides that face the garden across a wall the box
# nearly touches, where 0.60 m clears the wall and lands back out on the lawn. The
# right wing is boxed at x = 4.83, x = 7.33 and y = -5.23, all within 0.2 m of the
# box, and the worst of it was 2.5 m of grass east of the wing where the laser
# looks straight out through the south doorway. Its east side steps back inside the
# box, which already sat 7 cm beyond the wall. The main body's south wall is the
# other one: its outer face runs at y = -0.225, so 0.60 m of margin held on to
# 2.3 sq.m of lawn below it, and pulling in to 0.20 m drops the free cells that sit
# against unknown from 257 to 157 - the crop line stops being a ragged edge in the
# grass and becomes the wall itself.
#
# Sides that face another region keep the default. The right wing's north side is
# the one that matters: its box has to keep overlapping the main body's or the wing
# is sealed off from the house.
REGIONS = [
    # (x0, y0, x1, y1), then metres to widen each side by (west, south, east, north)
    ((-7.4, -3.80, -5.25, 0.85), (0.60, 0.60, 0.60, 0.60)),   # left wing
    ((-7.4, -0.05, 7.4, 5.15), (0.60, 0.20, 0.60, 0.60)),     # main body
    ((5.00, -5.15, 7.4, -0.30), (0.25, 0.10, -0.05, 0.60)),   # right wing
]

FREE, UNKNOWN, OCCUPIED = 254, 205, 0


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise SystemExit(f"{path} is not a binary PGM")
    # header: P5 <width> <height> <maxval>, with # comments anywhere between
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        start = i
        while i < len(data) and not data[i:i + 1].isspace():
            i += 1
        fields.append(int(data[start:i]))
    i += 1  # single whitespace after maxval
    width, height, _ = fields
    grid = np.frombuffer(data[i:i + width * height], dtype=np.uint8)
    return grid.reshape(height, width).copy(), data[:i]


def load_yaml(path):
    import yaml
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('pgm', help='path to map.pgm (map.yaml must sit beside it)')
    args = parser.parse_args()

    pgm = os.path.abspath(args.pgm)
    yml = os.path.splitext(pgm)[0] + '.yaml'
    if not os.path.exists(yml):
        raise SystemExit(f"no yaml beside the pgm: {yml}")

    meta = load_yaml(yml)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    # Every run crops what gmapping recorded, not what the last run left behind.
    # A cell turned to 205 cannot be given back, so a second pass over its own
    # output could only ever remove more - widen a margin and nothing would happen.
    # The first run puts the original aside, every later one reads from it.
    orig = pgm + '.orig'
    if not os.path.exists(orig):
        shutil.copy2(pgm, orig)
    grid, header = read_pgm(orig)
    height, width = grid.shape

    # pgm row 0 is the TOP of the image, while the origin refers to the
    # BOTTOM-LEFT pixel - the same flip PatrolSim has to undo for goals.
    cols = np.arange(width)
    rows = np.arange(height)
    xs = ox + (cols + 0.5) * res
    ys = oy + (height - 1 - rows + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)

    inside = np.zeros_like(grid, dtype=bool)
    for (x0, y0, x1, y1), (west, south, east, north) in REGIONS:
        inside |= ((gx >= x0 - west) & (gx <= x1 + east) &
                   (gy >= y0 - south) & (gy <= y1 + north))

    before_free = int((grid == FREE).sum())
    outside_free = int(((grid == FREE) & ~inside).sum())
    grid[~inside] = UNKNOWN
    after_free = int((grid == FREE).sum())

    with open(pgm, 'wb') as f:
        f.write(header)
        f.write(grid.tobytes())

    area = lambda n: n * res * res
    print(f"free before : {area(before_free):7.1f} sq.m ({before_free} cells)")
    print(f"outside     : {area(outside_free):7.1f} sq.m removed")
    print(f"free after  : {area(after_free):7.1f} sq.m ({after_free} cells)")
    print(f"values now  : {sorted(np.unique(grid).tolist())}")
    print(f"original kept at {orig}")


if __name__ == '__main__':
    sys.exit(main())
