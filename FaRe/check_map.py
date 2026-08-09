#!/usr/bin/env python3
"""Checks that the selected map is something FaRe can actually plan on.

Run this after building a new map and before Surveillance.py. FaRe identifies
free space by *exact equality* with unexplored_value (254) - see
Scout_Multi_Processing.cast_fov() and find_frontier_cells() - so a map that
map_server renders perfectly can still be half-invisible to the planner. An
anti-aliased map exported from an image editor, for instance, carries cells at
255 and a spread of greys; those are free to move_base and non-free to FaRe,
and the only symptom is a coverage number that is quietly computed over half
the house.

    python3 FaRe/check_map.py                 # AWS house (default)
    FARE_MAP=house python3 FaRe/check_map.py  # turtlebot3_house

Exits non-zero if the map is unusable, so it can gate a run.
"""

import sys

import numpy as np
from scipy.ndimage import distance_transform_edt

from config import config, MAP_NAME
from MAP import Map_generator

# 0 = occupied, 205 = unknown, 254 = free. What map_saver writes, and the only
# three values the planner's equality checks recognise.
TRINARY = {0, 205, 254}


def main():
    free_value = config['unexplored_value']
    map_generator = Map_generator()
    grid_map = map_generator.load_pgm(config['pgm_filename'])
    yaml_data = map_generator.load_yaml(config['yaml_filename'])
    resolution = yaml_data['resolution']
    origin = yaml_data['origin']
    height, width = grid_map.shape

    print(f"map        : {MAP_NAME}")
    print(f"pgm        : {config['pgm_filename']}")
    print(f"size       : {width} x {height} cells @ {resolution} m = "
          f"{width * resolution:.2f} x {height * resolution:.2f} m")
    print(f"origin     : {origin}")

    ok = True

    # --- 1. value histogram -------------------------------------------------
    values, counts = np.unique(grid_map, return_counts=True)
    stray = {int(v): int(c) for v, c in zip(values, counts) if int(v) not in TRINARY}
    print("\nvalue histogram (cells):")
    for v, c in zip(values, counts):
        label = {0: 'occupied', 205: 'unknown', 254: 'free'}.get(int(v), 'NOT TRINARY')
        print(f"  {int(v):>3} : {int(c):>7}  {label}")

    if stray:
        stray_cells = sum(stray.values())
        print(f"\nFAIL: {stray_cells} cells ({100.0 * stray_cells / grid_map.size:.1f}%) "
              f"are outside {sorted(TRINARY)}.")
        print("      FaRe matches free space with '== 254' exactly, so these are invisible")
        print("      to the planner even though map_server renders them fine.")
        print("      Re-save with 'rosrun map_server map_saver' rather than an image editor.")
        ok = False

    # --- 2. free area -------------------------------------------------------
    free_area = map_generator.estimate_area(grid_map, yaml_data, free_value)
    free = grid_map == free_value
    print(f"\nfree space : {free_area:.1f} sq.m ({free.sum()} cells)")
    if free.sum() == 0:
        print("FAIL: no free space at all - wrong value convention or an empty map.")
        return 1

    # --- 3. start cell ------------------------------------------------------
    # Inverse of PatrolSim.grid_to_world_coords(), which is the authority: pgm
    # row 0 is the TOP of the image but the origin is the BOTTOM-LEFT pixel.
    row, col = config['starting_position'][0]
    print(f"\nstart cell : (row={row}, col={col})")
    if not (0 <= row < height and 0 <= col < width):
        print(f"FAIL: start cell is outside the {height} x {width} map.")
        ok = False
    else:
        x = col * resolution + origin[0]
        y = (height - 1 - row) * resolution + origin[1]
        value = int(grid_map[row, col])
        print(f"           -> world ({x:.3f}, {y:.3f}) m, cell value {value}")
        print("           spawn the robot here: "
              f"house_sim.launch x_pos:={x:.2f} y_pos:={y:.2f}")
        if value != free_value:
            print(f"FAIL: start cell is not free space ({value} != {free_value}); "
                  "the first goal would be unreachable.")
            ok = False

    # --- 4. clearance -------------------------------------------------------
    # Same measure diagnose_waypoints.py uses: unknown space counts as a barrier
    # alongside obstacles, since the robot cannot be driven through it either.
    clearance = distance_transform_edt(free) * resolution
    clear_free = clearance[free]
    radius = config['robot_radius']
    fits = float((clear_free >= radius).mean()) * 100.0
    print(f"\nclearance  : median {np.median(clear_free):.3f} m, "
          f"max {clear_free.max():.3f} m")
    print(f"           {fits:.1f}% of free space clears the {radius} m footprint half-width")

    print("\n" + ("PASS: map is usable." if ok else "FAIL: fix the above before planning."))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
