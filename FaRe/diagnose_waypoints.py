#!/usr/bin/env python3
"""Measures how much room each generated waypoint actually has around it.

Reports the true clearance (distance to the nearest barrier) per waypoint, so
goal failures can be attributed to placement rather than to DWA.

Generation now enforces the same clearance it measures - see
Exploration.placeable_mask() - so this is a check on a set, not a filter for one:
it catches waypoints carried over from another map, generated before a config
change, or produced by the C++ planner, which still uses the old square test.
"""

import csv
import os

import numpy as np
from scipy.ndimage import distance_transform_edt

from config import config
from MAP import Map_generator


def load_waypoints(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    wp = eval(lines[0].split('=')[1].strip())
    ori = eval(lines[1].split('=')[1].strip())
    return wp, ori


def clearance_grid(grid_map, resolution):
    """Distance in metres from each free cell to the nearest non-navigable cell.

    Unknown space (205) counts as a barrier alongside obstacles (0): the robot
    cannot be driven through unmapped area either.
    """
    free = grid_map == config['unexplored_value']
    return distance_transform_edt(free) * resolution


def main():
    output_dir = config['output_dir']
    robot_radius = config['robot_radius']
    waypoints_path = os.path.join(output_dir, 'wp_ori_data.txt')

    map_generator = Map_generator()
    grid_map = map_generator.load_pgm(config['pgm_filename'])
    yaml_data = map_generator.load_yaml(config['yaml_filename'])
    resolution = yaml_data['resolution']

    wp, _ = load_waypoints(waypoints_path)
    clearance = clearance_grid(grid_map, resolution)

    # DWA still has to fit the footprint plus some margin to manoeuvre, so flag a
    # band above the hard limit as "tight" rather than calling it safe. This is the
    # same threshold Scout_Multi_Processing.Exploration.placeable_mask() generates
    # against, so a freshly generated set should report no TIGHT at all; any that
    # appear came from a map or a config the waypoints were not generated for.
    tight_limit = config['waypoint_clearance']

    rows = []
    for i, (row, col) in enumerate(wp):
        c = clearance[row, col]
        if c < robot_radius:
            verdict = 'UNREACHABLE'
        elif c < tight_limit:
            verdict = 'TIGHT'
        else:
            verdict = 'OK'
        rows.append([i, row, col, round(float(c), 3), verdict])

    csv_path = os.path.join(output_dir, 'waypoint_clearance.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'grid_row', 'grid_col', 'clearance_m', 'verdict'])
        writer.writerows(rows)

    counts = {v: sum(1 for r in rows if r[4] == v) for v in ('UNREACHABLE', 'TIGHT', 'OK')}
    print(f'robot_radius: {robot_radius} m, tight below: {tight_limit} m, resolution: {resolution} m/cell')
    print(f'{"idx":>4} {"row":>5} {"col":>5} {"clearance_m":>12}  verdict')
    for r in rows:
        print(f'{r[0]:>4} {r[1]:>5} {r[2]:>5} {r[3]:>12}  {r[4]}')
    print(f'\nUNREACHABLE: {counts["UNREACHABLE"]}, TIGHT: {counts["TIGHT"]}, OK: {counts["OK"]}')
    print(f'saved {csv_path}')

    clear_free = clearance[grid_map == config['unexplored_value']]
    print(f'free-space clearance: median {np.median(clear_free):.3f} m, max {clear_free.max():.3f} m')


if __name__ == '__main__':
    main()
