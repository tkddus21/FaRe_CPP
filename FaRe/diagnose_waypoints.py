#!/usr/bin/env python3
"""Reports how much room the planned patrol actually leaves the robot.

Two things get measured, because the first one turned out not to explain the
goal failures observed in Gazebo:

  per waypoint  the clearance at the goal pose itself. `find_frontier_cells`
                only rejects cells whose square neighbourhood touches an
                obstacle within buffer_distance and never checks the footprint,
                so this is worth knowing -- but on the AWS house map all 26
                waypoints passed while 2 goals still failed.

  per segment   the tightest clearance anywhere along the route between
                consecutive waypoints. The robot wedges *on the way* to a goal,
                not at it, so this is where the risk actually lives.

When results/patrol_log.csv exists its statuses are joined in, so a run can be
read against the geometry that produced it.
"""

import csv
import os

import numpy as np

from config import config
from MAP import Map_generator
from traversability import TraversabilityGraph, clearance_grid, required_clearance


def load_waypoints(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    wp = eval(lines[0].split('=')[1].strip())
    ori = eval(lines[1].split('=')[1].strip())
    return wp, ori


def load_statuses(path, count):
    """Goal outcomes from a previous run, or None when there is no log to join."""
    if not os.path.exists(path):
        return None
    with open(path) as f:
        rows = list(csv.DictReader(f))
    if len(rows) != count:
        print(f'skipping {os.path.basename(path)}: it has {len(rows)} goals '
              f'but the current plan has {count}')
        return None
    return [r['status'] for r in rows]


def report_waypoints(wp, clearance, robot_radius):
    # DWA still has to fit the footprint plus some margin to manoeuvre, so flag a
    # band above the hard limit as "tight" rather than calling it safe.
    tight_limit = robot_radius * 2.0

    rows = []
    for i, (row, col) in enumerate(wp):
        c = float(clearance[row, col])
        if c < robot_radius:
            verdict = 'UNREACHABLE'
        elif c < tight_limit:
            verdict = 'TIGHT'
        else:
            verdict = 'OK'
        rows.append([i, row, col, round(c, 3), verdict])

    print(f'\n--- waypoint poses --- (tight below {tight_limit:.3f} m)')
    print(f'{"idx":>4} {"row":>5} {"col":>5} {"clearance_m":>12}  verdict')
    for r in rows:
        print(f'{r[0]:>4} {r[1]:>5} {r[2]:>5} {r[3]:>12}  {r[4]}')

    counts = {v: sum(1 for r in rows if r[4] == v) for v in ('UNREACHABLE', 'TIGHT', 'OK')}
    print(f'UNREACHABLE: {counts["UNREACHABLE"]}, TIGHT: {counts["TIGHT"]}, OK: {counts["OK"]}')
    return rows


def report_segments(graph, wp, statuses):
    rows = []
    for i in range(1, len(wp)):
        path, length = graph.route(wp[i - 1], wp[i])
        bottleneck = graph.bottleneck(path)
        status = statuses[i] if statuses else ''
        rows.append([i, round(bottleneck, 3) if path else '',
                     round(length, 2) if path else 'inf',
                     'BLOCKED' if not path else '', status])

    print(f'\n--- segments --- (footprint needs {required_clearance():.3f} m)')
    header = f'{"seg":>4} {"bottleneck_m":>13} {"length_m":>9}  {"note":<8}'
    print(header + ('  status' if statuses else ''))
    for r in rows:
        line = f'{r[0]:>4} {str(r[1]):>13} {str(r[2]):>9}  {r[3]:<8}'
        print(line + (f'  {r[4]}' if statuses else ''))

    tight = [r for r in rows if r[1] != '' and r[1] < required_clearance() * 1.5]
    blocked = [r for r in rows if r[3] == 'BLOCKED']
    if blocked:
        print(f'\n{len(blocked)} segment(s) have no drivable route at all: '
              f'{[r[0] for r in blocked]}')
    print(f'{len(tight)} segment(s) squeeze below {required_clearance() * 1.5:.3f} m: '
          f'{[r[0] for r in tight]}')

    if statuses:
        ok = [r[1] for r in rows if r[4] == 'SUCCEEDED' and r[1] != '']
        bad = [r[1] for r in rows if r[4] not in ('SUCCEEDED', '') and r[1] != '']
        if ok and bad:
            print(f'bottleneck of reached goals: median {np.median(ok):.3f} m, '
                  f'min {min(ok):.3f} m')
            print(f'bottleneck of failed goals:  {sorted(bad)}')
            print('Tightness is a risk factor, not a predictor: segments with the '
                  'same bottleneck as the failures were driven successfully.')
    return rows


def main():
    output_dir = config['output_dir']
    robot_radius = config['robot_radius']

    map_generator = Map_generator()
    grid_map = map_generator.load_pgm(config['pgm_filename'])
    yaml_data = map_generator.load_yaml(config['yaml_filename'])
    resolution = yaml_data['resolution']

    wp, _ = load_waypoints(os.path.join(output_dir, 'wp_ori_data.txt'))
    clearance = clearance_grid(grid_map, resolution)
    graph = TraversabilityGraph(grid_map, resolution)
    statuses = load_statuses(os.path.join(output_dir, 'patrol_log.csv'), len(wp))

    print(f'robot_radius: {robot_radius} m, footprint_padding: {config["footprint_padding"]} m, '
          f'resolution: {resolution} m/cell')

    waypoint_rows = report_waypoints(wp, clearance, robot_radius)
    segment_rows = report_segments(graph, wp, statuses)

    wp_csv = os.path.join(output_dir, 'waypoint_clearance.csv')
    with open(wp_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'grid_row', 'grid_col', 'clearance_m', 'verdict'])
        writer.writerows(waypoint_rows)

    seg_csv = os.path.join(output_dir, 'segment_clearance.csv')
    with open(seg_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['segment', 'bottleneck_m', 'length_m', 'note', 'status'])
        writer.writerows(segment_rows)

    clear_free = clearance[grid_map == config['unexplored_value']]
    print(f'\nfree-space clearance: median {np.median(clear_free):.3f} m, '
          f'max {clear_free.max():.3f} m')
    print(f'saved {wp_csv} and {seg_csv}')


if __name__ == '__main__':
    main()
