#!/usr/bin/env python3
"""Scores how many placed objects the planned patrol would have seen.

Purely geometric: replays each waypoint's FOV sector and checks which object
positions fall inside it. Run Surveillance.py first to produce wp_ori_data.txt.

The coverage grid from planning marks anything within surveillance_range (5 m)
as seen, which is optimistic for recognising a small object. This applies its
own, shorter detection range on top of the same occlusion-aware ray casting.
"""

import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

from config import config
from MAP import Map_generator
from Scout_Multi_Processing import Scout


def load_waypoints(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    wp = eval(lines[0].split('=')[1].strip())
    ori = eval(lines[1].split('=')[1].strip())
    return wp, ori


def load_trash(path):
    positions = []
    with open(path, 'r') as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line:
                continue
            row, col = (int(v) for v in line.replace(',', ' ').split())
            positions.append((row, col))
    return positions


def detection_mask(grid_map, wp, ori, radius):
    """Union of every waypoint's FOV sector at its planned orientation."""
    scout = Scout()
    unexplored = config['unexplored_value']
    explored = config['explored_value']

    mask = np.zeros(grid_map.shape, dtype=bool)
    for (row, col), theta in zip(wp, ori):
        # fov() re-picks the best of 4 headings; cast_fov honours the planned one
        seen = scout.cast_fov(grid_map, (row, col), radius, np.rad2deg(theta))
        mask |= (seen == explored) & (grid_map == unexplored)
    return mask


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--range', type=float, default=config['trash_detection_range'],
                        help='detection range in metres')
    parser.add_argument('--trash', default=os.path.join(os.path.dirname(__file__), 'trash_positions.txt'))
    args = parser.parse_args()

    output_dir = config['output_dir']
    map_generator = Map_generator()
    grid_map = map_generator.load_pgm(config['pgm_filename'])
    yaml_data = map_generator.load_yaml(config['yaml_filename'])
    resolution = yaml_data['resolution']

    wp, ori = load_waypoints(os.path.join(output_dir, 'wp_ori_data.txt'))
    trash = load_trash(args.trash)

    radius_cells = int(round(args.range / resolution))
    mask = detection_mask(grid_map, wp, ori, radius_cells)

    rows = []
    for i, (row, col) in enumerate(trash):
        on_free = grid_map[row, col] == config['unexplored_value']
        detected = bool(mask[row, col])
        rows.append([i, row, col, 'yes' if detected else 'no',
                     '' if on_free else 'NOT_ON_FREE_SPACE'])

    detected_count = sum(1 for r in rows if r[3] == 'yes')
    total = len(rows)
    percent = 100.0 * detected_count / total if total else 0.0

    csv_path = os.path.join(output_dir, 'trash_eval.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['index', 'grid_row', 'grid_col', 'detected', 'note'])
        writer.writerows(rows)

    rgb = map_generator.convert_map_to_rgb(grid_map, frontiers=False)
    rgb[grid_map == config['unexplored_value']] = [235, 235, 235]  # neutral, so markers stay legible
    rgb[mask] = [255, 140, 0]
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb)
    for r in rows:
        colour = 'lime' if r[3] == 'yes' else 'red'
        ax.plot(r[2], r[1], marker='o', color=colour, markersize=6,
                markeredgecolor='black', markeredgewidth=0.5)
    ax.set_title(f'Trash detection at {args.range} m: {detected_count}/{total} ({percent:.1f}%)\n'
                 'green = detected, red = missed, orange = within detection range')
    fig.savefig(os.path.join(output_dir, 'trash_eval.png'), dpi=200)
    plt.close(fig)

    print(f'detection range: {args.range} m ({radius_cells} cells)')
    print(f'detected {detected_count}/{total} ({percent:.1f}%)')
    for r in rows:
        note = f'  [{r[4]}]' if r[4] else ''
        print(f'  #{r[0]:>2} ({r[1]}, {r[2]}): {r[3]}{note}')
    print(f'saved {csv_path}')


if __name__ == '__main__':
    main()
