#!/usr/bin/env python3
"""Measures the coverage a recorded patrol actually achieved, not the one it planned.

Every coverage figure this repo reports - metrics.csv, coverage_map.png,
trash_eval.py - is computed at planning time. report_coverage() unions the FOV of
every waypoint as if the robot had reached all of them facing the right way. A run
that scores 25/26 did not: the failed goal left the robot metres away pointing
somewhere else, and nothing measured what that cost. Reporting the planned figure
beside a goal-success count invites reading the first as if it were the second.

So replay the same FOV model - Scout.cast_fov(), the same surveillance_range, the
same occupancy grid - from where the robot actually stood, read out of the run's
bag. The gap between the two numbers is what goal failures and goal tolerance cost.

    python3 FaRe/achieved_coverage.py <run_dir>

<run_dir> is a directory made by run_patrol_test.sh; it must hold patrol.bag and
patrol_log.csv. Writes achieved_coverage.csv and achieved_coverage.png beside them.

Poses come from tf (map -> odom -> base_footprint), not /amcl_pose: move_base
judges goals against the tf tree, and /amcl_pose only publishes at about 1.2 Hz,
stale by up to a metre of travel at full speed.

By default each goal contributes one FOV cast, from the pose the robot held when
that goal ended - the planner's model, with measured poses substituted for
intended ones. --along-path instead casts every `step` metres of the driven
trajectory, which answers a different question: what the sensor swept including
while driving. That number is not comparable to the planned one, so it is reported
separately rather than in place of it.
"""

import argparse
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from config import config
from MAP import Map_generator
from Scout_Multi_Processing import Scout

FREE = None  # filled from config below; named for readability in the rendering code


def robot_poses_from_bag(bag_path, base_frame='base_footprint'):
    """[(t, x, y, yaw_rad)] in the map frame, composed from /tf.

    map->odom comes from AMCL and odom->base_footprint from the robot; neither
    alone puts the robot in the map frame.
    """
    import rosbag
    from tf.transformations import quaternion_matrix

    def mat(tr):
        m = quaternion_matrix([tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
        m[0, 3], m[1, 3] = tr.translation.x, tr.translation.y
        return m

    poses, map_to_odom = [], None
    with rosbag.Bag(bag_path) as bag:
        for _, msg, t in bag.read_messages(topics=['/tf']):
            for tf_msg in msg.transforms:
                parent = tf_msg.header.frame_id.strip('/')
                child = tf_msg.child_frame_id.strip('/')
                if parent == 'map' and child == 'odom':
                    map_to_odom = mat(tf_msg.transform)
                elif parent == 'odom' and child == base_frame and map_to_odom is not None:
                    m = map_to_odom.dot(mat(tf_msg.transform))
                    poses.append((t.to_sec(), m[0, 3], m[1, 3], np.arctan2(m[1, 0], m[0, 0])))
    return poses


def goal_end_times(bag_path, rows):
    """When each row of patrol_log.csv stopped being the active goal.

    Rows are matched to /move_base/current_goal by pose, not by position in the
    list. A goal cancelled before move_base published it never appears on that
    topic, so counting messages off in order silently pairs every later row with
    the wrong window - which reads as metres of pose error that never happened.
    Matching scans forward from the last hit, so waypoints that repeat the same
    cell still line up in visiting order.

    A row with no message ran too briefly to move the robot, so it inherits the
    previous row's end - which is where the robot was standing.
    """
    import rosbag
    from tf.transformations import euler_from_quaternion

    goals = []
    with rosbag.Bag(bag_path) as bag:
        bag_end = bag.get_end_time()
        for _, msg, t in bag.read_messages(topics=['/move_base/current_goal']):
            q = msg.pose.orientation
            goals.append((t.to_sec(), msg.pose.position.x, msg.pose.position.y,
                          euler_from_quaternion([q.x, q.y, q.z, q.w])[2]))

    ends, matched, cursor = [], 0, 0
    for row in rows:
        want = (float(row['world_x']), float(row['world_y']), float(row['yaw_rad']))
        hit = None
        for j in range(cursor, len(goals)):
            _, gx, gy, gyaw = goals[j]
            dyaw = abs((gyaw - want[2] + np.pi) % (2 * np.pi) - np.pi)
            if abs(gx - want[0]) < 0.02 and abs(gy - want[1]) < 0.02 and dyaw < 0.05:
                hit = j
                break
        if hit is None:
            ends.append(ends[-1] if ends else bag_end)
        else:
            cursor = hit + 1
            matched += 1
            ends.append(goals[hit + 1][0] if hit + 1 < len(goals) else bag_end)
    return ends, matched, len(goals)


def world_to_grid(x, y, map_data, map_height):
    """Inverse of PatrolSim.grid_to_world_coords(); see the row-flip note there."""
    resolution = map_data['resolution']
    origin = map_data['origin']
    row = int(round((map_height - 1) - (y - origin[1]) / resolution))
    col = int(round((x - origin[0]) / resolution))
    return row, col


def world_yaw_to_grid_deg(yaw):
    """World yaw -> the grid-space angle cast_fov() sweeps in, in degrees.

    PatrolSim.grid_yaws_to_world() sends grid theta out as -theta because the pgm
    row axis grows downward while world y grows upward. Coming back the other way
    is the same negation.
    """
    return -np.degrees(yaw)


def cast_from(grid, scout, row, col, yaw, radius):
    """One FOV sector at a measured pose, marking seen free cells."""
    if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
        return grid
    return scout.cast_fov(grid, (row, col), radius, world_yaw_to_grid_deg(yaw))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir', help='a directory produced by run_patrol_test.sh')
    parser.add_argument('--along-path', action='store_true',
                        help='also cast along the driven trajectory, not only at goals')
    parser.add_argument('--step', type=float, default=0.25,
                        help='metres between casts for --along-path (default 0.25)')
    args = parser.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    bag_path = os.path.join(run_dir, 'patrol.bag')
    log_path = os.path.join(run_dir, 'patrol_log.csv')
    for p in (bag_path, log_path):
        if not os.path.exists(p):
            raise SystemExit(f"not a run directory: {p} is missing")

    map_generator = Map_generator()
    grid_map = map_generator.load_pgm(config['pgm_filename'])
    map_data = map_generator.load_yaml(config['yaml_filename'])
    height = grid_map.shape[0]
    radius = config['surveillance_range']
    free_value = config['unexplored_value']
    seen_value = config['explored_value']
    scout = Scout()

    rows = list(csv.DictReader(open(log_path)))
    poses = robot_poses_from_bag(bag_path)
    if not poses:
        raise SystemExit("no map->base_footprint transforms in the bag; was /tf recorded?")
    ends, matched, n_goals = goal_end_times(bag_path, rows)
    if matched != len(rows):
        print(f"note: {matched}/{len(rows)} goals matched to the {n_goals} on "
              "/move_base/current_goal; the rest were cancelled before move_base "
              "published them and reuse the preceding pose.", file=sys.stderr)

    # Achieved: one cast per goal, from the pose held when that goal ended.
    achieved = np.copy(grid_map)
    per_goal = []
    for i, row in enumerate(rows):
        t_end = ends[i]
        t, x, y, yaw = min(poses, key=lambda p: abs(p[0] - t_end))
        r, c = world_to_grid(x, y, map_data, height)
        before = int((achieved == seen_value).sum())
        achieved = cast_from(achieved, scout, r, c, yaw, radius)
        gained = int((achieved == seen_value).sum()) - before

        planned_x, planned_y = float(row['world_x']), float(row['world_y'])
        per_goal.append({
            'index': row['index'], 'status': row['status'],
            'planned_x': planned_x, 'planned_y': planned_y,
            'actual_x': round(x, 3), 'actual_y': round(y, 3),
            'pos_error_m': round(float(np.hypot(x - planned_x, y - planned_y)), 3),
            'yaw_error_deg': round(float(np.degrees(
                (yaw - float(row['yaw_rad']) + np.pi) % (2 * np.pi) - np.pi)), 1),
            'new_cells': gained,
        })

    # Planned: what Surveillance.py computed for this waypoint set, if it was kept.
    planned_grid_path = os.path.join(run_dir, 'coverage_grid.npy')
    planned = np.load(planned_grid_path) if os.path.exists(planned_grid_path) else None

    area = lambda n: n * map_data['resolution'] ** 2
    free_cells = int((grid_map == free_value).sum())
    ach_cells = int((achieved == seen_value).sum())

    print(f"run           : {run_dir}")
    print(f"goals         : {sum(1 for r in rows if r['status'] == 'SUCCEEDED')}/{len(rows)} SUCCEEDED")
    print(f"free space    : {area(free_cells):7.1f} sq.m")
    print(f"ACHIEVED      : {area(ach_cells):7.1f} sq.m  ({100.0*ach_cells/free_cells:5.1f}%)  "
          "from the poses the robot actually held")
    if planned is not None:
        pl_cells = int((planned == seen_value).sum())
        print(f"planned       : {area(pl_cells):7.1f} sq.m  ({100.0*pl_cells/free_cells:5.1f}%)  "
              "from the waypoints as generated")
        print(f"gap           : {area(pl_cells - ach_cells):7.1f} sq.m  "
              f"({100.0*(pl_cells-ach_cells)/free_cells:+5.1f} pp)")

    errs = [g['pos_error_m'] for g in per_goal]
    print(f"pose error    : median {np.median(errs):.3f} m, max {max(errs):.3f} m")

    if args.along_path:
        swept = np.copy(grid_map)
        last = None
        n = 0
        for t, x, y, yaw in poses:
            if last is not None and np.hypot(x - last[0], y - last[1]) < args.step:
                continue
            last = (x, y)
            r, c = world_to_grid(x, y, map_data, height)
            swept = cast_from(swept, scout, r, c, yaw, radius)
            n += 1
        sw_cells = int((swept == seen_value).sum())
        print(f"along path    : {area(sw_cells):7.1f} sq.m  ({100.0*sw_cells/free_cells:5.1f}%)  "
              f"from {n} poses every {args.step} m - includes what was swept while driving, "
              "so it is not comparable to the planned figure")

    csv_path = os.path.join(run_dir, 'achieved_coverage.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(per_goal[0].keys()))
        writer.writeheader()
        writer.writerows(per_goal)
    print(f"saved {csv_path}")

    # Free cells only: green both, orange planned-but-not-achieved, blue the other
    # way round, red seen by neither.
    rgb = np.zeros(grid_map.shape + (3,), dtype=np.uint8)
    rgb[grid_map == 0] = (40, 40, 40)
    rgb[grid_map == 205] = (200, 200, 200)
    free = grid_map == free_value
    ach = (achieved == seen_value) & free
    if planned is not None:
        pl = (planned == seen_value) & free
        rgb[free & pl & ach] = (0, 170, 0)
        rgb[free & pl & ~ach] = (255, 140, 0)
        rgb[free & ~pl & ach] = (0, 120, 255)
        rgb[free & ~pl & ~ach] = (220, 0, 0)
        title = (f'Achieved {100.0*ach_cells/free_cells:.1f}% vs planned '
                 f'{100.0*int((planned==seen_value).sum())/free_cells:.1f}%\n'
                 'green = both, orange = planned but not achieved, '
                 'blue = achieved only, red = neither')
    else:
        rgb[free & ach] = (0, 170, 0)
        rgb[free & ~ach] = (220, 0, 0)
        title = f'Achieved {100.0*ach_cells/free_cells:.1f}%\ngreen = seen, red = missed'

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(rgb)
    ax.set_title(title)
    png_path = os.path.join(run_dir, 'achieved_coverage.png')
    fig.savefig(png_path, dpi=200)
    plt.close(fig)
    print(f"saved {png_path}")


if __name__ == '__main__':
    sys.exit(main())
