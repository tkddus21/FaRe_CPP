import os
import re

# Configuration parameters for FireBot tool


# --- map selection ---------------------------------------------------------
# Everything below the presets is shared; only the map, where the patrol starts
# and where results land differ between environments. Pick one with FARE_MAP:
#
#   python3 FaRe/Surveillance.py                  # AWS house (default)
#   FARE_MAP=house python3 FaRe/Surveillance.py   # turtlebot3_house
#
# The default stays "aws" so every command, path and recorded result from before
# the house was added keeps working untouched.
#
# A preset gives its start point either as "starting_position" (grid cells, the
# form the planner wants) or as "start_world" (metres). Prefer start_world for
# maps built by gmapping: map_saver picks an arbitrary origin, so a hardcoded
# grid cell silently points somewhere else every time the map is rebuilt.

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

MAPS = {
    # AWS RoboMaker Small House - the environment the paper's numbers come from.
    # Grid cell kept verbatim: it is a measured, validated value, and this map is
    # a fixed checked-in artefact that no longer gets rebuilt.
    "aws": {
        "map_dir": "/home/tkddus21/catkin_ws/src/aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi",
        "starting_position": [(289, 343)],
        "output_dir": os.path.join(_REPO, "FaRe", "results"),
    },
    # Stock turtlebot3_house. Build the map first - see README "Mapping the
    # turtlebot3_house". (-3.0, 1.0) is the world's stock spawn point and the
    # default in launch/house_sim.launch, so the robot starts on waypoint 0.
    "house": {
        "map_dir": os.path.join(_REPO, "maps", "turtlebot3_house"),
        "start_world": (-3.0, 1.0),
        "output_dir": os.path.join(_REPO, "FaRe", "results", "turtlebot3_house"),
    },
}

MAP_NAME = os.environ.get("FARE_MAP", "aws")
if MAP_NAME not in MAPS:
    raise SystemExit(f"FARE_MAP={MAP_NAME!r} is not one of {sorted(MAPS)}")
_map = MAPS[MAP_NAME]

_pgm = os.path.join(_map["map_dir"], "map.pgm")
_yaml = os.path.join(_map["map_dir"], "map.yaml")


def _pgm_size(path):
    """(width, height) from a binary PGM header, without pulling in PIL.

    MAP.py imports this module, so it cannot be imported back from here.
    """
    with open(path, 'rb') as f:
        header = f.read(128)   # magic + comment + dimensions + maxval fit easily
    fields = re.sub(rb'#[^\n]*', b'', header).split()
    if len(fields) < 4 or fields[0] != b'P5':
        raise ValueError(f"{path} is not a binary PGM")
    return int(fields[1]), int(fields[2])


def _world_to_grid(xy, pgm_path, yaml_path):
    """Metres -> (row, col), inverting PatrolSim.grid_to_world_coords().

    map_server's pgm has row 0 at the TOP of the image while the origin refers to
    the BOTTOM-LEFT pixel, hence the row flip.
    """
    import yaml as _yamllib

    with open(yaml_path, 'r') as f:
        meta = _yamllib.safe_load(f)
    resolution, origin = meta['resolution'], meta['origin']
    _, height = _pgm_size(pgm_path)

    x, y = xy
    col = int(round((x - origin[0]) / resolution))
    row = int(round((height - 1) - (y - origin[1]) / resolution))
    return row, col


if "starting_position" in _map:
    _starting_position = _map["starting_position"]
else:
    if not os.path.exists(_pgm):
        raise SystemExit(
            f"FARE_MAP={MAP_NAME} needs a map at {_map['map_dir']}, which does not exist yet.\n"
            "Build it first - see README 'Mapping the turtlebot3_house'."
        )
    _starting_position = [_world_to_grid(_map["start_world"], _pgm, _yaml)]


config = {

    "starting_position": _starting_position,

    "explored_value": 150, # update cells in fov with explored value

    "unexplored_value": 254, # free space

    "state": 150,  # Should match explored_value

    "steps": 25, #number of iterations

    "surveillance_range": 100,  # Surveillance range (based on sensor range)

    "way_point_dropout": 0,  # Waypoint dropout

    "output_dir": _map["output_dir"], # give path to save results

    "pgm_filename": _pgm, #path where occupancy map is saved

    "yaml_filename": _yaml, #path for yaml data

    "optimizer_params": {

        "wp_threshold": 0.3,

        "num_iterations": 5

    }, # grasp threshold and num_of iterations

    # Costmap footprint half-width of the robot the runs are measured with, from
    # turtlebot3_navigation/param/costmap_common_params_<model>.yaml:
    # burger 0.105 m, waffle_pi 0.155 m. Set to waffle_pi, which is what both the
    # AWS and the house patrols run. Drop to 0.105 if you switch to burger, so
    # diagnose_waypoints.py judges clearance against the right footprint.
    "robot_radius": 0.155,  # metres

    # Minimum distance from a waypoint to any barrier - a wall (0) or an unmapped
    # cell (205), which the robot cannot be driven through either. Waypoints are
    # only placed on a map inflated by this, the planner's analogue of move_base's
    # costmap inflation. diagnose_waypoints.py judges against the same value, so a
    # freshly generated set reports no TIGHT and no UNREACHABLE by construction.
    #
    # Measured on the turtlebot3_house with a waffle_pi. The lower bound comes from
    # a patrol log: a goal 0.30 m from a wall ABORTED, one at 0.32 m SUCCEEDED, so
    # robot_radius * 2 = 0.31 sits exactly on the edge. The upper bound is
    # connectivity: at 0.45 m the placeable area of this map splits into two
    # components and half the house becomes unreachable from the other half. 0.35
    # keeps 64% of the free space placeable in ONE component, and every free cell
    # stays within surveillance_range of somewhere a waypoint can go.
    #
    # Re-derive with diagnose_waypoints.py if the robot or the map changes.
    "waypoint_clearance": 0.35,  # metres

    "trash_detection_range": 5.0, # metres; shorter than surveillance_range, which is optimistic for recognising objects

}

if not os.path.exists(config["output_dir"]):

    os.makedirs(config["output_dir"])
