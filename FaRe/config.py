import os



# Configuration parameters for FireBot tool



config = {

    "starting_position": [(289, 343)],

    "explored_value": 150, # update cells in fov with explored value

    "unexplored_value": 254, # free space

    "state": 150,  # Should match explored_value

    "steps": 25, #number of iterations

    "surveillance_range": 100,  # Surveillance range in CELLS (100 * 0.05 m = 5 m)

    # Horizontal field of view of the range sensor, in degrees. The paper models a
    # camera here, not the LiDAR: waffle_pi's Pi Camera is 62.2 deg (horizontal_fov
    # 1.085595 rad) while the LDS-01 LiDAR is omnidirectional, so a sector only makes
    # sense for the camera. Paper uses 90/120 on Warehouse and 58.4 on House.
    "fov_angle": 90,

    "way_point_dropout": 0,  # Waypoint dropout

    "output_dir": "/home/tkddus21/catkin_ws/src/FaRe_CPP/FaRe/results", # give path to save results

    "pgm_filename": "/home/tkddus21/catkin_ws/src/aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/map.pgm", #path where occupancy map is saved

    "yaml_filename": "/home/tkddus21/catkin_ws/src/aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/map.yaml", #path for yaml data

    "optimizer_params": {

        "wp_threshold": 0.3,

        "num_iterations": 5

    }, # grasp threshold and num_of iterations

    # Inscribed radius of TURTLEBOT3_MODEL=waffle_pi: its costmap footprint is the
    # rectangle [+-0.205, +-0.155], so 0.155 m is the half-width that has to fit
    # through a gap (turtlebot3_navigation/param/costmap_common_params_waffle_pi.yaml).
    # Set to 0.105 when running burger.
    "robot_radius": 0.155,  # metres

    # Mirrors footprint_padding in launch/costmap_override.yaml. Kept here so the
    # offline traversability check inflates obstacles by the same amount move_base
    # does at runtime; if they disagree, offline planning approves paths the local
    # planner then refuses to drive.
    "footprint_padding": 0.020,  # metres

    # Also mirrored from launch/costmap_override.yaml. The offline router reproduces
    # costmap_2d's inflation gradient with these so its paths keep to the middle of
    # corridors the way NavfnROS does, instead of hugging the obstacle boundary.
    "inflation_radius": 0.30,      # metres
    "cost_scaling_factor": 5.0,    # 1/metres

    # Stopping criteria from the paper (Sec. III-C). Defaults disable both so the
    # run length stays governed by "steps" unless explicitly opted into.
    "min_coverage_area": None,   # sq.m of explored area to stop at (paper's A_min)
    "coverage_epsilon": 0.0,     # stop when relative area gain per iteration drops below this

    "trash_detection_range": 5.0, # metres; shorter than surveillance_range, which is optimistic for recognising objects

}

if not os.path.exists(config["output_dir"]):

    os.makedirs(config["output_dir"])
