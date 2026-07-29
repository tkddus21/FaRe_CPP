import os



# Configuration parameters for FireBot tool



config = {

    "starting_position": [(289, 343)],

    "explored_value": 150, # update cells in fov with explored value

    "unexplored_value": 254, # free space

    "state": 150,  # Should match explored_value

    "steps": 25, #number of iterations

    "surveillance_range": 100,  # Surveillance range (based on sensor range)

    "way_point_dropout": 0,  # Waypoint dropout

    "output_dir": "/home/tkddus21/catkin_ws/src/FaRe_CPP/FaRe/results", # give path to save results

    "pgm_filename": "/home/tkddus21/catkin_ws/src/aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/map.pgm", #path where occupancy map is saved

    "yaml_filename": "/home/tkddus21/catkin_ws/src/aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/map.yaml", #path for yaml data

    "optimizer_params": {

        "wp_threshold": 0.3,

        "num_iterations": 5

    }, # grasp threshold and num_of iterations

    # Matches TURTLEBOT3_MODEL=burger, whose costmap footprint half-width is 0.105 m
    # Matches TURTLEBOT3_MODEL=waffle_pi, whose costmap footprint half-width is 0.155 m
    # (turtlebot3_navigation/param/costmap_common_params_burger.yaml). The map above is
    # the waffle_pi one but geometry-independent, so burger stays authoritative here.
    "robot_radius": 0.155,  # metres

    "trash_detection_range": 5.0, # metres; shorter than surveillance_range, which is optimistic for recognising objects

}

if not os.path.exists(config["output_dir"]):

    os.makedirs(config["output_dir"])
