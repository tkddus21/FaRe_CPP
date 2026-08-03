# FaRe-CPP: Fast Revisit Coverage Path Planning for Autonomous Mobile Patrol Robots Using Long-Range Sensor Information

> 한국어판: [README.ko.md](README.ko.md) · The two documents mirror each other, so a change to one needs the same change to the other.

## Overview
**FaRe-CPP** is an algorithm designed for efficient revisit coverage path planning for autonomous mobile patrol robots using long-range sensor information. This repository provides the tools for generating optimized patrol paths and executing them in simulation environments such as **AWS RoboMaker** and the **Dynamic Logistics Warehouse**.

If you decide to use our work, please cite it as such:
> S. Kachavarapu, T. Doernbach and R. Gerndt, "Fast-Revisit Coverage Path Planning for Autonomous Mobile Patrol Robots Using Long-Range Sensor Information," 2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Hangzhou, China, 2025, pp. 7677-7683, doi: [10.1109/IROS60139.2025.11246182](https://doi.org/10.1109/IROS60139.2025.11246182).

## Implementations

Two implementations of the FaRe-CPP algorithm are provided in this repository:

| Implementation | Directory | Description |
| -------------- | --------- | ----------- |
| **Python** | [`FaRe/`](FaRe/) | Reference implementation. Includes the offline path planner and the ROS/Gazebo online patrol simulation (`PatrolSim.py`). Easiest to read and extend. |
| **C++** | [`FaRe_CPP/`](FaRe_CPP/) | Faster, standalone port of the offline path planner (~10–100× faster, no Python runtime). See [`FaRe_CPP/README.md`](FaRe_CPP/README.md) for build and usage instructions. |

> Note: "CPP" in **FaRe-CPP** stands for **Coverage Path Planning** (the algorithm), not C++. The Python package lives in `FaRe/` and the C++ port lives in `FaRe_CPP/`.


## Getting Started

### 1. Download Simulation Environment Occupancy grid Maps

Before running the FaRe-CPP algorithm, download and set up one of the following simulation environments:
- [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse)

### 2. Clone the FaRe-CPP Repository

Clone this repository using the following command:

```bash
git clone https://github.com/Srinikstudent/FaRe_CPP.git
```
(or) for similar settings as IPA coverage path planning

```bash
git clone --branch FaRe_cpp_ipa_fov https://github.com/hcr-lab/FaRe-CPP.git
```

After cloning, update the file paths in `FaRe/config.py` to point to where your environment map files (.pgm and .yaml) are stored. Adjust the parameters to match your robot's sensor capabilities. (For the C++ implementation, edit `FaRe_CPP/include/Config.hpp` instead — see [`FaRe_CPP/README.md`](FaRe_CPP/README.md).)

3. Install the required dependencies:

    ```bash
    pip install -r requirements.txt
    ```

4. Execute the surveillance script:

    ```bash
    python FaRe/Surveillance.py
    ```

This will process the environment Occupancy grid map and save the waypoints in the output directory. These waypoints will be used for navigation in the simulation.

## Online Navigation(Patrolling)
To execute patrols in the simulation, please make sure ROS and Gazebo are installed and follow these steps.

Prerequisites:

1. ros-noetic and Gazebo simulation installed and working
2. Follow the procedure  on how to set up the simulation environment from here - [AWS RoboMaker Small House World](https://github.com/aws-robotics/aws-robomaker-small-house-world)  or - [Dynamic Logistics Warehouse](https://github.com/belal-ibrahim/dynamic_logistics_warehouse) or you can use environment of your choice.
3. Run `python3 FaRe/Surveillance.py` first, so `results/wp_ori_data.txt` holds the optimized waypoints for this map.

### The whole thing at a glance

```
offline, once per map            online, once per run
─────────────────────            ────────────────────────────────────
Surveillance.py                  T1  view_small_house.launch      world
  frontier + FOV selection       T2  spawn_turtlebot3.launch      robot
  GRASP waypoint ordering        T3  fare_navigation.launch       map_server + AMCL + move_base
  ↓                              T4  set_initial_pose.py          seed AMCL
results/wp_ori_data.txt  ──────────► run_patrol_test.sh           record + patrol
  (coordinates + headings only)      ↓
                                 results/<date>_<time>_<label>/
```

FaRe supplies **where to stand and which way to face** — nothing else. The route between waypoints is planned online by move_base (NavfnROS + DWA) against the live costmap; the A\* path in `path.png` is for metrics and plotting only and is never driven. That split explains why waypoints do not need regenerating when the robot model changes, and why goal failures are a navigation-stack matter rather than a planner one.

## Steps for Online Navigation

Use four terminals. In each one run `source ~/catkin_ws/devel/setup.bash` and `export TURTLEBOT3_MODEL=waffle_pi` (or `burger`; `waffle_pi` matches the scan height the bundled map was built at and measured better — 24/26 goals vs 20/26).

**Terminal 1 — launch the world**
```bash
roslaunch aws_robomaker_small_house_world view_small_house.launch
```
or
```bash
roslaunch dynamic_logistics_warehouse logistics_warehouse.launch
```

**Terminal 2 — spawn TurtleBot 3 into that world**
```bash
roslaunch aws_robomaker_small_house_world spawn_turtlebot3.launch x_pos:=4.65 y_pos:=-2.0
```
`x_pos`/`y_pos` are the world coordinates of the first waypoint (`starting_position` in `FaRe/config.py`), so the patrol starts where it was planned. Convert any grid cell `(row, col)` with
`x = col * resolution + origin[0]` and `y = (map_height - 1 - row) * resolution + origin[1]`.

> Do **not** use `roslaunch turtlebot3_gazebo turtlebot3_world.launch` here. That launch file starts its own gzserver with turtlebot3's own world file, so it does not spawn into the house — it opens a second, different world and clashes with Terminal 1 over the `gazebo` node name.

**Terminal 3 — navigation stack (map_server + AMCL + move_base)**
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch
```
`PatrolSim.py` sends goals to a `move_base` action server, which only exists once this is running.

This is the stock `turtlebot3_navigation.launch` plus `launch/costmap_override.yaml`, whose values were measured on this map — the stock params wedge the robot in doorways (see Known Limitations). Launch it by path because `FaRe_CPP` is not a catkin package. It defaults to the AWS house map; pass `map_file:=<path>` for another one, and keep it the same map `FaRe/config.py` points at.

> `open_rviz:=false` is deliberate. On integrated GPUs, running rviz alongside the Gazebo client can starve the renderer and take gzserver (and the ROS master with it) down mid-run. Use `open_rviz:=true` only if you have the GPU headroom and want to watch the costmaps.

**Terminal 4 — seed AMCL, then patrol**
```bash
python3 FaRe/set_initial_pose.py
./FaRe/run_patrol_test.sh              # recommended: records the run into its own folder
```
`set_initial_pose.py` reads the robot's true pose from Gazebo (`/gazebo/get_model_state`) and publishes it to `/initialpose` — the same thing rviz's "2D Pose Estimate" button does. AMCL cannot localise without an initial pose, and every goal fails until it does, so this step is required rather than optional. If you launched with `open_rviz:=true` you can click "2D Pose Estimate" instead.

`run_patrol_test.sh` wraps `PatrolSim.py` (which you can still run bare) and is what you want when the run is a measurement rather than a smoke test — see the next section. Either way the patrol drives the waypoints from `results/wp_ori_data.txt` and writes one row per goal (`SUCCEEDED` / `ABORTED` / `TIMEOUT`, plus duration) to `patrol_log.csv`. Each goal has a 120 s timeout so one unreachable waypoint cannot stall the whole run, and after any failed goal the robot clears its costmaps and reverses to un-wedge itself.

## Recording a run

`./FaRe/run_patrol_test.sh [label]` captures one patrol into `results/<date>_<time>_<label>/` so runs accumulate instead of overwriting each other. The label defaults to `$TURTLEBOT3_MODEL`, since that is usually what differs between runs.

```bash
./FaRe/run_patrol_test.sh              # -> results/20260729_2350_waffle_pi
./FaRe/run_patrol_test.sh infl03       # -> results/20260729_2350_infl03
RECORD_SCAN=1 ./FaRe/run_patrol_test.sh    # also record /scan (much larger bag)
```

Each folder ends up with the bag, `patrol_log.csv`, the `wp_ori_data.txt` that was actually driven, the coverage/metrics artefacts, and a `run_info.txt` recording the model, the costmap parameters in effect and the git revision — so the numbers stay interpretable months later.

Before recording anything the script checks that `move_base` is up, that waypoints exist, and **that AMCL agrees with Gazebo ground truth to within 0.5 m**. That last check matters: a diverged particle filter produces a run that looks like a navigation failure but is really a measurement failure, and it is not obvious from the logs. It also stops `rosbag` with SIGINT rather than SIGKILL, because a killed bag is left unindexed and may be unplayable.

### Replaying a run

```bash
rosparam set /use_sim_time true                     # before rviz starts, not after
rviz -d launch/patrol_playback.rviz
rosbag play --clock -d 5 -r 3 results/<run>/patrol.bag
```
`rviz` reads `use_sim_time` at startup, so setting it afterwards leaves rviz on wall time and nothing renders. The `-d 5` delay matters too: `/move_base/global_costmap/costmap` is published **once and not latched**, so without a delay it fires before rviz has finished subscribing and the global costmap never appears. (`/map` is latched, which is why it shows up regardless.)

## Verifying coverage and detection

```bash
python3 FaRe/diagnose_waypoints.py    # clearance per waypoint AND per segment, joined to the last run's statuses
python3 FaRe/trash_eval.py            # how many placed objects the planned path would see
python3 FaRe/trash_eval.py --range 5  # compare against the optimistic full sensor range
```
`Surveillance.py` also writes `results/coverage_map.png` (orange = seen by the sensor, red = free space missed) and appends the coverage percentage to `results/metrics.csv`. Object positions for `trash_eval.py` live in `FaRe/trash_positions.txt` as `row, col` grid cells.

## Known Limitations

- **`PatrolSim.py` coordinate conversion (fixed):** `grid_to_world_coords()` previously transposed row/col and didn't account for `map_server`'s vertical flip (pgm row 0 = top of image = *max* world y, with the map origin at the bottom-left pixel), so goals were sent to the wrong physical location. Confirmed by testing: on an open map every waypoint failed or landed in the wrong spot before the fix, and 13-14/14 succeeded after it. Also fixed in the same pass: an invalid goal quaternion (`orientation.z = theta` instead of a real yaw quaternion), a success check that trusted `wait_for_result()` alone (which returns `True` even for `ABORTED` goals), and an `IndexError` from `wp` (waypoints, includes a final return-to-start point) being one longer than `ori` (orientations).
- **Waypoint placement ignores the robot's body, and that is the paper's design, not a bug:** `find_frontier_cells()` in `FaRe/Scout_Multi_Processing.py` only checks that a candidate cell is `buffer_distance` cells (default 4, i.e. 0.2 m at 0.05 m/cell) from the *nearest* obstacle. It never checks the footprint. The paper says so outright — FaRe uses the sensor's field of view as the footprint "instead of considering the robot's physical dimensions" (Sec. III), and drops the map entirely during waypoint optimisation "to avoid computational complexity" (Sec. III-D).

   Measured over the 24/26 waffle_pi run, that abstraction costs less *at the goal* than expected: **all 26 waypoints cleared the footprint** (minimum 0.25 m, against the 0.175 m a padded waffle_pi needs), and goal clearance did not separate failures from successes — one *failed* goal sat in 1.412 m of open space. The failures happen **in transit**, not at the goal, which is what the next item is about. `FaRe/diagnose_waypoints.py` now reports both, per waypoint and per segment.

- **A single wedge used to kill the whole patrol (fixed):** on cluttered maps the robot can drive into a gap barely wider than itself while *travelling between* waypoints. move_base's own rotate recovery then refuses to act ("can't rotate in place because there is a potential collision. Cost: -1.00"), so the robot stays stuck and every remaining goal aborts against a robot that cannot move. `PatrolSim.py` now calls `/move_base/clear_costmaps` and reverses ~16 cm under direct `/cmd_vel` control after any non-`SUCCEEDED` goal, which move_base cannot do for itself. This one change took the AWS house patrol from 7/26 to 20/26 goals.

- **Stock costmap params are wrong for indoor maps (fixed in `launch/costmap_override.yaml`):** `turtlebot3_navigation` ships `inflation_radius: 1.0`, but the AWS house map's *median* free-space clearance is only 0.552 m, so over half the navigable area sits inside the gradient and almost nothing is genuinely cheap. Use `launch/fare_navigation.launch` instead of `turtlebot3_navigation.launch` to get the overrides.

   Measured on the AWS house map, 26 waypoints per run:

   | config | goals reached | note |
   | --- | --- | --- |
   | stock (`inflation_radius: 1.0`) | 7/26 | wedged at goal 8, everything after failed |
   | `inflation_radius: 0.30` | 7/26 | same wedge, robot squeezed 3 cm *deeper* in |
   | + `footprint_padding: 0.045` | 4/26 | **worse** — padding shrinks every corridor, not just the bad pinch |
   | + `footprint_padding: 0.020` and PatrolSim recovery | **20/26** | recovers instead of cascading |

   Note the third row: lowering inflation or padding the footprint is not a free win. Padding to an effective 0.150 m radius did block the one 0.112 m pinch, but then wedged the robot at a 0.180 m spot it had previously driven through fine.

- **The offline path was planned for a point robot (fixed in `FaRe/traversability.py`):** the old path search treated every free cell as drivable and moved in 4-connected steps, so it happily threaded gaps no robot could enter. Measured on the 26 waypoints of the 24/26 run, **the path it drew squeezed through a 0.050 m gap** — the waffle_pi needs 0.175 m. That is why `path.png` cut straight through the bench, and why the reported `path_length` described a route move_base would never follow.

   Routing now runs on a grid inflated by `robot_radius + footprint_padding`, the same radius `launch/costmap_override.yaml` hands move_base, and weights each cell by costmap_2d's inflation gradient so paths keep to the middle of corridors the way NavfnROS does instead of hugging the boundary. Same waypoints, both models (`results/routing_comparison.png`):

   | | point robot | footprint-aware |
   | --- | --- | --- |
   | path length | 69.60 m | 61.52 m |
   | tightest gap used | **0.050 m** | 0.200 m |
   | median gap | 0.250 m | 0.300 m |

   It comes out *shorter* because 8-connected steps drop the staircase overhead of 4-connected ones, which more than pays for going around the furniture. GRASP now orders waypoints on these real drivable distances too, instead of straight lines that ignore walls.

- **Geometry still does not predict which goals fail (open):** with the footprint-aware model every waypoint pair has a drivable route, and the tightest point of each segment does not separate outcomes: the two failed goals sat at 0.200 m and 0.292 m, but segments with those same bottlenecks were driven successfully, and the successful median is only 0.320 m. Tightness is a risk factor, not a predictor. What remains is runtime — DWA's trajectory search, AMCL noise, and recovery behaviour — which is why `PatrolSim.py` keeps its costmap-clear-and-reverse recovery rather than relying on the plan being safe. Run `FaRe/diagnose_waypoints.py` after a patrol and it will join `patrol_log.csv` statuses against the per-segment geometry.
   
