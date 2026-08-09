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

## Choosing the environment

Two environments are set up. `FARE_MAP` selects one, and defaults to `aws` — so every command in this README works unchanged if you only care about the original.

| `FARE_MAP` | World | Map | Results land in |
| --- | --- | --- | --- |
| `aws` (default) | AWS RoboMaker Small House | `aws-robomaker-small-house-world/maps/turtlebot3_waffle_pi/` | `FaRe/results/` |
| `house` | stock `turtlebot3_house` | [`maps/turtlebot3_house/`](maps/turtlebot3_house/) | `FaRe/results/turtlebot3_house/` |

```bash
export FARE_MAP=house      # in every terminal, alongside TURTLEBOT3_MODEL
```

Each environment keeps its own `output_dir` on purpose: `results/wp_ori_data.txt` is the handoff between the offline planner and the patrol, so a shared directory would let a house patrol drive the AWS waypoints and quietly produce a meaningless run.

Presets live in `MAPS` at the top of [`FaRe/config.py`](FaRe/config.py); add an environment by adding an entry. A preset gives its start point either as `starting_position` (grid cells) or `start_world` (metres, converted on load). Prefer `start_world` for maps you build yourself — `map_saver` picks an arbitrary origin, so a hardcoded grid cell silently points somewhere else every time the map is rebuilt.

### Check a map before planning on it

```bash
FARE_MAP=house python3 FaRe/check_map.py
```

FaRe identifies free space by **exact equality** with `unexplored_value` (254), so a map that renders perfectly in rviz can still be half-invisible to the planner. A map exported from an image editor carries cells at 255 plus a spread of anti-aliased greys; `map_server` treats those as free and FaRe does not, and the only symptom is a coverage number quietly computed over half the house. `check_map.py` fails on anything outside `{0, 205, 254}`, reports free area and clearance, and converts `starting_position` into the world coordinates to spawn at.

## Mapping the turtlebot3_house

The house ships no map, so build one with gmapping. Two things not to do: do not hand-edit a map in an image editor (see above), and do not borrow one built for a different world — a map registered a few centimetres off is the same order as `inflation_radius`, and it costs goals in a way that looks like a navigation failure.

```bash
sudo apt install ros-noetic-slam-gmapping     # not installed by default
```

Four terminals, each with `export TURTLEBOT3_MODEL=waffle_pi`:

```bash
# T1  world + robot
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_sim.launch
# T2  gmapping + rviz
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
# T3  drive every room until the walls close up
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
# T4  save
rosrun map_server map_saver -f ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map
```

Drive down the middle of wide rooms as well as around the walls: the waffle_pi's LDS stops at 3.5 m, and unmapped interior counts as a barrier — `diagnose_waypoints.py` treats unknown space as one, and waypoints are never placed there. Then run `check_map.py`, and `Surveillance.py` to generate the waypoints for this map.

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

Use four terminals. In each one run `source ~/catkin_ws/devel/setup.bash` and `export TURTLEBOT3_MODEL=waffle_pi` (or `burger`; `waffle_pi` matches the scan height the bundled map was built at and measured better — 24/26 goals vs 20/26). For the turtlebot3_house, also `export FARE_MAP=house`.

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
`x = col * resolution + origin[0]` and `y = (map_height - 1 - row) * resolution + origin[1]` — or just read the spawn command off `check_map.py`, which prints it.

> Do **not** use `roslaunch turtlebot3_gazebo turtlebot3_world.launch` here. That launch file starts its own gzserver with turtlebot3's own world file, so it does not spawn into the house — it opens a second, different world and clashes with Terminal 1 over the `gazebo` node name.

**Terminals 1 and 2, for the turtlebot3_house** — one launch file does both:
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_sim.launch
```
It defaults to the house's stock spawn point `(-3.0, 1.0)`, which is what the `house` preset resolves waypoint 0 to; override with `x_pos:=`/`y_pos:=` if you change `start_world`.

> Do **not** use `roslaunch turtlebot3_gazebo turtlebot3_house.launch` for this. It loads the same world, but spawns the Gazebo model under the bare name `turtlebot3`, while `set_initial_pose.py` and `run_patrol_test.sh`'s AMCL pre-check both look up `turtlebot3_$TURTLEBOT3_MODEL`. The pre-check has no override for that name, so it compares AMCL against a not-found model's zero pose and aborts every run.

**Terminal 3 — navigation stack (map_server + AMCL + move_base)**
```bash
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch
# turtlebot3_house:
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/fare_navigation.launch \
  map_file:=$HOME/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map.yaml
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

`./FaRe/run_patrol_test.sh [label]` captures one patrol into `<output_dir>/<date>_<time>_<label>/` so runs accumulate instead of overwriting each other. The label defaults to `${FARE_MAP}_${TURTLEBOT3_MODEL}`, the two things that usually differ between runs — with more than one environment in play, a directory named just `waffle_pi` no longer identifies the run.

```bash
./FaRe/run_patrol_test.sh                  # -> results/20260729_2350_aws_waffle_pi
FARE_MAP=house ./FaRe/run_patrol_test.sh   # -> results/turtlebot3_house/20260729_2350_house_waffle_pi
./FaRe/run_patrol_test.sh infl03           # -> results/20260729_2350_infl03
RECORD_SCAN=1 ./FaRe/run_patrol_test.sh    # also record /scan (much larger bag)
```

The script reads `output_dir` from `FaRe/config.py` rather than assuming `results/`, so it follows whichever environment `FARE_MAP` selects.

Each folder ends up with the bag, `patrol_log.csv`, the `wp_ori_data.txt` that was actually driven, the coverage/metrics artefacts, and a `run_info.txt` recording the model, the map, the costmap parameters in effect and the git revision — so the numbers stay interpretable months later.

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
python3 FaRe/check_map.py             # the map itself: trinary values, free area, start cell
python3 FaRe/diagnose_waypoints.py    # clearance per waypoint vs the robot footprint
python3 FaRe/trash_eval.py            # how many placed objects the planned path would see
python3 FaRe/trash_eval.py --range 5  # compare against the optimistic full sensor range
```
`trash_eval.py` is AWS-only: `FaRe/trash_positions.txt` holds grid cells for that map, which index meaningless spots on any other.
`Surveillance.py` also writes `results/coverage_map.png` (orange = seen by the sensor, red = free space missed) and appends the coverage percentage to `results/metrics.csv`. Object positions for `trash_eval.py` live in `FaRe/trash_positions.txt` as `row, col` grid cells.

## Known Limitations

- **`PatrolSim.py` coordinate conversion (fixed):** `grid_to_world_coords()` previously transposed row/col and didn't account for `map_server`'s vertical flip (pgm row 0 = top of image = *max* world y, with the map origin at the bottom-left pixel), so goals were sent to the wrong physical location. Confirmed by testing: on an open map every waypoint failed or landed in the wrong spot before the fix, and 13-14/14 succeeded after it. Also fixed in the same pass: an invalid goal quaternion (`orientation.z = theta` instead of a real yaw quaternion), a success check that trusted `wait_for_result()` alone (which returns `True` even for `ABORTED` goals), and an `IndexError` from `wp` (waypoints, includes a final return-to-start point) being one longer than `ori` (orientations).
- **Waypoint clearance doesn't account for robot footprint (open, but measured):** `find_frontier_cells()` in `FaRe/Scout_Multi_Processing.py` only checks that a candidate cell is `buffer_distance` cells (default 4, i.e. 0.2m at 0.05 resolution) away from the *nearest* obstacle cell — a single-point distance check, not a check that the corridor the robot has to actually drive through is wide enough. Use `FaRe/diagnose_waypoints.py` to measure the real clearance (distance transform, treating unknown space as a barrier too) per waypoint.

   Measured on the AWS Small House map: **all 26 waypoints cleared the burger footprint**, minimum clearance 0.25 m against a 0.105 m footprint half-width — so on this map the placement is not geometrically undrivable, and the earlier flakiness traced to the coordinate/quaternion/pairing bugs above rather than to gap width. After those fixes a 5-waypoint patrol scored 5/5 `SUCCEEDED`. The check is still worth keeping: it is a single-point test, so a different map can defeat it.

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

- **Waypoint clearance does not predict goal failure (hypothesis rejected):** it is tempting to blame `find_frontier_cells()` for placing waypoints in tight spots, but the measurement says otherwise. Over the 20/26 run, failed goals had a median clearance of 0.450 m and successful goals 0.480 m — statistically indistinguishable — and one *failed* goal sat in a 1.412 m open space. Every one of the 26 waypoints cleared the footprint. The failures happen **in transit**, where the global planner routes through pinches the waypoints themselves avoid, so fixing this belongs in the navigation config and in recovery behaviour, not in waypoint placement.
   
