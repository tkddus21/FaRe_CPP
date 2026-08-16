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

Three terminals, each with `export TURTLEBOT3_MODEL=waffle_pi`:

```bash
# T1  world + robot
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_sim.launch

# T2  gmapping + move_base
roslaunch ~/catkin_ws/src/FaRe_CPP/launch/house_mapping.launch open_rviz:=true

# T3  find the rooms, then finish them, then save
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/explore_for_mapping.py
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/tour_house_for_mapping.py
rosrun map_server map_saver -f ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map

# T3  put the garden back to unknown
python3 ~/catkin_ws/src/FaRe_CPP/FaRe/crop_map_to_house.py \
    ~/catkin_ws/src/FaRe_CPP/maps/turtlebot3_house/map.pgm
```

Then `check_map.py` to check it, and `Surveillance.py` to generate the waypoints for this map.

**`house_mapping.launch` is the whole mapping stack** — robot_state_publisher, gmapping and move_base. Do not also run `turtlebot3_slam.launch`: both start nodes under the same names, roslaunch resolves that by killing the older one, and the survivor is left with no scan transform and a `/map` that never publishes again. It looks like move_base failing to come up, and the map built so far is gone.

**Both driving scripts, in that order.** `explore_for_mapping.py` picks frontiers — free cells touching unmapped space — off gmapping's live `/map`, which makes it good at *finding* rooms and bad at *finishing* them: it stops as soon as no frontier is left, and a wall glimpsed once from four metres away satisfies that test while still being a smear. `tour_house_for_mapping.py` does the opposite. The house is a known static world, so it reads the room boxes out of `turtlebot3_gazebo`'s `model.sdf` and visits a 1.2 m grid over them — below gmapping's 3.0 m `maxUrange`, so every part of every room is seen from close enough for the scan to land. This is what replaces the old advice to drive down the middle of wide rooms by hand: unmapped interior counts as a barrier, and `diagnose_waypoints.py` never places a waypoint there. Running the tour after the explorer is safe — the same gmapping node keeps accumulating.

**Why the crop.** The house's east wall is split between y = -0.40 and y = 0.50 and the south wall has a gap around x = 5.0..5.8, so the laser looks straight out onto the lawn and gmapping records the garden as free space with nothing to bound it. FaRe would then place waypoints out there and report coverage over an area that includes the grass. `crop_map_to_house.py` bounds the map by the rooms in `model.sdf` — measured at 100.7 → 93.4 sq.m on the bundled map. This is *not* the image-editor touch-up warned about above: nothing is resampled, the only value written is 205, which the map already uses for unseen space, and the origin is untouched, so the map still lines up with Gazebo world coordinates. It keeps a `.orig` copy and re-reads it on every run, so widening a margin and re-running works.

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
- **Goal *heading* had the same frame flip as position (fixed):** `grid_to_world_coords()` was corrected for the pgm row flip, but the orientation travelling beside it was not. `cast_fov()` sweeps in grid coordinates where the row axis grows *downward*, while world y grows upward, so world yaw is `-theta`, not `theta`. `PatrolSim.py` sent it through unconverted. Because `Scout.fov()` quantises headings to `[0, 90, 180, 270]` degrees, the two vertical ones came out exactly reversed — **15 of the 26 AWS waypoints (58%) faced backwards**. Position was unaffected, so goals still reported `SUCCEEDED`, no log showed anything wrong, and the only cost was coverage the robot silently failed to observe. That is why it outlived the position fix. Now converted in `grid_yaws_to_world()` at the same boundary; `wp_ori_data.txt` stays in grid space, since `Surveillance.py` and `trash_eval.py` cast FOV in grid coordinates and must not be converted. `patrol_log.csv` gained a `grid_theta_rad` column so `yaw_rad` can be read as the yaw actually commanded.

  Runs recorded before this fix keep valid goal-success numbers — those measure whether the robot arrived — but their *achieved* coverage was lower than the planned figure.

- **Sensor model is a flat 2D sector (by design, worth knowing):** the FOV is a 90° sector ray-cast on the occupancy grid, 0.05–5 m, with occlusion (rays stop at obstacles) but no vertical extent, no sensor height and no mounting geometry — the grid *is* the scan plane, at whatever height the map was built at. Headings are chosen from only four candidates, `[0, 90, 180, 270]` degrees, by picking whichever sees the most area. Note `surveillance_range` is 5 m while the waffle_pi's LDS reaches 3.5 m, so planned coverage is optimistic on both maps. Unknown cells neither block rays nor count as seen. (`FaRe_CPP/src/FOV.cpp` computes a vertical FOV from a camera model, but nothing calls it — the C++ planner uses the same flat sector.)

- **Waypoint clearance didn't account for robot footprint (fixed):** `find_frontier_cells()` used to keep any candidate whose ±4-cell square neighbourhood held no wall. Two problems: the square measures *Chebyshev* distance, so it guaranteed only 5 cells = 0.25 m of clearance regardless of the robot; and it tested for walls (`0`) alone, so unmapped space (`205`) — including the boundary `crop_map_to_house.py` draws — was invisible to it. On the house map that put 14 of 26 waypoints exactly 0.25 m from a wall and one 0.05 m from the crop boundary.

  Generation now runs on an **inflated map**, the planner's counterpart to move_base's costmap inflation: `Exploration.placeable_mask()` inflates every barrier (walls *and* unmapped cells) by `config['waypoint_clearance']` with one distance transform, and waypoints go only where that survives. `diagnose_waypoints.py` measures against the same constant, so a freshly generated set reports no `TIGHT` and no `UNREACHABLE` by construction — it becomes a check that a waypoint file matches its map, not a filter.

  The mask constrains *positions only*. Frontier detection and `cast_fov()` still run on the real grid, so inflation costs no coverage: a cell the robot cannot stand in is still a cell it can see into.

  `config['waypoint_clearance']` is 0.35 m, measured. Below it: a goal 0.30 m from a wall `ABORTED` while one at 0.32 m `SUCCEEDED`, so `robot_radius * 2` = 0.31 sits on the edge. Above it: at 0.45 m the house map's placeable area splits into two components. 0.35 keeps 64% of the house's free space placeable in one component (AWS 69%), with every free cell still within `surveillance_range` of somewhere a waypoint can go.

  Measured end to end on the turtlebot3_house with a waffle_pi: **12/26 goals before, 25/26 after** (`results/turtlebot3_house/20260811_1736_house_waffle_pi`), 10.7 minutes, median goal 17 s. Planned coverage moved 98.80% → 98.13% and the tour got *shorter*, 94.85 → 76.45 m.

  The AWS house was run the same day and scored **25/26** (`results/20260811_1757_aws_waffle_pi`), 10.5 minutes, median goal 18.6 s, against 21/26 on the last comparable run before the change. Read that one cautiously: the baseline is from 2026-07-30 rather than a paired same-session run, and earlier AWS runs with this toolchain have landed anywhere from 20/26 to 24/26, so a 4-goal gain is within the spread this map already shows. What is solid on AWS is the static result — 5 `TIGHT` waypoints became none, coverage held at 96.2% — bought with a 19% longer tour. Inflation is not free on a map whose clearance was never the binding constraint; it just turned out not to cost goals either.

  The one remaining failure is the useful part, and it is neither a placement nor a transit-wedge problem — it is the local planner, traced from the recorded bag and written up under "DWA misbehaves in open space" below. With generation now enforcing the clearance, `diagnose_waypoints.py` can no longer explain a failed goal, and that is exactly what made the real cause legible.

  Note `FaRe_CPP/src/Scout.cpp` still uses the old square test — the two implementations have diverged here.

- **DWA misbehaves in open space (open; one fix tried and reverted):** the failures that survive the placement fix are all the local planner losing the plot somewhere it has plenty of room. Two instances, both traced from recorded bags:

  *Driving away from the goal.* In `results/turtlebot3_house/20260811_1736_house_waffle_pi`, goal #8 at (6.95, 0.10) `TIMEOUT`ed after 120 s. The waypoint was fine (0.350 m clearance, no wall within 0.79 m of the approach) and so was the plan — NavFn's first was a 1.4 m straight line and its endpoint stayed on the goal for all 120 s. The robot reversed in to within **0.193 m**, closing x from 6.51 to 6.78 while y stuck at 0.190 against a target of 0.10, then flipped to forward and drove **12 m west**, away from a plan pointing east, before turning back 3.1 m short when the timeout fired. Over 3719 tf samples it never came within 0.15 m of the goal.

  The stock `dwa_local_planner_params_waffle_pi.yaml` explains why it could not close the last 0.19 m: its shortest simulable move is `min_vel_trans * sim_time` = 0.13 × 2.0 = **0.26 m**, but `xy_goal_tolerance` is **0.05 m**. No candidate trajectory ends inside the tolerance, so every one overshoots, and past the goal the best-scoring direction flips. That form only bites on short goals — travelled / straight-line distance was 26× on #8, 38× on #16, 9× on #6, against 1.0–1.8× for every goal of 2.7 m or more. It matters because 10 of the 25 goals sit within 0.30 m of the previous one and three are on the *same cell* with only the heading differing; `set_goals()` re-picks near-identical frontier cells across iterations.

  *Oscillating on the spot.* In `results/turtlebot3_house/20260817_0152_arrival_radius`, the robot stopped at (5.73, −2.74) in the right wing and sat there for 60 s until move_base reported `Robot is oscillating. Even after executing recovery behaviors.` and aborted — taking the next three goals down with it. It had 0.492 m of static clearance, a 0.90 m wide free corridor to the north in its own local costmap, and a valid 96-pose 2.5 m global plan the whole time. Nothing was in the way.

  **The fix that was tried and reverted.** `PatrolSim.send_goal()` was changed to stop asking for precision the patrol does not need: a waypoint exists to place the sensor and `surveillance_range` is 5 m, so once the robot came within 0.25 m the goal was cancelled and the robot turned on the spot to the heading the FOV wanted. The mechanism worked exactly as designed — 22 of 26 goals finished that way, all within 0.038–0.240 m, and short goals collapsed (#16 40.8 s → 2.2 s, #12 13.5 s → 2.5 s). **The run still scored worse: 22/26 against 25/26.**

  Why is the useful part. The right wing has two parallel corridors either side of an obstacle at x ≈ 6.30. Finishing a goal by rotating leaves the robot up to 0.25 m off the waypoint, and starting the wing exit from (6.06, −4.75) instead of (6.25, −4.92) was enough to flip NavFn from the east corridor (x ≈ 6.75, which worked) to the west one (x ≈ 5.75, where it oscillated). So the change did not *cause* the failure — it changed which corridor was taken, and one of them hides a local-planner pathology. With one run per condition there is no way to separate that from ordinary variance, and 22/26 is worse than 25/26, so it was reverted rather than kept on a hunch.

  Still open, and worth knowing before trying again: the two failure forms above are the same suspect, and neither the stock DWA params nor `set_goals()`'s near-duplicate waypoints have been touched. Any retry should run each condition several times — single runs on this map do not separate a fix from a coin flip.

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

- **Whether waypoint clearance predicts goal failure depends on the map (both results stand):** it was first tested on the **AWS house with a burger** and rejected. Over the 20/26 run, failed goals had a median clearance of 0.450 m and successful goals 0.480 m — statistically indistinguishable — and one *failed* goal sat in a 1.412 m open space. Every waypoint cleared the 0.105 m footprint with room over, so clearance was simply not the binding constraint there; the failures happened **in transit**, where the global planner routed through pinches the waypoints themselves avoided.

  On the **turtlebot3_house with a waffle_pi** the same measurement comes out the opposite way, and cleanly:

  | clearance verdict | `SUCCEEDED` | failed |
  |---|---|---|
  | `OK` (≥ 0.31 m) | **11** | 0 |
  | `TIGHT` (0.155–0.31 m) | 1 | **13** |
  | `UNREACHABLE` (< 0.155 m) | 0 | **1** |

  Both results are real. The house is tighter (median free-space clearance 0.450 m vs 0.552 m) and the waffle_pi is half again as wide as a burger (0.155 m vs 0.105 m), so waypoints that were comfortable on one map sit inside the costmap's inflation gradient on the other and the global planner cannot produce a path to them at all. The lesson is not that either measurement was wrong but that **this one has to be re-run per map and per robot** — which is what `diagnose_waypoints.py` is for. The placement fix above addresses the house case; recovery behaviour and the costmap overrides still address the in-transit case.
   
