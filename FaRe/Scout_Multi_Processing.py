import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml
import time
from IPython.display import clear_output
from scipy.spatial.distance import pdist, squareform
from scipy.ndimage import distance_transform_edt
from itertools import permutations
from MAP import Map_generator
from config import config
import multiprocessing
from Multi_Processing import process_frontier
import random
import math
map_generator = Map_generator()

# Cells a robot can never occupy: walls, and unmapped space, which is not free
# just because nothing was seen there. Everything else (254 unexplored free,
# 150 already surveyed) is drivable.
BARRIER_VALUES = (0, 205)



class Scout:
    def __init__(self):
        pass

    def cast_fov(self, grid_map, start_pos, radius, base_angle, fov_angle=90):
        """Ray-casts one FOV sector centred on base_angle (degrees), marking visible cells."""
        grid = np.copy(grid_map)
        angles = np.deg2rad(np.linspace(base_angle - fov_angle / 2, base_angle + fov_angle / 2, num=400))
        y_start, x_start = start_pos  # Start position interpreted as (y, x)

        for angle in angles:
            for r in range(1, radius + 1):
                x = int(x_start + r * np.cos(angle))
                y = int(y_start + r * np.sin(angle))
                if 0 <= x < grid.shape[1] and 0 <= y < grid.shape[0]:
                    if grid[y, x] == 0:  # Occupied, block visibility
                        break
                    elif grid[y, x] == 254:  # Unoccupied, update to visible
                        grid[y, x] = 150
                else:
                    break

        return grid

    def fov(self, grid_map, start_pos, radius, fov_angle=90):

        base_angles = [0, 180, 270, 90]  # Angles for N, S, W, E
        best_grid = None
        max_area = -1
        best_angle = None

        for base_angle in base_angles:
            grid = self.cast_fov(grid_map, start_pos, radius, base_angle, fov_angle)

            explored_area = np.sum(grid == 150)
            if explored_area > max_area:
                max_area = explored_area
                best_grid = grid
                best_angle = base_angle

        return best_grid, best_angle*(math.pi / 180)

    def find_frontier_cells(self, grid_map, explored_value, unexplored_value, placeable):
        """Unexplored cells bordering surveyed space that the robot can actually reach.

        `placeable` is the inflated-map mask from Exploration.placeable_mask(): a
        candidate is kept only where the footprint fits with waypoint_clearance to
        spare. It replaces an earlier square-neighbourhood test that measured
        Chebyshev distance to the nearest *wall* and ignored unmapped space, which
        put waypoints 0.25 m from walls and one 0.05 m from a map boundary. Nothing
        move_base could plan a path to - see README "Known Limitations".

        The mask filters positions only. Frontier detection below still runs on the
        real grid, and so does cast_fov(), so inflation costs no coverage: cells the
        robot cannot stand in are still cells it can see into.
        """
        rows, cols = grid_map.shape
        frontier_cells = []

        for i in range(rows):
            for j in range(cols):
                if grid_map[i, j] == unexplored_value:
                    if ((i > 0 and grid_map[i-1, j] == explored_value) or
                        (i < rows - 1 and grid_map[i+1, j] == explored_value) or
                        (j > 0 and grid_map[i, j-1] == explored_value) or
                        (j < cols - 1 and grid_map[i, j+1] == explored_value)):

                        if placeable[i, j]:
                            frontier_cells.append((i, j))

        return frontier_cells
class Exploration:
    def __init__(self, grid_map, surveillance_range, free_cells, state, yaml_data):
        self.grid_map = grid_map
        self.surveillance_range = surveillance_range
        self.free_cells = free_cells
        self.state = state
        self.yaml_data = yaml_data
        self.scout = Scout()
        # Computed once from the map as loaded, not per iteration: surveying only
        # rewrites free cells (254 -> 150), so the barriers this depends on never
        # move while set_goals() runs.
        self.placeable = self.placeable_mask(grid_map, yaml_data)
        print('range:', self.surveillance_range )

    @staticmethod
    def placeable_mask(grid_map, yaml_data):
        """Cells far enough from every barrier to put a waypoint on.

        The planner's counterpart to move_base's costmap inflation: rather than
        checking each candidate against its neighbourhood, inflate the barriers once
        and place waypoints only on what survives. One distance transform replaces a
        square scan per free cell per iteration, and it measures true Euclidean
        distance instead of the Chebyshev distance a square neighbourhood implies.

        Uses the same computation as diagnose_waypoints.clearance_grid(), against
        the same config['waypoint_clearance'], so what this generates is what that
        script passes.
        """
        barrier = np.isin(grid_map, BARRIER_VALUES)
        clearance = distance_transform_edt(~barrier) * yaml_data['resolution']
        return clearance >= config['waypoint_clearance']
    def surveillance(self, iteration, frontiers, graph, area):
        
        with multiprocessing.Pool() as pool:
            # Prepare arguments for each process
            args = [(graph, frontier, self.scout, self.surveillance_range, self.free_cells, self.yaml_data, self.state) for frontier in frontiers]

            # Execute the function in parallel
            results = pool.starmap(process_frontier, args)

        
        max_area_dict = max(results, key=lambda x: x['area'])
        selected_frontier, area, graph, ori = max_area_dict['frontier'], max_area_dict['area'], max_area_dict['sub_graph'],max_area_dict['ori']
        
        return selected_frontier, area, graph,ori
    def set_goals(self, current_pos, explored_value, unexplored_value,steps, frontier_drop_rate):
        total_area = 0
        iteration = 0
        t_time = 0
        graph = self.grid_map
        area_goals = []

        for i in range(steps):
            start_time = time.time()
            # i == 0 takes the configured start verbatim rather than a frontier, so
            # the mask cannot strand a patrol whose start sits in a tight spot.
            frontiers = current_pos if i == 0 else self.scout.find_frontier_cells(
                graph, explored_value, unexplored_value, self.placeable)
            #file_path = f"D:\srinika\Research_Track\maps\cpp_house\goals\goal{i}.txt"

            #with open(file_path, 'r') as file:
                #frontiers = [tuple(map(int, line.strip().strip('()').split(', '))) for line in file]
            if i > 1:
                random.shuffle(frontiers)
            if frontier_drop_rate > 0:
                frontiers = [item for index, item in enumerate(frontiers) if index == 0 or (index + 1) % frontier_drop_rate == 0]
            if not frontiers:
                print("No more frontiers found. Stopping exploration.")
                break
            selected_frontier, area, updated_graph,ori = self.surveillance(iteration, frontiers, graph, total_area)
            total_area = area 
            iteration += 1 
            graph = updated_graph
            end_time = time.time()
            area_goals.append({'iteration': iteration, 'goals': {'goal': selected_frontier, 'area': area,'orientation':ori}, 'graph':graph, 'frontiers':frontiers})
            t_time += end_time - start_time
            
            print(f'steps: {iteration} goal : {selected_frontier,ori}   e_area : {int(area)} wp: {len(frontiers)} e_time: {int(end_time - start_time)} seconds t_time = {int(t_time)}  ' )
            
        return area_goals
    def optimize_goals(self,goal_points):
   
        points = [item['goals']['goal'] for item in goal_points]

        
        distance_matrix = squareform(pdist(points, 'euclidean'))

        
        def total_distance(path):
            return sum(distance_matrix[path[i], path[i+1]] for i in range(len(path) - 1))

        
        all_paths = permutations(range(1, len(points)))
        min_path = None
        min_distance = float('inf')

       
        for path in all_paths:
            current_path = (0,) + path + (0,)
            current_distance = total_distance(current_path)
            if current_distance < min_distance:
                min_distance = current_distance
                min_path = current_path

        min_path_points = [points[i] for i in min_path]

      
        coord_to_iter_index = {item['goals']['goal']: idx for idx, item in enumerate(goal_points)}

       
        ordered_goals = [goal_points[coord_to_iter_index[point]] for point in min_path_points[1:-1]]

        return min_path_points, min_distance, ordered_goals
