from MAP import Map_generator
import yaml
from Scout_Multi_Processing import Exploration
from path_generator import calculate_and_plot_path
from path_optimizer import WayPointOptimizer
from config import config 
import os
def main():
    map_generator = Map_generator()

    starting_position = config['starting_position']
    explored_value = config['explored_value']
    unexplored_value = config['unexplored_value']
    state = config['state']
    steps = config['steps']
    s_range = config['surveillance_range']
    WPD = config['way_point_dropout']
    output_dir = config['output_dir']
    pgm_filename = config['pgm_filename']
    yaml_filename = config['yaml_filename']
    file_path = os.path.join(output_dir,'wp_ori_data.txt') #output file path to save optimized waypoints
    grid_map = map_generator.load_pgm(pgm_filename)
    yaml_data = map_generator.load_yaml(yaml_filename)
    
    print(f'total free space: {int(map_generator.estimate_area(grid_map,yaml_data,unexplored_value))} sq.mtrs')

    #map_generator.plot_rgb_map(grid_map)

    explore = Exploration(grid_map, s_range, unexplored_value, state, yaml_data) #intializes the  way point generation object

    goals = explore.set_goals(starting_position, explored_value, unexplored_value, steps,WPD) # waypoints generation returns list of waypoints(cor_x, cor_y) and orientation theta
    #map_generator.plot_iterations(goals)
    wp = []
    ori = []
    for item in goals:
        wp.append(item['goals']['goal'])
        ori.append(item['goals']['orientation'])

    # Last iteration's graph is the union of every waypoint's FOV, i.e. the coverage map
    coverage_grid = goals[-1]['graph']

    t_c_rot = sum(ori) # total cumulative rotations
    resolution = yaml_data['resolution'] # resolution of grid cell
    optimizer = WayPointOptimizer(wp, 0.3, 5) # initializes waypoint optimization object
    order = optimizer.run() # visiting order as indices into wp
    best_wp = [wp[i] for i in order]
    best_ori = [ori[i] for i in order]
    with open(file_path, 'w') as f:
        f.write(f"wp = {best_wp}\n")
        f.write(f"ori = {best_ori}\n")

    coverage = map_generator.report_coverage(grid_map, coverage_grid, yaml_data, output_dir)

    metrics = calculate_and_plot_path(grid_map,wp,best_wp,resolution,t_c_rot,output_dir,coverage=coverage)
    print(metrics)
    
if __name__ == '__main__':
    main()
