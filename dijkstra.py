

# import path setting
import numpy as np
import sys
import heapq
from pathlib import Path

abs_dir_path = str(Path(__file__).absolute().parent)
relative_path = "/../../../components/"

sys.path.append(abs_dir_path + relative_path + "visualization")
sys.path.append(abs_dir_path + relative_path + "state")
sys.path.append(abs_dir_path + relative_path + "vehicle")
sys.path.append(abs_dir_path + relative_path + "obstacle")
sys.path.append(abs_dir_path + relative_path + "sensors")
sys.path.append(abs_dir_path + relative_path + "sensors/lidar")
sys.path.append(abs_dir_path + relative_path + "mapping/ndt")
sys.path.append(abs_dir_path + relative_path + "course/cubic_spline_course")
sys.path.append(abs_dir_path + relative_path + "control/pure_pursuit")

# import component modules
from global_xy_visualizer import GlobalXYVisualizer
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle
from obstacle_list import ObstacleList
from sensors import Sensors
from sensor_parameters import SensorParameters
from omni_directional_lidar import OmniDirectionalLidar
from ndt_global_mapper import NdtGlobalMapper
from cubic_spline_course import CubicSplineCourse
from pure_pursuit_controller import PurePursuitController

# flag to show plot figure
show_plot = True

def dijkstra(start, goal, obstacles, grid_size=1):
    """
    Implements Dijkstra's algorithm to find the shortest path while avoiding obstacles.
    """
    directions = [(-grid_size, 0), (grid_size, 0), (0, -grid_size), (0, grid_size)]
    queue = [(0, start)]
    visited = set()
    came_from = {}
    cost = {start: 0}
    
    while queue:
        current_cost, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in visited:
                continue
            
            if any(np.hypot(neighbor[0] - obs.state.x_m, neighbor[1] - obs.state.y_m) < 5 for obs in obstacles):
                continue  # Avoid obstacles
            
            new_cost = current_cost + np.hypot(dx, dy)
            if neighbor not in cost or new_cost < cost[neighbor]:
                cost[neighbor] = new_cost
                heapq.heappush(queue, (new_cost, neighbor))
                came_from[neighbor] = current
    
    return []  # No valid path found

def update_course(start, goal, speed, obst_list):
    """
    Compute new course using Dijkstra's algorithm.
    """
    obstacles = obst_list.get_list()
    path = dijkstra(start, goal, obstacles)
    
    if not path:
        return [], [], speed  # No valid path found
    
    x_points, y_points = zip(*path)
    return list(x_points), list(y_points), speed

def main():
    """
    Main process function
    """
    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False)

    # initialize start and goal points
    start = (0, 0)
    goal = (50, -13)
    speed = 20

    # create obstacle instances
    obst_list = ObstacleList()
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=15.0), length_m=10, width_m=8))
    obst_list.add_obstacle(Obstacle(State(x_m=40.0, y_m=0.0), length_m=2, width_m=10))
    obst_list.add_obstacle(Obstacle(State(x_m=10.0, y_m=-10.0, yaw_rad=np.rad2deg(45)), length_m=5, width_m=5))
    obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=15.0, yaw_rad=np.rad2deg(10)), length_m=5, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=50.0, y_m=15.0, yaw_rad=np.rad2deg(15)), length_m=5, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=0.0), length_m=2, width_m=2))
    obst_list.add_obstacle(Obstacle(State(x_m=35.0, y_m=-15.0), length_m=7, width_m=2))
    vis.add_object(obst_list)

    # update course dynamically using Dijkstra's algorithm
    x_points, y_points, speed = update_course(start, goal, speed, obst_list)
    course = CubicSplineCourse(x_points, y_points, speed)
    vis.add_object(course)

    # create vehicle instance
    spec = VehicleSpecification()
    pure_pursuit = PurePursuitController(spec, course)
    sensor_params = SensorParameters(lon_m=spec.wheel_base_m/2, max_m=15, dist_std_rate=0.05)
    lidar = OmniDirectionalLidar(obst_list, sensor_params)
    mapper = NdtGlobalMapper(sensor_params=sensor_params, center_x_m=25.0, center_y_m=5.0)
    vehicle = FourWheelsVehicle(State(color=spec.color), spec,
                                controller=pure_pursuit,
                                sensors=Sensors(lidar=lidar),
                                mapper=mapper,
                                show_zoom=False)
    vis.add_object(vehicle)

    # plot figure is not shown when executed as unit test
    if not show_plot:
        vis.not_show_plot()

    # show plot figure
    vis.draw()

# execute main process
if __name__ == "__main__":
    main()
