"""
ndt_map_construction.py

Author: Perfect Cube
"""

# import path setting
import numpy as np
import sys
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

MAPS = {
    "map1": [
        {"x": 10.0, "y": 15.0, "length": 10, "width": 3},
        {"x": 40.0, "y": 0.0, "length": 2, "width": 10}
    ],
    "map2": [
        {"x": 10.0, "y": -10.0, "length": 5, "width": 5},
        {"x": 30.0, "y": 15.0, "length": 5, "width": 2}
    ],
    "map3": [
        {"x": 50.0, "y": 15.0, "length": 5, "width": 2},
        {"x": 25.0, "y": -10.0, "length": 2, "width": 6}
    ]
}

# flag to show plot figure
show_plot = True

def update_course(x_points, y_points, speed, obst_list):
    """
    Update course dynamically based on obstacle positions.
    """
    new_x, new_y = [], []
    
    for x, y in zip(x_points, y_points):
        for obs in obst_list.get_list():
            obs_x, obs_y = obs.state.x_m, obs.state.y_m
            distance = np.hypot(x - obs_x, y - obs_y)
            if distance < 5.0:  # If obstacle is too close, adjust path
                y += 9  # Shift y-coordinate to avoid obstacle
        new_x.append(x)
        new_y.append(y)

    return new_x, new_y, speed


def main():
    """
    Main process function
    """
    selected_map = sys.argv[1] if len(sys.argv) > 1 else "map1"

    # set simulation parameters
    x_lim, y_lim = MinMax(-5, 55), MinMax(-20, 25)
    vis = GlobalXYVisualizer(x_lim, y_lim, TimeParameters(span_sec=25), show_zoom=False)

    # initialize course parameters
    x_points = [0.0, 10.0, 25, 40, 50]
    y_points = [0.0, 4, -12, 20, -13]
    speed = 20

    # create obstacle instances
    obst_list = ObstacleList()
    for obs in MAPS[selected_map]:
        obst_list.add_obstacle(Obstacle(State(x_m=obs["x"], y_m=obs["y"]), length_m=obs["length"], width_m=obs["width"]))
    vis.add_object(obst_list)

    # update course dynamically
    x_points, y_points, speed = update_course(x_points, y_points, speed, obst_list)
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
