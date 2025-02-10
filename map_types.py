import numpy as np
import random

def generate_obstacles(map_choice, vis):
    """
    Generate obstacles based on the selected map choice (1-10).
    """
    obst_list = ObstacleList()
    
    if map_choice == 1:
        # Simple Two Obstacles
        obst_list.add_obstacle(Obstacle(State(x_m=20.0, y_m=5.0), length_m=5, width_m=2))
        obst_list.add_obstacle(Obstacle(State(x_m=30.0, y_m=-5.0), length_m=5, width_m=2))
    
    elif map_choice == 2:
        # Four Corner Obstacles
        for x, y in [(10, 10), (10, -10), (40, 10), (40, -10)]:
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=y), length_m=4, width_m=4))
    
    elif map_choice == 3:
        # Central Blockage
        obst_list.add_obstacle(Obstacle(State(x_m=25.0, y_m=0.0), length_m=10, width_m=10))
    
    elif map_choice == 4:
        # Narrow Passage
        for x in [20, 22, 40, 42]:
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=5), length_m=6, width_m=2))
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=-5), length_m=6, width_m=2))
    
    elif map_choice == 5:
        # Zig-Zag Path
        for i in range(5, 50, 10):
            obst_list.add_obstacle(Obstacle(State(x_m=i, y_m=(-1)**(i//10) * 5), length_m=3, width_m=3))
    
    elif map_choice == 6:
        # Random Scattered Obstacles
        for _ in range(10):
            x = random.uniform(5, 50)
            y = random.uniform(-15, 15)
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=y), length_m=4, width_m=2))
    
    elif map_choice == 7:
        # Parallel Walls
        for x in range(5, 50, 10):
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=10), length_m=3, width_m=1))
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=-10), length_m=3, width_m=1))
    
    elif map_choice == 8:
        # Circular Cluster
        for angle in range(0, 360, 45):
            x = 25 + 8 * np.cos(np.radians(angle))
            y = 0 + 8 * np.sin(np.radians(angle))
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=y), length_m=3, width_m=3))
    
    elif map_choice == 9:
        # U-Turn Challenge
        for x, y in [(20, 0), (40, 0), (30, 10)]:
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=y), length_m=5, width_m=10))
    
    elif map_choice == 10:
        # Complex City-like Environment
        for x in range(10, 50, 10):
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=5), length_m=2, width_m=10))
            obst_list.add_obstacle(Obstacle(State(x_m=x, y_m=-5), length_m=2, width_m=10))
        for y in range(-10, 15, 10):
            obst_list.add_obstacle(Obstacle(State(x_m=30, y_m=y), length_m=10, width_m=2))
    
    else:
        print("Invalid map choice! Choose between 1 and 10.")
        return
    
    vis.add_object(obst_list)
    return obst_list
