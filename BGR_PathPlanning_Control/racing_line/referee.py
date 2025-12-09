import sys
import os
import time # ?
# Ensure 'sub_modules' (project folder) is on sys.path so "from sub_modules import fsds" works
from pathlib import Path
current_file = Path(__file__).resolve()
for p in current_file.parents:
    if (p / "sub_modules").is_dir():
        sys.path.insert(0, str(p))
        break

from sub_modules import fsds
import numpy as np
import matplotlib.pyplot as plt
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes
from line import Line

print("finished imports")

def main():
    # you have to load/get the data, this is just an example
    # global_cones, car_position, car_direction = load_data() 
    # global_cones is a sequence that contains 5 numpy arrays with shape (N, 2),
    # where N is the number of cones of that type

    # connect to the AirSim simulator 
    client = fsds.FSDSClient()

    # Check network connection, exit if not connected
    client.confirmConnection()

    # After enabling api controll only the api can controll the car. 
    # Direct keyboard and joystick into the simulator are disabled.
    client.enableApiControl(True)


    # This is information that in a physical world would be relevant to the referee. 
    # Currently, this includes a list of down or out cones and the timestamp of when they went down or out, 
    # a list of lap times and a list of when the car went off-track.
    referee_state = client.getRefereeState()

    cones_by_colors = {0: [], 1: [], 2: [], 3: [], 4: []}
    cones = referee_state.cones
    initial_position = referee_state.initial_position
    racing_line = Line(initial_position=(initial_position.x/100.0, initial_position.y/100.0), epsilon=0.75)

    print("finished initializing")

    path_planner = PathPlanner(MissionTypes.trackdrive)
    print("path_planner")

    # Build cone lists by color
    color_map = {0: "k", 1: "gold", 2: "royalblue", 3: "peru", 4: "darkorange"}
    label_map = {
        0: "Unknown",
        1: "Right (yellow)",
        2: "Left (blue)",
        3: "Orange small",
        4: "Orange big",        
    }
    
    for cone in cones:
        # Support both dict-like and attr-like cone objects
        color = cone["color"] # a numner from 0 to 4 probably
        x = cone["x"] 
        y = cone["y"] 

        # Make position relative to initial_position and convert cm -> m
        px = (x) / 100.0
        py = (y) / 100.0
        cones_by_colors[color].append((px, py))
        
    # print(f"cones_by_colors[0]: {cones_by_colors[0]}")

    # Plot cones by color
    fig, ax = plt.subplots()
    for color_number, points in cones_by_colors.items():
        if not points:
            continue
        points = np.array(points)
        ax.scatter(points[:, 0], points[:, 1], s=20, c=color_map[color_number], label=label_map[color_number])

    # Prepare cones by type for path planning
    # cones_by_colors - the cones organized by color(5 colors = 5 lists)
    # cones_by_type - the cones organized by ConeTypes (5 types = 5 lists)
    cones_by_type = [np.zeros((0, 2)) for _ in range(len(ConeTypes))] # the outer list has 5 elements, one for each ConeTypes, then each element is an array of shape (N, 2)
    cones_by_type[ConeTypes.LEFT] = np.array(cones_by_colors[ConeTypes.BLUE])
    cones_by_type[ConeTypes.RIGHT] = np.array(cones_by_colors[ConeTypes.YELLOW])
    cones_by_type[ConeTypes.START_FINISH_LINE] = np.array(cones_by_colors[ConeTypes.ORANGE_BIG]) 
    cones_by_type[ConeTypes.START_FINISH_AREA] = np.array(cones_by_colors[ConeTypes.ORANGE_SMALL])

    # car_position = np.array([initial_position.x/100.0, initial_position.y/100.0]) # cm -> m
    # car_direction = np.array([1.0, 0.0])  # Assuming facing along positive X axis
    # path = path_planner.calculate_path_in_global_frame(cones_by_type, car_position, car_direction) 
    # print(f"Calculated path with {path}.")
    # change the car position to be on the quarter of the path array
    # quarter_index = len(path) // 4
    # car_position = np.array([path[quarter_index, 1], path[quarter_index, 2]])
    # racing_line.add_stage(path[:, 1:3])  # Add only the (x, y) coordinates to the racing line

    x_point = initial_position.x/100.0
    y_point = initial_position.y/100.0
    car_position = np.array([x_point, y_point])
    car_direction = np.array([1.0, 0.0])
    i = 0
    while(not racing_line.did_end_lap(car_position) or i < 100):
        i += 1        
        path = path_planner.calculate_path_in_global_frame(cones_by_type, car_position, car_direction) 

        # add stage
        racing_line.add_stage(path[:, 1:3])  # Add only the (x, y) coordinates to the racing line
        
        # update position and direction for next iteration
        quarter_index = 2
        x_point = path[quarter_index, 1]
        y_point = path[quarter_index, 2]
        car_position = np.array([x_point, y_point])

        #update direction
        dx = path[quarter_index + 1, 1] - path[quarter_index, 1]
        dy = path[quarter_index + 1, 2] - path[quarter_index, 2] 
        theta = np.arctan2(dy, dx)
        car_direction = np.array([np.cos(theta), np.sin(theta)])        


    # Extract path coordinates
    # cx, cy = path[:, 1], path[:, 2]
    # curve = path[:, 3]
    racing_line.cut_end()
    racing_line.set_path(racing_line.smooth_path())
    cx, cy = racing_line.get_path()[:, 0], racing_line.get_path()[:, 1]
    

    ax.scatter(initial_position.x/100.0, initial_position.y/100.0, s=30, c="deeppink", label="speedy meCwin", marker='^')
    # Plot planned path as points instead of a connecting line
    ax.scatter(cx, cy, s=25, c="red", marker='o', label="Planned path (points)")
    ax.set_xlabel("X [m] (relative to start)")
    ax.set_ylabel("Y [m] (relative to start)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    ax.grid(True)
    ax.set_title("Referee cones")
    plt.show()


if __name__ == "__main__":
    main()
