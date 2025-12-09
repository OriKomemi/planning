import sys
import os
import time # \?
#nsure 'sub_modules' (project folder) is on sys.path so "from sub_modules import fsds" works

from pathlib import Path
current_file = Path(__file__).resolve()
for p in current_file.parents:
    if (p / "sub_modules").is_dir():
        sys.path.insert(0, str(p))
        break


from sub_modules import fsds
import numpy as np
import matplotlib.pyplot as plt
from racing_line.map import Map

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
    cones = referee_state.cones
    initial_position = referee_state.initial_position

    map = Map(initial_position, cones)
    map.plot_cones_by_color()

if __name__ == "__main__":
    main()
