#!/usr/bin/env python3
"""
Run either a simulation or calibration session with a pluggable provider/
nodes pipeline.  Use --help for options.
"""
import sys
import os
import time

# Add project root to sys.path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, project_root)

import argparse
import logging
from core.manager import Manager
from providers.sim.car_state import SimCarStateProvider
from providers.sim.cones     import SimConeProvider
from providers.sim.map_data  import SimMapProvider
from providers.sim.referee_state import SimRefereeStateProvider
from providers.camera.cam import CameraProvider

# Swap these two for ROS2 later:
# from providers.ros.car_state import ROSCarStateProvider
# from providers.ros.cones     import ROSConeProvider
# from providers.ros.map_data  import ROSMapProvider
from nodes.planner     import Planner
from nodes.controller  import Controller
from core.logger import init_logger


def cli() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Modular path-planning & control framework")
    p.add_argument("--mode",    choices=["simulation", "calibration"],
                   default="simulation")
    p.add_argument("--plot",    action="store_true",
                   help="Enable live plots / post-run analysis")
    p.add_argument("--dt", type=float, default=0.05,
                   help="Main-loop period [s]")
    p.add_argument("--output_dir", type=str, default="output",
                   help="Output directory for plots and data")
    p.add_argument("--simulation", action="store_true",)
    p.add_argument('--steering-controller',choices=['pure_pursuit', 'stanley'], default='pure_pursuit',
                   help='Choose steering controller type')
    p.add_argument("--global_ref", action="store_true",
                   help="global refrence frame for path planning or car reference frame")

    return p.parse_args()

def build_manager(args):
    # Providers (simulation flavour)
    if args.simulation:
        providers = [
            SimCarStateProvider(is_global_frame=args.global_ref),      # vehicle pose & twist
            SimConeProvider(is_global_frame=args.global_ref),          # simulated cones
            # SimRefereeStateProvider(is_global_frame=args.global_ref),  # referee state (Optional)
        ]
        if args.global_ref:
            providers.append(SimMapProvider())
    else:
        providers = [
            # SimCarStateProvider(),      # vehicle pose & twist
            # LidarConeRecordingProvider(recording_path="/home/bgr/Desktop/competition/Recording_2025_05_24_17_27_12/",start_frame_num=0),  # LiDAR-derived track cones from recording

            CameraProvider(),      # Camera provider for video streaming
        ]
    providers = [
        SimCarStateProvider(is_global_frame=args.global_ref),      # vehicle pose & twist
        SimConeProvider(is_global_frame=args.global_ref),          # simulated cones
        # SimRefereeStateProvider(is_global_frame=args.global_ref),  # referee state (Optional)
    ]
    nodes = [
        Planner(),               # produces path based on cones & map
        Controller(simulation=args.simulation,is_global_frame=args.global_ref,steering_controller=args.steering_controller)             # produces throttle / steer commands
    
    ]
    return Manager(providers, nodes, dt=args.dt, enable_plots=args.plot, output_dir=args.output_dir, simulation=args.simulation, exporter=None)

def main():
    args = cli()
    uid = time.strftime("%Y-%m-%d_%H-%M-%S")
    args.output_dir = args.output_dir + f"/{uid}"
    init_logger(f"output/{uid}/logs")


    mgr = build_manager(args)

    if args.mode == "calibration":
        mgr.run_calibration()
    else:
        mgr.run()

if __name__ == "__main__":
    main()
