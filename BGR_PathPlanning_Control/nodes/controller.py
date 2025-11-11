from typing import Dict
import numpy as np
import logging
from vehicle_config import Vehicle_config as conf
from core.controllers import AccelerationPIDController, PurePursuitController,StanleyController
from providers.sim.sim_util import sim_car_controls    # wrapper for AirSim command
from providers.sim.sim_util import FSDSClientSingleton
# from visualization import PlotManager
from core.data.car_state import State, States
import time
import math
# Get logger for this module
log = logging.getLogger("Controller")

class Controller:

    def __init__(self, simulation=False, is_global_frame=True, steering_controller="pure_pursuit"):
        log.info("Initializing Controller with PID and Pure Pursuit controllers")
        self.accel_ctrl = AccelerationPIDController(
            kp=conf.kp_accel,
            ki=conf.ki_accel,
            kd=conf.kd_accel,
            setpoint=conf.TARGET_SPEED
        )
        self.last_command = time.time()

        # Initialize steering controller based on user choice
        if steering_controller.lower() == "stanley":
            from core.controllers import StanleyController
            self.steer_ctrl = StanleyController(k=conf.k_stanley, k_soft=conf.k_soft_stanley)
            log.info("Using Stanley steering controller")
        elif steering_controller.lower() == "pure_pursuit":
            self.steer_ctrl = PurePursuitController()
            log.info("Using Pure Pursuit steering controller")
        else:
            log.warning(f"Unknown steering controller '{steering_controller}', defaulting to Pure Pursuit")
            self.steer_ctrl = PurePursuitController()

        self.steering_controller_type = steering_controller
        self.states = States()
        self.simulation = simulation


        self.first_time = True
        self.is_global_frame = is_global_frame


    def update(self, data: Dict):
        is_apply_controls = False
        if time.time() - self.last_command > 0.2:
            is_apply_controls = True
        try:
            path = data.get("path")
            cx = data.get("cx")
            cy = data.get("cy")
            curve = data.get("curve")
            car_state = data.get("car_state")
            referee_state = data.get("referee_state")
            target_ind = data.get("target_ind")
            gps_speed = data.get("gps_data")
            if gps_speed is not None:
                gps_speed = gps_speed.gps_speed # km/h
                gps_speed = gps_speed / 3.6 # convert to m/s



            if self.simulation:
                if any(x is None for x in [path, cx, cy, car_state]):
                    log.warning("Missing required data for controller update: "
                            f"path={path is not None}, cx={cx is not None}, "
                            f"cy={cy is not None}, car_state={car_state is not None}")
                    return
            # if not self.simulation:
                # create car state for gps velocity and 0,0 x y and 0 yaw because the world is coming to the car
            if not self.simulation:
                car_state = State(
                    x=0,
                    y=0,
                    yaw=0,
                    v=gps_speed)
            elif not self.is_global_frame:
                car_state = State(
                    x=0,
                    y=0,
                    yaw=0,
                    v=car_state.v)

            state_modifier = State(
                x=car_state.x,
                y=-car_state.y,
                yaw=car_state.yaw,
                v=car_state.v,
                v_linear= car_state.v_linear,
                v_angular= car_state.v_angular,
                a_angular= car_state.a_angular,
                a_linear= car_state.a_linear,
                timestamp=car_state.timestamp
            )
            data["car_state"] = state_modifier # dont delete this, expose to websocket exporter

            path_track = np.column_stack((np.arange(len(cx)), cx, cy))
            log.debug(f"Car state - Position: ({car_state.x:.2f}, {car_state.y:.2f}), "
                     f"Yaw: {car_state.yaw:.2f}, Velocity: {car_state.v:.2f}")

            # Calculate control inputs
            if self.steering_controller_type.lower() == "stanley":
                # Stanley controller needs dt parameter
                dt = 0.1  # or get from data if available
                steering, target_ind, lookahead = self.steer_ctrl.compute_steering(car_state, path_track, target_ind, dt)
            else:
                # Pure Pursuit controller
                steering, target_ind, lookahead = self.steer_ctrl.compute_steering(car_state, path_track, target_ind)

            # steering, target_ind = self.steer_ctrl.compute_steering(car_state, path_track, target_ind)
            curvature = curve[target_ind] if target_ind < len(curve) else curve[-1]
            acceleration = self.accel_ctrl.compute_acceleration(car_state.v, curvature)
            data["lookahead"] = lookahead  # Update lookahead distance in data
            current_time = data.get("current_time", 0)
            self.states.append(
                current_time,
                state_modifier,
                steering if steering else 0.0,
                acceleration if acceleration else 0.0,
                self.accel_ctrl.maxspeed if self.accel_ctrl.maxspeed else 0.0,
                v_linear= car_state.v_linear,
                v_angular=car_state.v_angular,
                a_linear=car_state.a_angular,
                a_angular=car_state.a_linear,
                timestamp=car_state.timestamp
                )
            data["states"] = self.states # expose to other nodes
            # Log control decisions
            log.debug(f"Control outputs - Steering: {steering:.3f}, Acceleration: {acceleration:.3f}, "
                     f"Target Index: {target_ind}, Curvature: {curvature:.3f}")

            # expose to other nodes
            data["v_log"] = self.accel_ctrl.maxspeed
            data["acceleration"] = acceleration
            data["steering"] = np.rad2deg(steering)


            # Apply controls and log the action
            log.info(f"Applying controls - Steering: {np.rad2deg(steering):.3f}, Acceleration: {acceleration:.3f}")
            if is_apply_controls:
                self.apply_controls(steering, acceleration)

        except Exception as e:
            log.error(f"Error in controller update: {str(e)}", exc_info=True)
            raise  # Re-raise the exception after logging

    def stop(self):
        log.info("Cleaning up Controller resources")
        if not self.simulation:
            pass
        else:
            self.apply_controls(0.0, -1.0,stopped_flag=True)  # Stop the vehicle in simulation
            

    def apply_controls(self, steering: float, acceleration: float, stopped_flag=False):
        """
        Apply control commands to the vehicle.
        """
        
        #if self.simulation:
        sim_car_controls(FSDSClientSingleton.instance(), -steering, acceleration, stopped_flag=stopped_flag)
            
            