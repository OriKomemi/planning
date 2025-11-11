from typing import Dict
import numpy as np
from providers.sim.sim_util import FSDSClientSingleton
from sub_modules.fsds.utils import to_eularian_angles
from core.data.car_state import State
import logging

log = logging.getLogger("SimLogger")
class SimCarStateProvider:
    def __init__(self, is_global_frame: bool = True):
        self.client = FSDSClientSingleton.instance()
        self.is_global_frame = is_global_frame
        log.debug(f"Provider {self.__class__.__name__} initialized successfully")


    def start(self):  pass
    def stop(self):   pass

    def read(self) -> Dict[str, np.ndarray]:
        car_state = self.client.getCarState()
        referee_state = self.client.getRefereeState()

        orientation = car_state.kinematics_estimated.orientation
        _, _, yaw = to_eularian_angles(orientation)
        yaw = (yaw + np.pi) % (2 * np.pi) - np.pi #normalize_yaw
        v = car_state.speed if car_state.speed > 0 else 0

        state = State(
            x = car_state.kinematics_estimated.position.x_val if self.is_global_frame else 0,
            y = car_state.kinematics_estimated.position.y_val if self.is_global_frame else 0,
            yaw = yaw if self.is_global_frame else 0,
            v = v,
            v_linear = car_state.kinematics_estimated.linear_velocity if self.is_global_frame else np.array([0, 0, 0]),
            v_angular = car_state.kinematics_estimated.angular_velocity if self.is_global_frame else np.array([0, 0, 0]),
            a_angular = car_state.kinematics_estimated.angular_acceleration if self.is_global_frame else np.array([0, 0, 0]),
            a_linear = car_state.kinematics_estimated.linear_acceleration if self.is_global_frame else np.array([0, 0, 0]),
            timestamp = car_state.timestamp if self.is_global_frame else 0
            )
        return {"car_state": state, "referee_state": referee_state}   # keep as raw object for now
