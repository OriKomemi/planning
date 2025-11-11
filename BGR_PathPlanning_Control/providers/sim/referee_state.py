from typing import Dict
import numpy as np
from providers.sim.sim_util import FSDSClientSingleton
from sub_modules.fsds.client import FSDSClient   # your existing AirSim wrapper
from sub_modules.fsds.utils import to_eularian_angles
from core.data.car_state import State
import logging

log = logging.getLogger("SimLogger")
class SimRefereeStateProvider:
    def __init__(self, is_global_frame: bool = True):
        self.client = FSDSClientSingleton.instance()
        self.is_global_frame = is_global_frame
        log.debug(f"Provider {self.__class__.__name__} initialized successfully")
        

    def start(self):  pass
    def stop(self):   pass

    def read(self) -> Dict[str, np.ndarray]:
        referee_state = self.client.getRefereeState()
        return {"referee_state": referee_state}
