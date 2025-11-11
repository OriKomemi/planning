import sys
import pathlib

sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent.parent))
print(sys.path)
from core.utils.vcu import VCU

import time

# orig
# vcu_steering = VCU(port_name="/dev/ttyACM0")
# vcu_tourqe = VCU(port_name="/dev/ttyACM1")
TIME_SLEEP = 0.2
wheel = 0
vcu_steering = VCU(port_name="/dev/ttyACM0")
# vcu_tourqe = VCU(port_name="/dev/ttyACM1")
time.sleep(0.5)  # Reduced delay for VCU initialization
vcu_steering.set_angle(float(0))
for i in range(30):
    for i in range(8):
        wheel -= 10.0    
        vcu_steering.set_angle(wheel)
        time.sleep(TIME_SLEEP)  # Reduced delay for each angle change
    for i in range(16):
        wheel += 10.0
        vcu_steering.set_angle(wheel)
        time.sleep(TIME_SLEEP)  # Reduced delay for each angle change
    for i in range(8):
        wheel -= 10.0    
        vcu_steering.set_angle(wheel)
        time.sleep(TIME_SLEEP)  # Reduced delay for each angle change
# for i in range(30):
#     vcu_steering.set_angle(i*-1.0)
#     time.sleep(TIME_SLEEP)  # Allow time for VCU to stabilize
#     vcu_steering.set_angle(i*1.0)
#     time.sleep(TIME_SLEEP)  # Allow time for VCU to stabilize
# time.sleep(TIME_SLEEP)  # Allow time for VCU to stabilize
# vcu_steering.set_angle(float(-70))
# time.sleep(TIME_SLEEP)  # Allow time for VCU to stabilize
# vcu_steering.set_angle(float(70))

time.sleep(TIME_SLEEP)  # Allow time for VCU to stabilize
vcu_steering.set_angle(float(0))
    
