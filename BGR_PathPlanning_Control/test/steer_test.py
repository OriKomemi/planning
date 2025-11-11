# from core.controllers import PurePursuitController
# from vehicle_config import Vehicle_config as conf
# from core.data.car_state import State

# import numpy as np

# # ─── Circle parameters ───────────────────────
# radius = 10.0                 # meters
# num_points = 100             # resolution
# center_x, center_y = 0.0, 0.0

# # ─── Generate path ───────────────────────────
# angles = np.linspace(0, 2 * np.pi, num_points)
# cx = center_x + radius * np.cos(angles)
# cy = center_y + radius * np.sin(angles)

# path_track = np.column_stack((np.arange(len(cx)), cx, cy))

# car_state = State(
#     x=0.0,  # Car's x position
#     y=0.0,  # Car's y position
#     yaw=0.0,  # Car's orientation in radians
#     v=5.0,  # Car's speed
# )

# steer_ctrl = PurePursuitController(
#         look_ahead_distance=conf.LOOKAHEAD_DISTANCE
#     )

# steering, target_ind = steer_ctrl.compute_steering(car_state, path_track, target_ind)
# curvature = curve[target_ind] if target_ind < len(curve) else curve[-1]

