# control.py

import numpy as np
import math
import logging
from collections import deque
import cvxpy as cp  # For MPC
from simple_pid import PID
from vehicle_config import Vehicle_config as conf
import control as ctrl
from fsd_path_planning import ConeTypes
# from visualization import Visualizer

logger = logging.getLogger('SimLogger')
# PID Controller for Acceleration
class AccelerationPIDController:
    def __init__(self, kp, ki, kd, setpoint):
        """
        PID Controller for acceleration (throttle control).

        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
            setpoint (float): Desired speed [m/s].
        """
        self.pid = PID(kp, ki, kd, setpoint)
        self.maxspeed = setpoint

    def compute_acceleration(self, current_speed, curvature=0.0):
        """
        Compute acceleration (throttle input) based on current speed.

        Args:
            current_speed (float): Current speed of the vehicle [m/s].

        Returns:
            float: Throttle input [0,1].
        """
        desired_v = self.compute_breaking(curvature)
        self.maxspeed = desired_v
        self.pid.setpoint = self.maxspeed
        acceleration = self.pid(current_speed)
        acceleration = acceleration/ conf.MAX_SPEED  # Normalize to [0, 1]
        acceleration = np.clip(acceleration, conf.MAX_DECEL, conf.MAX_ACCEL)
        return acceleration

    def compute_breaking_old(self, curve, target_ind):
        # Logistic decay
        b = 8 # Sensitivity parameter
        c = 1 # Power parameter
        v_logistic = self.maxspeed / (1 + b * np.abs(curve)**c)
        velocity = v_logistic[target_ind]
        self._update_desired_speed(velocity)
        return velocity

    def compute_breaking(self, curvature):
        """
        Compute the desired speed based on curvature.

        Args:
            curvature (float): Curvature at the target point.

        Returns:
            float: Desired speed [m/s].
        """
        # Define the relationship between curvature and desired speed
        a = conf.k_expo       # Sensitivity parameter for exponential decay

        max_speed = conf.TARGET_SPEED  # Maximum desired speed
        v_desired = max_speed * np.exp(-a * np.abs(curvature))
        return v_desired

    def _update_desired_speed(self, velocity):
        self.pid.setpoint = velocity


class PurePursuitController:
    """
    Pure Pursuit steering controller with aggressive look-ahead shrink
    for hairpins.

    Parameters
    ----------
    L_min : float
        Absolute lower bound for look-ahead [m].
    L_max : float
        Absolute upper bound for look-ahead [m].
    k_v : float
        Speed scale (≈ time gap) so Lf ≈ k_v · v  [s].
    k_kappa : float
        Curvature gain – bigger ⇒ shorter Lf in bends.
    kappa_window : int
        How many future path indices to scan for *maximum* curvature.
    tau : float
        Low-pass filter time constant for Lf [s].
    """

    def __init__(self,
                 L_min        = conf.L_min,     # smaller floor for hairpins
                 L_max        = conf.L_max,     # larger ceiling for straights
                 k_v          = conf.k_v,          # speed scale (≈ time gap) so Lf ≈ k_v · v  [s]
                 k_kappa      = conf.k_kappa,     # stronger curvature penalty
                 kappa_window = conf.kappa_window,       # look ~1 car-length ahead (assuming 0.2 m spacing)
                 tau          = conf.tau):   # faster response

        self.L_min        = L_min
        self.L_max        = L_max
        self.k_v          = k_v
        self.k_kappa      = k_kappa
        self.kappa_window = kappa_window
        self.tau          = tau

        self._Lf = L_min
        self._curv_hist = deque(maxlen=3)

    # --------------------------------------------------------------
    # -------- curvature helpers -----------------------------------
    @staticmethod
    def _curvature_three_point(xm, ym, xp, yp, xn, yn):
        """Unsigned curvature from three consecutive points."""
        # first and second derivatives (central difference)
        dx  = (xp - xm) * 0.5
        dy  = (yp - ym) * 0.5
        ddx = xn - xp
        ddy = yn - yp
        denom = (dx*dx + dy*dy)**1.5
        if denom < 1e-9:
            return 0.0
        return abs(dx*ddy - dy*ddx) / denom

    def _max_curvature_ahead(self, path, start_idx):
        """Return max |κ| in the next kappa_window points (inclusive of start)."""
        n = len(path)
        print(f"Path length: {n}, start_idx: {start_idx}, kappa_window: {self.kappa_window}")
        end = min(start_idx + self.kappa_window, n - 2)   # leave room for +1
        max_kappa = 0.0
        for i in range(max(1, start_idx), end):
            xm, ym = path[i-1, 1], path[i-1, 2]
            xp, yp = path[i  , 1], path[i  , 2]
            xn, yn = path[i+1, 1], path[i+1, 2]
            max_kappa = max(max_kappa,
                            self._curvature_three_point(xm, ym, xp, yp, xn, yn))
        return max_kappa

    # --------------------------------------------------------------
    # -------- look-ahead computation ------------------------------
    def _dynamic_Lf(self, speed, kappa, dt):
        # curvature shrink term
        # shrink = self.k_kappa / (kappa**1.2 + 1e-3)
        shrink = self.k_kappa / (kappa + 1e-3)
        # raw look-ahead
        raw = self.L_min + self.k_v * speed + shrink

        # hard clamp
        raw = max(self.L_min, min(raw, self.L_max))

        # 1-pole low-pass filter
        alpha = dt / (self.tau + dt) if dt else 1.0
        self._Lf = (1.0 - alpha) * self._Lf + alpha * raw
        return self._Lf

    # --------------------------------------------------------------
    def compute_steering(self, state, path, target_ind):
        # ------- quick access to columns ---------------------------
        cx, cy = path[:, 1], path[:, 2]
        idx    = target_ind
        speed  = state.v

        # ------- curvature (max ahead) ----------------------------
        kappa = self._max_curvature_ahead(path, idx)
        self._curv_hist.append(kappa)
        kappa = np.mean(self._curv_hist)  # mild smoothing

        # ------- time step (optional) -----------------------------
        dt = getattr(state, 'dt', 0.02)   # 50 Hz fallback

        # ------- compute look-ahead -------------------------------
        Lf = self._dynamic_Lf(speed, kappa, dt)
        # Lf=5.0
        logger.info(f"Dynamic look-ahead: {Lf:.2f} m, speed: {speed:.2f} m/s, ")
        # ------- find look-ahead target ---------------------------
        while idx < len(cx):
            if math.hypot(cx[idx] - state.x, cy[idx] - state.y) >= Lf:
                break
            idx += 1
        idx = min(idx, len(cx) - 1)


        # ------- steering geometry --------------------------------
        tx, ty = cx[idx], cy[idx]
        alpha  = math.atan2(ty - state.y, tx - state.x) - state.yaw
        alpha  = (alpha + math.pi) % (2.0 * math.pi) - math.pi
        bias = -math.pi / 50
        bias = 0
        steering = math.atan2(2.0 * conf.WB * math.sin(alpha + bias * np.sign(-math.sin(alpha))), Lf)
        return float(np.clip(steering, -conf.MAX_STEER, conf.MAX_STEER)), idx, Lf

# Stanley Controller
# class StanleyController: #old
#     def __init__(self, k=conf.k_stanley, k_soft=conf.k_soft_stanley, max_steer_rate=np.deg2rad(30), alpha=0.5, lookahead_distance=0.0):
#         """
#         Improved Stanley Steering Controller.

#         Args:
#             k (float): Proportional gain for the cross-track error.
#             k_soft (float): Softening constant to avoid division by zero.
#             max_steer_rate (float): Maximum rate of change of the steering angle [rad/s].
#             alpha (float): Smoothing factor for steering command (0 < alpha <= 1).
#             lookahead_distance (float): Optional look-ahead distance [m].
#         """
#         self.k = k
#         self.k_soft = k_soft
#         self.max_steer_rate = max_steer_rate
#         self.alpha = alpha  # For low-pass filtering
#         self.lookahead_distance = lookahead_distance
#         self.previous_steering = 0.0  # Initialize previous steering angle

#     def compute_steering(self, state, path, target_ind, dt):
#         """
#         Compute steering angle using the improved Stanley control algorithm.

#         Args:
#             state (State): Current state of the vehicle.
#             path (numpy.ndarray): Path coordinates [[index, x1, y1], [index, x2, y2], ...].
#             target_ind (int): Current target index on the path.
#             dt (float): Time step [s].

#         Returns:
#             float: Steering angle [rad].
#             int: Updated target index.
#         """
#         cx = path[:, 1]
#         cy = path[:, 2]

#         # Find the nearest path point (with optional look-ahead)
#         dx = cx - state.x
#         dy = cy - state.y
#         d = np.hypot(dx, dy)
#         target_ind = np.argmin(d)

#         # Apply look-ahead if specified
#         if self.lookahead_distance > 0:
#             while target_ind < len(cx) - 1:
#                 distance = np.hypot(cx[target_ind] - state.x, cy[target_ind] - state.y)
#                 if distance >= self.lookahead_distance:
#                     break
#                 target_ind += 1

#         nearest_x = cx[target_ind]
#         nearest_y = cy[target_ind]

#         # Path heading at the nearest point
#         if target_ind < len(cx) - 1:
#             path_dx = cx[target_ind + 1] - cx[target_ind]
#             path_dy = cy[target_ind + 1] - cy[target_ind]
#         else:
#             path_dx = cx[target_ind] - cx[target_ind - 1]
#             path_dy = cy[target_ind] - cy[target_ind - 1]

#         path_yaw = math.atan2(path_dy, path_dx)

#         # Heading error
#         theta_e = normalize_angle(path_yaw - state.yaw)

#         # Cross-track error using lateral error projection
#         dx = nearest_x - state.x
#         dy = nearest_y - state.y
#         e_ct = -dx * np.sin(path_yaw) + dy * np.cos(path_yaw)

#         # Adaptive gain scheduling based on speed
#         adaptive_k = self.k / (state.v + self.k_soft)

#         # Steering control law
#         steering = theta_e + math.atan2(adaptive_k * e_ct, 1.0)
#         steering = normalize_angle(steering)

#         # Limit steering rate
#         max_delta = self.max_steer_rate * dt
#         steering = np.clip(steering, self.previous_steering - max_delta, self.previous_steering + max_delta)

#         # Low-pass filter steering angle
#         steering = self.alpha * steering + (1 - self.alpha) * self.previous_steering

#         # Update previous steering angle
#         self.previous_steering = steering

#         # Clip to maximum steering angle
#         steering = np.clip(steering, -conf.MAX_STEER, conf.MAX_STEER)

#         return steering, target_ind
    # def __init__(self, k=1.0, k_soft=0.1):
    #     """
    #     Stanley Steering Controller.

    #     Args:
    #         k (float): Gain for the cross-track error.
    #         k_soft (float): Softening constant to avoid division by zero.
    #     """
    #     self.k = k
    #     self.k_soft = k_soft

    # def compute_steering1(self, state, path, target_ind):
    #     """
    #     Compute steering angle using Stanley control algorithm.

    #     Args:
    #         state (State): Current state of the vehicle.
    #         path (numpy.ndarray): Path coordinates [[index, x1, y1], [index, x2, y2], ...].
    #         target_ind (int): Current target index on the path.

    #     Returns:
    #         float: Steering angle [rad].
    #         int: Updated target index.
    #     """
    #     cx = path[:, 1]
    #     cy = path[:, 2]

    #     # Find the nearest path point
    #     dx = cx - state.x
    #     dy = cy - state.y
    #     d = np.hypot(dx, dy)
    #     target_ind = np.argmin(d)
    #     nearest_x = cx[target_ind]
    #     nearest_y = cy[target_ind]

    #     # Path heading at the nearest point
    #     if target_ind < len(cx) - 1:
    #         path_dx = cx[target_ind + 1] - cx[target_ind]
    #         path_dy = cy[target_ind + 1] - cy[target_ind]
    #     else:
    #         path_dx = cx[target_ind] - cx[target_ind - 1]
    #         path_dy = cy[target_ind] - cy[target_ind - 1]

    #     path_yaw = math.atan2(path_dy, path_dx)

    #     # Heading error
    #     theta_e = normalize_angle(path_yaw - state.yaw)

    #     # Cross-track error
    #     dx = nearest_x - state.x
    #     dy = nearest_y - state.y
    #     e_ct = -dx * np.sin(path_yaw) + dy * np.cos(path_yaw)

    #     # Steering control law
    #     steering = theta_e + math.atan2(self.k * e_ct, state.v + self.k_soft)
    #     steering = normalize_angle(steering)
    #     steering = np.clip(steering, -conf.MAX_STEER, conf.MAX_STEER)
    #     # Visualizer.cross_track_error(e_ct,path,cx,cy,theta_e)


    #     return steering, target_ind

class StanleyController:
    def __init__(self, k=1.5, k_soft=1.0, max_steer_rate=np.deg2rad(45), alpha=0.8):
        """
        Stanley Steering Controller - Improved Implementation.

        Args:
            k (float): Cross-track error gain (higher = more aggressive correction)
            k_soft (float): Speed normalization constant (prevents division by zero)
            max_steer_rate (float): Maximum steering rate change [rad/s]
            alpha (float): Smoothing factor for steering (0 < alpha <= 1)
        """
        self.k = k
        self.k_soft = k_soft
        self.max_steer_rate = max_steer_rate
        self.alpha = alpha
        self.previous_steering = 0.0

    def compute_steering(self, state, path, target_ind, dt):
        """
        Compute steering using Stanley control law.
        """
        cx = path[:, 1]
        cy = path[:, 2]

        # Ensure target_ind is within bounds
        target_ind = max(0, min(target_ind, len(cx) - 1))

        # 1. Find closest point on path
        distances = np.sqrt((cx - state.x)**2 + (cy - state.y)**2)
        closest_ind = np.argmin(distances)

        # 2. Use closest point or advance based on speed
        lookahead_dist = max(2.0, state.v * 0.5)  # Dynamic lookahead

        # Search forward from closest point
        search_ind = closest_ind
        while search_ind < len(cx) - 1:
            dist = np.sqrt((cx[search_ind] - state.x)**2 + (cy[search_ind] - state.y)**2)
            if dist >= lookahead_dist:
                break
            search_ind += 1

        target_ind = search_ind

        # 3. Calculate path heading using multiple points for smoothness
        if target_ind < len(cx) - 2:
            # Use 2-point ahead for better smoothness
            path_dx = cx[target_ind + 2] - cx[target_ind]
            path_dy = cy[target_ind + 2] - cy[target_ind]
        elif target_ind < len(cx) - 1:
            path_dx = cx[target_ind + 1] - cx[target_ind]
            path_dy = cy[target_ind + 1] - cy[target_ind]
        else:
            path_dx = cx[target_ind] - cx[target_ind - 1]
            path_dy = cy[target_ind] - cy[target_ind - 1]

        path_yaw = math.atan2(path_dy, path_dx)

        # 4. Heading error
        heading_error = normalize_angle(path_yaw - state.yaw)

        # 5. Cross-track error (distance from vehicle to path)
        # Vector from vehicle to target point
        dx = cx[target_ind] - state.x
        dy = cy[target_ind] - state.y

        # Project onto path normal (positive = left of path)
        cross_track_error = dx * math.sin(path_yaw) - dy * math.cos(path_yaw)

        # 6. Stanley control law
        speed = max(0.1, state.v)  # Avoid division by zero
        cross_track_term = math.atan2(self.k * cross_track_error, speed + self.k_soft)

        # Total steering command
        steering = heading_error + cross_track_term
        steering = normalize_angle(steering)

        # 7. Rate limiting
        if dt > 0:
            max_delta = self.max_steer_rate * dt
            steering = np.clip(steering,
                             self.previous_steering - max_delta,
                             self.previous_steering + max_delta)

        # 8. Smoothing filter
        steering = self.alpha * steering + (1 - self.alpha) * self.previous_steering

        # 9. Final limits
        steering = np.clip(steering, -conf.MAX_STEER, conf.MAX_STEER)

        # Update previous steering
        self.previous_steering = steering

        return steering, target_ind, lookahead_dist

class MPCController:
    def __init__(self, N=10, dt=0.1):
        """
        Model Predictive Controller using a linearized vehicle model with fixed speed.

        Args:
            N (int): Prediction horizon.
            dt (float): Time step size [s].
        """
        self.N = N  # Prediction horizon
        self.dt = dt  # Time step
        # Define weights for the cost function
        self.Q = np.diag([1.0, 1.0, 0.5, 0.1])  # State weighting matrix
        self.R = np.diag([0.1, 0.1])  # Control weighting matrix

    def compute_control(self, state, path):
        """
        Compute the optimal control inputs using MPC with a fixed speed.

        Args:
            state (State): Current state of the vehicle.
            path (numpy.ndarray): Path coordinates [[index, x1, y1], [index, x2, y2], ...].

        Returns:
            float: Acceleration [m/s^2].
            float: Steering angle [rad].
        """
        # Extract reference trajectory
        ref_x = path[:, 1]
        ref_y = path[:, 2]

        # Define optimization variables
        x = cp.Variable((self.N + 1, 4))  # States: [x, y, yaw, v]
        u = cp.Variable((self.N, 2))      # Controls: [acceleration, steering]

        # Define constraints and cost
        constraints = []
        cost = 0

        # Initial condition
        constraints += [x[0, :] == [state.x, state.y, state.yaw, state.v]]

        # Precompute cos and sin of the current yaw angle
        cos_theta = np.cos(state.yaw)
        sin_theta = np.sin(state.yaw)

        # Fix the speed to current value
        v_current = state.v

        for k in range(self.N):
            # System dynamics with fixed speed
            constraints += [
                x[k + 1, 0] == x[k, 0] + v_current * cos_theta * self.dt,
                x[k + 1, 1] == x[k, 1] + v_current * sin_theta * self.dt,
                x[k + 1, 2] == x[k, 2] + v_current / conf.WB * u[k, 1] * self.dt,
                x[k + 1, 3] == x[k, 3] + u[k, 0] * self.dt
            ]

            # Control constraints
            constraints += [
                cp.abs(u[k, 0]) <= conf.MAX_ACCEL,
                cp.abs(u[k, 1]) <= conf.MAX_STEER
            ]

            # State constraints (optional)
            constraints += [x[k + 1, 3] >= 0]  # Speed cannot be negative

            # Reference tracking
            if k < len(ref_x):
                ref_state = np.array([ref_x[k], ref_y[k], 0.0, conf.TARGET_SPEED])
            else:
                ref_state = np.array([ref_x[-1], ref_y[-1], 0.0, conf.TARGET_SPEED])

            # Cost function
            cost += cp.quad_form(x[k, :] - ref_state, self.Q) + cp.quad_form(u[k, :], self.R)

        # Terminal cost
        cost += cp.quad_form(x[self.N, :] - ref_state, self.Q)

        # Define and solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)

        if prob.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            # Extract control inputs
            acceleration = u.value[0, 0]
            steering = u.value[0, 1]
        else:
            # If optimization fails, use default values
            acceleration = 0.0
            steering = 0.0
            print("MPC optimization failed. Using zero acceleration and steering.")

        # Clip control inputs to limits
        acceleration = np.clip(acceleration, conf.MAX_DECEL, conf.MAX_ACCEL)
        steering = np.clip(steering, -conf.MAX_STEER, conf.MAX_STEER)

        return acceleration, -steering


# Factory function to get steering controller by name
def get_steering_controller(name):
    """
    Factory function to get the steering controller instance based on name.

    Args:
        name (str): Name of the steering controller ('pure_pursuit', 'stanley', 'mpc').

    Returns:
        An instance of the requested steering controller.
    """
    if name == 'pure_pursuit':
        return PurePursuitController(look_ahead_distance=conf.LOOKAHEAD_DISTANCE)
    elif name == 'stanley':
        return StanleyController(k=1.0, k_soft=0.1)
    elif name == 'mpc':
        return MPCController(N=10, dt=0.05)
    else:
        raise ValueError(f"Unknown steering controller name: {name}")

# Utility functions
def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi].

    Args:
        angle (float): Angle [rad].

    Returns:
        float: Normalized angle [rad].
    """
    angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return angle
