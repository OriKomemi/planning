import time
import matplotlib.pyplot as plt               # NEW
from core.controllers import AccelerationPIDController, PurePursuitController
from vehicle_config import Vehicle_config as conf
from providers.sim.sim_util import FSDSClientSingleton, sim_car_controls
from providers.gps_imu.python.gps_imu_provider import GPSIMUProvider
from nodes.controller import Controller


def main():
    LIVE = False                    # Set True for live mode
    # ─── Controllers ────────────────────────────────────────────────
    accel_controller = AccelerationPIDController(
        kp=conf.kp_accel,
        ki=conf.ki_accel,
        kd=conf.kd_accel,
        setpoint=conf.TARGET_SPEED
    )
    pp_controller = PurePursuitController(
        look_ahead_distance=conf.LOOKAHEAD_DISTANCE
    )
    controller = Controller(simulation=(not LIVE), is_global_frame=True)

    # ─── Data storage for plotting ─────────────────────────────────
    t_hist: list[float] = []        # time stamps (s)
    v_hist: list[float] = []        # measured speed (m/s)

    # ─── Providers / client init ───────────────────────────────────
    if LIVE:
        gps_provider = GPSIMUProvider().start()
    else:
        client = FSDSClientSingleton.instance()

    # ─── Main loop ─────────────────────────────────────────────────
    t0 = time.perf_counter()
    duration = 10.0       # seconds
    while (t := time.perf_counter()) - t0 < duration:
        # --- Acquire speed ---
        if LIVE:
            gps_data = gps_provider.read()
            current_speed = gps_data.gps_speed / 3.6   # km/h → m/s
        else:
            current_speed = client.getCarState().speed

        # --- Control & actuation ---
        acceleration = accel_controller.compute_acceleration(current_speed, curvature=0.0)
        controller.apply_controls(steering=0.0, acceleration=acceleration)

        # --- Log for plotting ---
        t_hist.append(t - t0)
        v_hist.append(current_speed)

    # Stop the car
    controller.apply_controls(0.0, -1000)

    # ─── Plot results ──────────────────────────────────────────────
    plt.figure(figsize=(8, 4))
    plt.plot(t_hist, v_hist, label="Current speed")
    plt.axhline(conf.TARGET_SPEED, linestyle="--", label=f"TARGET_SPEED = {conf.TARGET_SPEED:.2f} m/s")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Vehicle Speed vs. Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
