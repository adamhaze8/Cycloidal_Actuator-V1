import serial
import struct
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
PORT = "COM11"  # Update to your target port
BAUD_RATE = 115200


def run_friction_calibration(port):
    """Smoothly ramps torque and uses absolute position to find breakaway."""
    print(f"\n--- RUNNING SMART FRICTION CALIBRATION ON {port} ---")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException:
        print(f"\nError: Could not open {port}.")
        return None

    time.sleep(1)

    # --- Phase 1: Smooth Breakaway Probe (POSITION BASED) ---
    print("Gently ramping torque to find static breakaway...")
    breakaway_tau = 0.0
    ramp_duration = 5.0  # Take 5 seconds to slowly ramp up
    max_safe_tau = 20.0  # Absolute maximum limit

    # Send an initial zero command to wake it up and get the STARTING POSITION
    ser.reset_input_buffer()
    ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
    while ser.in_waiting < 16:
        pass
    start_q, _, _, _ = struct.unpack("<ffff", ser.read(16))

    start_time = time.perf_counter()

    # High-speed continuous polling loop
    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed >= ramp_duration:
            break

        current_tau = (elapsed / ramp_duration) * max_safe_tau

        # Command the torque and read the CURRENT POSITION
        ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, current_tau))
        while ser.in_waiting < 16:
            pass
        q_curr, _, _, _ = struct.unpack("<ffff", ser.read(16))

        # Calculate how far it has actually moved from the start line
        position_delta = abs(q_curr - start_q)

        # If the physical angle changes by more than 0.1 radians (~5.7 degrees), it slipped!
        if position_delta > 0.1:
            breakaway_tau = current_tau
            print(f"\n-> Breakaway detected cleanly at {breakaway_tau:.2f} Nm!")

            # INSTANTLY CUT POWER
            ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
            break

        if int(elapsed * 100) % 10 == 0:
            print(
                f"\r  Ramping... {current_tau:.2f} Nm | Moved: {position_delta:.3f} rad",
                end="",
            )

    # Ensure motor is off and let inertia settle
    ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
    time.sleep(2.0)

    if breakaway_tau == 0.0:
        print("\n[!] Error: Hit 3.5 Nm limit without moving. Is the joint stuck?")
        ser.close()
        return pd.DataFrame(
            [[0, 0, "Forward"]],
            columns=["Applied_Torque_Nm", "Terminal_Velocity_rad_s", "Direction"],
        )

    # --- Phase 2: Generate Dynamic Test Points ---
    # Create 4 test points, starting one full increment ABOVE breakaway.
    torque_step = 0.4  # Nm to increment each test by
    test_torques = [round(breakaway_tau + (i * torque_step), 2) for i in range(1, 5)]

    print(f"Generated dynamic test points: {test_torques} Nm\n")
    data_log = []

    # --- Phase 3: Run the Data Collection ---
    for tau_ff in test_torques:
        for direction, signed_tau in [("Forward", tau_ff), ("Reverse", -tau_ff)]:
            print(
                f"  Mapping {direction} at {abs(signed_tau):.2f} Nm...",
                end="",
                flush=True,
            )

            ser.reset_input_buffer()

            # Spin up to terminal velocity
            start_spin = time.perf_counter()
            while (time.perf_counter() - start_spin) < 1.0:
                ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, signed_tau))
                while ser.in_waiting < 16:
                    pass
                ser.read(16)

            # Measure velocity for 2 seconds
            start_measure = time.perf_counter()
            velocities = []
            while (time.perf_counter() - start_measure) < 2.0:
                ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, signed_tau))
                while ser.in_waiting < 16:
                    pass
                _, dq_curr, _, _ = struct.unpack("<ffff", ser.read(16))
                velocities.append(dq_curr)

            # Stop motor to reset for the next direction
            ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
            time.sleep(1.5)

            avg_velocity = np.mean(velocities)
            print(f" Settled at {abs(avg_velocity):.4f} rad/s")
            data_log.append([abs(signed_tau), abs(avg_velocity), direction])

    ser.close()
    return pd.DataFrame(
        data_log, columns=["Applied_Torque_Nm", "Terminal_Velocity_rad_s", "Direction"]
    )


def analyze_and_plot_friction(df):
    """Calculates damping/friction and displays graph for 5 seconds."""
    summary = (
        df.groupby("Applied_Torque_Nm")["Terminal_Velocity_rad_s"].mean().reset_index()
    )
    torques = summary["Applied_Torque_Nm"].values
    velocities = summary["Terminal_Velocity_rad_s"].values

    m, b = np.polyfit(velocities, torques, 1)  # m = damping, b = friction

    plt.figure(figsize=(8, 5))
    df_fwd = df[df["Direction"] == "Forward"]
    df_rev = df[df["Direction"] == "Reverse"]
    plt.scatter(
        df_fwd["Terminal_Velocity_rad_s"],
        df_fwd["Applied_Torque_Nm"],
        label="Forward",
        color="blue",
    )
    plt.scatter(
        df_rev["Terminal_Velocity_rad_s"],
        df_rev["Applied_Torque_Nm"],
        label="Reverse",
        color="orange",
    )

    x_line = np.linspace(0, max(velocities) * 1.1, 100)
    plt.plot(x_line, m * x_line + b, color="red", label=f"Fit (b={b:.4f}, m={m:.4f})")
    plt.title("Constant Torque Friction Map")
    plt.xlabel("Terminal Velocity (rad/s)")
    plt.ylabel("Applied Torque (Nm)")
    plt.legend()

    # Block=False allows code execution to continue while plot remains open
    plt.show(block=False)
    plt.pause(5)
    plt.close()

    return float(b), float(m)


if __name__ == "__main__":
    try:
        friction_df = run_friction_calibration(PORT)

        if friction_df is not None:
            new_friction, new_damping = analyze_and_plot_friction(friction_df)

            print(f"\nCalculated Friction: {new_friction:.4f} Nm")
            print(f"Calculated Damping: {new_damping:.4f} Nms/rad")
            print("\nSweep Complete.")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Ensure motor is safe.")
