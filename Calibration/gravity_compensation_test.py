import serial
import struct
import math
import time
import sys

# --- CONFIGURATION ---
PORT = "/dev/ttyUSB0"  # Change to your specific serial port (e.g., COM3 on Windows)
BAUD_RATE = 115200

# The peak holding torque required when the arm is perfectly horizontal
MAX_GRAVITY_TORQUE_NM = 8.16314625


def run_gravity_compensation(port_name):
    print(f"Connecting to actuator on {port_name}...")
    try:
        ser = serial.Serial(port_name, BAUD_RATE, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    print("Connected. Initiating pure Feed-Forward Gravity Compensation.")
    print("Lift the arm. It should feel weightless.")
    print("Press Ctrl+C to drop the arm and exit.\n")

    # Flush any stale data in the serial buffer
    ser.reset_input_buffer()

    # Send an initial zero-command to kick off the STM32's response loop
    ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))

    try:
        while True:
            # Wait for the 16-byte state packet from the STM32
            if ser.in_waiting >= 16:
                state_data = ser.read(16)

                # Unpack the 4 floats returning from the firmware
                q_curr, dq_curr, tau_curr, motor_q = struct.unpack("<ffff", state_data)

                # --- GRAVITY MATH ---
                # Assuming 0 is straight down:
                # sin(0) = 0.0 (Requires 0 torque)
                # sin(pi/2) = 1.0 (Requires max torque at horizontal)
                tau_ff = MAX_GRAVITY_TORQUE_NM * math.sin(q_curr)

                # NOTE: If the arm pulls aggressively DOWNWARDS when you lift it,
                # the motor phasing is inverted relative to the encoder.
                # Just change the line above to: tau_ff = -MAX_GRAVITY_TORQUE_NM * math.sin(q_curr)

                # Send command: q_des=0, kp=0, kd=0, tau_ff=calculated_torque
                ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, tau_ff))

    except KeyboardInterrupt:
        print("\n[!] Ctrl+C Detected. Sending zero-torque command to go limp...")
        ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
        time.sleep(0.1)
        ser.close()
        print("Actuator safely disabled.")


if __name__ == "__main__":
    # If passing port via terminal: python grav_comp.py /dev/ttyUSB1
    target_port = sys.argv[1] if len(sys.argv) > 1 else PORT
    run_gravity_compensation(target_port)
