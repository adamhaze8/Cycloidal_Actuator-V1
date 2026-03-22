import serial
import struct
import time

# --- CONFIGURATION ---
PORT = 'COM12'  # Update this to your actuator's port
BAUD_RATE = 115200

def manual_zeroing_tool():
    print(f"Connecting to {PORT}...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # Give STM32 a moment to reboot on connection
        
        print("\n--- MANUAL ZEROING MODE ---")
        print("Motor is Limp (kp=0, kd=0). Move the joint by hand.")
        print("Align it perfectly in your rig, then copy the Joint Angle below.")
        print("Press Ctrl+C to exit.\n")
        print("-" * 80)
        print(f"{'Live Joint Angle (q_curr)':<25} | {'Velocity':<15} | {'Effort':<10} | {'Motor Angle'}")
        print("-" * 80)

        # Clear any junk out of the buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while True:
            # 1. SEND LIMP COMMAND (16 Bytes)
            cmd_data = struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)
            ser.write(cmd_data)

            # 2. WAIT FOR & RECEIVE STATE (16 Bytes)
            if ser.in_waiting >= 16:
                state_data = ser.read(16)
                ser.reset_input_buffer() # Flush to maintain instant sync
                
                # 3. UNPACK THE RESPONSE (4 Floats)
                q_curr, dq_curr, tau_cmd, motor_q = struct.unpack('<ffff', state_data)

                # 4. DISPLAY THE DATA
                print(f"\r      {q_curr:>8.4f} rad            |  {dq_curr:>8.4f} rad/s | {tau_cmd:>6.2f} Nm  |  {motor_q:>8.4f} rad", end="")
                
                # Sleep briefly so the terminal is readable by human eyes (~20Hz display)
                time.sleep(0.05) 

    except serial.SerialException:
        print(f"\nError: Could not open {PORT}. Is it plugged in and not in use?")
    except KeyboardInterrupt:
        print("\n\nZeroing session closed. Update your robot_config.json!")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    manual_zeroing_tool()