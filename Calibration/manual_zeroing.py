import serial
import serial.tools.list_ports
import struct
import time
import math

# --- CONFIGURATION ---
BAUD_RATE = 115200

def scan_for_actuator():
    """Finds the connected ST-Link automatically."""
    print("Scanning for ST-LINK Motor Controllers...\n")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            print(f"Found Actuator on {port.device}")
            return port.device
            
    print("No recognized actuators found connected.")
    return None

def manual_zeroing_tool(port):
    print(f"Connecting to {port}...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2) # Give STM32 a moment to reboot on connection
        
        print("\n--- MANUAL ZEROING MODE ---")
        print("Motor is Limp (kp=0, kd=0). Move the joint by hand.")
        print("Align it perfectly in your rig, then copy the Joint Angle below.")
        print("Press Ctrl+C to exit.\n")
        print("-" * 105)
        print(f"{'Live Joint Angle (q_curr)':<25} | {'Raw Encoder Angle':<18} | {'Velocity':<15} | {'Effort':<10} | {'Motor Angle'}")
        print("-" * 105)

        # Clear any junk out of the buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        while True:
            # 1. SEND LIMP COMMAND (16 Bytes)
            cmd_data = struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)
            ser.write(cmd_data)
            
            # Give the firmware a tiny fraction of a second to process and reply
            time.sleep(0.05) 

            # 2. TELEMETRY-AGNOSTIC READ
            bytes_waiting = ser.in_waiting
            if bytes_waiting >= 4:
                state_data = ser.read(bytes_waiting)
                ser.reset_input_buffer() # Flush to maintain instant sync
                
                # 3. UNPACK DYNAMICALLY BASED ON AVAILABLE BYTES
                if len(state_data) >= 16:
                    # Full Calibration Firmware (Unpack all 4 floats)
                    q_curr, dq_curr, tau_cmd, motor_q = struct.unpack('<ffff', state_data[:16])
                    
                    # Calculate raw output encoder angle (0 to 2PI)
                    raw_encoder = (q_curr * 2.0) % (2 * math.pi)
                    
                    print(f"\r      {q_curr:>8.4f} rad            |     {raw_encoder:>8.4f} rad     |  {dq_curr:>8.4f} rad/s | {tau_cmd:>6.2f} Nm  |  {motor_q:>8.4f} rad", end="")
                else:
                    # Biped Firmware (Unpack only the first float, ignore the rest)
                    q_curr = struct.unpack('<f', state_data[:4])[0]
                    
                    # Calculate raw output encoder angle (0 to 2PI)
                    raw_encoder = (q_curr * 2.0) % (2 * math.pi)
                    
                    print(f"\r      {q_curr:>8.4f} rad            |     {raw_encoder:>8.4f} rad     |  {'N/A':>8}        | {'N/A':>6}      |  {'N/A':>8}    ", end="")

    except serial.SerialException:
        print(f"\nError: Could not open {port}. Is it plugged in and not in use?")
    except KeyboardInterrupt:
        print("\n\nZeroing session closed. Update your robot_config.json!")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    found_port = scan_for_actuator()
    if found_port:
        manual_zeroing_tool(found_port)