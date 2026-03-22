import serial
import serial.tools.list_ports
import struct
import time
import math
import json

# --- CONFIGURATION ---
BAUD_RATE = 115200
CONFIG_FILE = "robot_config.json"

def get_actuator_from_json(target_serial):
    """Cross-references the connected hardware serial with the JSON config."""
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
            
        for joint_name, data in config['actuators'].items():
            if data['hw_serial'] == target_serial:
                return joint_name, data
    except FileNotFoundError:
        print(f"Error: Could not find '{CONFIG_FILE}'.")
        
    return None, None

def scan_and_match():
    """Finds the connected ST-Link and matches it to the JSON."""
    print("Scanning for ST-LINK Motor Controllers...\n")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            hw_serial = port.serial_number
            joint_name, act_data = get_actuator_from_json(hw_serial)
            
            if joint_name:
                print(f"Match found! Actuator: {joint_name} ({act_data['motor_type']}) on {port.device}")
                return port.device, joint_name, act_data['zero_offset']
                
    print("No recognized actuators found connected.")
    return None, None, None

def move_to_home(port, joint_name, target_q, kp=20.0, kd=0.2):
    """Smoothly glides the joint to its 90-degree home posture with compliant gains."""
    print(f"\nConnecting to {port}...")
    
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(1) # Wait for buffer to clear after connection
        ser.reset_input_buffer()
        
        # 1. Send Limp Command to wake board and get starting position
        ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
        time.sleep(0.05) # Give firmware time to reply
        
        # 2. Telemetry-Agnostic Read (Handles any number of bytes)
        if ser.in_waiting >= 4:
            data = ser.read(ser.in_waiting) # Read everything in the buffer
            # Unpack ONLY the first 4 bytes as the current position float
            start_q = struct.unpack('<f', data[:4])[0] 
        else:
            print("Error: Actuator did not respond. Is it powered on?")
            ser.close()
            return

        print(f"Current Position : {start_q:.4f} rad")
        print(f"Target Home (90°) : {target_q:.4f} rad")
        print(f"Executing smooth transition (kp={kp}, kd={kd})...")

        # 3. Smooth S-Curve Transition
        transition_time = 2.0
        start_time = time.perf_counter()
        
        while True:
            elapsed = time.perf_counter() - start_time
            if elapsed >= transition_time:
                break
                
            # Calculate S-Curve progression
            progress = elapsed / transition_time
            ease = (1.0 - math.cos(progress * math.pi)) / 2.0
            current_target = start_q + ((target_q - start_q) * ease)
            
            # Send command using the updated kp and kd
            ser.write(struct.pack('<ffff', current_target, kp, kd, 0.0))
            
            # Flush incoming buffer to prevent Overflow Errors
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)
                
            # Run loop at approx 500Hz to not overwhelm the serial line
            time.sleep(0.002) 
            
        # 4. Hold position briefly
        print("Holding at home (90°) for 3 seconds...")
        hold_start = time.perf_counter()
        while (time.perf_counter() - hold_start) < 3.0:
            # Continue holding with the updated kp and kd
            ser.write(struct.pack('<ffff', target_q, kp, kd, 0.0))
            if ser.in_waiting > 0:
                ser.read(ser.in_waiting)
            time.sleep(0.002)

    except serial.SerialException as e:
        print(f"\nSerial Error: {e}")
    finally:
        # 5. Always disengage motor safely before exiting
        if 'ser' in locals() and ser.is_open:
            print("Disengaging motor...")
            ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
            time.sleep(0.1)
            ser.close()
            print("Done!")

if __name__ == "__main__":
    port, joint_name, home_offset = scan_and_match()
    
    if port:
        move_to_home(port, joint_name, home_offset)