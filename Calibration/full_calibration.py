import serial
import serial.tools.list_ports
import struct
import time
import math
import os
import json
import subprocess
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- AUTOMATION CONFIGURATION ---
BAUD_RATE = 115200
CONFIG_FILE = "robot_config.json"

# Your exact Arduino CLI FQBN string:
FQBN = "STMicroelectronics:stm32:Disco:upload_method=swdMethod"

# Base paths for firmware (relative to this script's location)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FW_CALIB_LARGE = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Large", "calibrationFirmware", "calibrationFirmware.ino"))
FW_CALIB_SMALL = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Small", "calibrationFirmware", "calibrationFirmware.ino"))
FW_BIPED_LARGE = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Large", "bipedFirmware", "bipedFirmware.ino"))
FW_BIPED_SMALL = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Small", "bipedFirmware", "bipedFirmware.ino"))

# --- HELPER FUNCTIONS ---

def get_actuator_from_json(target_serial):
    """Cross-references the connected hardware serial with the JSON config."""
    with open(CONFIG_FILE, 'r') as f:
        config = json.load(f)
        
    for joint_name, data in config['actuators'].items():
        if data['hw_serial'] == target_serial:
            return joint_name, data, config
    return None, None, config

def scan_and_match():
    """Finds the connected ST-Link and matches it to the JSON."""
    print("Scanning for ST-LINK Motor Controllers...\n")
    ports = serial.tools.list_ports.comports()
    
    for port in ports:
        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            hw_serial = port.serial_number
            print(f"Found Hardware Serial: {hw_serial} on {port.device}")
            
            joint_name, act_data, full_config = get_actuator_from_json(hw_serial)
            if joint_name:
                print(f"Match found! This is the {act_data['motor_type']} actuator for: {joint_name}")
                return port.device, hw_serial, joint_name, act_data, full_config
            else:
                print(f"Warning: Serial {hw_serial} not found in {CONFIG_FILE}.")
                
    print("No recognized actuators found connected.")
    return None, None, None, None, None

def flash_firmware(port, ino_path):
    """Flashes firmware using Arduino CLI."""
    print(f"\n--- FLASHING FIRMWARE: {os.path.basename(ino_path)} ---")
    if not os.path.exists(ino_path):
        print(f"Error: Could not find firmware at {ino_path}")
        exit(1)
        
    cmd = ["arduino-cli.exe", "compile", "--upload", "-b", FQBN, "-p", port, ino_path]
    
    try:
        subprocess.run(cmd, check=True)
        print("Flashing successful!")
        time.sleep(5) # Wait for STM32 to reboot
    except subprocess.CalledProcessError as e:
        print(f"Error flashing firmware: {e}")
        exit(1)

def command_position_smooth(port, target_q, transition_time=2.0, hold_time=5.0, kp=20.0, kd=0.2):
    """Smoothly glides the joint to the target angle using an S-Curve, then holds."""
    print(f"\nMoving smoothly to zero position ({target_q:.4f} rad)...")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(1) # Reboot buffer
    ser.reset_input_buffer()
    
    # 1. Ping the motor to find out where it is currently sitting
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
    while ser.in_waiting < 16: pass
    start_q, _, _, _ = struct.unpack('<ffff', ser.read(16))
    
    # 2. Execute the Smooth S-Curve Transition
    start_time = time.perf_counter()
    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed >= transition_time:
            break
            
        # Calculate S-Curve progression (0.0 to 1.0)
        progress = elapsed / transition_time
        ease = (1.0 - math.cos(progress * math.pi)) / 2.0
        
        # Interpolate between start and target
        current_target = start_q + ((target_q - start_q) * ease)
        
        ser.write(struct.pack('<ffff', current_target, kp, kd, 0.0))
        while ser.in_waiting < 16: pass
        ser.read(16)
        
    # 3. Hold the final target position
    print(f"Holding at {target_q:.4f} rad for {hold_time} seconds...")
    hold_start = time.perf_counter()
    while (time.perf_counter() - hold_start) < hold_time:
        ser.write(struct.pack('<ffff', target_q, kp, kd, 0.0))
        while ser.in_waiting < 16: pass
        ser.read(16)
    
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)) # Limp motor
    ser.close()

def run_encoder_check(port, gear_ratio):
    """Runs the high-speed encoder sweep and displays graph for 5 seconds."""
    print(f"\n--- RUNNING ENCODER CHECK (Ratio: {gear_ratio}:1) ---")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(1)
    
    ser.reset_input_buffer()
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
    while ser.in_waiting < 16: pass 
    state_data = ser.read(16)
    start_q, _, _, start_motor_q = struct.unpack('<ffff', state_data)
    
    motor_start_offset = (start_motor_q / gear_ratio) - start_q
    
    times_log, targets_log, actuals_log, motor_q_log, raws_log = [], [], [], [], []
    start_time = time.perf_counter()
    
    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed > 10.0: break 
            
        target_q = start_q + ((elapsed / 10.0) * (2 * math.pi))
        ser.reset_input_buffer()
        ser.write(struct.pack('<ffff', target_q, 40.0, 0.0, 0.0))
        while ser.in_waiting < 16: pass
        q_curr, dq_curr, tau_curr, motor_q = struct.unpack('<ffff', ser.read(16))
        
        motor_q_joint_space = (motor_q / gear_ratio) - motor_start_offset
        raw_angle = (q_curr * 2.0) % (2 * math.pi)

        times_log.append(elapsed)
        targets_log.append(target_q)
        actuals_log.append(q_curr)
        motor_q_log.append(motor_q_joint_space)
        raws_log.append(raw_angle)
        
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
    ser.close()

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(times_log, targets_log, label='Target', linestyle='--')
    plt.plot(times_log, actuals_log, label='Joint Encoder', linewidth=2)
    plt.plot(times_log, motor_q_log, label='Motor Encoder', linestyle=':')
    plt.title(f'Encoder Sweep Verification ({gear_ratio}:1 Ratio)')
    plt.legend()
    plt.subplot(2, 1, 2)
    plt.plot(times_log, raws_log, label='Raw Sensor Angle', color='red', marker='.', linestyle='None')
    plt.axhline(y=2*math.pi, color='black')
    plt.show(block=False)
    plt.pause(5)
    plt.close()

def run_friction_calibration(port):
    """Smoothly ramps torque and uses absolute position to find breakaway."""
    print("\n--- RUNNING SMART FRICTION CALIBRATION ---")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(1)
    
    # --- Phase 1: Smooth Breakaway Probe (POSITION BASED) ---
    print("Gently ramping torque to find static breakaway...")
    breakaway_tau = 0.0
    ramp_duration = 5.0   # Take 5 seconds to slowly ramp up
    max_safe_tau = 3.5    # Absolute maximum limit
    
    # Send an initial zero command to wake it up and get the STARTING POSITION
    ser.reset_input_buffer()
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
    while ser.in_waiting < 16: pass
    start_q, _, _, _ = struct.unpack('<ffff', ser.read(16))
    
    start_time = time.perf_counter()
    
    # High-speed continuous polling loop
    while True:
        elapsed = time.perf_counter() - start_time
        if elapsed >= ramp_duration:
            break
            
        current_tau = (elapsed / ramp_duration) * max_safe_tau
        
        # Command the torque and read the CURRENT POSITION
        ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, current_tau))
        while ser.in_waiting < 16: pass
        q_curr, _, _, _ = struct.unpack('<ffff', ser.read(16))
        
        # Calculate how far it has actually moved from the start line
        position_delta = abs(q_curr - start_q)
        
        # If the physical angle changes by more than 0.1 radians (~5.7 degrees), it slipped!
        if position_delta > 0.1: 
            breakaway_tau = current_tau
            print(f"\n-> Breakaway detected cleanly at {breakaway_tau:.2f} Nm!")
            
            # INSTANTLY CUT POWER
            ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
            break
            
        if int(elapsed * 100) % 10 == 0:
            print(f"\r  Ramping... {current_tau:.2f} Nm | Moved: {position_delta:.3f} rad", end="")

    # Ensure motor is off and let inertia settle
    ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0)) 
    time.sleep(2.0) 

    if breakaway_tau == 0.0:
        print("\n[!] Error: Hit 3.5 Nm limit without moving. Is the joint stuck?")
        ser.close()
        return pd.DataFrame([[0,0,'Forward']], columns=["Applied_Torque_Nm", "Terminal_Velocity_rad_s", "Direction"])

    # --- Phase 2: Generate Dynamic Test Points ---
    # Create 4 test points, starting one full increment ABOVE breakaway.
    # This guarantees the motor is safely in the kinetic friction zone.
    torque_step = 0.2  # Nm to increment each test by
    test_torques = [round(breakaway_tau + (i * torque_step), 2) for i in range(1, 5)]
    
    print(f"Generated dynamic test points: {test_torques} Nm\n")
    data_log = []
    
    # --- Phase 3: Run the Data Collection ---
    for tau_ff in test_torques:
        for direction, signed_tau in [("Forward", tau_ff), ("Reverse", -tau_ff)]:
            print(f"  Mapping {direction} at {abs(signed_tau):.2f} Nm...", end="", flush=True)
            
            ser.reset_input_buffer()
            
            # Spin up to terminal velocity
            start_spin = time.perf_counter()
            while (time.perf_counter() - start_spin) < 1.0:
                ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, signed_tau))
                while ser.in_waiting < 16: pass
                ser.read(16)
            
            # Measure velocity for 2 seconds
            start_measure = time.perf_counter()
            velocities = []
            while (time.perf_counter() - start_measure) < 2.0:
                ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, signed_tau))
                while ser.in_waiting < 16: pass
                _, dq_curr, _, _ = struct.unpack('<ffff', ser.read(16))
                velocities.append(dq_curr)
                
            # Stop motor to reset for the next direction
            ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
            time.sleep(1.5)
            
            avg_velocity = np.mean(velocities)
            print(f" Settled at {abs(avg_velocity):.4f} rad/s")
            data_log.append([abs(signed_tau), abs(avg_velocity), direction])
            
    ser.close()
    return pd.DataFrame(data_log, columns=["Applied_Torque_Nm", "Terminal_Velocity_rad_s", "Direction"])

def analyze_and_plot_friction(df):
    """Calculates damping/friction and displays graph for 5 seconds."""
    summary = df.groupby('Applied_Torque_Nm')['Terminal_Velocity_rad_s'].mean().reset_index()
    torques = summary['Applied_Torque_Nm'].values
    velocities = summary['Terminal_Velocity_rad_s'].values

    m, b = np.polyfit(velocities, torques, 1) # m = damping, b = friction
    
    plt.figure(figsize=(8, 5))
    df_fwd = df[df['Direction'] == 'Forward']
    df_rev = df[df['Direction'] == 'Reverse']
    plt.scatter(df_fwd['Terminal_Velocity_rad_s'], df_fwd['Applied_Torque_Nm'], label='Forward', color='blue')
    plt.scatter(df_rev['Terminal_Velocity_rad_s'], df_rev['Applied_Torque_Nm'], label='Reverse', color='orange')
    
    x_line = np.linspace(0, max(velocities) * 1.1, 100)
    plt.plot(x_line, m * x_line + b, color='red', label=f'Fit (b={b:.4f}, m={m:.4f})')
    plt.title('Constant Torque Friction Map')
    plt.legend()
    plt.show(block=False)
    plt.pause(5)
    plt.close()
    
    return float(b), float(m)

# --- MASTER WORKFLOW ---

def master_calibration():
    # 1. Scan and Match
    port, hw_serial, joint_name, act_data, full_config = scan_and_match()
    if not port:
        return
    
    motor_type = act_data['motor_type']
    zero_offset = act_data['zero_offset']
    gear_ratio = act_data.get('gear_ratio', 20.0)
    
    # Select firmware paths based on motor size
    calib_fw = FW_CALIB_LARGE if motor_type == "G80" else FW_CALIB_SMALL
    biped_fw = FW_BIPED_LARGE if motor_type == "G80" else FW_BIPED_SMALL

    # 2. Flash Calibration Firmware
    flash_firmware(port, calib_fw)
    
    # 3. Command to Zero Position Smoothly (2s move, 5s hold)
    command_position_smooth(port, target_q=zero_offset, transition_time=2.0, hold_time=5.0)
    
    # 4. Encoder Check (Graphs for 5s)
    run_encoder_check(port, gear_ratio)
    
    # 5 & 6. Friction Test & Analysis (Graphs for 5s)
    friction_df = run_friction_calibration(port)
    new_friction, new_damping = analyze_and_plot_friction(friction_df)
    
    print(f"\nCalculated Friction: {new_friction:.4f} Nm")
    print(f"Calculated Damping: {new_damping:.4f} Nms/rad")
    
    # 7. Update JSON File
    full_config['actuators'][joint_name]['friction'] = round(new_friction, 4)
    full_config['actuators'][joint_name]['damping'] = round(new_damping, 4)
    
    with open(CONFIG_FILE, 'w') as f:
        json.dump(full_config, f, indent=2)
    print(f"\n{CONFIG_FILE} updated successfully!")
    
    command_position_smooth(port, target_q=zero_offset, transition_time=2.0, hold_time=5.0)

    # 8. Flash Biped Firmware
    flash_firmware(port, biped_fw)
    
    # 9. Final Command to Zero Smoothly (2s move, 5s hold)
    print("\nFinal system check. Waking up in operational biped mode...")
    
    print(f"\n=== CALIBRATION COMPLETE FOR {joint_name} ===")

if __name__ == "__main__":
    master_calibration()