import serial
import serial.tools.list_ports
import struct
import time
import os
import json
import subprocess

# --- AUTOMATION CONFIGURATION ---
BAUD_RATE = 115200
CONFIG_FILE = "biped_actuator_config.json"
FQBN = "STMicroelectronics:stm32:Disco:upload_method=swdMethod"

# Base paths for firmware
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FW_CALIB_LARGE = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Large", "calibrationFirmware", "calibrationFirmware.ino"))
FW_CALIB_SMALL = os.path.normpath(os.path.join(BASE_DIR, "..", "Firmware", "Small", "calibrationFirmware", "calibrationFirmware.ino"))

# --- HELPER FUNCTIONS ---

def get_actuator_from_json(target_serial):
    try:
        with open(CONFIG_FILE, "r") as f:
            config = json.load(f)
        for joint_name, data in config["actuators"].items():
            if data["hw_serial"] == target_serial:
                return joint_name, data
    except Exception as e:
        print(f"Error loading config: {e}")
    return None, None

def scan_and_match():
    print("Scanning for ST-LINK Motor Controllers...\n")
    ports = serial.tools.list_ports.comports()

    for port in ports:
        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            hw_serial = port.serial_number
            print(f"Found Hardware Serial: {hw_serial} on {port.device}")

            joint_name, act_data = get_actuator_from_json(hw_serial)
            if joint_name:
                print(f"Match found! This is the {act_data['motor_type']} actuator for: {joint_name}")
                return port.device, joint_name, act_data
            else:
                print(f"Warning: Serial {hw_serial} not found in {CONFIG_FILE}.")

    print("No recognized actuators found connected.")
    return None, None, None

def flash_firmware(port, ino_path):
    print(f"\n--- FLASHING FIRMWARE: {os.path.basename(ino_path)} ---")
    if not os.path.exists(ino_path):
        print(f"Error: Could not find firmware at {ino_path}")
        exit(1)

    cmd = ["arduino-cli.exe", "compile", "--clean", "--upload", "-b", FQBN, "-p", port, ino_path]

    try:
        subprocess.run(cmd, check=True)
        print("Flashing successful!")
        time.sleep(5)  # Wait for STM32 to reboot
    except subprocess.CalledProcessError as e:
        print(f"Error flashing firmware: {e}")
        exit(1)

# --- BREAK-IN ROUTINE ---

def run_break_in_routine(port, duration_minutes=15, target_vel=4.0, sweep_rad=3.0, kp_vel=0.8, max_tau=3.5):
    """
    Runs a continuous P-controller to maintain target_vel, reversing direction 
    whenever the actuator travels further than sweep_rad from the starting position.
    """
    print(f"\n--- STARTING BREAK-IN ROUTINE ({duration_minutes} minutes) ---")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(1)
    ser.reset_input_buffer()
    
    # 1. Ping the motor to find the starting position
    ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
    while ser.in_waiting < 16:
        pass
    start_q, _, _, _ = struct.unpack("<ffff", ser.read(16))
    
    print(f"Starting Position: {start_q:.4f} rad")
    print(f"Sweep Bounds: ±{sweep_rad} rad")
    print(f"Target Speed: {target_vel} rad/s")
    print("Press Ctrl+C to stop early.\n")
    
    start_time = time.perf_counter()
    duration_sec = duration_minutes * 60.0
    
    direction = 1
    tau_cmd = 0.0
    
    try:
        while (time.perf_counter() - start_time) < duration_sec:
            # Send the pure feedforward torque command (position, Kp, and Kd are 0.0)
            ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, tau_cmd))
            
            while ser.in_waiting < 16:
                pass
            
            q_curr, dq_curr, tau_curr, _ = struct.unpack("<ffff", ser.read(16))
            
            # 2. Check bounds to reverse direction
            if q_curr > start_q + sweep_rad:
                direction = -1
            elif q_curr < start_q - sweep_rad:
                direction = 1
                
            # 3. Proportional Velocity Control (P-Controller on Torque)
            current_target_vel = target_vel * direction
            vel_error = current_target_vel - dq_curr
            tau_cmd = kp_vel * vel_error
            
            # 4. Hard Safety Clamp
            tau_cmd = max(min(tau_cmd, max_tau), -max_tau)
            
            # Print status update at roughly 10Hz to avoid console spam
            elapsed = time.perf_counter() - start_time
            if int(elapsed * 100) % 10 == 0:
                print(f"\rTime: {elapsed:.1f}s | Pos: {q_curr-start_q:+.2f} rad | Vel: {dq_curr:+.2f} rad/s | CMD Tau: {tau_cmd:+.2f} Nm    ", end="")
                
    except KeyboardInterrupt:
        print("\n\n[!] Break-in stopped manually by user.")
        
    finally:
        # Guarantee motor is safed before exiting
        print("\nSafing motor...")
        ser.write(struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0))
        ser.close()
        print("Break-in complete.")

# --- MASTER WORKFLOW ---

def main():
    # 1. Scan and Match
    port, joint_name, act_data = scan_and_match()
    if not port:
        return

    motor_type = act_data["motor_type"]
    calib_fw = FW_CALIB_LARGE if motor_type == "G80" else FW_CALIB_SMALL

    # 2. Flash Calibration Firmware (which supports raw tau_ff inputs)
    flash_firmware(port, calib_fw)

    # 3. Run the Break-in Control Loop
    # You can tune duration_minutes, target_vel, sweep_rad, and kp_vel here:
    run_break_in_routine(port, duration_minutes=15, target_vel=4.0, sweep_rad=3.0, kp_vel=0.8, max_tau=3.5)

if __name__ == "__main__":
    main()