import serial
import struct
import time
import math
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
PORT = 'COM4' 
BAUD_RATE = 115200

KP = 40.0
KD = 0.0
TAU_FF = 0.0

GEAR_RATIO = 20.0
SWEEP_DURATION = 10.0  

def high_res_encoder_sweep():
    print(f"Connecting to {PORT}...")
    
    times_log = []
    targets_log = []
    actuals_log = []
    raws_log = []
    motor_q_log = [] # NEW: Ground truth log
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2) 
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Wake up board (16 bytes)
        ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
        
        # WAITING FOR 16 BYTES NOW
        while ser.in_waiting < 16: pass 
            
        state_data = ser.read(16)
        # Unpack 4 floats
        start_q, _, _, start_motor_q = struct.unpack('<ffff', state_data)
        
        # We need an offset to align the motor encoder to the joint encoder's starting point
        motor_start_offset = (start_motor_q / GEAR_RATIO) - start_q
        
        print(f"\nJoint found at {start_q:.4f} rad.")
        print("Running UNTHROTTLED High-Speed Sweep...")
        print("-" * 75)
        print(f"{'Target (q_des)':<15} | {'Joint (q_curr)':<15} | {'Motor (Ground)':<15} | {'Loop Hz'}")
        print("-" * 75)

        start_time = time.perf_counter()
        previous_raw = None
        
        loop_count = 0
        last_print_time = start_time
        current_hz = 0

        while True:
            current_time = time.perf_counter()
            elapsed = current_time - start_time
            
            if elapsed > SWEEP_DURATION: break 
                
            target_q = start_q + ((elapsed / SWEEP_DURATION) * (2 * math.pi))

            ser.reset_input_buffer()
            ser.write(struct.pack('<ffff', target_q, KP, KD, TAU_FF))

            # WAIT FOR 16 BYTES
            while ser.in_waiting < 16: pass
                
            state_data = ser.read(16)
            
            # UNPACK 4 FLOATS
            q_curr, dq_curr, tau_curr, motor_q = struct.unpack('<ffff', state_data)
            loop_count += 1

            # Transform motor angle to joint space and align it to the start position
            motor_q_joint_space = (motor_q / GEAR_RATIO) - motor_start_offset

            raw_angle = (q_curr * 2.0) % (2 * math.pi)

            # LOG DATA
            times_log.append(elapsed)
            targets_log.append(target_q)
            actuals_log.append(q_curr)
            motor_q_log.append(motor_q_joint_space)
            raws_log.append(raw_angle)

            if previous_raw is not None:
                delta = abs(raw_angle - previous_raw)
                if delta > 0.5 and delta < (2 * math.pi - 0.5): 
                    print(f"\n[!!!] MAGNETIC BOUNDARY JUMP DETECTED [!!!]")
                    print(f"Time: {elapsed:.4f}s | Previous: {previous_raw:.4f} rad -> New: {raw_angle:.4f} rad")
                    print("-" * 75)
                    
            previous_raw = raw_angle

            if current_time - last_print_time >= 0.1:
                current_hz = loop_count / (current_time - last_print_time)
                print(f"\r{target_q:10.4f} rad   | {q_curr:10.4f} rad   | {motor_q_joint_space:10.4f} rad   | {current_hz:5.0f} Hz", end="")
                loop_count = 0
                last_print_time = current_time

        print("\n\nSweep complete. Disengaging motor...")

    except serial.SerialException:
        print(f"\nError: Could not open {PORT}.")
    except KeyboardInterrupt:
        print("\n\nSweep safely interrupted.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
            ser.close()
            
        if len(times_log) > 0:
            print("Generating cross-reference plots...")
            plt.figure(figsize=(12, 8))

            # Top Plot: Target vs Joint Sensor vs Motor Sensor
            plt.subplot(2, 1, 1)
            plt.plot(times_log, targets_log, label='Target Trajectory', color='gray', linestyle='--', linewidth=1)
            plt.plot(times_log, actuals_log, label='AS5048A Joint Encoder (q_curr)', color='blue', alpha=0.8, linewidth=3)
            plt.plot(times_log, motor_q_log, label='Incremental Motor Encoder (Ground Truth)', color='lime', linestyle=':', linewidth=2)
            plt.title('Absolute Joint Sensor vs. Incremental Motor Sensor', fontweight='bold')
            plt.ylabel('Angle (radians)')
            plt.grid(True, linestyle=':', alpha=0.7)
            plt.legend()

            # Bottom Plot: Raw Sensor
            plt.subplot(2, 1, 2)
            plt.plot(times_log, raws_log, label='Raw Sensor Angle', color='red', marker='.', markersize=2, linestyle='None')
            plt.title('Raw Magnetic Sensor Signal (0 to 2π)', fontweight='bold')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Raw Angle (radians)')
            plt.axhline(y=0, color='black', linewidth=1)
            plt.axhline(y=2*math.pi, color='black', linewidth=1, label='Max Wrap (2π)')
            plt.grid(True, linestyle=':', alpha=0.7)
            plt.legend()

            plt.tight_layout()
            plt.show()

if __name__ == "__main__":
    high_res_encoder_sweep()