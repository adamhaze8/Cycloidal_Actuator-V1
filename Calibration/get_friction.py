import serial
import struct
import time
import csv
import numpy as np

# --- CONFIGURATION ---
PORT = 'COM12'  
BAUD_RATE = 115200

# We completely disable the impedance controller. Pure open-loop torque.
KP = 0.0
KD = 0.0

# The constant torques we will apply (Nm). 
# Note: Start slightly above your known ~0.8Nm baseline so it actually spins!
TEST_TORQUES = [1.2, 1.5, 2] 
TEST_DURATION = 3.0   # Seconds to hold the torque and measure velocity
SETTLE_TIME = 1.0     # Seconds to wait for terminal velocity before measuring

DATA_FILE = "friction_torque_step_map.csv"

def map_constant_torque_friction():
    print(f"Connecting to {PORT}...")
    
    try:
        ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
        time.sleep(2) 
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Wake up / Ping (16 Bytes)
        ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
        while ser.in_waiting < 16: pass 
        ser.read(16)
        
        data_log = [] 

        for tau_ff in TEST_TORQUES:
            print(f"\n--- TESTING CONSTANT TORQUE: {tau_ff} Nm ---")
            
            # We will test both Forward (+tau) and Reverse (-tau)
            for direction, signed_tau in [("Forward", tau_ff), ("Reverse", -tau_ff)]:
                print(f"{direction} Spin ({signed_tau} Nm)... ", end="", flush=True)
                
                # 1. Spool Up: Apply torque and let it reach terminal velocity
                ser.reset_input_buffer()
                ser.write(struct.pack('<ffff', 0.0, KP, KD, signed_tau))
                time.sleep(SETTLE_TIME)
                
                # 2. Measure: Record the velocity at steady state
                start_time = time.perf_counter()
                velocities = []
                
                while (time.perf_counter() - start_time) < TEST_DURATION:
                    # Keep applying the constant torque just to keep the watchdog alive if you have one
                    ser.reset_input_buffer()
                    ser.write(struct.pack('<ffff', 0.0, KP, KD, signed_tau))
                    
                    while ser.in_waiting < 16: pass
                    _, dq_curr, _, _ = struct.unpack('<ffff', ser.read(16))
                    
                    velocities.append(dq_curr)
                
                # 3. Stop the motor and let inertia bleed off
                ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
                time.sleep(1.5)
                
                # Calculate the average terminal velocity
                avg_velocity = np.mean(velocities)
                print(f"Settled at {avg_velocity:8.4f} rad/s")
                
                # Save the absolute magnitudes for Isaac Lab math
                data_log.append([abs(signed_tau), abs(avg_velocity), direction])

        print(f"\nSweep complete. Saving data...")
        
        with open(DATA_FILE, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Applied_Torque_Nm", "Terminal_Velocity_rad_s", "Direction"])
            writer.writerows(data_log)
        print(f"Data saved to {DATA_FILE}!")

    except serial.SerialException:
        print(f"\nError opening {PORT}.")
    except KeyboardInterrupt:
        print("\n\nInterrupted.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.write(struct.pack('<ffff', 0.0, 0.0, 0.0, 0.0))
            ser.close()

if __name__ == "__main__":
    map_constant_torque_friction()