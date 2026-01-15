import serial
import time
import csv

# --- CONFIGURATION ---
SERIAL_PORT = "COM4"  # Ensure this matches your device
BAUD_RATE = 115200
OUTPUT_FILE = "actuator_data.csv"


def log_serial_data():
    ser = None
    try:
        # 1. Open Serial Connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT}. Initializing...")

        # 2. Wait for Board Reset/Boot
        time.sleep(2)

        # Clear any startup garbage
        ser.reset_input_buffer()

        # 3. Send Start Command
        print("Sending Start Command ('S')...")
        ser.write(b"S")

        # 4. Open CSV and Start Logging
        with open(OUTPUT_FILE, "w", newline="") as csvfile:
            print(f"Logging data to {OUTPUT_FILE}...")

            while True:
                if ser.in_waiting > 0:
                    try:
                        # Read line, decode, and strip whitespace
                        line = ser.readline().decode("utf-8", errors="ignore").strip()

                        if line:
                            # Show progress in terminal (always print)
                            print(line)

                            # Check for STOP condition
                            if "STOP" in line:
                                print("\n--- CAPTURE COMPLETE ---")
                                print(f"Data saved to {OUTPUT_FILE}")
                                break

                            # ONLY write to file if it's NOT the stop message
                            # (The 'break' above handles the exit, but just to be safe/clear)
                            csvfile.write(line + "\n")

                    except UnicodeDecodeError:
                        continue

    except serial.SerialException:
        print(
            f"Error: Could not open {SERIAL_PORT}. Check connections and close Arduino IDE."
        )
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed.")


if __name__ == "__main__":
    log_serial_data()
