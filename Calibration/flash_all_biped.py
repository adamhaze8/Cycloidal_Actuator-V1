import serial.tools.list_ports
import json
import os
import subprocess
import time

# --- AUTOMATION CONFIGURATION ---
CONFIG_FILE = "biped_actuator_config.json"
FQBN = "STMicroelectronics:stm32:Disco:upload_method=swdMethod"

# Your explicit path to the STM32 Programmer
STM32_PROG = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

# Base paths for firmware
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FW_BIPED_LARGE = os.path.normpath(
    os.path.join(BASE_DIR, "..", "Firmware", "Large", "bipedFirmware")
)
FW_BIPED_SMALL = os.path.normpath(
    os.path.join(BASE_DIR, "..", "Firmware", "Small", "bipedFirmware")
)

# DEDICATED BUILD FOLDERS (The Fix)
BUILD_DIR_LARGE = os.path.join(BASE_DIR, "build", "Large")
BUILD_DIR_SMALL = os.path.join(BASE_DIR, "build", "Small")


def load_hardware_map():
    try:
        with open(CONFIG_FILE, "r") as f:
            return json.load(f).get("actuators", {})
    except Exception as e:
        print(f"[!] FATAL: Could not load {CONFIG_FILE}. ({e})")
        exit(1)


def scan_for_targets(hw_config):
    print("Scanning USB bus for ST-Links...\n")
    ports = serial.tools.list_ports.comports()
    targets = []
    seen_joints = set()

    for port in ports:
        if not port.serial_number:
            continue

        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            hw_serial = str(port.serial_number).strip()

            for joint_name, data in hw_config.items():
                config_serial = str(data.get("hw_serial", "")).strip()
                if not config_serial or config_serial == "None":
                    continue

                if config_serial in hw_serial and joint_name not in seen_joints:
                    motor_type = data.get("motor_type", "Unknown")
                    targets.append(
                        {"joint": joint_name, "serial": hw_serial, "type": motor_type}
                    )
                    seen_joints.add(joint_name)
                    break
    return targets


def pre_compile_firmwares(targets):
    """Compiles the required firmwares ONCE to save massive amounts of time."""
    needs_large = any(t["type"] == "G80" for t in targets)
    needs_small = any(t["type"] != "G80" for t in targets)

    print("==================================================")
    print(" PRE-COMPILING FIRMWARES (CLEAN BUILD)")
    print("==================================================")

    if needs_large:
        print(" -> Compiling Large (G80) Firmware...")
        os.makedirs(BUILD_DIR_LARGE, exist_ok=True)
        # Added --clean flag to force a fresh build
        cmd = f'arduino-cli compile --clean -b {FQBN} --output-dir "{BUILD_DIR_LARGE}" "{FW_BIPED_LARGE}"'
        subprocess.run(cmd, check=True, shell=True, capture_output=True)
        print("    [+] Large compilation successful.")

    if needs_small:
        print(" -> Compiling Small (G60) Firmware...")
        os.makedirs(BUILD_DIR_SMALL, exist_ok=True)
        # Added --clean flag to force a fresh build
        cmd = f'arduino-cli compile --clean -b {FQBN} --output-dir "{BUILD_DIR_SMALL}" "{FW_BIPED_SMALL}"'
        subprocess.run(cmd, check=True, shell=True, capture_output=True)
        print("    [+] Small compilation successful.")


def rapid_flash(target):
    """Directly injects the pre-compiled .bin file."""
    joint_name = target["joint"]
    hw_serial = target["serial"]

    # Route to the correct dedicated build folder
    if target["type"] == "G80":
        bin_file = os.path.join(BUILD_DIR_LARGE, "bipedFirmware.ino.bin")
    else:
        bin_file = os.path.join(BUILD_DIR_SMALL, "bipedFirmware.ino.bin")

    print(f" -> Flashing {joint_name} ({target['type']})...", end="", flush=True)

    flash_cmd = [
        STM32_PROG,
        "-c",
        "port=SWD",
        f"sn={hw_serial}",
        "-w",
        bin_file,
        "0x08000000",
        "-v",
        "-rst",
    ]

    try:
        # Capture output so it doesn't spam the console, keeping it clean
        subprocess.run(flash_cmd, check=True, capture_output=True)
        print(f" [SUCCESS]")
        return True
    except subprocess.CalledProcessError:
        print(f" [FAILED]")
        return False


def main():
    hw_config = load_hardware_map()
    targets = scan_for_targets(hw_config)

    if not targets:
        print("No recognized actuators found connected to this computer.")
        return

    print(f"Found {len(targets)} recognized actuators.")

    # 1. Compile everything once
    try:
        pre_compile_firmwares(targets)
    except subprocess.CalledProcessError as e:
        print("\n[!] FATAL: A compilation error occurred!")
        print(e.stderr.decode())
        return

    # 2. Rapid-fire flash the entire robot
    print("\n==================================================")
    print(" RAPID DEPLOYMENT SEQUENCE")
    print("==================================================")

    success_count = 0
    for target in targets:
        if rapid_flash(target):
            success_count += 1
            time.sleep(0.5)  # Just enough time for the USB bus to breathe

    print("\n" + "=" * 50)
    print(
        f"DEPLOYMENT COMPLETE: {success_count}/{len(targets)} Actuators fully updated."
    )
    print("=" * 50)


if __name__ == "__main__":
    main()
