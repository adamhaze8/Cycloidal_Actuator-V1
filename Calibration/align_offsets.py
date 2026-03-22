import json
import math

CONFIG_FILE = "robot_config.json"
TARGET_ANGLE = math.pi / 2
INCREMENT = (2 * math.pi) / 6     # The physical symmetry step (pi/3 radians)

def align_zero_offsets():
    # 1. Load the current JSON configuration
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find {CONFIG_FILE}. Make sure it is in the same folder.")
        return

    print(f"{'Actuator':<15} | {'Old Offset':<12} | {'Steps Shifted':<15} | {'New Offset'}")
    print("-" * 65)

    # 2. Iterate through every actuator and math out the new offset
    for joint_name, data in config['actuators'].items():
        current_offset = data['zero_offset']
        
        # Calculate how far we are from the new target (pi/4)
        difference = TARGET_ANGLE - current_offset
        
        # Determine how many "increments" of pi/3 we need to add or subtract.
        # round() automatically handles negatives (subtracting) if the difference is negative.
        steps_needed = round(difference / INCREMENT)
        
        # Calculate the new offset and round it to 4 decimal places for clean JSON
        new_offset = current_offset + (steps_needed * INCREMENT)
        new_offset = round(new_offset, 4)
        
        # Update the dictionary
        config['actuators'][joint_name]['zero_offset'] = new_offset
        
        # Print the change so you can verify it
        print(f"{joint_name:<15} | {current_offset:>8.4f} rad | {steps_needed:>5} steps       | {new_offset:>8.4f} rad")

    # 3. Save the updated configuration back to the JSON file
    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)
        
    print("-" * 65)
    print(f"Success! All zero offsets have been aligned to ~0.7854 rad (pi/4).")
    print(f"File saved: {CONFIG_FILE}")

if __name__ == "__main__":
    align_zero_offsets()