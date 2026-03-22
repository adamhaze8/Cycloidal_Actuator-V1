import math

# ==========================================
#   USER CONFIGURATION
# ==========================================
BELT_LENGTH = 180  # The specific belt length (mm)
TARGET_RATIO = 2.0  # Desired Ratio (e.g., 2.0 for 2:1)
MIN_CENTER_DIST = 35  # Minimum allowed distance (mm)
MAX_CENTER_DIST = 45 # Maximum allowed distance (mm)

# GT2 Specs
PITCH = 2.0  # Distance between teeth (mm)
PITCH_LINE_OFFSET = 0.51  # Reduction to get Outside Diameter (mm)

# ==========================================
#   CALCULATION LOGIC
# ==========================================


def calculate_center_distance(belt_len, d_large, d_small):
    """
    Calculates the center-to-center distance based on belt length and pitch diameters.
    """
    if d_large == d_small:
        return (belt_len - (math.pi * d_large)) / 2.0

    P = belt_len - (math.pi / 2.0) * (d_large + d_small)
    discriminant = P**2 - 2 * (d_large - d_small) ** 2

    if discriminant < 0:
        return None

    return (P + math.sqrt(discriminant)) / 4.0


def find_pulleys():
    print(
        f"\nConfiguration: Belt={BELT_LENGTH}mm | Ratio={TARGET_RATIO}:1 | Dist Range={MIN_CENTER_DIST}-{MAX_CENTER_DIST}mm"
    )
    print("=" * 110)
    # Header
    print(
        f"{'TEETH':<10} | {'SMALL PULLEY (mm)':<22} | {'LARGE PULLEY (mm)':<22} | {'CENTER':<10}"
    )
    print(
        f"{'(S / L)':<10} | {'Pitch Dia  /  Outer Dia':<22} | {'Pitch Dia  /  Outer Dia':<22} | {'DIST'}"
    )
    print("-" * 110)

    found_solution = False

    for t_small in range(10, 150):
        t_large = t_small * TARGET_RATIO

        if not t_large.is_integer():
            continue
        t_large = int(t_large)

        # Calculate Diameters
        pd_small = (t_small * PITCH) / math.pi
        pd_large = (t_large * PITCH) / math.pi

        od_small = pd_small - PITCH_LINE_OFFSET
        od_large = pd_large - PITCH_LINE_OFFSET

        # Calculate Distance
        c_dist = calculate_center_distance(BELT_LENGTH, pd_large, pd_small)

        if c_dist is None:
            continue

        # Check Range
        if MIN_CENTER_DIST <= c_dist <= MAX_CENTER_DIST:
            found_solution = True

            # Formatting strings for cleaner output
            small_dims = f"{pd_small:6.2f}     {od_small:6.2f}"
            large_dims = f"{pd_large:6.2f}     {od_large:6.2f}"

            print(
                f"{t_small:<3} / {t_large:<4} | {small_dims:<22} | {large_dims:<22} | {c_dist:.2f} mm"
            )

    if not found_solution:
        print("\nNo exact matches found.")


if __name__ == "__main__":
    find_pulleys()
