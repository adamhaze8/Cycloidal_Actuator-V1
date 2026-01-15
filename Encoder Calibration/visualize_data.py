import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ==========================================
#           USER CONFIGURATION
# ==========================================
FILE_PATH = "actuator_data.csv"

# 1. RANGE TO ANALYZE (In Output Degrees)
# Define the window of motion you want to inspect.
RANGE_START_DEG = -300
RANGE_END_DEG = -660

# 2. ANALYSIS PARAMETERS
# We ignore points that are physically close to each other (e.g., within 5 degrees).
# We only care if a point looks like another point "far away" (a Ghost).
PHYSICAL_SEPARATION_THRESHOLD = 5.0  # Degrees

# ==========================================
#           DATA PROCESSING
# ==========================================
try:
    # Load and Filter
    df = pd.read_csv(FILE_PATH, skipinitialspace=True)

    lower_bound = min(RANGE_START_DEG, RANGE_END_DEG)
    upper_bound = max(RANGE_START_DEG, RANGE_END_DEG)

    subset = df[
        (df["Output_Accum"] >= lower_bound) & (df["Output_Accum"] <= upper_bound)
    ].copy()

    if len(subset) < 10:
        print("Error: Not enough data points to analyze.")
        exit()

    print(f"--- PROCESSING ---")
    print(f"Analyzing {len(subset)} points from {lower_bound}° to {upper_bound}°...")

except Exception as e:
    print(f"Error: {e}")
    exit()

# ==========================================
#        AMBIGUITY ANALYSIS (VECTORIZED)
# ==========================================
# We want to find the "Nearest Sensor Neighbor" for every point,
# excluding points that are physically nearby.

# 1. Prepare Arrays
motor_raw = subset["Motor_Raw"].values
output_raw = subset["MT6_Raw"].values
output_accum = subset["Output_Accum"].values
N = len(subset)


# 2. Helper: Vectorized Angular Distance (0-360 handling)
def ang_diff(a, b):
    d = np.abs(a - b)
    return np.minimum(d, 360.0 - d)


# 3. Calculate Uniqueness Score for each point
# (This can be slow for massive datasets, but fine for <10k points)
uniqueness_scores = []

for i in range(N):
    # Get current point's sensors and physical position
    m_i = motor_raw[i]
    o_i = output_raw[i]
    pos_i = output_accum[i]

    # Calculate physical distance to ALL other points
    # We only care about comparing against points > 5 degrees away
    physical_dists = np.abs(output_accum - pos_i)
    valid_mask = physical_dists > PHYSICAL_SEPARATION_THRESHOLD

    if not np.any(valid_mask):
        # If no points are far enough away, we can't calculate a ghost score.
        # Assign a default "Safe" value (e.g., 180 deg) or NaN
        uniqueness_scores.append(180.0)
        continue

    # Get the sensors of the valid candidates
    m_candidates = motor_raw[valid_mask]
    o_candidates = output_raw[valid_mask]

    # Calculate Sensor Distance (Euclidean distance in Sensor Space)
    # Dist = sqrt( (DeltaMotor)^2 + (DeltaOutput)^2 )
    d_motor = ang_diff(m_i, m_candidates)
    d_output = ang_diff(o_i, o_candidates)

    # Combined sensor difference
    sensor_distances = np.sqrt(d_motor**2 + d_output**2)

    # The Score is the MINIMUM distance found.
    # High Score = Unique. Low Score = Ambiguous.
    min_dist = np.min(sensor_distances)
    uniqueness_scores.append(min_dist)

subset["Uniqueness_Score"] = uniqueness_scores

# ==========================================
#              PLOTTING
# ==========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# PLOT 1: Raw Data
ax1.plot(
    subset["Output_Accum"],
    subset["Motor_Raw"],
    label="Motor Raw",
    color="green",
    alpha=0.6,
    lw=1,
)
ax1.plot(
    subset["Output_Accum"],
    subset["MT6_Raw"],
    label="Output Raw",
    color="red",
    alpha=0.6,
    lw=1,
)
ax1.set_title(f"1. Raw Sensor Data ({lower_bound}° to {upper_bound}°)")
ax1.set_ylabel("Raw Angle (0-360°)")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)

# PLOT 2: Uniqueness / Ambiguity Analysis
# We define a "Danger Zone" threshold (e.g., < 10 degrees difference)
DANGER_THRESHOLD = 3.0

ax2.plot(
    subset["Output_Accum"],
    subset["Uniqueness_Score"],
    color="blue",
    lw=2,
    label="Uniqueness Margin",
)
ax2.axhline(
    DANGER_THRESHOLD, color="orange", linestyle="--", label="Warning Threshold (10°)"
)
ax2.fill_between(
    subset["Output_Accum"],
    0,
    subset["Uniqueness_Score"],
    where=(subset["Uniqueness_Score"] < DANGER_THRESHOLD),
    color="red",
    alpha=0.3,
    label="Ambiguity Zone",
)

ax2.set_title("2. Position Uniqueness Analysis (Higher is Safer)")
ax2.set_xlabel("Actuator Output Angle (Degrees)")
ax2.set_ylabel("Sensor Difference (Deg)")
ax2.set_ylim(0, 100)  # Zoom in to see the danger zone clearly
ax2.legend(loc="upper right")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

# Report min margin
min_score = np.min(subset["Uniqueness_Score"])
print(f"Minimum Uniqueness Score: {min_score:.2f}°")
if min_score < DANGER_THRESHOLD:
    print("VERDICT: POTENTIAL AMBIGUITY DETECTED (See Red Zones)")
else:
    print("VERDICT: SAFE (No ambiguous positions found in this range)")
