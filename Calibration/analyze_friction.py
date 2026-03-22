import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DATA_FILE = "friction_torque_step_map.csv"

def analyze_constant_torque_friction():
    print(f"Loading {DATA_FILE}...\n")
    try:
        df = pd.read_csv(DATA_FILE)
    except FileNotFoundError:
        print(f"Error: Could not find '{DATA_FILE}'.")
        return

    # Average the Forward and Reverse terminal velocities for each applied torque
    summary = df.groupby('Applied_Torque_Nm')['Terminal_Velocity_rad_s'].mean().reset_index()
    
    torques = summary['Applied_Torque_Nm'].values
    velocities = summary['Terminal_Velocity_rad_s'].values

    # Perform a 1st-degree polynomial fit (Linear Regression: y = mx + b)
    # y = Torque, x = Velocity
    m, b = np.polyfit(velocities, torques, 1)

    print("==================================================")
    print("   ISAAC LAB DYNAMICS ANALYSIS (STEADY STATE) ")
    print("==================================================")
    for index, row in summary.iterrows():
        print(f" Applied {row['Applied_Torque_Nm']:.2f} Nm -> Settled at {row['Terminal_Velocity_rad_s']:.4f} rad/s")
    
    print("-" * 50)
    print(f" Coulomb Friction (b) : {b:.4f} Nm")
    print(f" Viscous Damping  (m) : {m:.4f} Nms/rad")
    print("==================================================\n")

    # --- PLOTTING ---
    plt.figure(figsize=(10, 6))

    # Scatter the raw Forward and Reverse data
    df_fwd = df[df['Direction'] == 'Forward']
    df_rev = df[df['Direction'] == 'Reverse']
    plt.scatter(df_fwd['Terminal_Velocity_rad_s'], df_fwd['Applied_Torque_Nm'], color='dodgerblue', s=40, label='Forward')
    plt.scatter(df_rev['Terminal_Velocity_rad_s'], df_rev['Applied_Torque_Nm'], color='darkorange', s=40, label='Reverse')

    # Line of Best Fit
    x_line = np.linspace(0, max(velocities) * 1.1, 100)
    y_line = m * x_line + b
    plt.plot(x_line, y_line, color='red', linewidth=2, linestyle='--', label=f'Linear Fit (y = {m:.4f}x + {b:.4f})')

    plt.title('Constant Torque System Identification', fontsize=14, fontweight='bold')
    plt.xlabel('Terminal Velocity (rad/s)', fontsize=12)
    plt.ylabel('Applied Constant Torque (Nm)', fontsize=12)
    plt.xlim(left=0)   
    plt.ylim(bottom=0) 
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='lower right')

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    analyze_constant_torque_friction()