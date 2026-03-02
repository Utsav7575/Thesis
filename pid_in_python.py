import pandas as pd
import matplotlib.pyplot as plt
import json

CSV_FILE = "ball_tracker_pid_python.csv"
OUT_PREFIX = "archC_pid"

df = pd.read_csv(CSV_FILE)
df['timestamp'] = pd.to_datetime(df['timestamp'])
t0 = df['timestamp'].iloc[0]
df['t_s'] = (df['timestamp'] - t0).dt.total_seconds()

if 'detected' not in df.columns:
    df['detected'] = (~df['ball_x_norm'].isna() & ~df['ball_y_norm'].isna()).astype(int)

# Full ball positions
plt.figure(figsize=(12,6))
plt.plot(df['t_s'], df['ball_x_norm'], label='ball_x_norm')
plt.plot(df['t_s'], df['ball_y_norm'], label='ball_y_norm')
plt.xlabel("Time (s)")
plt.ylabel("Normalized position")
plt.title("Architecture C: Ball Position vs Time (All rows)")
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUT_PREFIX}_ballpos_full.png", dpi=150)
plt.close()

with open(f"{OUT_PREFIX}_ballpos_full.png.meta.json",'w') as f:
    json.dump({
        "caption":"Architecture C: ball_x_norm & ball_y_norm vs time (all rows)",
        "description":"Includes detected=0 rows from PID log."
    }, f, indent=2)

# Detected-only positions
df_det = df[df['detected'] == 1]
plt.figure(figsize=(12,6))
plt.plot(df_det['t_s'], df_det['ball_x_norm'], marker='o', label='ball_x_norm')
plt.plot(df_det['t_s'], df_det['ball_y_norm'], marker='o', label='ball_y_norm')
plt.xlabel("Time (s)")
plt.ylabel("Normalized position")
plt.title("Architecture C: Ball Position vs Time (Detected Only)")
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUT_PREFIX}_ballpos_detected.png", dpi=150)
plt.close()

with open(f"{OUT_PREFIX}_ballpos_detected.png.meta.json",'w') as f:
    json.dump({
        "caption":"Architecture C: ball_x_norm & ball_y_norm vs time (detected only)",
        "description":"Only frames where detected == 1 from PID log."
    }, f, indent=2)

# Servo commands (full)
plt.figure(figsize=(12,6))
plt.plot(df['t_s'], df['servo_x'], label='servo_x')
plt.plot(df['t_s'], df['servo_y'], label='servo_y')
plt.xlabel("Time (s)")
plt.ylabel("Servo angle (deg)")
plt.title("Architecture C: Servo Commands vs Time (All rows)")
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUT_PREFIX}_servo_full.png", dpi=150)
plt.close()

with open(f"{OUT_PREFIX}_servo_full.png.meta.json",'w') as f:
    json.dump({
        "caption":"Architecture C: servo_x & servo_y vs time (all rows)",
        "description":"Servo commands over time including detected=0 rows."
    }, f, indent=2)

# Export cleaned CSV
df[['timestamp','t_s','ball_x_norm','ball_y_norm','servo_x','servo_y','detected']].to_csv(f"{OUT_PREFIX}_processed.csv", index=False)