import pandas as pd
import matplotlib.pyplot as plt
import json

# ---------- CONFIG ----------
CSV_FILE = "ball_tracker_simple.csv"
OUT_PREFIX = "archA_simple"
# -----------------------------

# Load CSV
df = pd.read_csv(CSV_FILE)
df['timestamp'] = pd.to_datetime(df['timestamp'])
t0 = df['timestamp'].iloc[0]
df['t_s'] = (df['timestamp'] - t0).dt.total_seconds()

# Fill detected if missing
if 'detected' not in df.columns:
    df['detected'] = (~df['x_norm'].isna() & ~df['y_norm'].isna()).astype(int)

# ---------- PLOTS ----------

# Full positions
plt.figure(figsize=(12,6))
plt.plot(df['t_s'], df['x_norm'], label='x_norm')
plt.plot(df['t_s'], df['y_norm'], label='y_norm')
plt.xlabel("Time (s)")
plt.ylabel("Normalized position")
plt.title("Architecture A: Position vs Time (All rows)")
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUT_PREFIX}_pos_full.png", dpi=150)
plt.close()

with open(f"{OUT_PREFIX}_pos_full.png.meta.json",'w') as f:
    json.dump({
        "caption":"Architecture A: x_norm & y_norm vs time (all rows)",
        "description":"Includes detected=0 rows."
    }, f, indent=2)

# Detected-only positions
df_det = df[df['detected'] == 1]
plt.figure(figsize=(12,6))
plt.plot(df_det['t_s'], df_det['x_norm'], marker='o', label='x_norm')
plt.plot(df_det['t_s'], df_det['y_norm'], marker='o', label='y_norm')
plt.xlabel("Time (s)")
plt.ylabel("Normalized position")
plt.title("Architecture A: Position vs Time (Detected Only)")
plt.legend()
plt.tight_layout()
plt.savefig(f"{OUT_PREFIX}_pos_detected.png", dpi=150)
plt.close()

with open(f"{OUT_PREFIX}_pos_detected.png.meta.json",'w') as f:
    json.dump({
        "caption":"Architecture A: x_norm & y_norm vs time (detected only)",
        "description":"Only frames where detected == 1."
    }, f, indent=2)

# Export cleaned CSV
df[['timestamp','t_s','x_norm','y_norm','detected']].to_csv(f"{OUT_PREFIX}_processed.csv", index=False)