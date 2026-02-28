import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load cleaned CSVs
files = {
    'A_simple': 'archA_simple_processed.csv',
    'B_kalman': 'archB_kalman_processed.csv',
    'C_pid': 'archC_pid_processed.csv'
}

common = {}
for k, f in files.items():
    df = pd.read_csv(f)
    # Normalize columns
    cols = df.columns.str.lower()
    df.columns = cols
    # Rename PID columns to x_norm / y_norm
    if 'ball_x_norm' in df.columns and 'ball_y_norm' in df.columns:
        df = df.rename(columns={'ball_x_norm':'x_norm','ball_y_norm':'y_norm'})
    common[k] = df

# ---------- METRICS ----------
metrics=[]
for k, df in common.items():
    dfv = df[df['detected']>0]
    if 'x_norm' not in dfv.columns or 'y_norm' not in dfv.columns:
        continue
    e = np.sqrt(dfv['x_norm']**2 + dfv['y_norm']**2)
    metrics.append({
        'architecture': k,
        'n_samples': len(dfv),
        'rmse': np.sqrt(np.mean(e**2)),
        'mae': np.mean(np.abs(e))
    })

metrics_df = pd.DataFrame(metrics)
metrics_df.to_csv('tracking_metrics.csv', index=False)

# ---------- ERROR OVER TIME ----------
plt.figure(figsize=(12,6))
for k, df in common.items():
    dfv = df[df['detected']>0]
    if 'x_norm' not in dfv.columns or 'y_norm' not in dfv.columns:
        continue
    e = np.sqrt(dfv['x_norm']**2 + dfv['y_norm']**2)
    dt = np.median(np.diff(dfv['t_s'])) if len(dfv) > 5 else 0.02
    win = max(1,int(0.5/dt))
    e_roll = pd.Series(e).rolling(win,min_periods=1).mean()
    plt.plot(dfv['t_s'], e_roll, label=k)
plt.xlabel("Time (s)")
plt.ylabel("|e| (norm units)")
plt.title("Tracking error over time (rolling mean)")
plt.grid(True,alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("error_over_time.png", dpi=200)
plt.close()

# ---------- X/Y OVER TIME ----------
fig, axs = plt.subplots(2,1, figsize=(12,8), sharex=True)
for k, df in common.items():
    dfv = df[df['detected']>0]
    if 'x_norm' not in dfv.columns or 'y_norm' not in dfv.columns:
        continue
    axs[0].plot(dfv['t_s'], dfv['x_norm'], label=k)
    axs[1].plot(dfv['t_s'], dfv['y_norm'], label=k)
axs[0].axhline(0,color='k',alpha=0.5)
axs[1].axhline(0,color='k',alpha=0.5)
axs[0].set_ylabel('x_norm')
axs[1].set_ylabel('y_norm')
axs[1].set_xlabel('Time (s)')
axs[0].set_title('Ball position vs time')
axs[0].legend()
for ax in axs:
    ax.grid(True,alpha=0.3)
plt.tight_layout()
plt.savefig("xy_over_time.png", dpi=200)
plt.close()