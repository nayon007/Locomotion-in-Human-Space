#!/usr/bin/env python3
import sys, csv, numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("usage: plot_trajectories.py <timeseries.csv> [out.png]")
    sys.exit(1)

ts_csv = sys.argv[1]
out_png = sys.argv[2] if len(sys.argv) > 2 else "fig4_like.png"

rows = []
with open(ts_csv, 'r') as f:
    r = csv.DictReader(f)
    for row in r:
        rows.append(row)

def f(row,k):
    try:
        s = row[k]
        return float(s) if s not in ("", "nan", "NaN") else np.nan
    except: return np.nan

t = np.array([float(r['t']) for r in rows])
x = np.array([f(r,'x_base') for r in rows])
y = np.array([f(r,'y_base') for r in rows])
ex = np.array([f(r,'ex_px') for r in rows])
band = np.array([int(r['in_band']) for r in rows])

# Plot top: approach trajectory in base (x forward, y left)
plt.figure(figsize=(7,4))
plt.plot(x, y, linewidth=2)
plt.axvline(x=0.60, linestyle='--', alpha=0.5)  # nominal stop distance
plt.xlabel('x (m)'); plt.ylabel('y (m)'); plt.title('Approach trajectory (base frame)')
plt.grid(True)
plt.tight_layout()
plt.savefig(out_png.replace('.png','_traj.png'), dpi=200)

# Plot bottom: centering error over time with in-band shading
plt.figure(figsize=(7,4))
plt.plot(t, ex, linewidth=2)
yb = np.where(band==1, ex, np.nan)
plt.plot(t, yb, linewidth=4, alpha=0.4)  # highlight in-band segments
plt.xlabel('time (s)'); plt.ylabel('e_x (px)'); plt.title('Centering error over time')
plt.grid(True)
plt.tight_layout()
plt.savefig(out_png.replace('.png','_ex.png'), dpi=200)

print("Saved:", out_png.replace('.png','_traj.png'), "and", out_png.replace('.png','_ex.png'))
