#!/usr/bin/env python3
import sys, os, csv, glob

if len(sys.argv) < 2:
    print("usage: summarize_metrics.py <experiments_root>")
    sys.exit(1)

root = sys.argv[1]
rows = []
for trial_dir in sorted(glob.glob(os.path.join(root, '*'))):
    mpath = os.path.join(trial_dir, 'metrics.csv')
    if os.path.exists(mpath):
        with open(mpath, 'r') as f:
            r = list(csv.reader(f))
            if len(r) >= 2:
                rows.append(r[1])

if not rows:
    print("No metrics found under", root); sys.exit(0)

out = os.path.join(root, 'summary.csv')
with open(out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['trial_id','entry_ex_px','entry_ey_px','entry_ed_cm',
                'time_to_entry_s','dwell_achieved_s','residual_inband_cm',
                'residual_inband_px','jerk_proxy_mps3','fresh_fraction'])
    for r in rows: w.writerow(r)

print("Wrote summary:", out)
