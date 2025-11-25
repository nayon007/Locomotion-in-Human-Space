#!/usr/bin/env python3
import os, glob, argparse, json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_runs(patterns):
    files = []
    for p in patterns:
        files += glob.glob(p)
    runs = []
    for f in sorted(files):
        base = f[:-4]
        meta = base + '.meta.txt'
        label = 'unknown'
        entry_t = settle_t = None
        if os.path.exists(meta):
            md = {}
            with open(meta) as m:
                for line in m:
                    if '=' in line:
                        k,v = line.strip().split('=',1)
                        md[k]=v
            label = md.get('label', label)
            entry_t = float(md.get('entry_t','nan')) if md.get('entry_t','') not in ('None','nan','') else None
            settle_t = float(md.get('settle_t','nan')) if md.get('settle_t','') not in ('None','nan','') else None
        df = pd.read_csv(f)
        runs.append({'file':f,'label':label,'entry_t':entry_t,'settle_t':settle_t,'df':df})
    return runs

def plot_trajectories(runs, outpng):
    plt.figure()
    for r in runs:
        df = r['df']
        plt.plot(df['x'], df['y'], alpha=0.8, label=f"{r['label']}:{os.path.basename(r['file'])}")
    plt.axhline(0, lw=0.5, color='k')
    plt.gca().set_aspect('equal', 'box')
    plt.xlabel('Range x (m)')
    plt.ylabel('Lateral y (m)')
    plt.title('Approach Trajectories (base frame)')
    plt.legend(fontsize=7, loc='best')
    plt.grid(True, alpha=0.3)
    plt.savefig(outpng, dpi=200, bbox_inches='tight')
    print('Saved', outpng)

def plot_centering_error(runs, outpng):
    plt.figure()
    for r in runs:
        df = r['df']
        if r['entry_t'] is None:   # align by earliest time if no entry
            t0 = df['t'].iloc[0]
        else:
            t0 = r['entry_t']
        t_al = df['t'] - t0
        plt.plot(t_al, np.abs(df['y']), alpha=0.8, label=f"{r['label']}:{os.path.basename(r['file'])}")
    plt.xlabel('Time aligned at band entry t - t_entry (s)')
    plt.ylabel('|y| (m)')
    plt.title('Centering Error Over Time (aligned at band entry)')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=7, loc='best')
    plt.savefig(outpng, dpi=200, bbox_inches='tight')
    print('Saved', outpng)

def plot_jerk_vs_range(runs, outpng):
    plt.figure()
    for r in runs:
        df = r['df']
        j = df['jerk'].rolling(5, center=True, min_periods=1).mean()
        plt.plot(df['x'], j, alpha=0.8, label=f"{r['label']}:{os.path.basename(r['file'])}")
    plt.xlabel('Range x (m)')
    plt.ylabel('jerk proxy (m/s³)')
    plt.title('Jerk Proxy vs Range (decrease near person expected)')
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=7, loc='best')
    plt.savefig(outpng, dpi=200, bbox_inches='tight')
    print('Saved', outpng)

def summarize_metrics(runs):
    rows = []
    for r in runs:
        df = r['df']
        label = r['label']
        entry_t = r['entry_t']; settle_t = r['settle_t']
        dwell = residual = np.nan
        if settle_t is not None:
            dff = df[df['t'] >= settle_t]
            if len(dff) >= 3:
                dwell = dff['t'].iloc[-1] - settle_t
                residual = np.sqrt((dff['speed']**2).mean())
        # errors at entry: lateral |y| and range x
        if entry_t is not None:
            dfe = df.iloc[(df['t']-entry_t).abs().argsort()[:1]]
            e_lat = float(abs(dfe['y'].iloc[0]))
            e_rng = float(dfe['x'].iloc[0])
        else:
            e_lat = float(abs(df['y'].iloc[0])); e_rng = float(df['x'].iloc[0])
        rows.append({
            'file': os.path.basename(r['file']),
            'label': label,
            'e_lat_at_entry': e_lat,
            'x_at_entry': e_rng,
            'settle_t': settle_t,
            'dwell': dwell,
            'residual_rms': residual
        })
    import pandas as pd
    tab = pd.DataFrame(rows)
    print('\n=== METRICS ===')
    print(tab.to_string(index=False))
    outcsv = os.path.commonpath([os.path.dirname(r['file']) for r in runs])
    outcsv = os.path.join(outcsv, 'csr_metrics_summary.csv')
    tab.to_csv(outcsv, index=False)
    print('Saved', outcsv)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('files', nargs='+', help='CSV files or patterns (e.g., results/csr_*.csv)')
    ap.add_argument('--out_dir', default='.')
    args = ap.parse_args()
    runs = load_runs(args.files)
    os.makedirs(args.out_dir, exist_ok=True)
    plot_trajectories(runs, os.path.join(args.out_dir, 'fig4_trajectories.png'))
    plot_centering_error(runs, os.path.join(args.out_dir, 'fig4_centering_error.png'))
    plot_jerk_vs_range(runs, os.path.join(args.out_dir, 'fig4_jerk_vs_range.png'))
    summarize_metrics(runs)

if __name__ == '__main__':
    main()
