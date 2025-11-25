#!/usr/bin/env python3
import glob, os, sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    data_dir = sys.argv[1] if len(sys.argv)>1 else 'wbc_data'
    files = sorted(glob.glob(os.path.join(data_dir,'wbc_metrics_*.csv')))
    if not files:
        print("No metrics found in", data_dir); return
    dfs=[]
    for f in files:
        df=pd.read_csv(f)
        dfs.append(df)
    df=pd.concat(dfs, ignore_index=True)
    # pivot by mode
    pivot=df.groupby('mode').agg({
        'E_hold':'mean','jerk_proxy':'mean','A_omega':'mean',
        'rho_min_m':'mean','p_rho_lt0_pct':'mean',
        'solve_ms_median':'mean','solve_ms_p95':'mean','fallback_pct':'mean',
    })
    if 'raw' not in pivot.index:
        print("RAW baseline missing (mode=raw). Plotting absolute."); base=None
    else:
        base=pivot.loc['raw']

    # build ratios to raw where meaningful
    to_ratio=['E_hold','jerk_proxy','A_omega']
    ratios=pivot.copy()
    if base is not None:
        for k in to_ratio:
            ratios[k]=pivot[k]/max(1e-6, base[k])
    # Bars
    labels=['raw','pocs','osqp']
    cols=[c for c in to_ratio]
    fig,ax=plt.subplots(figsize=(7.0,3.8))
    width=0.24
    x=range(len(labels))
    for i,c in enumerate(cols):
        y=[ratios[c].get(l, float('nan')) for l in labels]
        ax.bar([xx+i*width for xx in x], y, width, label=c)
    ax.set_xticks([xx+width for xx in x])
    ax.set_xticklabels([s.upper() for s in labels])
    ax.set_ylabel('ratio to RAW (lower is better)')
    ax.set_title('Velocity-level stabilizer (ratios to RAW)')
    ax.legend()
    fig.tight_layout()
    out=os.path.join(data_dir,'fig_wbc_graph.png')
    plt.savefig(out, dpi=160)
    print("Saved", out)

if __name__=='__main__':
    main()
