#!/usr/bin/env bash
set -e
trial_id="${1:-trial_000}"
out_dir="${2:-$HOME/go2_ws/experiments}"
stop_d="${3:-0.60}"
band="${4:-0.08}"
yaw_dead="${5:-0.12}"
tau_enter="${6:-0.25}"
tau_hold="${7:-1.00}"
Tmax="${8:-60.0}"

echo "[run_trial] trial_id=$trial_id out_dir=$out_dir stop_d=$stop_d band=$band yaw_dead=$yaw_dead tau_enter=$tau_enter tau_hold=$tau_hold Tmax=$Tmax"

ros2 launch go2_d1_eval csr_eval.launch.py \
  trial_id:=$trial_id \
  out_dir:=$out_dir \
  stop_distance_m:=$stop_d \
  stop_band_m:=$band \
  yaw_dead_rad:=$yaw_dead \
  tau_enter_s:=$tau_enter \
  tau_hold_s:=$tau_hold \
  max_duration_s:=$Tmax
