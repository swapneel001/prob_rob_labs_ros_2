#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd

CSV_PATH = "measurement_errors.csv"

if not os.path.isfile(CSV_PATH):
    raise FileNotFoundError(f"CSV file not found at {CSV_PATH}")

df = pd.read_csv(CSV_PATH)
print(f"loaded {len(df)} samples from {CSV_PATH}")

# keep only finite
cols = ["d_meas", "theta_meas", "d_err", "theta_err"]
df = df[cols].replace([np.inf, -np.inf], np.nan).dropna()

d_meas = df["d_meas"].to_numpy()
th_meas = df["theta_meas"].to_numpy()
d_err = df["d_err"].to_numpy()
th_err = df["theta_err"].to_numpy()

# define “reasonable” region
dist_mask = (d_meas >= 1.0) & (d_meas <= 10.0)
theta_mask = (np.abs(th_meas) <= 0.6)

# compute base variances in reasonable region
var_d_base = np.var(d_err[dist_mask], ddof=1) if dist_mask.sum() > 0 else 0.0
var_th_base = np.var(th_err[theta_mask], ddof=1) if theta_mask.sum() > 0 else 0.0

print(f"base var_d={var_d_base:.6f}, base var_theta={var_th_base:.6f}")

# stepwise inflation rule
def sigma_d2(d):
    if d < 1.0 or d > 10.0:
        return 3.0 * var_d_base
    return var_d_base

def sigma_theta2(theta):
    if abs(theta) > 0.6:
        return 3.0 * var_th_base
    return var_th_base

# example output
for test_d in [0.5, 2.0, 12.0]:
    print(f"d={test_d:.1f}, sigma_d^2={sigma_d2(test_d):.6f}")
for test_th in [0.3, 0.8, -1.0]:
    print(f"theta={test_th:.2f}, sigma_theta^2={sigma_theta2(test_th):.6f}")

print("done.")
