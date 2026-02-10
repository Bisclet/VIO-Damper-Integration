#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys

fs = 200.0
dt = 1.0 / fs

if len(sys.argv) < 2:
    print("Usage: python allan_imu.py imu.csv")
    sys.exit(1)

fname = sys.argv[1]

# Load CSV
df = pd.read_csv(fname)

acc = df[["ax", "ay", "az"]].values
gyro = df[["gx", "gy", "gz"]].values

def allan_deviation(data, fs):
    data = np.asarray(data)
    N = len(data)

    taus = np.logspace(0, np.log10(N/10), 80)
    taus = np.unique(np.round(taus)).astype(int)

    adev = []

    cumsum = np.cumsum(np.insert(data, 0, 0))

    for m in taus:
        if 2*m >= N:
            break

        # fast window averages
        y = (cumsum[m:] - cumsum[:-m]) / m

        diff = y[2*m:] - 2*y[m:-m] + y[:-2*m]

        sigma2 = 0.5 * np.mean(diff**2)
        adev.append(np.sqrt(sigma2))

    taus = taus[:len(adev)] / fs
    adev = np.array(adev)

    return taus, adev



def analyze(signal, name):
    taus, adev = allan_deviation(signal, fs)

    # Fit white noise region (slope -1/2)
    mask_w = (taus > 0.02) & (taus < 1.0)
    p_w = np.polyfit(np.log10(taus[mask_w]),
                     np.log10(adev[mask_w]), 1)
    sigma_w = 10**np.polyval(p_w, np.log10(1.0))

    # Fit bias region (slope +1/2)
    mask_b = (taus > 3) & (taus < 30)
    p_b = np.polyfit(np.log10(taus[mask_b]),
                     np.log10(adev[mask_b]), 1)
    sigma_b = 10**np.polyval(p_b, np.log10(3.0))

    print(f"\n{name}")
    print("White noise σw =", sigma_w)
    print("Bias RW σb    =", sigma_b)

    plt.loglog(taus, adev, label=name)

    return sigma_w, sigma_b

plt.figure(figsize=(8,6))

print("Accelerometer:")
for i, ax in enumerate(["x","y","z"]):
    analyze(acc[:,i], f"acc_{ax}")

print("\nGyroscope:")
for i, ax in enumerate(["x","y","z"]):
    analyze(gyro[:,i], f"gyro_{ax}")

# ===== Convert to Basalt format =====
rate = fs
scale = np.sqrt(rate)

acc_sigma_w = []
acc_sigma_b = []
gyro_sigma_w = []
gyro_sigma_b = []

# recompute to store values
for i in range(3):
    w, b = analyze(acc[:,i], f"acc_export_{i}")
    acc_sigma_w.append(w)
    acc_sigma_b.append(b)

for i in range(3):
    w, b = analyze(gyro[:,i], f"gyro_export_{i}")
    gyro_sigma_w.append(w)
    gyro_sigma_b.append(b)

acc_noise = np.array(acc_sigma_w) * scale
gyro_noise = np.array(gyro_sigma_w) * scale
acc_bias  = np.array(acc_sigma_b) * scale
gyro_bias = np.array(gyro_sigma_b) * scale

print("\n===== Basalt IMU block =====\n")
print('"accel_noise_std":', list(acc_noise))
print('"gyro_noise_std": ', list(gyro_noise))
print('"accel_bias_std":', list(acc_bias))
print('"gyro_bias_std": ', list(gyro_bias))

# ===== Static bias export (Basalt) =====

g = 9.81

acc_mean = np.mean(acc, axis=0)
gyro_mean = np.mean(gyro, axis=0)

# ===== Correct gravity removal =====

g = 9.81

acc_mean = np.mean(acc, axis=0)
gyro_mean = np.mean(gyro, axis=0)

gravity_dir = acc_mean / np.linalg.norm(acc_mean)
gravity_vec = gravity_dir * g

acc_bias = acc_mean - gravity_vec


print("\n===== Basalt bias block =====\n")

print('"calib_accel_bias":',
      list(acc_bias) + [0]*6)

print('"calib_gyro_bias":',
      list(gyro_mean) + [0]*9)


plt.xlabel("Integration time τ [s]")
plt.ylabel("Allan deviation")
plt.grid(True, which="both")
plt.legend()
plt.title("IMU Allan deviation")
plt.show()
