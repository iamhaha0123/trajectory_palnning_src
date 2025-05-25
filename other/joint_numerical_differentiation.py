import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

# 讀取規劃軌跡 (100Hz 插值後的)
df_planned = pd.read_csv("interpolated_trajectory_100Hz.csv")

# 讀取執行軌跡 (時間歸零後的)
df_executed = pd.read_csv("active_motion_zeroed.csv")

# **對執行軌跡做內插，確保時間點對齊**
time_new = df_planned["time"]  # 以規劃軌跡的時間為基準
df_executed_interp = pd.DataFrame({"time": time_new})

for i in range(1, 7):  # joint1 ~ joint6
    f_interp = interp1d(df_executed["time"], df_executed[f"joint{i}"], kind="linear", fill_value="extrapolate")
    df_executed_interp[f"joint{i}"] = f_interp(time_new)

# **計算數值微分 (速度)**
df_planned_speed = df_planned.copy()
df_executed_speed = df_executed_interp.copy()

for i in range(1, 7):
    dt_planned = np.gradient(df_planned["time"])  # 計算時間間隔
    dt_executed = np.gradient(df_executed_interp["time"])

    # **避免時間間隔過小**
    dt_planned[dt_planned < 1e-6] = 1e-6
    dt_executed[dt_executed < 1e-6] = 1e-6

    # **使用中心差分法**
    df_planned_speed[f"joint{i}"] = np.gradient(df_planned[f"joint{i}"], dt_planned, edge_order=2)
    df_executed_speed[f"joint{i}"] = np.gradient(df_executed_interp[f"joint{i}"], dt_executed, edge_order=2)

# **計算速度誤差**
df_speed_error = df_executed_speed.copy()
for i in range(1, 7):
    df_speed_error[f"joint{i}"] = df_executed_speed[f"joint{i}"] - df_planned_speed[f"joint{i}"]

# **繪製速度誤差曲線**
plt.figure(figsize=(10, 5))

for i in range(1, 7):
    plt.plot(df_speed_error["time"], df_speed_error[f"joint{i}"], label=f"Joint {i} Speed Error")

plt.axhline(0, color='black', linestyle='--')  # 畫出 0 軸，方便判斷
plt.xlabel("Time (s)")
plt.ylabel("Speed Error (Executed - Planned)")
plt.title("Joint Speed Error (Executed - Planned)")
plt.legend()
plt.grid()
plt.show()
