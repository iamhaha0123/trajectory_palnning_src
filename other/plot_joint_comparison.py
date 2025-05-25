import pandas as pd
import matplotlib.pyplot as plt

# 讀取規劃軌跡 (100Hz 插值後的)
planned_file = "interpolated_trajectory_100Hz.csv"
df_planned = pd.read_csv(planned_file)

# 讀取實際執行軌跡 (時間歸零後的)
executed_file = "active_motion_zeroed.csv"
df_executed = pd.read_csv(executed_file)

# 繪製比較圖
plt.figure(figsize=(10, 5))

for i in range(1, 7):  # joint1 ~ joint6
    plt.plot(df_planned["time"], df_planned[f"joint{i}"], '--', label=f"Planned Joint {i}")  # 規劃軌跡 (虛線)
    plt.plot(df_executed["time"], df_executed[f"joint{i}"], '-', label=f"Executed Joint {i}")  # 實際軌跡 (實線)

plt.xlabel("Time (s)")
plt.ylabel("Joint Position")
plt.title("Comparison: Planned vs Executed Trajectory")
plt.legend()
plt.grid()
plt.show()
