import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# **讀取原始 CSV 檔案**
input_file = "planned_trajectory.csv"  # 你的原始 CSV 檔案名稱
output_file = "interpolated_trajectory_100Hz.csv"  # 100Hz 插值後的 CSV

# 讀取 CSV
df = pd.read_csv(input_file)
df.columns = ["time", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

# **確保時間軸嚴格遞增**
df = df.sort_values(by="time").drop_duplicates(subset="time")

# **確保所有數據類型為 float**
time_original = df["time"].astype(float).values
joint_data = df.iloc[:, 1:].astype(float).values

# **設定新的時間軸 (100Hz, 每 0.01s 一個點)**
time_new = np.arange(time_original[0], time_original[-1], 0.01)

# **對每個關節進行線性插值 (增加 fill_value="extrapolate")**
interp_funcs = [
    interp1d(time_original, joint_data[:, i], kind="linear", fill_value="extrapolate")
    for i in range(joint_data.shape[1])
]
new_joint_data = np.array([f(time_new) for f in interp_funcs]).T

# **建立新的 DataFrame**
df_new = pd.DataFrame(np.column_stack((time_new, new_joint_data)), columns=df.columns)

# **儲存新的 CSV**
df_new.to_csv(output_file, index=False)

print(f"✅ 插值完成！新檔案已儲存為 {output_file}")
