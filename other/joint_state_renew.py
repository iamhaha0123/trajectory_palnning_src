import pandas as pd
import numpy as np

# 讀取原始數據
input_file = "joint_states.csv"  # 原始數據檔案
output_file = "active_motion_zeroed.csv"  # 時間歸零後的作動區間 CSV 檔案

df = pd.read_csv(input_file)

# 設定變化閾值 (決定機械手臂是否在動)
threshold = 1e-3  # 變化小於這個值視為靜止

# 計算關節變化率 (Δjoint)
df_diff = df.iloc[:, 1:].diff().abs()  # 計算每個關節的變化量
df["max_diff"] = df_diff.max(axis=1)  # 找出每一行的最大變化量

# 找到機械手臂 **開始** 及 **結束** 動作的時間
motion_start = df[df["max_diff"] > threshold]["time"].min()
motion_end = df[df["max_diff"] > threshold]["time"].max()

if pd.isna(motion_start) or pd.isna(motion_end):
    print("❌ 無法偵測到機械手臂的明顯運動")
else:
    print(f"✅ 機械手臂動作範圍: {motion_start:.3f}s ~ {motion_end:.3f}s")

    # 擷取作動區間數據
    df_active = df[(df["time"] >= motion_start) & (df["time"] <= motion_end)].drop(columns=["max_diff"])

    # **讓時間從 0 開始**
    df_active["time"] = df_active["time"] - motion_start

    # 存成新的 CSV 檔案
    df_active.to_csv(output_file, index=False)
    print(f"✅ 作動數據已歸零時間，已儲存為 {output_file}")
