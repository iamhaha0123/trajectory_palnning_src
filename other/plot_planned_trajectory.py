import pandas as pd
import matplotlib.pyplot as plt

def plot_planned_trajectory(csv_file="interpolated_trajectory_100Hz.csv"):
    # 讀取 CSV 檔案
    df_planned = pd.read_csv(csv_file)

    # 確保時間軸從 0 開始
    df_planned["time"] -= df_planned["time"].iloc[0]

    # 繪製 6 個關節的變化圖
    plt.figure(figsize=(10, 6))

    for i in range(6):
        joint_name = f"joint{i+1}"
        plt.plot(df_planned["time"], df_planned[joint_name], label=f"Joint {i+1}")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint Angle (rad)")
    plt.title("Planned Joint Angles Over Time")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    plot_planned_trajectory()
