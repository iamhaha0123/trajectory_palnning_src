import pandas as pd
import matplotlib.pyplot as plt

def plot_joint_states(csv_file="active_motion_zeroed.csv"):
    try:
        # 讀取 joint_states.csv
        df_actual = pd.read_csv(csv_file)

        # 確保時間軸從 0 開始
        df_actual["time"] -= df_actual["time"].iloc[0]

        # 繪製 6 個關節的變化圖
        plt.figure(figsize=(10, 6))

        for i in range(6):
            joint_name = f"joint{i+1}"
            plt.plot(df_actual["time"], df_actual[joint_name], label=f"Joint {i+1}")

        plt.xlabel("Time (s)")
        plt.ylabel("Joint Angle (rad)")
        plt.title("Actual Joint Angles Over Time")
        plt.legend()
        plt.grid()
        plt.show()

    except FileNotFoundError:
        print(f"❌ 檔案 '{csv_file}' 未找到，請確認 `joint_states.csv` 是否存在並在正確目錄下運行程式。")

if __name__ == "__main__":
    plot_joint_states()
