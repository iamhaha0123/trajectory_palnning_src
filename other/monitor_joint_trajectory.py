import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import csv
import time

class JointDataRecorder(Node):
    def __init__(self):
        super().__init__('joint_data_recorder')

        # 訂閱關節狀態（實際執行）
        self.subscription_actual = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # 訂閱規劃軌跡（MoveIt 2 計算結果）
        self.subscription_planned = self.create_subscription(
            DisplayTrajectory, '/display_planned_path', self.planned_trajectory_callback, 10)

        # 記錄開始時間
        self.start_time = time.time()
        self.execution_start_time = None
        self.planning_start_time = None

        # 開啟 CSV 檔案寫入
        self.csv_file_actual = open('joint_states.csv', 'w', newline='')
        self.csv_file_planned = open('planned_trajectory.csv', 'w', newline='')

        self.csv_writer_actual = csv.writer(self.csv_file_actual)
        self.csv_writer_planned = csv.writer(self.csv_file_planned)

        # 寫入標題
        self.csv_writer_actual.writerow(["time", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.csv_writer_planned.writerow(["time", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])

    def joint_state_callback(self, msg):
        """記錄機械手臂的實際執行關節角度"""
        if self.execution_start_time is None:
            self.execution_start_time = time.time()

        current_time = time.time() - self.execution_start_time
        joint_positions = list(msg.position[:6])  # 取前6個關節
        self.csv_writer_actual.writerow([current_time] + joint_positions)

    def planned_trajectory_callback(self, msg):
        """記錄 MoveIt 2 的完整規劃軌跡"""
        if len(msg.trajectory) == 0:
            return

        if self.planning_start_time is None:
            self.planning_start_time = time.time()

        # 取得第一條軌跡
        joint_trajectory = msg.trajectory[0].joint_trajectory

        for point in joint_trajectory.points:
            planned_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            joint_positions = list(point.positions[:6])
            self.csv_writer_planned.writerow([planned_time] + joint_positions)

    def close_csv(self):
        """關閉 CSV 檔案"""
        self.csv_file_actual.close()
        self.csv_file_planned.close()

def main():
    rclpy.init()
    node = JointDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.close_csv()
        print("\n✅ `joint_states.csv` & `planned_trajectory.csv` 已儲存！")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
