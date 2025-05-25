from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 設定 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("cobot").to_moveit_configs()

    # 定義 hello_moveit 節點
    hello_moveit_node = Node(
        package="hello_moveit",
        executable="simple_planning",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # 載入 URDF
            moveit_config.robot_description_semantic,  # 載入 SRDF
            moveit_config.robot_description_kinematics,  # 載入 kinematics.yaml
        ],
    )

    return LaunchDescription([hello_moveit_node])
