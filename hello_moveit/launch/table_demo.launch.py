from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # ▶ MSA 原生 demo（完全不改 robot_description）
    moveit = MoveItConfigsBuilder("lrmate-200id",
                                  package_name="cobot_moveit_config").to_moveit_configs()
    demo = generate_demo_launch(moveit)

    # ▶ 桌子自己獨立 publish
    table_urdf = os.path.join(
        os.path.expanduser("~"),
        "ws_moveit", "src", "lrmate_200id-main", "models", "table" ,"table.urdf")

    table_rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="table_state_pub",
        output="screen",
        parameters=[{"robot_description": open(table_urdf).read()}]
    )

    # ▶ world ➜ table_link  (桌子放原點)
    tf_world_table = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_table",
        arguments=["0", "0", "0", "0", "0", "0",  # xyz rpy
                   "world", "table_link"]
    )

    # ▶ table_link ➜ Base  (把 Base 提到 0.405 m)
    tf_table_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_table_base",
        arguments=["0", "0", "0.81", "0", "0", "0",
                   "table_link", "Base"]
    )

    return LaunchDescription(
        demo.entities + [table_rsp, tf_world_table, tf_table_base]
    )
