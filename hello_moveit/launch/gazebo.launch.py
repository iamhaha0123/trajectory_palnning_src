import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


import xacro
import re

def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    package_name = 'lrmate_200id'
    pkg_share = FindPackageShare(package='lrmate_200id').find('lrmate_200id')
    
    # 找到 URDF 檔案
    
    urdf_name = "LRMate-200iD_convex.urdf"
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    robot_name_in_model = 'cobot'
    
    # 找到 World 檔案
    
    world_file_name = 'conveyor_belt_world.world'  
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    
    # 設定 GAZEBO_MODEL_PATH確保 Gazebo 可以找到 ConveyorBelt
    model_path = os.path.join(pkg_share, 'models')
    resource_path = pkg_share  # 包含 worlds 等資源

    # 設定 GAZEBO_MODEL_PATH 確保 Gazebo 可以找到模型
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_share, 'models') + ":" + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # 確保不覆蓋 Gazebo 內建資源
    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_share) + ":" + os.environ.get('GAZEBO_RESOURCE_PATH', '') + ":/usr/share/gazebo-11"
    )
    # 啟動 Gazebo
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', world_path , '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 解析 URDF
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': remove_comments(doc.toxml())}

    # robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # 生成機器人
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-topic',  'robot_description',
            # 以下是「相對 table::link」的位姿
            '-x', '0.0',            # 桌面 X 軸中央
            '-y', '0.0',            # 桌面 Y 軸中央
            '-z', '0.0',           # 放在桌面高度
            '-R', '0', '-P', '0', '-Y', '0',
            '-reference_frame', 'table::link_1'   
        ],
        output='screen'
    )

    # joint_state_broadcaster
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # arm_controller
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # 設定啟動順序
    close_evt1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_joint_state_controller]
        )
    )

    close_evt2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_controller]
        )
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    return ld


