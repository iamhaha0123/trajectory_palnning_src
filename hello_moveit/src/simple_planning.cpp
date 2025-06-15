#include <memory>
#include <thread>  
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "robot_interfaces/msg/arm_attitude.hpp"

void sub_test(const robot_interfaces::msg::ArmAttitude::SharedPtr msg, rclcpp::Logger logger) {
  RCLCPP_INFO(logger, "Received Plug Attitude: '%f , %f , %f , %f , %f , %f '",
    msg->x, msg->y, msg->z, msg->w, msg->p, msg->r);
}

int main(int argc, char *argv[])
{
  //初始化 ROS 2 節點
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "simple_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("simple_planning");

  //publisher
  auto trajectory_pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
  "planned_trajectory", 10);
  auto attitude_sub = node->create_subscription<robot_interfaces::msg::ArmAttitude>(
    "fan_attitude", 10, 
    [logger](const robot_interfaces::msg::ArmAttitude::SharedPtr msg) {
      sub_test(msg, logger);});

      

  // 建立 SingleThreadedExecutor 並將節點加入
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 啟動執行緒執行 executor 的 spin()
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 建立 MoveGroupInterface，指定規劃群組名稱
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // 設定規劃參數
  move_group_interface.setPlanningTime(1.0);  // 降低規劃時間，提高頻率
  move_group_interface.setGoalPositionTolerance(0.01);
  move_group_interface.setGoalOrientationTolerance(0.01);
  move_group_interface.setMaxVelocityScalingFactor(0.5);  // 降低速度比例，確保點數足夠
  move_group_interface.setMaxAccelerationScalingFactor(0.5);
  // 設定目標位姿
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.5;
  target_pose.position.y = 0.0;
  target_pose.position.z = 1.6;
  move_group_interface.setPoseTarget(target_pose);

  // 建立並初始化 MoveItVisualTools
  namespace rvt = rviz_visual_tools;
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "world", rvt::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();  // 清除所有標記
  moveit_visual_tools.loadRemoteControl(); // 載入遠端控制

  // 建立視覺化
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto text_pose = [] {
      Eigen::Isometry3d msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    moveit_visual_tools.trigger();
  };

  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path =
    [&moveit_visual_tools,
     jmg = move_group_interface.getRobotModel()->getJointModelGroup("arm")]
    (auto const trajectory) {
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      moveit_visual_tools.trigger();
    };

  // 定義原始姿態（Home Position）
  std::map<std::string, double> home_joint_values;
  home_joint_values["J1"] = 0.0;  
  home_joint_values["J2"] = 0.0;
  home_joint_values["J3"] = 0.0;
  home_joint_values["J4"] = 0.0;
  home_joint_values["J5"] = 0.0;
  home_joint_values["J6"] = 0.0;

  // **先宣告 `recording` 參數**
  node->declare_parameter("recording", false);

  // 顯示標題
  draw_title("Simple Planning Demo");

  // 提示使用者按下 "Next"
  prompt("Press 'Next' in Rviz to continue...");

  // 嘗試生成規劃
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    RCLCPP_INFO(logger, "Planning successful, executing...");

    trajectory_pub->publish(plan.trajectory_.joint_trajectory);
    RCLCPP_INFO(logger, "Published trajectory to topic 'planned_trajectory'");
    
    // 視覺化規劃路徑
    draw_trajectory_tool_path(plan.trajectory_);

    // 提示使用者按 "Next" 來執行路徑
    prompt("Press 'Next' in Rviz to execute the trajectory...");
    draw_title("Executing");

    // **開始記錄 JointState**
    node->set_parameter(rclcpp::Parameter("recording", true));

    move_group_interface.execute(plan);

    RCLCPP_INFO(logger, "Target location reached！");
  
    // **在 Return Home 執行前，停止記錄 JointState**
    node->set_parameter(rclcpp::Parameter("recording", false));

    // 顯示 "回到原點" 按鈕
    draw_title("Press 'Return Home' to go back to Home Position");
    prompt("Press 'Return Home' in Rviz...");

    // 執行歸位動作
    RCLCPP_INFO(logger, "Returning to home position...");
    move_group_interface.setJointValueTarget(home_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group_interface.plan(home_plan)) {
      move_group_interface.execute(home_plan);
      RCLCPP_INFO(logger, "return home！");
    } else {
      RCLCPP_ERROR(logger, "Failed to plan to home position!");
    }

  } else {
    draw_title("Planning Failed!");
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // 關閉 ROS 2，並等待執行緒結束
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
