#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <robot_interfaces/msg/arm_attitude.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fk_publisher_node");

  // 建立 publisher
  auto pose_pub = node->create_publisher<robot_interfaces::msg::ArmAttitude>("end_effector_pose", 10);

  // 載入機器人模型
  robot_model_loader::RobotModelLoader model_loader(node, "robot_description");
  auto kinematic_model = model_loader.getModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
  robot_state->setToDefaultValues();

  // 訂閱planned_trajectory
  auto sub = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "/planned_trajectory", 10,
    [=](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
      if (msg->points.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty trajectory.");
        return;
      }

      const std::string ee_link = "link_J6"; 
      rclcpp::Rate rate(100);

      for (size_t i = 0; i < msg->points.size(); ++i)
      {
        const auto &pt = msg->points[i];

        if (msg->joint_names.size() != pt.positions.size()) {
          RCLCPP_WARN(node->get_logger(), "Mismatch between joint_names and positions at point %zu", i);
          continue;
        }

        // 更新 joint 狀態並計算 FK
        robot_state->setVariablePositions(msg->joint_names, pt.positions);
        robot_state->update();
        auto tf = robot_state->getGlobalLinkTransform(ee_link);

        /* 位置 */
        const auto &p = tf.translation();

        // 轉換為由拉角（RPY，單位：角度）
        Eigen::Vector3d rpy = tf.rotation().eulerAngles(2, 1, 0); 
        double roll = rpy[2] * 180.0 / M_PI;
        double pitch = rpy[1] * 180.0 / M_PI;
        double yaw = rpy[0] * 180.0 / M_PI;

        /* 填入 ArmAttitude */
        robot_interfaces::msg::ArmAttitude att;
        att.x = p.x();
        att.y = p.y();
        att.z = p.z();

        att.w = yaw;    
        att.p = pitch;  
        att.r = roll;   
        att.t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;

        pose_pub->publish(att);
        rate.sleep();  

        // 顯示 RPY 與位置資訊
        RCLCPP_INFO(node->get_logger(),
          "[%.3f s] Pos:[%.3f, %.3f, %.3f]  RPY:[%.2f°, %.2f°, %.2f°]",
          att.t, att.x, att.y, att.z, roll, pitch, yaw);
      }
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
