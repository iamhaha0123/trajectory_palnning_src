#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fk_publisher_node");

  // 建立 publisher
  auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("end_effector_pose", 10);

  // 載入機器人模型
  robot_model_loader::RobotModelLoader model_loader(node, "robot_description");
  auto kinematic_model = model_loader.getModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(kinematic_model);
  robot_state->setToDefaultValues();

  // 訂閱 /planned_trajectory
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

        // 發布 FK 結果
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = "world";
        pose_msg.header.stamp = rclcpp::Time(
          static_cast<int64_t>(pt.time_from_start.sec) * 1000000000LL + pt.time_from_start.nanosec
        );
        pose_msg.pose.position.x = tf.translation().x();
        pose_msg.pose.position.y = tf.translation().y();
        pose_msg.pose.position.z = tf.translation().z();

        Eigen::Quaterniond q(tf.rotation());
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_pub->publish(pose_msg);
        rate.sleep();  
        RCLCPP_INFO(node->get_logger(), "Published FK pose [%.3f s]", pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9);
      }
    });

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
