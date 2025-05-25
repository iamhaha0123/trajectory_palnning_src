#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <iostream>

class TrajectoryListener : public rclcpp::Node
{
public:
  TrajectoryListener() : Node("trajectory_to_ee_pose")
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
    kinematic_model_ = robot_model_loader_->getModel();
    joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

    if (!joint_model_group_) {
      RCLCPP_ERROR(get_logger(), "JointModelGroup 'arm' not found!");
      return;
    }

    sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "planned_trajectory", 10,
      std::bind(&TrajectoryListener::trajectoryCallback, this, std::placeholders::_1));
  }
private:
  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu points", msg->points.size());

    moveit::core::RobotState robot_state(kinematic_model_);
    robot_state.setToDefaultValues();

    for (const auto& pt : msg->points)
    {
      robot_state.setJointGroupPositions(joint_model_group_, pt.positions);

      const auto& pose = robot_state.getGlobalLinkTransform("ee_link");  // ⚠️ 根據你實際末端連結修改
      auto pos = pose.translation();
      Eigen::Quaterniond quat(pose.rotation());

      std::cout << "Time " << pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                << "s - Position: [" << pos.x() << ", " << pos.y() << ", " << pos.z()
                << "], Orientation (wxyz): [" << quat.w() << ", " << quat.x() << ", "
                << quat.y() << ", " << quat.z() << "]" << std::endl;
    }
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryListener>());
  rclcpp::shutdown();
  return 0;
}
