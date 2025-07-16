#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class InitJointStateNode : public rclcpp::Node
{
public:
  InitJointStateNode() : Node("init_joint_state_node")
  {
    /* ---- 參數：關節名稱與 home 角度 ---- */
    this->declare_parameter<std::vector<std::string>>(
        "joint_names", {"J1", "J2", "J3", "J4", "J5", "J6"});

    this->declare_parameter<std::vector<double>>(
        "home_values", {0.0, 0.0, 0.2, 0.0, 0.5, 0.0});

    auto names = this->get_parameter("joint_names").as_string_array();
    auto vals  = this->get_parameter("home_values").as_double_array();

    if (names.size() != vals.size()) {
      RCLCPP_FATAL(get_logger(),
                   "joint_names (%zu) 與 home_values (%zu) 數量不符！",
                   names.size(), vals.size());
      rclcpp::shutdown();
      return;
    }

    msg_.name     = names;
    msg_.position = vals;

    /* ---- 發佈器 ---- */
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    /* ---- 每 100 ms 發一次，共發 2 秒 ---- */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&InitJointStateNode::tick, this));
  }

private:
  void tick()
  {
    msg_.header.stamp = now();
    pub_->publish(msg_);

    if (++counter_ >= 20) {            // 10 Hz × 2 秒
      RCLCPP_INFO(get_logger(), "已廣播 home 關節角度，結束初始化節點");
      rclcpp::shutdown();
    }
  }

  sensor_msgs::msg::JointState  msg_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr  timer_;
  int counter_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitJointStateNode>());
  return 0;
}
