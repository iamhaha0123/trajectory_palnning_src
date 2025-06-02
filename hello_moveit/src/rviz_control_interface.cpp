#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>

using std::placeholders::_1;

class RvizButtonInterface
{
public:
  RvizButtonInterface(const rclcpp::Node::SharedPtr &node)
  : node_(node)
  {
    // 正確建構方式：傳入 server 名稱與 node
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "button_server", node_);

    pub_ = node_->create_publisher<std_msgs::msg::String>("/rviz_control/command", 10);

    create_button("plan", -0.3, 0.5);
    create_button("execute", 0.0, 0.5);
    create_button("home", 0.3, 0.5);

    server_->applyChanges();
    RCLCPP_INFO(node_->get_logger(), "RViz button interface ready.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  void create_button(const std::string &name, float x, float y)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.name = name + "_button";
    int_marker.description = name;
    int_marker.scale = 0.3;
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = 1.2;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    control.always_visible = true;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.05;
    marker.color.r = 0.2f;
    marker.color.g = 0.5f;
    marker.color.b = 0.9f;
    marker.color.a = 1.0f;

    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
      [this, name](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &)
      {
        std_msgs::msg::String msg;
        msg.data = name;
        pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Button pressed: %s", name.c_str());
      });
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("rviz_button_interface");
  RvizButtonInterface button_interface(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
