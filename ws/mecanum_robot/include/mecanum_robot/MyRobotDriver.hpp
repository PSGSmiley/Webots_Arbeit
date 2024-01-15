#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

namespace my_robot_driver {
class MyRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
  

private:
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
//  void publishMotorData();
//  rclcpp::Publisher<std_msgs::msg::float64>::SharedPtr
//      wheelSpeedPublisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag front_right_motor;
  WbDeviceTag front_left_motor;
  WbDeviceTag back_right_motor;
  WbDeviceTag back_left_motor;
};
} // namespace my_robot_driver
#endif