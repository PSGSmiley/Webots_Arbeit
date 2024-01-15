#ifndef WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP
#define WEBOTS_ROS2_PLUGIN_EXAMPLE_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

namespace webots_motor_publisher {
  class WebotsMotorPublisher : public webots_ros2_driver::PluginInterface {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
    //WebotsMotorPublisher() : Node("webots_motor_publisher") {
      // Initialize ROS 2 publisher for wheel speed
      //wheelSpeedPublisher_ = create_publisher<std_msgs::msg::Float64>("wheel_speed", 10);

      // Set the update rate for publishing
      //timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&WebotsMotorPublisher::publishMotorData, this));
    //}

  private:
    void publishMotorData() {
      // Get wheel speed from Webots (assuming motor name is "wheel_motor")
      double wheelSpeed = getWheelSpeed("front_right_motor");

      // Publish wheel speed
      //auto wheelSpeedMsg = std_msgs::msg::Float64();
      wheelSpeedMsg.data = wheelSpeed;
      //wheelSpeedPublisher_->publish(wheelSpeedMsg);
    }

    double getWheelSpeed(const std::string &motorName) {
      // Use the Webots API to get motor speed
      WbDeviceTag motor = wb_robot_get_device(motorName.c_str());
      double wheelSpeed = wb_motor_get_velocity(motor);
      return wheelSpeed;
    }

    //rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheelSpeedPublisher_;
    //rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace webots_motor_publisher
#endif