#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <webots/motor.h>
#include <webots/robot.h>
#include <cstdio>
#include <functional>
#include "mecanum_robot/WebotsMotorPublisher.hpp"

namespace my_robot_driver {
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {


class WebotsMotorPublisher : public rclcpp::Node {
public:
  WebotsMotorPublisher() : Node("webots_motor_publisher") {
    // Initialize ROS 2 publisher for wheel speed
    wheelSpeedPublisher_ = create_publisher<std_msgs::msg::Float64>("wheel_speed", 10);

    // Set the update rate for publishing
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&WebotsMotorPublisher::publishMotorData, this));
  }

private:
  void publishMotorData() {
    // Get wheel speed from Webots (assuming motor name is "wheel_motor")
    double wheelSpeed = getWheelSpeed("front_right_motor");

    // Publish wheel speed
    auto wheelSpeedMsg = std_msgs::msg::Float64();
    wheelSpeedMsg.data = wheelSpeed;
    wheelSpeedPublisher_->publish(wheelSpeedMsg);
  }

  double getWheelSpeed(const std::string &motorName) {
    // Use the Webots API to get motor speed
    WbDeviceTag motor = wb_robot_get_device(motorName.c_str());
    double wheelSpeed = wb_motor_get_velocity(motor);
    return wheelSpeed;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheelSpeedPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebotsMotorPublisher>());
  rclcpp::shutdown();
  return 0;
}


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

extern "C" {
  #include <webots/robot.h>
  #include <webots/motor.h>
}

class WebotsMotorPublisher : public rclcpp::Node {
public:
  WebotsMotorPublisher() : Node("webots_motor_publisher") {
    // Initialize ROS 2 publisher for wheel speed
    wheelSpeedPublisher_ = create_publisher<std_msgs::msg::Float64>("wheel_speed", 10);

    // Create a timer with a callback function to publish motor data
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&WebotsMotorPublisher::publishMotorData, this));
  }

private:
  void publishMotorData() {
    // Get wheel speed from Webots (assuming motor name is "wheel_motor")
    double wheelSpeed = getWheelSpeed("wheel_motor");

    // Publish wheel speed
    auto wheelSpeedMsg = std_msgs::msg::Float64();
    wheelSpeedMsg.data = wheelSpeed;
    wheelSpeedPublisher_->publish(wheelSpeedMsg);
  }

  double getWheelSpeed(const std::string &motorName) {
    // Use the Webots API to get motor speed
    WbDeviceTag motor = wb_robot_get_device(motorName.c_str());
    double wheelSpeed = wb_motor_get_velocity(motor);
    return wheelSpeed;
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheelSpeedPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebotsMotorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
