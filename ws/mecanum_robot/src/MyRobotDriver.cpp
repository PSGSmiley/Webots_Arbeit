#include "mecanum_robot/MyRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include "std_msgs/msg/float64.hpp"

#define wheel_radius_ 0.123
#define wheel_separation_width_ 0.35  // lateral distance from robot's COM to wheel [m]. 
#define wheel_separation_length_ 0.22  // longitudinal distance from robot's COM to wheel [m]. 

namespace my_robot_driver {
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  front_right_motor = wb_robot_get_device("front_right_motor");
  front_left_motor = wb_robot_get_device("front_left_motor");
  back_right_motor = wb_robot_get_device("back_right_motor");
  back_left_motor = wb_robot_get_device("back_left_motor");

  wb_motor_set_position(front_right_motor, INFINITY);
  wb_motor_set_position(front_left_motor, INFINITY);
  wb_motor_set_position(back_right_motor, INFINITY);
  wb_motor_set_position(back_left_motor, INFINITY);
  
  wb_motor_set_velocity(front_right_motor, 0.0);
  wb_motor_set_velocity(front_left_motor, 0.0);
  wb_motor_set_velocity(back_right_motor, 0.0);
  wb_motor_set_velocity(back_left_motor, 0.0);

  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&MyRobotDriver::cmdVelCallback, this, std::placeholders::_1));

//  wheelSpeedPublisher_ = node->create_publisher<std_msgs::msg::Float64>(
//    "/motor_speed", 10);

//  timer_ = node->create_wall_timer(std::chrono::milliseconds(100), 
//    std::bind(&MyRobotDriver::publishMotorData, this));

}

void MyRobotDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

// void MyRobotDriver::publishMotorData() {
//   double wheelSpeed = getWheelSpeed("front_right_motor");
//   auto wheelSpeedMsg = std_msgs::msg::Float64();
//   wheelSpeedMsg.data = wheelSpeed;
//   wheelSpeedPublisher_->publish(wheelSpeedMsg);
// }

double getWheelSpeed(const std::string &motorName) {
  // Use the Webots API to get motor speed
  WbDeviceTag motor = wb_robot_get_device(motorName.c_str());
  double wheelSpeed = wb_motor_get_velocity(motor);
  return wheelSpeed;
}

void MyRobotDriver::step() {
  auto forward_speed  = cmd_vel_msg.linear.x;
  auto sideways_speed = cmd_vel_msg.linear.y;
  auto angular_speed  = cmd_vel_msg.angular.z;

  auto fl_wheel_velocity = (1 / wheel_radius_) * (forward_speed - sideways_speed - (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
  auto fr_wheel_velocity = (1 / wheel_radius_) * (forward_speed + sideways_speed + (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
  auto bl_wheel_velocity = (1 / wheel_radius_) * (forward_speed + sideways_speed - (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
  auto br_wheel_velocity = (1 / wheel_radius_) * (forward_speed - sideways_speed + (wheel_separation_width_ + wheel_separation_length_) * angular_speed);

  wb_motor_set_velocity(front_left_motor, fl_wheel_velocity);
  wb_motor_set_velocity(front_right_motor, fr_wheel_velocity);
  wb_motor_set_velocity(back_left_motor, bl_wheel_velocity);
  wb_motor_set_velocity(back_right_motor, br_wheel_velocity);

  //getWheelSpeed("front_right_motor");

      
}
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)
