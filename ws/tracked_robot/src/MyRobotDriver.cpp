#include "tracked_robot/MyRobotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 1
#define WHEEL_RADIUS 0.2

namespace my_robot_driver {
void MyRobotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  lw1m = wb_robot_get_device("lw1m");
  lw2m = wb_robot_get_device("lw2m");
  lw3m = wb_robot_get_device("lw3m");
  lw4m = wb_robot_get_device("lw4m");

  rw1m = wb_robot_get_device("rw1m");
  rw2m = wb_robot_get_device("rw2m");
  rw3m = wb_robot_get_device("rw3m");
  rw4m = wb_robot_get_device("rw4m");

  wb_motor_set_position(lw1m, INFINITY);
  wb_motor_set_position(lw2m, INFINITY);
  wb_motor_set_position(lw3m, INFINITY);
  wb_motor_set_position(lw4m, INFINITY);

  wb_motor_set_position(rw1m, INFINITY);
  wb_motor_set_position(rw2m, INFINITY);
  wb_motor_set_position(rw3m, INFINITY);
  wb_motor_set_position(rw4m, INFINITY);

  wb_motor_set_velocity(lw1m, 0.0);
  wb_motor_set_velocity(lw2m, 0.0);
  wb_motor_set_velocity(lw3m, 0.0);
  wb_motor_set_velocity(lw4m, 0.0);

  wb_motor_set_velocity(rw1m, 0.0);
  wb_motor_set_velocity(rw2m, 0.0);
  wb_motor_set_velocity(rw3m, 0.0);
  wb_motor_set_velocity(rw4m, 0.0);


  cmd_vel_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&MyRobotDriver::cmdVelCallback, this, std::placeholders::_1));
}

void MyRobotDriver::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_msg.linear = msg->linear;
  cmd_vel_msg.angular = msg->angular;
}

void MyRobotDriver::step() {
  auto forward_speed = cmd_vel_msg.linear.x;
  auto angular_speed = cmd_vel_msg.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(lw1m, command_motor_left);
  wb_motor_set_velocity(lw2m, command_motor_left);
  wb_motor_set_velocity(lw3m, command_motor_left);
  wb_motor_set_velocity(lw4m, command_motor_left);

  wb_motor_set_velocity(rw1m, command_motor_right);
  wb_motor_set_velocity(rw2m, command_motor_right);
  wb_motor_set_velocity(rw3m, command_motor_right);
  wb_motor_set_velocity(rw4m, command_motor_right);
}
} // namespace my_robot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_driver::MyRobotDriver,
                       webots_ros2_driver::PluginInterface)
