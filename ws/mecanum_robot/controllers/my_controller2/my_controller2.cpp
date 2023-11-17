#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 32
#define wheel_radius_ 0.123
#define wheel_separation_width_ 0.35  // lateral distance from robot's COM to wheel [m]. 
#define wheel_separation_length_ 0.22  // longitudinal distance from robot's COM to wheel [m]. 

using namespace webots;


int main() {
  Robot *robot = new Robot();

  Motor *front_right_motor = robot->getMotor("front_right_motor");
  Motor *front_left_motor = robot->getMotor("front_left_motor");
  Motor *back_right_motor = robot->getMotor("back_right_motor");
  Motor *back_left_motor = robot->getMotor("back_left_motor");
  
  front_right_motor->setPosition(INFINITY);
  front_left_motor->setPosition(INFINITY);
  back_right_motor->setPosition(INFINITY);
  back_left_motor->setPosition(INFINITY);
  
  front_right_motor->setVelocity(0.0);
  front_left_motor->setVelocity(0.0);
  back_right_motor->setVelocity(0.0);
  back_left_motor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {


    // actuate wheel motors

    auto forward_speed  = 0;
    auto sideways_speed = 0;
    auto angular_speed  = 1;

    auto fl_wheel_velocity = (1 / wheel_radius_) * (forward_speed - sideways_speed - (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
    auto fr_wheel_velocity = (1 / wheel_radius_) * (forward_speed + sideways_speed + (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
    auto bl_wheel_velocity = (1 / wheel_radius_) * (forward_speed + sideways_speed - (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
    auto br_wheel_velocity = (1 / wheel_radius_) * (forward_speed - sideways_speed + (wheel_separation_width_ + wheel_separation_length_) * angular_speed);
  
    front_right_motor->setVelocity(fr_wheel_velocity);
    front_left_motor->setVelocity(fl_wheel_velocity);
    back_right_motor->setVelocity(br_wheel_velocity);
    back_left_motor->setVelocity(bl_wheel_velocity);
    
  }

  delete robot;
  return 0;
}
