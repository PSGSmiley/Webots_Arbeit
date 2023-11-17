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

  Motor *front_right_motor = robot->getMotor("right_wheel_joint");
  Motor *front_left_motor = robot->getMotor("left_wheel_joint");
  
  front_right_motor->setPosition(INFINITY);
  front_left_motor->setPosition(INFINITY);
  
  front_right_motor->setVelocity(0.0);
  front_left_motor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {

    front_right_motor->setVelocity(1);
    front_left_motor->setVelocity(1);
    
  }

  delete robot;
  return 0;
}