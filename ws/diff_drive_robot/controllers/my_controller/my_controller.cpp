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

  Motor *right_motor = robot->getMotor("rightmotor");
  Motor *left_motor = robot->getMotor("leftmotor");
  
  right_motor->setPosition(INFINITY);
  left_motor->setPosition(INFINITY);
  
  right_motor->setVelocity(0.0);
  left_motor->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {

    right_motor->setVelocity(5);
    left_motor->setVelocity(0);
    
  }

  delete robot;
  return 0;
}