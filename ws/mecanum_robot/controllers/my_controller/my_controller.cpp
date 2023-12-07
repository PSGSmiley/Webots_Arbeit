#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 32

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
    front_right_motor->setVelocity(1);
    front_left_motor->setVelocity(1);
    back_right_motor->setVelocity(1);
    back_left_motor->setVelocity(1);
    
  }

  delete robot;
  return 0;
}
