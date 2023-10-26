#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 32

using namespace webots;
int vel = 8;

int main() {
  Robot *robot = new Robot();

  Motor *lw1m = robot->getMotor("lw1m");
  Motor *lw2m = robot->getMotor("lw2m");
  Motor *lw3m = robot->getMotor("lw3m");
  Motor *lw4m = robot->getMotor("lw4m");
  Motor *rw1m = robot->getMotor("rw1m");
  Motor *rw2m = robot->getMotor("rw2m");
  Motor *rw3m = robot->getMotor("rw3m");
  Motor *rw4m = robot->getMotor("rw4m");
  
  lw1m->setPosition(INFINITY);
  lw2m->setPosition(INFINITY);
  lw3m->setPosition(INFINITY);
  lw4m->setPosition(INFINITY);
  rw1m->setPosition(INFINITY);
  rw2m->setPosition(INFINITY);
  rw3m->setPosition(INFINITY);
  rw4m->setPosition(INFINITY);
  
  lw1m->setVelocity(0.0);
  lw2m->setVelocity(0.0);
  lw3m->setVelocity(0.0);
  lw4m->setVelocity(0.0);
  rw1m->setVelocity(0.0);
  rw2m->setVelocity(0.0);
  rw3m->setVelocity(0.0);
  rw4m->setVelocity(0.0);

  while (robot->step(TIME_STEP) != -1) {


    // actuate wheel motors
    lw1m->setVelocity(vel);
    lw2m->setVelocity(vel);
    lw3m->setVelocity(vel);
    lw4m->setVelocity(vel);
    rw1m->setVelocity(vel);
    rw2m->setVelocity(vel);
    rw3m->setVelocity(vel);
    rw4m->setVelocity(vel);
    
  }

  delete robot;
  return 0;
}
