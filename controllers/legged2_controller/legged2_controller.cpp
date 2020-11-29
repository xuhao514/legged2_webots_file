
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "arm.h"

using namespace webots;
using namespace std;

Arm arml,armr;
Motor *motor_rc1,*motor_rc2,*motor_rc3,*motor_lc1,*motor_lc2,*motor_lc3;
Motor *motor_legrc1,*motor_legrc2,*motor_legrc3,*motor_legrc4,*motor_legrc5;
Motor *motor_leglc1,*motor_leglc2,*motor_leglc3,*motor_leglc4,*motor_leglc5;
void sendConMsg();

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  arml.init(90,90,0);armr.init(90,90,0);
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

 motor_rc1 = robot->getMotor("joint_rc1");
 motor_rc2 = robot->getMotor("joint_rc2");
 motor_rc3 = robot->getMotor("joint_rc3");
  motor_lc1 = robot->getMotor("joint_lc1");
motor_lc2 = robot->getMotor("joint_lc2");
 motor_lc3 = robot->getMotor("joint_lc3");
 motor_legrc1 = robot->getMotor("jointlegr1");
motor_legrc2 = robot->getMotor("jointlegr2");
motor_legrc3 = robot->getMotor("jointlegr3");
motor_legrc4 = robot->getMotor("jointlegr4");
motor_legrc5 = robot->getMotor("jointlegr5");
motor_leglc1 = robot->getMotor("jointlegl1");
motor_leglc2 = robot->getMotor("jointlegl2");
motor_leglc3 = robot->getMotor("jointlegl3");
motor_leglc4 = robot->getMotor("jointlegl4");
motor_leglc5 = robot->getMotor("jointlegl5");
 
 motor_rc1->setPosition(0);
 motor_rc2->setPosition(0);
 motor_rc3->setPosition(0);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  double val = 0;
  while (robot->step(timeStep) != -1) {

    // motor_rc1->setPosition(90);
     arml.update(timeStep);
     armr.update(timeStep);
    // arml.setJointAng(0,0,0);
     armr.setJointAng(0,0,0);
     arml.setPos(0.0,0,0.14);
     sendConMsg();
     val+=1;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}


void sendConMsg()
{
  motor_rc1->setPosition(-armr.get_c1_relative(false));
  motor_rc2->setPosition(-armr.get_c2_relative(false));
  motor_rc3->setPosition(-armr.get_c3_relative(false));
    motor_lc1->setPosition(arml.get_c1_relative(false));
  motor_lc2->setPosition(arml.get_c2_relative(false));
  motor_lc3->setPosition(-arml.get_c3_relative(false));
 
}
