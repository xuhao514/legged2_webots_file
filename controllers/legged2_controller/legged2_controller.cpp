
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "arm.h"
#include "leg.h"
#include "walk_leg.h"
#include <iostream>
#include <fstream>

using namespace webots;
using namespace std;

Arm arml,armr;
Leg legl,legr;
WalkLegClass walk_leg_l,walk_leg_r;
Motor *motor_rc1,*motor_rc2,*motor_rc3,*motor_lc1,*motor_lc2,*motor_lc3;
Motor *motor_legrc1,*motor_legrc2,*motor_legrc3,*motor_legrc4,*motor_legrc5;
Motor *motor_leglc1,*motor_leglc2,*motor_leglc3,*motor_leglc4,*motor_leglc5;
void updateMotor();
ofstream outfile("log.txt", ios::trunc);  //log文件
void logTxt();
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  arml.init(90,90,0);armr.init(90,90,0);   //与计算坐标系的角度之差
  legl.init(0,0,15,-15,0); legr.init(0,0,15,-15,0);
  walk_leg_l.init(legl,190,0,15,20,30,2000,1,3.0/4,0,1);
  walk_leg_r.init(legr,190,0,15,20,30,2000,1,3.0/4,1.0,1);
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
  walk_leg_l.move2InitPos(1000);
  walk_leg_l.startAct(true);
  walk_leg_r.move2InitPos(1000);
 walk_leg_r.startAct(true);
  while (robot->step(timeStep) != -1) {
   //printf("%d\n",timeStep);  timeStep运行一次的时间 ms
    // motor_rc1->setPosition(90);
    //  arml.update(timeStep);
    //  armr.update(timeStep);
      // arml.setJointAng(0,0,0);
      // armr.setJointAng(0,0,0);   
    //  arml.setPos(0.0,0,0.14);

    // legl.setJointAng(90,0,0,0,0);
    //  legr.setJointAng(0,0,15,-15,0);
     //legl.setPos(0.15,0.0,-0.1,0,0);
    //  legr.setPos(0.18,0.0,-0.05,0,0);
     walk_leg_l.update(timeStep);
     walk_leg_r.update(timeStep);
     updateMotor();
     val+=1;
    // logTxt();
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}


void updateMotor()
{
  //先测试正负号
  //左右镜向对称
  motor_rc1->setPosition(-armr.get_c1_relative(false));
  motor_rc2->setPosition(-armr.get_c2_relative(false));
  motor_rc3->setPosition(-armr.get_c3_relative(false));
    motor_lc1->setPosition(arml.get_c1_relative(false));
  motor_lc2->setPosition(arml.get_c2_relative(false));
  motor_lc3->setPosition(-arml.get_c3_relative(false));
  //逆时针旋转为正  腿的前与身体前相同   但c3的角度超范围  腿干涉
  //   motor_legrc1->setPosition(legr.get_c1_relative(false));
  // motor_legrc2->setPosition(legr.get_c2_relative(false));
  // motor_legrc3->setPosition(-legr.get_c3_relative(false));
  // motor_legrc4->setPosition(-legr.get_c4_relative(false));
  // motor_legrc5->setPosition(legr.get_c5_relative(false));
  //   motor_leglc1->setPosition(-legl.get_c1_relative(false));
  // motor_leglc2->setPosition(legl.get_c2_relative(false));
  // motor_leglc3->setPosition(-legl.get_c3_relative(false));
  // motor_leglc4->setPosition(-legl.get_c4_relative(false));
  // motor_leglc5->setPosition(-legl.get_c5_relative(false));
  //顺时针旋转为正  腿的前与身体前相反
   motor_legrc1->setPosition(legr.get_c1_relative(false));
  motor_legrc2->setPosition(-legr.get_c2_relative(false));
  motor_legrc3->setPosition(legr.get_c3_relative(false));
  motor_legrc4->setPosition(legr.get_c4_relative(false));
  motor_legrc5->setPosition(legr.get_c5_relative(false));
    motor_leglc1->setPosition(-legl.get_c1_relative(false));
  motor_leglc2->setPosition(-legl.get_c2_relative(false));
  motor_leglc3->setPosition(legl.get_c3_relative(false));
  motor_leglc4->setPosition(legl.get_c4_relative(false));
  motor_leglc5->setPosition(-legl.get_c5_relative(false));
}


void logTxt()
{
  outfile<<legr.get_c1(false)<<" "<<legr.get_c2(false)<<" "<<legr.get_c3(false)<<" "<<legr.get_c4(false)<<" "<<legr.get_c5(false)
  <<" "<<legr.get_x()<<" "<<legr.get_z()<<"\n";
}