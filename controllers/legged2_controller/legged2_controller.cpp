
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>   //imu
#include <webots/Accelerometer.hpp>  //加速度计
#include <webots/GPS.hpp>

#include "arm.h"
#include "leg.h"
#include "walk_leg.h"
#include "log.hpp"

using namespace webots;
using namespace std;
 Robot *robot;
Arm arml,armr;
Leg legl,legr;
WalkLegClass walk_leg_l,walk_leg_r;
Zmp zmp;
Motor *motor_rc1,*motor_rc2,*motor_rc3,*motor_lc1,*motor_lc2,*motor_lc3;
Motor *motor_legrc1,*motor_legrc2,*motor_legrc3,*motor_legrc4,*motor_legrc5;
Motor *motor_leglc1,*motor_leglc2,*motor_leglc3,*motor_leglc4,*motor_leglc5;
InertialUnit *imu;
Accelerometer *accelerometer;
GPS *gps;
RPY body_rpy;
Accel acc;
void updateMotor();
void initMotor();
void setSensor();
//LogFile log_file;  
void logTxt();
int main(int argc, char **argv) {
  // create the Robot instance.
  robot = new Robot();
  
  arml.init(90,90,0);armr.init(90,90,0);   //与计算坐标系的角度之差
  legl.init(0,0,15,-15,0); legr.init(0,0,15,-15,0);
  walk_leg_l.init(legl,170,0,15,30,30,500,1,3.0/4,0,1);
  walk_leg_r.init(legr,170,0,15,30,30,500,1,3.0/4,1.0,1);
  walk_leg_l.initRPY(body_rpy);
  walk_leg_r.initRPY(body_rpy);
  zmp.init(legl,legr,walk_leg_l,walk_leg_r,acc,body_rpy);
  int timeStep = (int)robot->getBasicTimeStep();  //基本频率 32  在界面中可改
  //int timeStep = 10;
  robot->step(1000);   //延时函数

  imu= robot->getInertialUnit("imu");
  imu->enable(timeStep);
  accelerometer=robot->getAccelerometer("accelerometer");
  accelerometer->enable(timeStep);
  gps = robot->getGPS("gps_body");
  gps->enable(timeStep);
  initMotor();
 
  double val = 0;
  walk_leg_r.move2InitPos(500);
  walk_leg_r.startAct(true);
  walk_leg_l.move2InitPos(500);
  walk_leg_l.startAct(true);

  while (robot->step(timeStep) != -1) {   //固定32ms运行一次
     setSensor();
    // motor_rc1->setPosition(90);
    //  arml.update(timeStep);
    //  armr.update(timeStep);
    // arml.setJointAng(0,0,0);
    // armr.setJointAng(0,0,0);   
    //  arml.setPos(0.0,0,0.14);

    // legl.setJointAng(90,0,0,0,0);
    // legr.setJointAng(-25,0,15,-15,0);
    //  legl.setPos(0.15,0.0,0.0,0,0);
    // legr.setPos(0.18,0.050,0,0,0);
     legl.update(timeStep); legr.update(timeStep);
     walk_leg_l.update(timeStep);walk_leg_r.update(timeStep);
     //zmp.update(timeStep);

     updateMotor();
     val+=0.1;
    
    logTxt();
    cout<<"time:"<<(robot->getTime())<<"\n";
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
  
}


void updateMotor()
{
  //先测试正负号
  //手左右镜向对称
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
   //腿左右镜向  
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
  LogFile::logMsg("c1",legr.get_c1(false));
  LogFile::logMsg("x",legr.get_x()); LogFile::logMsg("z",legr.get_z());
  // outfile<<legr.get_c1(false)<<" "<<legr.get_c2(false)<<" "<<legr.get_c3(false)<<" "<<legr.get_c4(false)<<" "<<legr.get_c5(false)
  // <<" "<<legr.get_x()<<" "<<legr.get_z()<<"\n";
}

void setSensor()
{
    std::cout<< "IMU roll: " <<imu->getRollPitchYaw()[0]<< " pitch: " <<imu->getRollPitchYaw()[1]
    << " yaw: " <<imu->getRollPitchYaw()[2] <<std::endl;    
    std::cout<< "Accel X: " <<accelerometer->getValues()[0]
    << " Y: " <<accelerometer->getValues()[1]<< " Z: " <<accelerometer->getValues()[2] <<std::endl;  
  body_rpy.R = imu->getRollPitchYaw()[0];  // rad
  body_rpy.P = imu->getRollPitchYaw()[1];  // rad
  body_rpy.Y = imu->getRollPitchYaw()[2];  // rad
  acc.ax = accelerometer->getValues()[0];  //m/s^2
  acc.ay = accelerometer->getValues()[1];
  acc.az = accelerometer->getValues()[2];
  // std::cout<< "robot_pos Value X: " <<gps->getValues()[0]
  //   << " Y: " <<gps->getValues()[1]<< " Z: " <<gps->getValues()[2] <<std::endl;    
}

//resetRobot():  move robot to initial position


void initMotor()
{
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
}