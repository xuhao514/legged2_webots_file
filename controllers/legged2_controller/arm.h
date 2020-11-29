#ifndef __ARM_H
#define __ARM_H
#include "utils.h"

class Arm
{
public:
    //初始化  角度初值，单位度
    void init(float _c10,float _c20,float _c30);
	void update(float _dt);
	//设定关节角度
    void setJointAng(float _c1,float _c2,float _c3);
    //设定关节弧度
    void setJointrRad(float _c1,float _c2,float _c3);
	//设定末端坐标
	void setPos(float _x,float _y,float _z);
	//设定速度运动到目标位置 _use_t_long:移动使用时长/ms： 
    void moveToPos(float _target_x,float _target_y,float _target_z,float _use_t_long);

     ArmAng getAng();
     //获取绝对角度  单位  默认度
     float get_c1(bool _ang=true){if(_ang) return c1*180/my_pi;else return c1;};
     float get_c2(bool _ang=true){if(_ang) return c2*180/my_pi;else return c2;};
     float get_c3(bool _ang=true){if(_ang) return c3*180/my_pi;else return c3;};
    //获取相对初始角度的角度  单位  默认度
     float get_c1_relative(bool _ang=true){if(_ang) return (c1-c10)*180/my_pi;else return (c1-c10);};
     float get_c2_relative(bool _ang=true){if(_ang) return (c2-c20)*180/my_pi;else return (c2-c20);};
     float get_c3_relative(bool _ang=true){if(_ang) return (c3-c30)*180/my_pi;else return (c3-c30);};
     float get_x(){return x;};
     float get_y(){return y;};
     float get_z(){return z;};
private:
	double c1, c2, c3;          //对应弧度   弧度
	double x, y, z;               //足底曲线坐标    m
	float c10, c20, c30;          //上电初始角度
    ArmAng arm_ang;
	//机构参数
	float L1, L2, L3;   //杆长
	DH dh1, dh2, dh3;   //DH参数
	TrMatrix tr1, tr2, tr3;//转化矩阵
	TrMatrix pos_tr;   //正解求得矩阵
   //正解  由(c1,c2,c3)求(x,y,z) 弧度 m
	void fk_3(float _c1, float _c2, float _c3);
    //逆解 由(x,y,z)求(c1,c2,c3) false为无解  m 弧度
    bool ik_3(double _x, double _y, double _z);

    float target_x, target_y ,target_z; //目标位置
    float start_x, start_y,start_z;  //设定目标位置时的起始位置
	float use_t_long;    //运动到目标时长  ms
	float use_t;   //移动已经使用时长

	bool set_move_to_pos;
};
#endif
