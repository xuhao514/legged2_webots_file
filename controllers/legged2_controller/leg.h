//#pragma once
#ifndef __LEG_H
#define __LEG_H

#include "utils.h"

class Leg
{
public:
    //初始化 单位度    角度初值：与计算坐标系的角度之差
	void init(float _c10,float _c20,float _c30,float _c40,float _c50);
    //更新  dt ms
    void update(float _dt);
    //设定关节角度  
    void setJointAng(float _c1,float _c2,float _c3,float _c4,float _c5);
    //设定关节弧度 并限定角度范围
    void setJointrRad(float _c1,float _c2,float _c3,float _c4,float _c5);
    //设定末端坐标  m  弧度   flase为无解  x:竖直 y:侧向 正朝里 z:前方 正朝后  P:前后转(抬腿) 正逆时针  Y:左右转（侧向） 正：身体向外
    bool setPos(float _x,float _y,float _z,float _P,float _Y);
    //设定速度运动到目标位置 _use_t_long:移动使用时长/ms：
    void moveToPos(float _target_x,float _target_y,float _target_z,float _target_P,float _target_Y,float _use_t_long);

    //获取绝对角度
     LegAng getAng();
     //获取相对初始角度的角度
     LegAng getAngRelative();
     //获取绝对角度  单位  默认度
     float get_c1(bool _ang=true){if(_ang) return c1*180/my_pi;else return c1;};
     float get_c2(bool _ang=true){if(_ang) return c2*180/my_pi;else return c2;};
     float get_c3(bool _ang=true){if(_ang) return c3*180/my_pi;else return c3;};
     float get_c4(bool _ang=true){if(_ang) return c4*180/my_pi;else return c4;};
     float get_c5(bool _ang=true){if(_ang) return c5*180/my_pi;else return c5;};
    //获取相对初始角度的角度  单位  默认度
     float get_c1_relative(bool _ang=true){if(_ang) return (c1-c10)*180/my_pi;else return (c1-c10);};
     float get_c2_relative(bool _ang=true){if(_ang) return (c2-c20)*180/my_pi;else return (c2-c20);};
     float get_c3_relative(bool _ang=true){if(_ang) return (c3-c30)*180/my_pi;else return (c3-c30);};
     float get_c4_relative(bool _ang=true){if(_ang) return (c4-c40)*180/my_pi;else return (c4-c40);};
     float get_c5_relative(bool _ang=true){if(_ang) return (c5-c50)*180/my_pi;else return (c5-c50);};
     ////高度 m
     float get_x(){return x;};   
     //侧向 m
     float get_y(){return y;};   
     //前向   m
     float get_z(){return z;};  
     float get_R(){return R;};
     float get_P(){return P;};
     float get_Y(){return Y;};
     
     // 设定关节转动角度范围  度
     void set_c1_rang(float _min,float _max){c1_rang.min =_min; c1_rang.max =_max; };
     void set_c2_rang(float _min,float _max){c2_rang.min =_min; c2_rang.max =_max; };
     void set_c3_rang(float _min,float _max){c3_rang.min =_min; c3_rang.max =_max; };
     void set_c4_rang(float _min,float _max){c4_rang.min =_min; c4_rang.max =_max; };
     void set_c5_rang(float _min,float _max){c5_rang.min =_min; c5_rang.max =_max; };
     //获取角度范围  度
     ValueRang get_c1_rang(){return c1_rang; };
     ValueRang get_c2_rang(){return c2_rang; };
     ValueRang get_c3_rang(){return c3_rang; };
     ValueRang get_c4_rang(){return c4_rang; };
     ValueRang get_c5_rang(){return c5_rang; };

private:
    double c1, c2, c3, c4, c5;          //关节转角  弧度
    double c10, c20, c30,c40,c50;        //上电初始弧度
	RPY rpy;                           //足底角度 正解解算值 
    LegAng ang;
    double x, y, z,R,P,Y;               //足底曲线坐标  设定值  m 弧度
    ValueRang c1_rang,c2_rang,c3_rang,c4_rang,c5_rang;  //角度范围 度

	//机构参数
	float L1, L2, L3,L4,L5;   //杆长  m
	DH dh1, dh2, dh3,dh4,dh5;   //DH参数
	TrMatrix tr1, tr2, tr3, tr4, tr5;//转化矩阵
	TrMatrix pos_tr;   //正解求得矩阵
   //正解  由求(x,y,z)
	void fk_5(float _c1, float _c2, float _c3, float _c4, float _c5);
	//逆解 由(x,y,z,P,Y)求(c1,c2,c3)
    bool ik_5(double _x, double _y, double _z, double _P, double _Y);


    float target_x, target_y,target_z,target_P,target_Y; //目标位置
    float start_x, start_y,start_z,start_P,start_Y;  //设定目标位置时的起始位置
	float use_t_long;    //运动到目标时长  ms
	float use_t;   //移动已经使用时长
	bool set_move_to_pos;
};
#endif
