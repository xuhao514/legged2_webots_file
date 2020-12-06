#ifndef WALK_LEG_H
#define WALK_LEG_H

#include "leg.h"

struct WalkLegState   //记录状态
{
    bool to_init_pos;
    bool in_delay;
    bool walking;
    bool in_air_else_floor;
    bool start_act;  //开始运动
    bool is_add_start_tn;
    float x,y;
    float tn;
};

//设置单腿运动  单位统一为mm  ms 度
class WalkLegClass
{
public:
    //(x0,y0,z0) 行走曲线中心点 L:步长 H:步高  _t_floor:着地相时长 _tair_tfloor_scale:Tair与t_floor的比值 _start_scale_pos:起始位置在整个轨迹的比值,从腾空相开始 _delayt_tfloor_scale：delay_t与t_floor的比值
    //当设定起始位置相同时，使用_delayt_tfloor_scale来获得相位差；  设定起始位置为相位初始位置时， _delayt_tfloor_scale为0，_start_scale_pos设定成相位差处的值
    void init(Leg &_leg,float _x0,float _y0,float _z0,float _L,float _H0,float _t_floor,float _tair_tfloor_scale,float _start_scale_pos,float _delayt_tfloor_scale,int _forward);
    void update(float _dt);
    // 返回速度 m/s    speed：速度    _t_floor:着地相时长
    float setLegSpeed(float speed,float _t_floor);
    //设定着地相长度
    void set_t_floor(float _t_floor);
    //设置贝赛尔曲线的控制点 (_x0,_y0)：中心点  _L：步长  _H0：步高  _forward：是否反向
    void setBezier(float _x0,float _y0,float _L,float _H0,float _tair_tfloor_scale,int _forward);
    void reStart();
    void move2InitPos(float _use_t_long);   //移动动初始位置
    void startAct(bool _start);    //运动或者暂停
    WalkLegState walk_leg_state;
    float get_tfloor(){return t_floor;};
private:
    void calPos(float _tn);  //计算位置  _tn:周期中的时刻
    void calTnNow(float _dt);
private:
    float x0,y0,z0;       //(x0,y0,z0) 行走曲线中心点 mm
    float L;              //步长  当前的步长   mm
    float H0;             //步高
    float t_floor;        //着地相时长 ms
    float t_air;           //腾空相时长	ms
    float tair_tfloor_scale;  //Tair与t_floor的比值
    float start_scale_pos;    //起始位置在整个轨迹的比值
    float start_tn_tfloor_scale;        //周期的起始时刻与t_floor的比值
    float start_tn;        //周期的起始时刻
    float delayt_tfloor_scale;  //delay_t与t_floor的比值

    float alp;            // 曲线旋转角度  弧度 //上坡时可能用到
    const int bezier_len = 12;  //11阶贝塞尔曲线 12个控制点
    float wx12[12];       //11阶贝赛尔曲线
    float wy12[12];
    float wx2[2];         //1阶贝赛尔曲线  用于走直线
    float wy2[2];         //
    int forward;          //足端曲线的前方与车身前身的正反号  当前后腿的安装方向相反时，需要注意
    Leg *leg;         //与腿绑定

    bool over_flag;            // 一步完成的标志  从抬腿开始 Tn 上一时刻>t_air  这一时刻<t_air  即置true  否则false
    bool fixPointFlag;        //曲线不动点  此时可进行大幅度参数修改 如Lst等  x0不可在此时改变   时间只能渐变
   // ActionPhaseMode phaFlag; //所在相位标志
    int walk_leg_num;          //单腿步数
    float v_leg;         //单腿的速度

    bool prep;
    float prepc1,prepc4;  //记录需要运动到的位置
    float max_L,min_L;      //最大最小步长  mm

    float x,y,z;  //计算的坐标 mm

    struct CalParam //计算用参数
    {
      float  use_t_long;        //移动到初始位置用时
      float delay_t;            //延时时长
      float tn_now ,tn_last;   //记录一个周期中的相位  //记录在周期中运行的时刻  范围：0~Tm+Tm1
      float T_last,T_now;      //一个周期的总长度 周期长期可变 记录上周期与此时周期时间长度
    }cal_param;

};









#endif // WALK_LEG_H
