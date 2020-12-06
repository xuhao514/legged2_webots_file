#include "walk_leg.h"

void WalkLegClass::init(Leg &_leg,float _x0,float _y0,float _z0,float _L,float _H0,float _t_floor,float _tair_tfloor_scale,float _start_scale_pos,float _delayt_tfloor_scale,int _forward)
{
     max_L=100,min_L=-100;   //最大最小步长
     v_leg=_L/_t_floor;

     leg=&_leg; over_flag=false; fixPointFlag=false;
     prep=false; walk_leg_num=0;
     t_floor=_t_floor;
     tair_tfloor_scale=_tair_tfloor_scale; t_air=t_floor*tair_tfloor_scale;
     start_scale_pos = _start_scale_pos;
     if(start_scale_pos<0.5)  start_tn= 2*start_scale_pos*t_air;
     else start_tn = 2*(start_scale_pos-0.5)*t_floor+t_air;
     delayt_tfloor_scale = _delayt_tfloor_scale;
     alp=0;

     x0=_x0,y0=_y0,z0=_z0;
     setBezier( z0, x0, _L, _H0,tair_tfloor_scale, _forward);
     x=y=z=0;

     cal_param.delay_t = delayt_tfloor_scale * t_floor;
     cal_param.use_t_long = 1000;
     cal_param.tn_last=cal_param.tn_now=0;
     cal_param.T_now=cal_param.T_last=t_floor+t_air;
     walk_leg_state.is_add_start_tn = false;
     walk_leg_state.in_delay = true;
     walk_leg_state.to_init_pos = true;
     walk_leg_state.walking = false;
     walk_leg_state.in_air_else_floor = false;
     walk_leg_state.start_act =false;
}

void WalkLegClass::move2InitPos(float _use_t_long=1000)
{
    calPos(start_tn);
    cal_param.use_t_long = _use_t_long;
    leg->moveToPos(x/1000,y/1000,z/1000,0,0,_use_t_long);
}

//更新腿位置  _dt:调用时间间隔 ms
void WalkLegClass::update(float _dt)
{
    calTnNow(_dt);
    leg->update(_dt);
    walk_leg_state.x=leg->get_x()*1000;
    walk_leg_state.y=leg->get_y()*1000;
}
//_dt:ms
void WalkLegClass::calTnNow(float _dt)
{
    if(walk_leg_state.to_init_pos) //没到初始位置
    {
        cal_param.use_t_long -=_dt;
        if(cal_param.use_t_long<=0) walk_leg_state.to_init_pos = false;
        return;
    }

    if(! walk_leg_state.start_act)
        return;

    if(walk_leg_state.in_delay)
    {
        cal_param.delay_t-=_dt;
        if(cal_param.delay_t<=0) walk_leg_state.in_delay = false;
        return;
    }
    cal_param.T_last = cal_param.T_now;
    cal_param.T_now = t_floor + t_air;
    cal_param.tn_last = cal_param.tn_now;
    cal_param.tn_now = cal_param.tn_last/cal_param.T_last * cal_param.T_now + _dt;    //如果周期突变，保证在周期中时刻的比例不变
    if(!walk_leg_state.is_add_start_tn)    //初始相位差只加一次
    {
      cal_param.tn_now += start_tn ;             //当前腿时刻
      walk_leg_state.is_add_start_tn=true;
    }
    if(cal_param.tn_now>cal_param.T_now) cal_param.tn_now-= cal_param.T_now;  //在周期中的时刻

    if(cal_param.tn_last>=t_air && cal_param.tn_now<t_air)  //判断是否完成一个周期
         over_flag=true;
    else over_flag=false;

    if(over_flag==true) walk_leg_num++;

    if(cal_param.tn_now>=t_floor/2+t_air && cal_param.tn_last<t_floor/2+t_air)
        fixPointFlag=true;
    else fixPointFlag=false;

    walk_leg_state.walking = true;
    calPos(cal_param.tn_now);
    printf("xyz %f %f %f\n",x,y,z);
    leg->setPos(x/1000,y/1000,z/1000,0,0);
}

void WalkLegClass::calPos(float _t_now)
{
    if(_t_now<t_air)                  //曲线段  由wx12 wy12控制
    {
        walk_leg_state.in_air_else_floor = true;
        Bezier(&z,&x,wx12,wy12,bezier_len,_t_now/t_air);
    }
    else if(_t_now<=t_floor+t_air)    //直线段
    {
        walk_leg_state.in_air_else_floor = false;

        Bezier(&z,&x,wx2,wy2,2,(_t_now-t_air)/t_floor);	    //均匀的直线
    }

   //if(alp!=0) Rotate(&x,&y,L5/2,0,forward*alp);  //围绕腿的原点旋转

   walk_leg_state.tn = _t_now;
}

void WalkLegClass::startAct(bool _start)
{
     walk_leg_state.start_act = _start;
    if(! walk_leg_state.start_act)
        walk_leg_state.walking = false;
}

void WalkLegClass::reStart()  //有问题 没找到
{
    cal_param.delay_t = delayt_tfloor_scale * t_floor;
    cal_param.use_t_long = 1000;
    cal_param.tn_last=cal_param.tn_now=0;
    cal_param.T_now=cal_param.T_last=t_floor+t_air;
    walk_leg_state.is_add_start_tn = false;
    walk_leg_state.in_delay = true;
    walk_leg_state.to_init_pos = true;
    walk_leg_state.walking = false;
    walk_leg_state.in_air_else_floor = false;
    walk_leg_state.start_act =false;

    move2InitPos(1000);

}

void WalkLegClass::set_t_floor(float _t_floor)
{
    t_floor = _t_floor;
    t_air = t_floor * tair_tfloor_scale;
}

float WalkLegClass::setLegSpeed(float speed,float _t_floor )
{
    t_floor=_t_floor; t_air=tair_tfloor_scale*_t_floor;

    float dspeed=speed-v_leg;
    L+=dspeed*_t_floor ;
    L =constrainValue(L,min_L,max_L);

    setBezier(z0,x0,L,H0,tair_tfloor_scale,forward);   //更新贝赛尔点

    v_leg=L/_t_floor;
    return v_leg;
}

void WalkLegClass::setBezier(float _x0,float _y0,float _L,float _H0,float _tair_tfloor_scale,int _forward)
{
     float dL;  //贝赛尔曲线水平段至曲线段的控制长度
     int i=0;
     float k = 2048.0f/1859;  //高度系数
    dL = _L /(bezier_len-1) * _tair_tfloor_scale;
    wx12[0]=_L/2;         wx12[1]=_L/2+dL;              wx12[2]=_L/2+2*dL;      wx12[3]=_L/2+3*dL;
    wx12[4]=_L/2+4*dL;    wx12[5]=-_L/2+4*dL;           wx12[6]=-_L/2-4*dL;     wx12[7]=-_L/2-4*dL;
    wx12[8]=-_L/2-3*dL;   wx12[9]=-_L/2-2*dL;           wx12[10]=-_L/2-dL;      wx12[11]=-_L/2;
    wy12[0]=0;            wy12[1]=0;                    wy12[2]=0;              wy12[3]=-_H0*k;
    wy12[4]=-_H0*k;       wy12[5]=-_H0*k;               wy12[6]=-_H0*k;;        wy12[7]=-_H0*k;
    wy12[8]=-_H0*k;       wy12[9]=0;                    wy12[10]=0;             wy12[11]=0;

    wx2[0]=-_L/2;  wx2[1]=_L/2;
    wy2[0]=0;     wy2[1]=0;

    for(i=0;i<bezier_len;i++)
    {
      wx12[i]+=_x0;
      wy12[i]+=_y0;
    }
    wx2[0]=-_L/2+_x0;  wx2[1]=_L/2+_x0;
    wy2[0]=_y0;       wy2[1]=_y0;

    if(forward==-1)                                      //关于中心对称
    {
        for(i=0;i<12;i++)
        {
            wx12[i]=-wx12[i];
        }
        wx2[0]=-wx2[0];  wx2[1]=-wx2[1];
    }
}
