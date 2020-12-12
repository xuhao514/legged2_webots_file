#ifndef __PID_H
#define __PID_H

class PID
{
public:
    float threshold;  //阈值
    float max_out;    //最大输出值
    void pidInit(float _k,float p,float i,float d,float _threshold,float _max_out)
    {
        k=_k;kp=p;ki=i;kd=d;
        threshold=_threshold; //阈值
        max_out=_max_out;    //最大输出
        setNum=0;     //设定值
        actualNum=0;  //实际值
        err=0;   //偏差值
        errNext=0;
        errNN=0;
        loc_sum=0;
        pos=0;
        max_sum = 10000000;
    };
    //增量式pid
    float pidIncUpdate(float _setNum,float _actualNum)
    {
        setNum=_setNum;
        actualNum=_actualNum;
        err=setNum - actualNum;
        if((err>threshold)||(err<-threshold))
        incrementSpeed=kp*(err-errNext)+ki*err+kd*(err-2*errNext+errNN);
        else incrementSpeed=0;

        errNext=err;       //errNext 上一次偏差，  errNN 上上次偏差
        errNN=errNext;

        incrementSpeed*=k;
        if(incrementSpeed > max_out )  incrementSpeed=max_out;
        if(incrementSpeed < -max_out) incrementSpeed=-max_out;

        return incrementSpeed;
    };
    //位置式pid
    float pidPosUpdate(float _setNum,float _actualNum)
    {
        setNum=_setNum;
        actualNum=_actualNum;
        err=setNum - actualNum;
        if((err>threshold)||(err<-threshold))
        loc_sum += err;
        if(loc_sum >= max_sum) loc_sum = max_sum;
        else if(loc_sum <= -max_sum) loc_sum= -max_sum;

        if(loc_sum > max_sum) loc_sum = max_sum;
        else if(loc_sum < -max_sum) loc_sum = -max_sum;

        pos = kp * err + ki * loc_sum  + kd * (err - errNext);
        errNext=err;

        pos*=k;
        if(pos > max_out )  pos = max_out;
        if(pos < -max_out) pos = -max_out;
        return pos;
    };
    void setPid(float p,float i,float d)
    {
        kp=p;ki=i;kd=d;
    };
private:
    float k;           //结果乘以个系数
    float kp;
    float ki;
    float kd;
    float setNum;     //设定值
    float actualNum;  //实际值

    float err;
    float errNext;
    float errNN;    //偏差值
    float loc_sum;     //累计积分位置
    float incrementSpeed;   //增量  调节量
    float pos;         //位置式输出
    float max_sum;

};


#endif
