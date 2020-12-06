#include "arm.h"

void Arm::init(float _c10,float _c20,float _c30)
{
    L1 = 0.02; L2 = 0.05; L3 = 0.07;
    dh1 = { 0,L1,0,my_pi / 2 }; dh2 = { 0,0,L2,0 }; dh3 = { 0,0,L3,0 };
    c1=c10=_c10*my_pi/180;c2= c20=_c20*my_pi/180;c3=c30=_c30*my_pi/180;

    fk_3(c10,c20,c30);
}

void Arm::update(float _dt)
{
    if(set_move_to_pos)
    {
        if(use_t>=use_t_long)  //到达目标
            set_move_to_pos = false;
        use_t+=_dt;
        setPos(Lerp(start_x,target_x,use_t/use_t_long) , Lerp(start_y,target_y,use_t/use_t_long), Lerp(start_z,target_z,use_t/use_t_long));
    }
}

 void Arm::moveToPos(float _target_x,float _target_y,float _target_z,float _use_t_long)
 {
     set_move_to_pos = true;
     target_x = _target_x; target_y = _target_y; target_z = _target_z;
     use_t_long = _use_t_long;
     use_t = 0;
     start_x = x; start_y = y;start_z = z;
 }

void Arm::setJointAng(float _c1,float _c2,float _c3)
{
    setJointrRad(_c1*my_pi/180,_c2*my_pi/180,_c3*my_pi/180);
}

void Arm::setJointrRad(float _c1,float _c2,float _c3)
{
    _c1 = constrainValue(_c1,c1_rang.min*my_pi/180,c1_rang.max*my_pi/180);
    _c2 = constrainValue(_c2,c2_rang.min*my_pi/180,c2_rang.max*my_pi/180);
    _c3 = constrainValue(_c3,c3_rang.min*my_pi/180,c3_rang.max*my_pi/180);
    fk_3(_c1,_c2,_c3);
    c1=_c1;c2=_c2;c3=_c3;
}

bool Arm::setPos(float _x,float _y,float _z)
{
    return (ik_3(_x,_y,_z));
}

ArmAng  Arm::getAng()
{
    arm_ang.c1 = c1*180/my_pi;
    arm_ang.c2 = c2*180/my_pi;
    arm_ang.c3 = c3*180/my_pi;
    return  arm_ang;
}

ArmAng Arm::getAngRelative()
{
    arm_ang.c1=get_c1_relative();
    arm_ang.c2=get_c2_relative();
    arm_ang.c3=get_c3_relative();
    return  arm_ang;
}

void Arm::fk_3(float _c1, float _c2, float _c3)
{
    dh1.theta = _c1;  dh2.theta = _c2;  dh3.theta = _c3;
    Dh2Tr(&dh1, &tr1); Dh2Tr(&dh2, &tr2); Dh2Tr(&dh3, &tr3);
    //pos_tr = TrxTr(tr1, tr2);
    //printfTr(&pos_tr);
    pos_tr = TrxTr(TrxTr(tr1, tr2), tr3);
    //printfTr(&pos_tr);
    x = pos_tr.tr[0][3]; y = pos_tr.tr[1][3]; z = pos_tr.tr[2][3];
}

bool Arm::ik_3(double _x, double _y, double _z)
{
    double _c1, _c2, _c3;
    _c1 = atan(_y / _x);

    double m1 = _z - L1;
    double n1 = _x * cos(_c1) + _y * sin(_c1);
    double m2 = 2 * m1*L2;
    double n2 = 2 * n1*L2;
    double k = m1 * m1 + n1 * n1 + L2 * L2 - L3 * L3;
    double _det = 4 * n2*n2*k*k - 4 * (k*k - m2 * m2)*(m2*m2 + n2 * n2);  //防止sqrt内小于0 当角度为0时 可能会出现小的负值
    //printf("_det: %lf\n", _det);
    if (_det < 0  && _det >-0.00001) _det = 0;

    double _cos_val = (2 * n2*k + sqrt(_det)) / (2 * (m2*m2 + n2 * n2));   //可以差一点点
    //printf("_cos_val: %lf\n", _cos_val);
    if (_cos_val > 1 && _cos_val < 1+ 0.00001) _cos_val = 1;
    if (_cos_val < -1 && _cos_val > -1 - 0.00001) _cos_val = -1;
    _c2 = acos(_cos_val);

     _cos_val = ((n1 - L2 * cos(_c2)) / L3);   //可以差一点点
     if (_cos_val > 1 && _cos_val < 1 + 0.00001) _cos_val = 1;
     if (_cos_val < -1 && _cos_val > -1 - 0.00001) _cos_val = -1;
     _c3 = acos(_cos_val) - _c2;

     if (isNan(_c1) || isNan(_c2) ||isNan(_c3))
        return false;

    //c1 = _c1; c2 = _c2; c3 = _c3;
    setJointrRad( _c1, _c2, _c3);
    return true;

}
