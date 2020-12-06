#include "leg.h"

void Leg::init(float _c10,float _c20,float _c30,float _c40,float _c50)
{
	L1 = 0.04; L2 = 0.052; L3 = 0.06; L4 = 0.043; L5 = 0.017;
    dh1 = { 0,0,L1,my_pi / 2 }; dh2 = { 0,0,L2,0 }; dh3 = { 0,0,L3,0 };
    dh4 = { 0,0,L4,my_pi/2 }; dh5 = { 0,0,L5,0 };

    c1=c10=_c10*my_pi/180;c2= c20=_c20*my_pi/180;c3=c30=_c30*my_pi/180;
    c4=c40=_c40*my_pi/180;c5= c50=_c50*my_pi/180;
    fk_5(c10,c20,c30,c40,c50);
}

void Leg::update(float _dt)
{
    if(set_move_to_pos)
    {
        if(use_t>=use_t_long)  //到达目标
            set_move_to_pos = false;
        use_t+=_dt;
        setPos(Lerp(start_x,target_x,use_t/use_t_long) , Lerp(start_y,target_y,use_t/use_t_long),Lerp(start_z,target_z,use_t/use_t_long),
               Lerp(start_P,target_P,use_t/use_t_long), Lerp(start_Y,target_Y,use_t/use_t_long) );
    }
}

 void Leg::moveToPos(float _target_x,float _target_y,float _target_z,float _target_P,float _target_Y,float _use_t_long)
 {

     set_move_to_pos = true;
     target_x = _target_x; target_y = _target_y;target_z = _target_z; target_P = _target_P;target_Y = _target_Y;
     use_t_long = _use_t_long;
     use_t = 0;
     start_x = x; start_y = y;start_z = z; start_P = P; start_Y = Y;
 }


void Leg::setJointAng(float _c1,float _c2,float _c3,float _c4,float _c5)
{
    setJointrRad(_c1*my_pi/180,_c2*my_pi/180,_c3*my_pi/180,_c4*my_pi/180,_c5*my_pi/180);
}

void Leg::setJointrRad(float _c1,float _c2,float _c3,float _c4,float _c5)
{
    _c1 = constrainValue(_c1,c1_rang.min*my_pi/180,c1_rang.max*my_pi/180);
    _c2 = constrainValue(_c2,c2_rang.min*my_pi/180,c2_rang.max*my_pi/180);
    _c3 = constrainValue(_c3,c3_rang.min*my_pi/180,c3_rang.max*my_pi/180);
    _c4 = constrainValue(_c4,c4_rang.min*my_pi/180,c4_rang.max*my_pi/180);
    _c5 = constrainValue(_c5,c5_rang.min*my_pi/180,c5_rang.max*my_pi/180);

    fk_5(_c1,_c2,_c3,_c4,_c5);
    c1=_c1;c2=_c2;c3=_c3;c4=_c4;c5=_c5;
}

//设定末端坐标
bool Leg::setPos(float _x,float _y,float _z,float _P,float _Y)
{
   return ik_5(_x,_y,_z,_P,_Y);
}

LegAng Leg::getAng()
{
    ang.c1 = c1*180/my_pi;
    ang.c2 = c2*180/my_pi;
    ang.c3 = c3*180/my_pi;
    ang.c4 = c4*180/my_pi;
    ang.c5 = c5*180/my_pi;
    return  ang;
}

LegAng Leg::getAngRelative()
{
    ang.c1 = get_c1_relative();
    ang.c2 = get_c2_relative();
    ang.c3 = get_c3_relative();
    ang.c4 = get_c4_relative();
    ang.c5 = get_c5_relative();
    return  ang;
}

void Leg::fk_5(float _c1, float _c2, float _c3, float _c4, float _c5)
{
	dh1.theta = _c1;  dh2.theta = _c2;  dh3.theta = _c3; dh4.theta = _c4;  dh5.theta = _c5;
	Dh2Tr(&dh1, &tr1); Dh2Tr(&dh2, &tr2); Dh2Tr(&dh3, &tr3);  Dh2Tr(&dh4, &tr4); Dh2Tr(&dh5, &tr5);
	//pos_tr = TrxTr(tr1, tr2);
	//printfTr(&pos_tr);
	pos_tr = TrxTr( TrxTr( TrxTr( TrxTr( tr1, tr2), tr3) , tr4 ) , tr5);
	//printfTr(&pos_tr);
	x = pos_tr.tr[0][3]; y = pos_tr.tr[1][3]; z = pos_tr.tr[2][3];
	Tr2Rpy(&pos_tr, &rpy);
}

bool Leg::ik_5(double _x, double _y, double _z , double _P ,double _Y)
{
    double _c1, _c2, _c3,_c4,_c5;
    _c1 = atan((_y - L5 * cos(_P)*sin(_Y)) / (_x - L5 * cos(_P)*cos(_Y)));
    _c5 = asin((_x*sin(_c1) - _y * cos(_c1)) / L5);

    R = asin(sin(_P)*sin(_c5) / cos(_P) / cos(_c5));

    double sin234 = -sin(_P) / cos(_c5);
    double cos234 = cos(_P)*cos(_Y - _c1) / cos(_c5);
    double m1 = _z  + L5 * sin(_P) - L4 * sin234;
    double n1 = _x * cos(_c1) - L5 * (cos(_P)*cos(_Y)*cos(_c1) + cos(_P)*sin(_Y)*sin(_c1)) - L1 + _y * sin(_c1) - L4 * cos234;

	double m2 = 2 * m1*L2;
	double n2 = 2 * n1*L2;
	double k = m1 *m1 + n1 * n1 + L2 *L2  - L3 *L3;
    //使用sin求解
    double _det = 4 * m2 *m2 * k *k - 4 * (k *k - n2 * n2)*(m2 *m2 + n2 * n2);  //防止sqrt内小于0  可能会小于一点点
	if (_det < 0  && _det >-0.00001) _det = 0;
    double _val = (2 * m2*k - sqrt(_det)) / (2 * (m2 *m2 + n2 *n2));
    if (_val > 1 && _val < 1 + 0.00001) _val = 1;
    if (_val < -1 && _val > -1 - 0.00001) _val = -1;
    _c2 = asin(_val);
    _val = ((m1 - L2 * sin(_c2)) / L3);   //防止acos内大于1或者小于-1  可能会大于一点点
    if (_val > 1 && _val < 1 + 0.00001) _val = 1;
    if (_val < -1 && _val > -1 - 0.00001) _val = -1;
    _c3 = asin(_val) - _c2;
    _c4 = asin(sin234) - _c2 - _c3;
    printf("c %f %f %f %f %f\n",_c1,_c2,_c3,_c4,_c5);
    //使用cos求解
	// double _det = 4 * n2 *n2 * k *k - 4 * (k *k - m2 * m2)*(m2 *m2 + n2 * n2);  //防止sqrt内小于0  可能会小于一点点
	// if (_det < 0  && _det >-0.00001) _det = 0;
    // double _cos_val = (2 * n2*k + sqrt(_det)) / (2 * (m2 *m2 + n2 *n2));
    // if (_cos_val > 1 && _cos_val < 1 + 0.00001) _cos_val = 1;
    // if (_cos_val < -1 && _cos_val > -1 - 0.00001) _cos_val = -1;
    // _c2 = acos(_cos_val);
    // _cos_val = ((n1 - L2 * cos(_c2)) / L3);   //防止acos内大于1或者小于-1  可能会大于一点点
    // if (_cos_val > 1 && _cos_val < 1 + 0.00001) _cos_val = 1;
    // if (_cos_val < -1 && _cos_val > -1 - 0.00001) _cos_val = -1;
    // _c3 = acos(_cos_val) - _c2;
    // _c4 = acos(cos234) - _c2 - _c3;
    // printf("c %f %f %f %f %f\n",_c1,_c2,_c3,_c4,_c5);

    if (isNan(_c1) || isNan(_c2) ||isNan(_c3) || isNan(_c4) || isNan(_c5))
       return false;

   // c1 = _c1; c2 = _c2; c3 = _c3; c4 = _c4; c5 = _c5;
   setJointrRad(_c1,_c2,_c3,_c4,_c5);
   return true;
}
