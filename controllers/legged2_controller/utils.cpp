#include "utils.h"

//被Bezier()调用  绘制单方向的贝赛尔曲线
float decas(int degree, float coeff[], float t) {
	int r, i;
	float point[13];   //最大12阶贝赛尔曲线    
	for (i = 0; i < degree; i++) {
		point[i] = coeff[i];
	}
	for (r = 1; r < degree; r++) {
		for (i = 0; i < degree - r; i++) {
			point[i] = (1 - t)*point[i] + t * point[i + 1];
		}
	}
	return (point[0]);
}

//绘制n阶贝赛尔曲线   获得坐标(x,y);   mx[],my[] 控制点坐标数组;  n 数组长度
void Bezier(float *x, float *y, float mx[], float my[], int n, float t) {
	*x = decas(n, mx, t);
	*y = decas(n, my, t);
}


//坐标旋转  (x0,y0)旋转中心;(x,y)旋转值   a 旋转弧度 逆时针为正
void Rotate(float*x, float *y, float x0, float y0, float a) {
	float x1 = *x;
	float y1 = *y;
	*x = (x1 - x0)*cos(a) - (y1 - y0)*sin(a) + x0;
	*y = (y1 - y0)*cos(a) + (x1 - x0)*sin(a) + y0;
}

//平面围绕原点旋转
void Roat(float *x, float *y, float c) {
	float mx = *x; float my = *y;
	*x = mx * cos(c) - my * sin(c);
	*y = mx * sin(c) + my * cos(c);
}

//线性插值  ；_start 起始值;  _end终值；  _t ：0-1 比例，大于1返回1
float Lerp(float _start, float _end, float _t)
{
	if (_t > 1) _t = 1;
	return (_end - _start)* _t + _start;
}

//线性改变值  val：目标变量； setval：设定值； k：单次变化值； _threshold：阈值
void LineToValue(float *val, float setval, float k, float _threshold)
{
	if (fabs(*val - setval) > _threshold)
		*val -= k * (*val - setval) / fabs(*val - setval);
	else *val = setval;
}

void Dh2Tr(DH *_dh, TrMatrix* _tr)
{
	_tr->tr[0][0] = cos(_dh->theta); _tr->tr[0][1] = -sin(_dh->theta)*cos(_dh->alpha); _tr->tr[0][2] =  sin(_dh->theta)*sin(_dh->alpha);  _tr->tr[0][3] = _dh->a * cos(_dh->theta);
	_tr->tr[1][0] = sin(_dh->theta); _tr->tr[1][1] =  cos(_dh->theta)*cos(_dh->alpha); _tr->tr[1][2] = -cos(_dh->theta)*sin(_dh->alpha); _tr->tr[1][3] = _dh->a * sin(_dh->theta);
	_tr->tr[2][0] = 0; _tr->tr[2][1] = sin(_dh->alpha); _tr->tr[2][2] = cos(_dh->alpha);  _tr->tr[2][3] = _dh->d;
	_tr->tr[3][0] = 0; _tr->tr[3][1] = 0; _tr->tr[3][2] = 0;  _tr->tr[3][3] = 1;
}

void Tr2Rpy(TrMatrix* _tr, RPY* _rpy)
{
	_rpy->R = atan2(_tr->tr[2][1], _tr->tr[2][2]);
	_rpy->P = atan2(-_tr->tr[2][0], sqrt(_tr->tr[2][1] * _tr->tr[2][1] + _tr->tr[2][2] * _tr->tr[2][2]));
	_rpy->Y = atan2(_tr->tr[1][0], _tr->tr[0][0]);
}

TrMatrix TrxTr(TrMatrix _m1, TrMatrix _m2)
{
	TrMatrix ba_;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			ba_.tr[i][j] = 0;
			for (int k = 0; k < 4; k++) {
				ba_.tr[i][j] += (_m1.tr[i][k] * _m2.tr[k][j]);
			}
		}
	}
	return ba_;
}

void printfTr(TrMatrix *_tr)
{
	printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n\n", _tr->tr[0][0], _tr->tr[0][1], _tr->tr[0][2], _tr->tr[0][3],
		_tr->tr[1][0], _tr->tr[1][1], _tr->tr[1][2], _tr->tr[1][3],
		_tr->tr[2][0], _tr->tr[2][1], _tr->tr[2][2], _tr->tr[2][3],
		_tr->tr[3][0], _tr->tr[3][1], _tr->tr[3][2], _tr->tr[3][3]);
}

void printfDH(DH *dh)
{
	printf("theta:%f, d:%f, a:%f, alpha:%f\n", dh->theta, dh->d, dh->a , dh->alpha);
}

bool isNan(double x)
{
    return x != x;
}
