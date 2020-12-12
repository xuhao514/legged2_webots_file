#ifndef __UTILS_H
#define __UTILS_H
#include "math.h"
#include "stdio.h"
#include "float.h"
//#pragma pack(1)  //结构体对齐

#define abs(x) ((x)>0?(x):-(x))
#define my_pi 3.1415926535
#define MAXVALUE 100000000000
//转化矩阵
struct TrMatrix
{
	//数据
	float tr[4][4];
};
//DH参数 theta,d,a,alpha;
struct DH
{
	//theta：绕Z轴旋转角度；d:绕Z轴平移距离; a:绕X轴平移距离;  alpha:绕X轴旋转角度
	float theta, d, a, alpha;
};
//RPY欧拉角
struct RPY
{
	//绕X轴旋转roll，绕Y轴旋转pitch，绕Z轴旋转yaw
	float R, P, Y;
};
//度
struct ArmAng
{
  float c1,c2,c3;
};
struct LegAng
{
  float c1,c2,c3,c4,c5;
};
//三维坐标
struct Pos
{
	float x,y,z;
};
//加速度
struct Accel
{
	float ax,ay,az;
};
//值的范围
class ValueRang
{
public:
    //ValueRang(){min = DBL_MIN; max =DBL_MAX;};  //无限大  无限小
	ValueRang(){min = -MAXVALUE; max =MAXVALUE;};
    double min,max;
};
//将数字约束在一个范围内
float constrainValue(float _val,float _min,float _max);
// DH参数->转化矩阵
void Dh2Tr(DH *_dh, TrMatrix* _tr);
// 转化矩阵-> 欧拉角
void Tr2Rpy(TrMatrix* _tr, RPY* _rpy);
//转化矩阵相乘
TrMatrix TrxTr(TrMatrix _tr1, TrMatrix _tr2);
//打印
void printfTr(TrMatrix *tr);
void printfDH(DH *dh);
//判断是否为一个数
bool isNan(double x);
void Bezier(float *x, float *y, float mx[], float my[], int n, float t);
void Rotate(float*x, float *y, float x0, float y0, float a);
void Roat(float *x, float *y, float c);
float Lerp(float _start, float _end, float _t);
void LineToValue(float *val, float setval, float k, float _threshold);


#endif
