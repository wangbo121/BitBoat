/*
 * SIM_Vehicle.cpp
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <sys/time.h>

#include "SIM_Vehicle.h"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
#endif

template <typename T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0)
    {
        res += M_2PI;
    }
    return res;
}

template float wrap_2PI<int>(const int radian);
template float wrap_2PI<short>(const short radian);
template float wrap_2PI<float>(const float radian);
template float wrap_2PI<double>(const double radian);

template <typename T>
float wrap_PI(const T radian)
{
    float res = wrap_2PI(radian);
    if (res > M_PI) {
        res -= M_2PI;
    }
    return res;
}

template float wrap_PI<int>(const int radian);
template float wrap_PI<short>(const short radian);
template float wrap_PI<float>(const float radian);
template float wrap_PI<double>(const double radian);

Watercraft::Watercraft(const char *home_str, const char *frame_str)
{
	math_heading = 0.0;
	math_omega_z = 0.0;

	position_x = 0.0;
	position_y = 0.0;
	velocity_x = 0.0;
	velocity_y = 0.0;

	char *saveptr=NULL;
	char *s = strdup(home_str);
	char *lat_s = strtok_r(s, ",", &saveptr);
	char *lon_s = strtok_r(NULL, ",", &saveptr);
	char *alt_s = strtok_r(NULL, ",", &saveptr);
	//char *yaw_s = strtok_r(NULL, ",", &saveptr);

	memset(&home, 0, sizeof(home));
	home.lat = atof(lat_s) * 1.0e7;
	home.lng = atof(lon_s) * 1.0e7;
	home.alt = atof(alt_s) * 1.0e2;
	location = home;


}

/*
 * 一步一步更新模拟器，也就是动力模型的差分方程
  update the multicopter simulation by one time step
 */
void Watercraft::update(const struct sitl_input &input)
{
	float motor_speed[4];

	float max_delta = 35 * DEG_TO_RAD;

	motor_speed[0] = ( (float)input.servos[0] -1500.0) / 500.0;//把pwm转化为-1.0 ～ +0.1
	motor_speed[1] = ( (float)input.servos[1] -1000.0) / 1000.0;//把pwm转化为百分比 0.0～1.0

	//根据Nomoto野本一阶模型更新航向角 psi
	float delta_time = 0.01;//0.01表示0.01秒
	static float psi; //偏航角
	static float r;//偏航角的角速度
	float K_usv = 0.5;
	float T_usv =2;
	float delta;//表示方向舵的舵角

	delta = max_delta * motor_speed[0];//对方向舵的角度进行了限制

	psi = r * delta_time + psi;
	r = ( K_usv * delta - r ) / T_usv * delta_time + r;
	psi = wrap_PI(psi);

	math_heading = psi;
	math_omega_z = r;

	float U = 10;// 假设速度是10米每秒，先跟油门量无关，以后修改
	velocity_x = U * cosf(psi);
	velocity_y = U *sinf(psi);

	position_x += velocity_x * delta_time;
	position_y += velocity_y * delta_time;

	update_position();
}

/*
 * 从位置更新经度纬度，经度纬度是location，位置是position
 * 利用速度计算得到position也就是位移量，然后加上前一时刻的经度纬度
 * 就是当前的经度和纬度
*/
void Watercraft::update_position(void)
{
    float bearing = RAD_TO_DEG * (atan2f(position_y, position_x));//location_update函数中的参数需要 角度 不是弧度
    float distance = sqrtf(powf(position_x,2) + powf(position_y,2));

    location = home;
    location_update(location, bearing, distance);
}

/*
 * 把模拟的计算出来的状态填入到一个仿真的动力模型结构中去
 * 然后这个飞行动力模型就是这个源文件的输出
*/
void Watercraft::fill_fdm( struct sitl_fdm &fdm) const
{
	fdm.latitude = (float)location.lat * 1e-7;
	fdm.longitude = (float)location.lng * 1e-7;//正常度数*1*10^7

	fdm.heading = math_heading * RAD_TO_DEG ;

	fdm.speedN = velocity_x;// m/s
	fdm.speedE = velocity_y;// m/s
}

float constrain_value(const float amt, const float low, const float high)
{
    if (isnan(amt))
    {
        return (low + high) / 2;
    }

    if (amt < low)
    {
        return low;
    }

    if (amt > high)
    {
        return high;
    }

    return amt;
}

float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

float longitude_scale(const struct Location &loc)
{
    float scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    return constrain_float(scale, 0.01f, 1.0f);
}

unsigned char is_zero(float x)
{
	if(x==0.0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 * 通过北向东向的距离的偏移量计算下一时刻的经度纬度
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (!is_zero(ofs_north) || !is_zero(ofs_east))
    {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}

/*
 * 通过距离和方位角计算下一时刻的经度和纬度
 */
void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(DEG_TO_RAD * (bearing)) * distance;
    float ofs_east  = sinf(DEG_TO_RAD * (bearing)) * distance;
    location_offset(loc, ofs_north, ofs_east);
}
