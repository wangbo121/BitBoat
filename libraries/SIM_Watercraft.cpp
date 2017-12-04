/*
 * SIM_Watercraft.cpp
 *
 *  Created on: 2017-11-25
 *      Author: wangbo
 */

#include "SIM_Watercraft.h"
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h> //#define	RAND_MAX	2147483647
#include <iostream>
#include <string.h>

#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)
#endif

template <typename T>
float wrap_2PI(const T radian)
{
    float res = fmodf(static_cast<float>(radian), M_2PI);
    if (res < 0) {
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



	char *saveptr=NULL;
	char *s = strdup(home_str);
	char *lat_s = strtok_r(s, ",", &saveptr);
	char *lon_s = strtok_r(NULL, ",", &saveptr);
	char *alt_s = strtok_r(NULL, ",", &saveptr);
	char *yaw_s = strtok_r(NULL, ",", &saveptr);

	memset(&home, 0, sizeof(home));
	home.lat = atof(lat_s) * 1.0e7;
	home.lng = atof(lon_s) * 1.0e7;
	home.alt = atof(alt_s) * 1.0e2;
	location = home;
	ground_level = home.alt*0.01;//20170818已测试，当home.alt=0.0时，ground_level也是0，所以on_ground函数，只需要判断position.z是不是大于0，大于0则是到地下了

	//std::cout<<"location.lng="<<location.lng<<std::endl;
	//std::cout<<"location.lat="<<location.lat<<std::endl;
	//std::cout<<" ground_level="<< ground_level<<std::endl;//20170818已测试
}

/*
  update the multicopter simulation by one time step
 */
void Watercraft::update(const struct sitl_input &input)
{



	//float motor_speed[frame->num_motors];
	float motor_speed[4];

	//假设
	//motro_speed[0] : 方向舵
	//motro_speed[1] : 推进器

	float max_delta = 35 * DEG_TO_RAD;
	float U_max_speed = 3;//3m/s 最大速度 这个是沿船头方向的的速度

	motor_speed[0] = ( (float)input.servos[0] -1500.0) / 500.0;//把pwm转化为-1.0 ～ +0.1
	motor_speed[1] = ( (float)input.servos[1] -1000.0) / 1000.0;//把pwm转化为百分比 0.0～1.0

	//根据nomoto野本一阶模型更新航向角 psi
	float delta_time = 0.01;//0.01表示0.01秒
	static float psi; //偏航角
	static float r;//偏航角的角速度
	float K_usv = 0.5;
	float T_usv =2;
	float delta;//表示方向舵的舵角

	delta = max_delta * motor_speed[0];//对方向舵的角度进行了限制

	//printf("motor_speed[0] = %f\n",motor_speed[0]);
	//printf("delta = %f\n",delta);

	psi = r * delta_time + psi;
	r = ( K_usv * delta - r ) / T_usv * delta_time + r;
	//printf("r = %f\n",r);
#if 0
	//psi = psi % (2 * M_PI);
	psi = fmod( psi, (2 * M_PI) );

	if( psi > M_PI)
	{
		psi = psi - 2 * M_PI;
	}
#else
	psi = wrap_PI(psi);
#endif


	math_heading = psi;
	math_omega_z = r;

	//printf("psi = %f\n",psi);

	/*
	 * 更新位置
	 */
	//psi = -1.57;//20171204已测试
	//float U = U_max_speed * 1;//20171204已测试
	//float U = U_max_speed * motor_speed[1];
	float U = 10;
	velocity_x = U * cosf(psi);
	velocity_y = U *sinf(psi);

	//printf("velocity_x = %f\n",velocity_x);
	// new position vector
	position_x += velocity_x * delta_time;
	position_y += velocity_y * delta_time;


	update_position();



}


/*
   update location from position
*/
void Watercraft::update_position(void)
{
    float bearing = RAD_TO_DEG * (atan2f(position_y, position_x));//location_update函数中的参数需要 角度 不是弧度
    float distance = sqrtf(powf(position_x,2) + powf(position_y,2));

    location = home;



    location_update(location, bearing, distance);

    //Location &location_ref = location;
   // location_update(location_ref, bearing, distance);

    //location.alt  = home.alt - position.z*100.0f;


}

/*
   fill a sitl_fdm structure from the simulator state
*/
void Watercraft::fill_fdm( struct sitl_fdm &fdm) const
{
	/*
	 * fdm其实相当于整个飞控的外部设备，比如让正阳从别的设备读过来的
	 */


	//printf("location.lat = %d\n",location.lat);//20171204已测试

	fdm.latitude = (float)location.lat * 1e-7;
	fdm.longitude = (float)location.lng * 1e-7;//正常度数*1*10^7

	fdm.heading = math_heading * RAD_TO_DEG ;

	fdm.speedN = velocity_x;//m/s
	fdm.speedE = velocity_y;//m/s



}














float constrain_value(const float amt, const float low, const float high)
{
    // the check for NaN as a float prevents propagation of floating point
    // errors through any function that uses constrain_value(). The normal
    // float semantics already handle -Inf and +Inf
    if (isnan(amt)) {
        return (low + high) / 2;
    }

    if (amt < low) {
        return low;
    }

    if (amt > high) {
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
 *  extrapolate latitude/longitude given distances north and east
 *   extrapolate : （由已知资料对未知事实或价值）推算，推断
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

void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(DEG_TO_RAD * (bearing)) * distance;
    float ofs_east  = sinf(DEG_TO_RAD * (bearing)) * distance;
    location_offset(loc, ofs_north, ofs_east);
}






