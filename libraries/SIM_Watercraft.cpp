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

#include <math.h>


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

	float motor_speed[frame->num_motors];

	//假设
	//motro_speed[0] : 方向舵
	//motro_speed[1] : 推进器

	float max_delta = 35 * DEG_TO_RAD;
	float U_max_speed = 3;//3m/s 最大速度 这个是沿船头方向的的速度

	motor_speed[0] = ( (float)input.servos[0] -1500.0) / 500.0;//把pwm转化为-1.0 ～ +0.1
	motor_speed[1] = ( (float)input.servos[0] -1000.0) / 1000.0;//把pwm转化为百分比 0.0～1.0

	//根据nomoto野本一阶模型更新航向角 psi
	float delta_time = 0.01;//0.01表示0.01秒
	static float psi; //偏航角
	static float r;//偏航角的角速度
	float K_usv = 0.5;
	float T_usv =2;
	float delta;//表示方向舵的舵角

	delta = max_delta * motor_speed[0];//对方向舵的角度进行了限制

	psi = r * delta_time + psi;
	r = ( K_usv * delta - r ) / T_usv * delta_time + r;

	psi = psi % (2 * M_PI);

	if( psi > M_PI)
	{
		psi = psi - 2 * M_PI;
	}
	math_heading = psi;
	math_omega_z = r;



	/*
	 * 更新位置
	 */
	float U = U_max_speed * motor_speed[1];
	velocity_x = U * cosf(psi);
	velocity_y = U *sinf(psi);

	// new position vector
	position_x = velocity_x * delta_time;
	position_y = velocity_y * delta_time;


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

	fdm.latitude = location.lat;
	fdm.longitude = location.lng;//正常度数*1*10^7
	fdm.heading = math_heading;//弧度
	fdm.speedN = velocity_x;//m/s
	fdm.speedE = velocity_y;//m/s



}

