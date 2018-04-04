/*
 * navigation.h
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#ifndef HEADERS_NAVIGATION_H_
#define HEADERS_NAVIGATION_H_

//#include "boatlink.h"
#include "global.h"
#include "gps.h"

#define NAVIGATION_HEADING_ANGLE 0
#define NAVIGATION_COURSE_ANGLE 1

struct T_NAVIGATION
{
	/*
	 * in表示需要从外部传输进navigation这个循环的参数
	 */
	unsigned int in_arrive_radius;
	unsigned char in_total_wp_num;
	unsigned char in_wp_guide_no;//引导模式时，指定的航点编号
	unsigned char in_work_mode;//工作模式
	float in_CTE_p;
	float in_CTE_i;
	float in_CTE_d;


	struct T_LOCATION *previous_target_loc;
	struct T_LOCATION *current_target_loc;
	struct T_LOCATION *current_loc;

	/*
	 * course angle跟heading angle不同，
	 * heading angle指的是船头船尾连线 与 正北（x轴）的夹角，heading angle通常与yaw是相等的
	 * course  angle指的是前进速度方向 与 正北（x轴）的夹角
	 * 对我们有用的是前进速度方向，希望船体本身可以沿着期望的航线前进
	 * 但是我们能控制的是船体的摆动，也就是改变船头朝向
	 * 所以我们是通过改变船头朝向（heading angle）间接改变前进速度方向（course angle）
	 */
	float command_heading_angle_radian;//单位[弧度] 期望的 船头船尾 角度
	float gps_heading_angle_radian;//单位[弧度] 期望的 船头船尾 角度
	float gps_heading_angle_degree;//单位[度] 期望的 船头船尾 角度

	float command_course_angle_radian;//单位[弧度] 期望的 速度航向 角度
    float gps_course_angle_radian;//单位[弧度] 实际的 速度航向 角度

	unsigned int current_target_wp_cnt;//当前第几个目标航点
	unsigned int total_wp_num;//所有的航点总数
	unsigned int arrive_radius;//[米]这里直接用米，把gcs2ap的乘以10赋值给它

	int gps_lng;/*[度*0.00001]*/
	int gps_lat;/*[度*0.00001]*/
	int gps_spd;/*[knot*0.01]*/
};

extern struct T_NAVIGATION auto_navigation;

void navigation_init(void);

int navigation_loop(struct T_NAVIGATION *ptr_auto_navigation,\
										struct WAY_POINT *ptr_wp_data,\
										nmea_msg *ptr_gps_data);

float get_command_heading(struct T_LOCATION *previous_target_wp, struct T_LOCATION *target_wp, struct T_LOCATION *current_loc);

/*
 * 准确地说，get_command_heading_NED应该是get_command_course_radian_NED
 * 是获取前进速度航向角course angle，真实的heading需要用course angle减去侧滑角slide angle
 * 也就是heading加上侧滑角slide angle是速度航向角course angle
 */
float get_command_heading_NED(struct T_LOCATION *previous_target_loc,  struct T_LOCATION *current_loc, struct T_LOCATION *target_loc);

/*
 * 这个函数是获取期望的船的航向速度与正北的夹角
 */
float get_command_course_radian_NED(struct T_LOCATION *previous_target_loc,  struct T_LOCATION *current_loc, struct T_LOCATION *target_loc);

#endif /* HEADERS_NAVIGATION_H_ */
