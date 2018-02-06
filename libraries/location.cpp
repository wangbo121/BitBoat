/*
 * struct T_LOCATION.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <stdio.h>
#include <math.h>

#include "global.h"
#include "pid.h"
#include "location.h"
#include "boatlink.h"
#include "Boat.h"

static int sgn(float x)
{
	if (x>0)
	{
		return 1;
	}
	else
	{
		return 0;
	}

	return 0;
}

static float haversin_equation(float theta)
{
	float v;

	v = sin(theta / 2);

	return v * v;
}

float convert_degree_to_radian(float angle_degree)
{
	return angle_degree * M_PI / 180;
}

float convert_radian_to_degree(float angle_radian)
{
	return angle_radian * 180.0 / M_PI;
}

/*
 * 把角度的弧度值转化为-PI～+PI 也就是类似于转化为+-180度内
 * wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
*/
float wrap_PI(float angle_radian)
{
	if (angle_radian > 10 * M_PI || angle_radian < -10 * M_PI)
	{
		// for very large numbers use modulus
	    angle_radian = fmodf(angle_radian, 2 * M_PI);
	}
	while (angle_radian > M_PI) angle_radian -= 2 * M_PI;
	while (angle_radian < -M_PI) angle_radian += 2 * M_PI;

	return angle_radian;
}

/*
 * 把弧度值转为0～2PI之间
 * wrap an angle in radians to 0..2PI
*/
float wrap_2PI(float angle_radian)
{
	if (angle_radian > 10 * M_PI || angle_radian < -10 * M_PI)
	{
		// for very large numbers use modulus
	    angle_radian = fmodf(angle_radian, 2 * M_PI);
	}
	while (angle_radian > 2 * M_PI) angle_radian -= 2 * M_PI;
	while (angle_radian < 0) angle_radian += 2 * M_PI;

	return angle_radian;
}

float get_distance_loc_to_loc(struct T_LOCATION *start_loc,struct T_LOCATION *end_loc)
{
	float longitude1=0.0, latitude1=0.0;
	float longitude2 = 0.0, latitude2 = 0.0;
	float delta_lng, delta_lat, h, distance_pt2pt;

	longitude1 = convert_degree_to_radian(start_loc->lng);
	latitude1 = convert_degree_to_radian(start_loc->lat);
	longitude2 = convert_degree_to_radian(end_loc->lng);
	latitude2 = convert_degree_to_radian(end_loc->lat);

	delta_lng = fabs(longitude1 - longitude2);
	delta_lat = fabs(latitude1 - latitude2);

	h = haversin_equation(delta_lat) + cos(latitude1) * cos(latitude2) * haversin_equation(delta_lng);

	distance_pt2pt = 2 * EARTH_RADIUS * sin(sqrt(h));

	return distance_pt2pt;
}

float get_mercator_x(struct T_LOCATION *loc)
{
	float longitude_radian = 0.0;
	float lambda_0 = 0.0;

	longitude_radian = convert_degree_to_radian(loc->lng);
	lambda_0 = REFERENCE_LONGITUDE_MERCATOR;

	return EARTH_CIRCUMFERENCE*(longitude_radian - lambda_0);
}

float get_mercator_y(struct T_LOCATION *loc)
{
	float phi_0 = 0.0;

	phi_0 = convert_degree_to_radian(loc->lat);

	return EARTH_CIRCUMFERENCE*log(tan(M_PI / 4 + phi_0 / 2));
}

unsigned char arrive_specific_location_radius(struct T_LOCATION *current_loc,struct T_LOCATION *specific_loc,unsigned int arrive_radius)
{
	float distance_pt2pt = 0.0;

	distance_pt2pt = get_distance_loc_to_loc(current_loc, specific_loc);

	if (distance_pt2pt < arrive_radius)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

/*****************************从这里开始按照我的论文里的理论设计制导guidance方法***************/
/*****************************/

/*
 * Function:       get_bearing_point_2_point_NED
 * Description:  输入是两个北东地坐标系中的两个的坐标点，
 *                        输出是二者之间的连线与正北也就是NED坐标系中x轴的夹角，-180-+180，正北为0，东为正，西为负
 *                        这个函数其实是获取方位角度的，
 *                        方位角定义：2点连线与正北的夹角，而这个方位角其实就是我们不考虑偏航距离的期望航迹角
 */
float get_bearing_point_2_point_NED(struct T_LOCATION *previous_target_loc, struct T_LOCATION *target_loc)
{
    struct T_NED start_point;
    struct T_NED end_point;

    /*在北东地坐标系中，北是x轴，东是y轴，朝地是z轴*/
    float vector_x = 0.0, vector_y = 0.0;

    start_point.x = get_mercator_y(previous_target_loc);
    start_point.y = get_mercator_x(previous_target_loc);
    end_point.x = get_mercator_y(target_loc);
    end_point.y = get_mercator_x(target_loc);

    vector_x = end_point.x - start_point.x;
    vector_y = end_point.y - start_point.y;

    return atan2(vector_y,vector_x);
}

unsigned char arrive_specific_location_over_line_project_NED(struct T_LOCATION *previous_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc)
{
    static unsigned char arrive_cnt;

    struct T_NED start_point;
    struct T_NED end_point;
    struct T_NED current_point;
    float vector_end2start_x = 0.0, vector_end2start_y = 0.0;
    float vector_end2cur_x = 0.0, vector_end2cur_y = 0.0;
    float dot_multi_NED;//内积 NED 北东地坐标系内积

    start_point.x = get_mercator_y(previous_target_loc);
    start_point.y = get_mercator_x(previous_target_loc);
    end_point.x = get_mercator_y(target_loc);
    end_point.y = get_mercator_x(target_loc);
    current_point.x = get_mercator_y(current_loc);
    current_point.y = get_mercator_x(current_loc);

    vector_end2start_x = start_point.x - end_point.x;
    vector_end2start_y = start_point.y - end_point.y;
    vector_end2cur_x = current_point.x - end_point.x;
    vector_end2cur_y = current_point.y - end_point.y;

    dot_multi_NED = vector_end2start_x * vector_end2cur_x + vector_end2start_y * vector_end2cur_y;

    if(!sgn(dot_multi_NED))
    {
        arrive_cnt++;
    }
    else if(arrive_cnt<=5)
    {
        arrive_cnt=0;
    }

    if(arrive_cnt>5)
    {
        arrive_cnt=0;
        return 1;
    }

    return 0;
}

/*
 * 输入三个点的经纬度，输出是中间的current的点到另外2点连线的垂直距离
 * 也就是输出偏航距，也就是船本身位置距离航线的垂直距离，单位是米
 */
float get_cross_track_error_NED(struct T_LOCATION *previous_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc)
{
    struct T_NED end_point;
    struct T_NED current_point;

    float kai_radian=0;
    float cte_m;//返回的偏航距，单位[m]

    end_point.x = get_mercator_y(target_loc);
    end_point.y = get_mercator_x(target_loc);
    current_point.x = get_mercator_y(current_loc);
    current_point.y = get_mercator_x(current_loc);

    /*kai是希腊字母的名称，这里代表前一目标点与当前目标点连线 与 正北方向的夹角*/
    kai_radian = get_bearing_point_2_point_NED(previous_target_loc,target_loc);

    cte_m = -sin(kai_radian) * (end_point.x - current_point.x) + cos(kai_radian) * (end_point.y - current_point.y);

    return cte_m;
}

float get_cross_track_error_correct_radian_NED(struct T_LOCATION *last_target_loc,struct T_LOCATION *current_loc, struct T_LOCATION *target_loc)
{
    float CTE_m=0.0;/*计算得到的偏航的距离，单位：米[m]*/

    /*
     * 乘以偏航距离 比例系数后的数值，偏航距比例系数0--100/10.00
     * 这个其实由偏航距离计算得到的补偿的目标期望航向角
     * 这个单位是[弧度]
     */
    float gamma_CTE = 0.0;
    float gamma_CTE_max_radian=0.0;//最大偏航距补偿角度[弧度]

    float CTE_p=0.0;
    float CTE_i=0.0;
    float CTE_d=0.0;

    float distance=0.0;

    CTE_m=get_cross_track_error_NED(last_target_loc, current_loc, target_loc);
    //printf("偏航距离是 NED=%f[m]\n",CTE_m);//20170508已测试
    global_bool_boatpilot.cte_distance_error=(short)fabs(CTE_m)*100;//这个是为了距离扩大100倍发送到地面站

#if 0
	CTE_p=(float)gcs2ap_radio_all.cte_p * 0.1;
	CTE_i=(float)gcs2ap_radio_all.cte_i *   0.01;
	CTE_d=(float)gcs2ap_radio_all.cte_d * 0.1;
    gamma_CTE = boat.pid_CTE.get_pid(CTE_m, 20, 1);//最终由偏航距计算的修正的补偿方向舵角
#else
    // 可以选择用反正切的
    CTE_p=(float)gcs2ap_radio_all.cte_p * 0.1;
    CTE_i=(float)gcs2ap_radio_all.cte_i *   0.01;
    CTE_d=(float)gcs2ap_radio_all.cte_d * 0.1;

	boat.pid_CTE.set_kP(CTE_p);
	boat.pid_CTE.set_kI(CTE_i);
	boat.pid_CTE.set_kD(CTE_d);

    float atan_cte=0.0;
    atan_cte = 0.5 * atan(CTE_m);// 2/pi*arctan(CTE_m) 限制幅度是pi/4
    //printf("atan_cte = %f \n",atan_cte);
    gamma_CTE = boat.pid_CTE.get_pid(atan_cte, 20, 1);
#endif
    //printf("gamma_CTE=%f\n",gamma_CTE);//20170508已测试

    gamma_CTE_max_radian=convert_degree_to_radian((float)gcs2ap_radio_all.cte_max_degree);
    if(gamma_CTE_max_radian>=MAX_CTE_CORRECT_RADIAN)
    {
        gamma_CTE_max_radian=MAX_CTE_CORRECT_RADIAN;
    }
    else if(gamma_CTE_max_radian<=0)
    {
        gamma_CTE_max_radian=0;
    }

    if (gamma_CTE>gamma_CTE_max_radian)
    {
        gamma_CTE = gamma_CTE_max_radian;
    }
    else if (gamma_CTE<-gamma_CTE_max_radian)
    {
        gamma_CTE = -gamma_CTE_max_radian;
    }
    global_bool_boatpilot.cte_error_check_radian=(short)gamma_CTE*1000;// 把数据放大1000倍，成为整型，然后通过实时数据发送给地面站

    if(CTE_m >= 0)
    {
        //printf("在航线的左侧了，打右舵gamma_CTE=%f\n",convert_radian_to_degree(gamma_CTE));//20170508已测试
    }
    else
    {
        //printf("在航线的右侧了，打左舵gamma_CTE=%f\n",convert_radian_to_degree(gamma_CTE));//20170508已测试
    }

    //获取前一航点到当前航点距离
    distance=get_distance_loc_to_loc(last_target_loc, target_loc);
    //只有距离大于30米时，才进行偏航距的补偿
    if(distance > 30.0)
    {
        return gamma_CTE;
    }
    else
    {
        return 0;
    }

    return 0;
}
