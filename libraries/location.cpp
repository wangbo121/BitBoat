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
wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
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
 * Function:     get_bearing_point_2_point_NED
 * Description:  输入是两个北东地坐标系中的两个的坐标点，
 *               输出是二者之间的连线与正北也就是NED坐标系中x轴的夹角，-180-+180，正北为0，东为正，西为负
 *               这个函数其实是获取方位角度的，
 *               方位角定义：2点连线与正北的夹角，而这个方位角其实就是我们不考虑偏航距离的期望航迹角
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

#if 0
    /*
     * 这是以前用东北天坐标系时采用的，实验室飞控用的这个，
     * 但是呢后来改成了北东地NED坐标就不再需要
     * 20170508
     */
    float return_bearing_radian;
    return_bearing_radian=atan2(vector_y,vector_x);
    printf("return_bearing_radian=%f\n",return_bearing_radian);
#endif

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
 * 输入三个点的经纬度，输出单位是米的偏航距
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
    //CTE_m=fabs(CTE_m);//计算偏航距离时，已经考虑了在目标航线左侧时为正，右侧时为负，所以不需要绝对值，直接再用pid控制方法就可以了
    global_bool_boatpilot.cte_distance_error=(short)fabs(CTE_m)*100;
    //printf("偏航距离是 NED=%f[m]\n",CTE_m);//20170508已测试

    //0.001对应1000米对应1弧度也就是57度左右，0.0001对应10000米对应57度

    CTE_p=(float)gcs2ap_radio_all.cte_p*0.0001;
    CTE_i=(float)gcs2ap_radio_all.cte_i*0.1*0.000039215;
    CTE_d=(float)gcs2ap_radio_all.cte_d*0.1;

    gamma_CTE = get_pid_cte(CTE_m, 1,CTE_p,CTE_i,CTE_d);//最终由偏航距计算的修正的补偿方向舵角
    gamma_CTE_max_radian=convert_degree_to_radian((float)gcs2ap_radio_all.cte_max_degree);
    //printf("gamma_CTE=%f\n",gamma_CTE);//20170508已测试

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
    global_bool_boatpilot.cte_error_check_radian=(short)gamma_CTE*1000;

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

#if 0
/**************************没有采用北东地坐标系之前的************************************/
/*
 * Function:     get_bearing_loc_to_loc
 * Description:  输入是两个location的坐标点，输出是二者之间的夹角，-180-+180，正北为0，东为正，西为负，
 *               这个函数其实是获取方位角度的，
 *               方位角定义：2点连线与正北的夹角，而这个方位角其实就是我们不考虑偏航距离的期望航迹角
 */
float get_bearing_loc_to_loc(struct T_LOCATION *start_loc, struct T_LOCATION *end_loc)
{
    float start_mercator_x = 0.0, start_mercator_y = 0.0;
    float loc_end_mercator_x = 0.0, loc_end_mercator_y = 0.0;
    float vector_x = 0.0, vector_y = 0.0;
    float ret_degree = 0.0;

    unsigned char bool_north = 0, bool_east = 0;
    unsigned char bool_great_than_45deg = 0;
    float change_temp = 0.0;

    start_mercator_x = get_mercator_x(start_loc);
    start_mercator_y = get_mercator_y(start_loc);
    loc_end_mercator_x = get_mercator_x(end_loc);
    loc_end_mercator_y = get_mercator_y(end_loc);

    vector_x = loc_end_mercator_x - start_mercator_x;
    vector_y = loc_end_mercator_y - start_mercator_y;

    if (vector_x > 0)
        bool_east = 1;
    else
        bool_east = 0;

    if (vector_y > 0)
        bool_north = 1;
    else
        bool_north = 0;

    vector_x = fabsf(vector_x);
    vector_y = fabsf(vector_y);

    if (vector_x > vector_y)
    {
        change_temp = vector_x;
        vector_x = vector_y;
        vector_y = change_temp;
        bool_great_than_45deg = 1;
    }
    else
    {
        bool_great_than_45deg = 0;
    }

    //if ((vector_y != 0) && (!isnan(atan(vector_x / vector_y))))
    if ((vector_y != 0))
        ret_degree = atan(vector_x / vector_y);  //0~PI/4
    else
        ret_degree = M_PI / 2;

    if (bool_great_than_45deg)
        ret_degree = M_PI / 2 - ret_degree;      //PI/4~PI/2
    if (bool_north && bool_east)
        ret_degree = ret_degree+0;               //0~PI/4
    if (bool_north && !bool_east)
        ret_degree = 0 - ret_degree;
    if (!bool_north && !bool_east)
        ret_degree = ret_degree - M_PI;
    if (!bool_north && bool_east)
        ret_degree = M_PI - ret_degree;

    return ret_degree;
}

/*
 * Function:     arrive_specific_location_over_line
 * Description:  判断是否到达目标点，根据是过线，
 *               即计算 上一目标点到过其与当前目标点的连线的垂线的距离 和
 *               当前位置与过上一目标点与当前目标点的连线的垂线的距离 的乘积，如果是正说明还没有到达
 *               如果是负说明过了这个目标点了。
 *               这个是根据以前实验室无人机的算法写，但是用投影的正负来判断更简单，所以有了另外的算法
 */
unsigned char arrive_specific_location_over_line(struct T_LOCATION *previous_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc)
{
    float start_mercator_x = 0.0, start_mercator_y = 0.0;
    float end_mercator_x = 0.0, end_mercator_y = 0.0;
    float current_mercator_x=0.0,current_mercator_y=0.0;
    float vector_x = 0.0, vector_y = 0.0;
    float distance_start_to_line;
    float distance_current_to_line;
    float sin_theta;
    float cos_theta;

    static unsigned char arrive_cnt;

    start_mercator_x = get_mercator_x(previous_target_loc);
    start_mercator_y = get_mercator_y(previous_target_loc);
    end_mercator_x = get_mercator_x(target_loc);
    end_mercator_y = get_mercator_y(target_loc);
    current_mercator_x=get_mercator_x(current_loc);
    current_mercator_y=get_mercator_y(current_loc);

    vector_x = end_mercator_x - start_mercator_x;
    vector_y = end_mercator_y - start_mercator_y;

    distance_start_to_line=get_distance_loc_to_loc(previous_target_loc,target_loc);

    if(distance_start_to_line!=0)
    {
        sin_theta=vector_y/distance_start_to_line;
        cos_theta=vector_x/distance_start_to_line;
    }
    else
    {
        sin_theta=0;
        cos_theta=0;
    }

    vector_x = end_mercator_x - current_mercator_x;
    vector_y = end_mercator_y - current_mercator_y;

    distance_current_to_line=vector_x*cos_theta+vector_y*sin_theta;

    if(!sgn(distance_start_to_line*distance_current_to_line))
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

unsigned char arrive_specific_location_over_line_project(struct T_LOCATION *previous_target_loc,struct T_LOCATION *current_loc,struct T_LOCATION *target_loc)
{
    float start_mercator_x = 0.0, start_mercator_y = 0.0;
    float end_mercator_x = 0.0, end_mercator_y = 0.0;
    float current_mercator_x=0.0,current_mercator_y=0.0;
    float vector_target2previous_x = 0.0, vector_target2previous_y = 0.0;
    float vector_target2cur_x = 0.0, vector_target2cur_y = 0.0;
    float dot_multi;//内积

    static unsigned char arrive_cnt;

    start_mercator_x = get_mercator_x(previous_target_loc);
    start_mercator_y = get_mercator_y(previous_target_loc);
    end_mercator_x = get_mercator_x(target_loc);
    end_mercator_y = get_mercator_y(target_loc);
    current_mercator_x=get_mercator_x(current_loc);
    current_mercator_y=get_mercator_y(current_loc);

    vector_target2previous_x = -(end_mercator_x - start_mercator_x);
    vector_target2previous_y = -(end_mercator_y - start_mercator_y);

    vector_target2cur_x = -(end_mercator_x - current_mercator_x);
    vector_target2cur_y = -(end_mercator_y - current_mercator_y);

    dot_multi=vector_target2previous_x*vector_target2cur_x+vector_target2previous_y*vector_target2cur_y;

    if(!sgn(dot_multi))
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
 * Function:     get_cross_track_error_correct_radian
 * Description:  根据当前位置，目标航点，以及命令航向（规划的路径航向），
 *               计算得出当前位置到规划航线的距离，从而进行目标航向的补偿，
 *               目的是让船体尽快靠近航线。
 */
float get_cross_track_error_correct_radian(struct T_LOCATION *target_loc, struct T_LOCATION *current, float command_radian)
{
    float vector_longitude = 0.0;/*转换为莫卡托坐标系的向量经度，单位：米[m]*/
    float vector_latitude = 0.0;/*转换为莫卡托坐标系的向量纬度，单位：米[m]*/

    float cross_track_error = 0.0;/*计算得到的偏航的距离，单位：米[m]*/
    float gamma_CTE = 0.0;/*乘以偏航距 比例系数后的数值，偏航距比例系数0--100/10.00*/
    float cte_max_radian=0.0;

    float current_to_target_radian=0.0;

    float delta_bearing=0.0;
    float cte_p=0.0;
    float cte_i=0.0;
    float cte_d=0.0;

    vector_longitude = get_mercator_x(target_loc) - get_mercator_x(current);
    vector_latitude = get_mercator_y(target_loc) - get_mercator_y(current);

    current_to_target_radian = get_bearing_loc_to_loc(current, target_loc);
    cross_track_error = sqrt(powf(vector_longitude, 2.0) + powf(vector_latitude, 2.0))*sin(current_to_target_radian - command_radian);

    /*
     * cross_track_error这个偏航距离很容易就是+300或者-300，太大了，基本都是超过了cte_max_radian，也就没有意义了
     * 如何把这个距离差值映射到-pi--+pi呢
     * 目前暂时定义为距离10000米对应1弧度，1弧度是57度
     * cross_track_error的单位是[米]
     */
    cte_p=(float)global_bool_boatpilot.cte_p*0.0001;//10000米对应1弧度也就是57度左右
    cte_i=(float)global_bool_boatpilot.cte_i*0.1*0.000039215;
    cte_d=(float)global_bool_boatpilot.cte_d*0.1;
    gamma_CTE = get_pid(cross_track_error, 1,cte_p,cte_i,cte_d);

    cte_max_radian=convert_degree_to_radian((float)gcs2ap_radio_all.cte_max_degree);

    if(cte_max_radian>=MAX_CTE_CORRECT)
    {
        cte_max_radian=MAX_CTE_CORRECT;
    }
    else if(cte_max_radian<=0)
    {
        cte_max_radian=0;
    }

    if (gamma_CTE>cte_max_radian)
    {
        gamma_CTE = cte_max_radian;
    }
    else if (gamma_CTE<-cte_max_radian)
    {
        gamma_CTE = -cte_max_radian;
    }

    /*如果船舶现在目标航线的右侧，得到的结果为负值，然后就更加的往左打舵，如果为正值，就更加的往右打舵*/
    delta_bearing=current_to_target_radian-command_radian;
    if(delta_bearing>0)
    {
        //船当前位置在航线的左侧，打右舵，所以返回正
        printf("在航线的左侧了，打右舵gamma_CTE=%f\n",gamma_CTE);
    }
    else
    {
        //船当前位置在航线的右侧，打左舵，所以返回负
        printf("在航线的右侧了，打左舵gamma_CTE=%f\n",gamma_CTE);
        gamma_CTE=-gamma_CTE;//因为前面计算偏航距离计算的是绝对值，没有正负所以这里符号需要取反
    }

    return gamma_CTE;
}

/*
 * Function:     get_cross_track_error_correct_radian
 * Description:  根据当前位置，目标航点，以及命令航向（规划的路径航向），
 *               计算得出当前位置到规划航线的距离，从而进行目标航向的补偿，
 *               目的是让船体尽快靠近航线。
 *               这个是根据坐标系的旋转求得的偏航距离，上面那个是以前无人机上用的。
 */
float get_cross_track_error_correct_radian2(struct T_LOCATION *previous_target_loc, struct T_LOCATION *current_loc, struct T_LOCATION *target_loc)
{
    float vector_longitude = 0.0;/*转换为莫卡托坐标系的向量经度，单位：米[m]*/
    float vector_latitude = 0.0;/*转换为莫卡托坐标系的向量纬度，单位：米[m]*/

    float cross_track_error = 0.0;/*计算得到的偏航的距离，单位：米[m]*/
    float gamma_CTE = 0.0;/*乘以偏航距 比例系数后的数值，偏航距比例系数0--100/10.00*/
    float cte_max_radian=0.0;
    float current_to_target_radian=0.0;

    float delta_bearing=0.0;
    float cte_p=0.0;
    float cte_i=0.0;
    float cte_d=0.0;

    float command_path_radian=0.0;

    float sin_t=0.0;
    float cos_t=0.0;

    float distance=0.0;

    //获取前一航点到当前航点距离
    distance=get_distance_loc_to_loc(previous_target_loc,target_loc);

    vector_longitude = get_mercator_x(target_loc) - get_mercator_x(previous_target_loc);
    vector_latitude = get_mercator_y(target_loc) - get_mercator_y(previous_target_loc);
    if(distance>30)
    {
        //距离大于30米时才算，如果太近就不用偏航距离了，直接到达即可
        sin_t=vector_latitude/distance;
        cos_t=vector_longitude/distance;
    }
    else
    {
        sin_t=0.0;
        cos_t=0.0;
    }

    vector_longitude = get_mercator_x(target_loc) - get_mercator_x(current_loc);
    vector_latitude = get_mercator_y(target_loc) - get_mercator_y(current_loc);
    cross_track_error=-sin_t*vector_longitude+cos_t*vector_latitude;
    cross_track_error=fabs(cross_track_error);
    global_bool_boatpilot.cte_distance_error=(short)cross_track_error;
    printf("偏航距离是 第一种方法=%f[m]\n",cross_track_error);

/************************************************************************************/
    command_path_radian = get_bearing_loc_to_loc(previous_target_loc, target_loc);

    vector_longitude = get_mercator_x(target_loc) - get_mercator_x(current_loc);
    vector_latitude = get_mercator_y(target_loc) - get_mercator_y(current_loc);

    current_to_target_radian = get_bearing_loc_to_loc(current_loc, target_loc);

    cross_track_error = sqrt(powf(vector_longitude, 2.0) + powf(vector_latitude, 2.0))*sin(current_to_target_radian - command_path_radian);
    cross_track_error=fabs(cross_track_error);
    printf("偏航距离是 第二种方法=%f[m]\n",cross_track_error);

    cte_p=(float)global_bool_boatpilot.cte_p*0.0001;//10000米对应1弧度也就是57度左右
    cte_i=(float)global_bool_boatpilot.cte_i*0.1*0.000039215;
    cte_d=(float)global_bool_boatpilot.cte_d*0.1;
    gamma_CTE = get_pid_cte(cross_track_error, 1,cte_p,cte_i,cte_d);

    cte_max_radian=convert_degree_to_radian((float)gcs2ap_radio_all.cte_max_degree);

    if(cte_max_radian>=MAX_CTE_CORRECT)
    {
        cte_max_radian=MAX_CTE_CORRECT;
    }
    else if(cte_max_radian<=0)
    {
        cte_max_radian=0;
    }

    if (gamma_CTE>cte_max_radian)
    {
        gamma_CTE = cte_max_radian;
    }
    else if (gamma_CTE<-cte_max_radian)
    {
        gamma_CTE = -cte_max_radian;
    }

    /*如果船舶现在目标航线的右侧，得到的结果为负值，然后就更加的往左打舵，如果为正值，就更加的往右打舵*/
    delta_bearing=current_to_target_radian-command_path_radian;
    if(delta_bearing>0)
    {
        //船当前位置在航线的左侧，打右舵，所以返回正
        printf("在航线的左侧了，打右舵gamma_CTE=%f\n",convert_radian_to_degree(gamma_CTE));

    }
    else
    {
        //船当前位置在航线的右侧，打左舵
        gamma_CTE=-gamma_CTE;//因为前面计算偏航距离计算的是绝对值，没有正负所以这里符号需要取反
        printf("在航线的右侧了，打左舵gamma_CTE=%f\n",convert_radian_to_degree(gamma_CTE));
    }

    global_bool_boatpilot.cte_error_check_radian=(short)gamma_CTE*1000;

    return gamma_CTE;
}
#endif
