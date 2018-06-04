/*
 * navigation.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "global.h"
#include "boatlink_udp.h"
#include "gps.h"
#include "location.h"
#include "pid.h"

#include "navigation.h"
#include "BIT_Math.h"

//struct T_NAVIGATION auto_navigation;

struct T_NAVI_PARA                   navi_para;
struct T_NAVI_INPUT                  navi_input;
struct T_NAVI_OUTPUT                 navi_output;


static int get_navigation_parameter();
static int get_navigation_input();
static int get_navigation_output(struct T_NAVIGATION *ptr_auto_navigation,\
                                 struct WAY_POINT *ptr_wp_data,\
                                 nmea_msg *ptr_gps_data);

static int get_navigation_output(struct T_NAVI_OUTPUT    *ptr_navi_output,\
                                 struct T_NAVI_INPUT     *ptr_navi_input,\
                                 struct T_NAVI_PARA      *ptr_navi_para);
/*
 * 获取下一个航点编号
 */
static unsigned int get_next_wp_num(struct WAY_POINT *ptr_wp_data,\
                                    struct T_LOCATION *current_loc,\
                                    unsigned int current_target_wp_num,\
                                    unsigned int total_wp_num,\
                                    unsigned int arrive_radius);
/*
 * 这个函数是获取期望的船的航向速度与正北的夹角
 */
static float get_command_course_radian_NED(struct T_LOCATION *previous_target_loc,  struct T_LOCATION *current_loc, \
                                           struct T_LOCATION *target_loc, struct T_GUIDANCE_CONTROLLER *ptr_guidance);
void navigation_init(void)
{
	/*
	 * 结构体成员是指针，该成员在使用之前必须初始化
	 * 结构体内有不是例如char int等基本结构时，需要分配空间
	 * char int等基本类型，在声明时已经分配了空间，但是自己定义的结构是没有分配空间的
	 */
//    auto_navigation.current_loc            = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));
//	auto_navigation.current_target_loc     = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));
//	auto_navigation.previous_target_loc    = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));

	navi_output.current_loc            = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));
	navi_output.current_target_loc     = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));
	navi_output.previous_target_loc    = (struct T_LOCATION *)malloc(sizeof (struct T_LOCATION));



	navi_para.arrive_radius_low            = 10;
	navi_para.arrive_radius_high           = 100;
}

int get_navigation_parameter()
{
    navi_para.arrive_radius = (unsigned int)gcs2ap_all_udp.arrive_radius * 10; // gcs2ap_all中的5其实表示的是50米，要扩大10倍
    navi_para.arrive_radius = constrain_value(navi_para.arrive_radius, navi_para.arrive_radius_low, navi_para.arrive_radius_high);

    navi_para.wp_guide_no   = gcs2ap_all_udp.wp_guide_no;
    navi_para.workmode      = gcs2ap_all_udp.workmode;
    navi_para.auto_workmode = gcs2ap_all_udp.auto_workmode;
    navi_para.CTE_P         = (float)gcs2ap_all_udp.cte_p * 0.1;
    navi_para.CTE_I         = (float)gcs2ap_all_udp.cte_i * 0.01;
    navi_para.CTE_D         = (float)gcs2ap_all_udp.cte_d * 0.1;

    navi_para.total_wp_num  = global_bool_boatpilot.wp_total_num;
    navi_para.total_wp_num  = gcs2ap_all_udp.wp_total_num;


    return 0;
}

int navigation_loop( void )
{
    get_navigation_parameter();
	get_navigation_input();
	//get_navigation_output(&auto_navigation, wp_data, &gps_data);
	get_navigation_output(&navi_output, &navi_input, &navi_para);

	return 0;
}

int get_navigation_input()
{
    navi_input.ptr_wp_data            = wp_data;
    navi_input.ptr_gps_data           = &gps_data;

	return 0;
}


static int get_navigation_output(struct T_NAVI_OUTPUT    *ptr_navi_output,\
                                 struct T_NAVI_INPUT     *ptr_navi_input,\
                                 struct T_NAVI_PARA      *ptr_navi_para)
{
    unsigned int target_wp_num = 0;

    /*
     * 获取制导控制器 guidance_ctrl 的参数
     */
    static struct T_GUIDANCE_CONTROLLER guidance_ctrl;//局部变量

    guidance_ctrl.input.CTE_P = ptr_navi_para->CTE_P;
    guidance_ctrl.input.CTE_I = ptr_navi_para->CTE_I;
    guidance_ctrl.input.CTE_D = ptr_navi_para->CTE_D;

    /*
     * 获取位置环的反馈值，gps经纬度坐标
     */
    struct T_LOCATION current_loc;
    current_loc.lng = (float)(ptr_navi_input->ptr_gps_data->longitude)   * GPS_SCALE;
    current_loc.lat = (float)(ptr_navi_input->ptr_gps_data->latitude)    * GPS_SCALE;

    switch(ptr_navi_para->workmode)
    {
    case STOP_MODE:
        break;
    case RC_MODE:
        break;
    case AUTO_MODE:
        switch(ptr_navi_para->auto_workmode)
        {
        case AUTO_MISSION_MODE:
            break;
        case AUTO_GUIDE_MODE:
            break;
        case AUTO_LOITER_MODE:
            break;
        default:
            break;
        }

        if (ptr_navi_output->current_target_wp_cnt >= ptr_navi_para->total_wp_num)
        {
           ptr_navi_output->current_target_wp_cnt = 0;
        }


        /*1. 获取当前目标航点的标号*/
        target_wp_num  =  get_next_wp_num(ptr_navi_input->ptr_wp_data,\
                                          &current_loc,\
                                          ptr_navi_output->current_target_wp_cnt,\
                                          ptr_navi_para->total_wp_num,\
                                          ptr_navi_para->arrive_radius);

        /*2. 更新当前目标航点的信息*/
        ptr_navi_output->current_target_wp_cnt   = target_wp_num;
        ptr_navi_output->current_target_loc->lng = ((float)ptr_navi_input->ptr_wp_data[target_wp_num].lng) * WP_SCALE;
        ptr_navi_output->current_target_loc->lat = ((float)ptr_navi_input->ptr_wp_data[target_wp_num].lat) * WP_SCALE;

        if(target_wp_num>=1)
        {
            ptr_navi_output->previous_target_loc->lng = ((float)ptr_navi_input->ptr_wp_data[target_wp_num-1].lng) * WP_SCALE;
            ptr_navi_output->previous_target_loc->lat = ((float)ptr_navi_input->ptr_wp_data[target_wp_num-1].lat) * WP_SCALE;
        }
        else
        {
            ptr_navi_output->previous_target_loc->lng = ((float)ptr_navi_input->ptr_wp_data[ptr_navi_para->total_wp_num-1].lng) * WP_SCALE;
            ptr_navi_output->previous_target_loc->lat = ((float)ptr_navi_input->ptr_wp_data[ptr_navi_para->total_wp_num-1].lat) * WP_SCALE;
        }

        /*3. 获取 1期望course angle 2当前实际course angle 3期望heading angle 4当前实际heading angle */
        if ((ptr_navi_input->ptr_gps_data->longitude != 0 )&& (ptr_navi_input->ptr_gps_data->latitude != 0))
        {
            ptr_navi_output->command_course_angle_radian = get_command_course_radian_NED(ptr_navi_output->previous_target_loc,\
                                                                                         &current_loc,\
                                                                                         ptr_navi_output->current_target_loc,\
                                                                                         &guidance_ctrl);
            ptr_navi_output->gps_course_angle_radian     = ptr_navi_input->ptr_gps_data->course_radian;
        }

        ptr_navi_output->current_to_target_radian          = guidance_ctrl.output.current_to_target_radian;
        ptr_navi_output->current_to_target_degree          = guidance_ctrl.output.current_to_target_degree;
        ptr_navi_output->command_course_angle_radian       = guidance_ctrl.output.command_course_radian;
        ptr_navi_output->command_course_angle_degree       = guidance_ctrl.output.command_course_degree;
        break;
    case RTL_MODE:
        break;
    default:
        break;
    }

    return 0;
}














static unsigned int get_next_wp_num(struct WAY_POINT *ptr_wp_data,\
                                    struct T_LOCATION *current_loc,\
                                    unsigned int current_target_wp_cnt,\
                                    unsigned int total_wp_num,\
                                    unsigned int arrive_radius)
{
    unsigned char bool_arrive_point_radius = 0;//利用是否到达该航点某半径内 判断是否到达该航点
    unsigned char bool_arrive_point = 0;//利用是否冲过 上一目标航点到当前目标航点连线的垂线 判断是否到达该航点

    struct T_LOCATION last_target;
    struct T_LOCATION specific_location;
    struct T_LOCATION *specific_loc = NULL;

    specific_loc = &specific_location;

    if(current_target_wp_cnt >= 1)
    {
        last_target.lng = ((float)ptr_wp_data[current_target_wp_cnt-1].lng) * WP_SCALE;
        last_target.lat =((float)ptr_wp_data[current_target_wp_cnt-1].lat)  * WP_SCALE;
    }
    else
    {
        last_target.lng = ((float)ptr_wp_data[total_wp_num-1].lng) * WP_SCALE;
        last_target.lat = ((float)ptr_wp_data[total_wp_num-1].lat) * WP_SCALE;
    }

    specific_loc->lng = ((float)ptr_wp_data[current_target_wp_cnt].lng) * WP_SCALE;
    specific_loc->lat = ((float)ptr_wp_data[current_target_wp_cnt].lat) * WP_SCALE;

    bool_arrive_point_radius = arrive_specific_location_radius(current_loc, specific_loc, arrive_radius);               //这是利用  到达半径  判断
    bool_arrive_point        = arrive_specific_location_over_line_project_NED(&last_target, current_loc, specific_loc); //这是利用  过线  判断

    /*
     * 或者到达指定航点的某个半径范围的圆圈内
     * 或者超过了指定航点和下一航点的连线
     * 我们都认为是到达了该航点
     */
    if (bool_arrive_point || bool_arrive_point_radius)
    {
        if (current_target_wp_cnt >= (total_wp_num - 1))
        {
            current_target_wp_cnt = 0;
        }
        else
        {
            current_target_wp_cnt++;
        }
    }

    return current_target_wp_cnt;
}

/*
 * 这个函数是获取期望的船的航向速度与正北的夹角
 */
static float get_command_course_radian_NED(struct T_LOCATION *previous_target_loc,  struct T_LOCATION *current_loc, \
                                           struct T_LOCATION *target_loc,           struct T_GUIDANCE_CONTROLLER *ptr_guidance)
{
    float current_to_target_radian = 0.0;//当前航点与目标航点方位角的弧度值
    float command_course_radian = 0.0;//期望航向角heading或者说是yaw
    float cross_track_error_correct_radian = 0.0;

    current_to_target_radian = get_bearing_point_2_point_NED(current_loc, target_loc);
    current_to_target_radian = wrap_PI(current_to_target_radian);//从当前位置直接冲向目标航点，从小圈也就是小于180的方向转

    static BIT_PID guidance_pid;
    float CTE_P=0.0;
    float CTE_I=0.0;
    float CTE_D=0.0;

    CTE_P = ptr_guidance->input.CTE_P;
    CTE_I = ptr_guidance->input.CTE_I;
    CTE_D = ptr_guidance->input.CTE_D;
    guidance_pid.set_kP(CTE_P);
    guidance_pid.set_kI(CTE_I);
    guidance_pid.set_kD(CTE_D);
    cross_track_error_correct_radian = get_cross_track_error_correct_radian_NED_PID(previous_target_loc, current_loc, target_loc, &guidance_pid);

    /*
     * 虽然经过偏航距离修正，但是还是要朝着当前位置到目标航点直接方位角的小于180度方向走
     * 例如从当前位置直接到目标航点的角度为175度
     * 如果经过偏航距离修正后，成为185度，wrap_PI函数会导致目标航向-175度，这不行，得还是朝着175的那个小圈走
     */
    //command_heading_radian = wrap_PI(command_heading_radian); // 暂时勿删除，不需要这个wrap_PI函数，用了反而错误 要朝着小圈走
    command_course_radian = current_to_target_radian + cross_track_error_correct_radian;
    command_course_radian = constrain_value(command_course_radian, -(float)M_PI, (float)M_PI);

    /*
     * 把一些计算得到的中间变量送出去
     */
    ptr_guidance->output.current_to_target_radian    = current_to_target_radian;
    ptr_guidance->output.current_to_target_degree    = convert_radian_to_degree(current_to_target_radian);
    ptr_guidance->output.command_course_radian       = command_course_radian;
    ptr_guidance->output.command_course_degree       = convert_radian_to_degree(command_course_radian);

    return command_course_radian;
}



















