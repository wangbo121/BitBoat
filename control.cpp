/*
 * control.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "global.h"
#include "location.h"
#include "navigation.h"
#include "gps.h"
#include "pid.h"
#include "radio.h"
#include "servo.h"
#include "utility.h"
#include "save_data.h"
#include "control.h"
#include "Boat.h"
#include "boatlink_udp.h"

struct CTRL_PARA ctrlpara;
struct CTRL_INPUT ctrlinput;
struct CTRL_OUTPUT ctrloutput;

static int get_ctrlpara();
static int get_ctrlinput();
static int get_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput,struct CTRL_INPUT *ptr_ctrlinput,struct CTRL_PARA *ptr_ctrlpara);

static float cal_throttle_control(float command_throttle,float current_throttle, float change_time);
static float cal_rudder_control(float command_heading,float current_track_heading,struct T_PID pid);

static float convert_to_pwm(unsigned char min,unsigned char max,unsigned char input );
static float constrain_pwm(float input_pwm, float min_pwm, float max_pwm);

int control_loop(void)
{
    get_ctrlpara();
    get_ctrlinput();
    get_ctrloutput(&ctrloutput,&ctrlinput,&ctrlpara);

	return 0;
}

int execute_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput)
{
	float motor_left_pwm_out=0.0;
	float motor_right_pwm_out=0.0;

	ptr_ctrloutput->mmotor_onoff_pwm = constrain_pwm(ptr_ctrloutput->mmotor_onoff_pwm, 1000.0, 2000.0);
	ptr_ctrloutput->rudder_pwm = constrain_pwm(ptr_ctrloutput->rudder_pwm, 1000.0, 2000.0);

	unsigned char turn_mode = global_bool_boatpilot.turn_mode;
    float rudder2motor=0.0;
    int delta_rudder=0;
	switch(turn_mode)
	{
	case TURN_MODE_DIFFSPD:
        /*
         * 增加差速，在油门的基础上，添加舵效，分为5个档位
         */
        motor_left_pwm_out  = ptr_ctrloutput->mmotor_onoff_pwm;
        motor_right_pwm_out = ptr_ctrloutput->mmotor_onoff_pwm;

        rudder2motor = ptr_ctrloutput->rudder_pwm-1500;
        //DEBUG_PRINTF("ptr_ctrloutput->rudder_pwm = %f \n",ptr_ctrloutput->rudder_pwm);
        if(rudder2motor > 50)
        {
            //右舵
            delta_rudder        =   (int)rudder2motor;
            delta_rudder        =   fabs(delta_rudder);
            motor_left_pwm_out  =   motor_left_pwm_out  + (delta_rudder / 100) * 100;
            motor_right_pwm_out =   motor_right_pwm_out - (delta_rudder / 100) * 100;
        }
        else if(rudder2motor < -50)
        {
            //左舵
            delta_rudder        =   (int)rudder2motor;
            delta_rudder        =   fabs(delta_rudder);
            motor_left_pwm_out  =   motor_left_pwm_out  - (delta_rudder / 100) * 100;
            motor_right_pwm_out =   motor_right_pwm_out + (delta_rudder / 100) * 100;
        }
        motor_left_pwm_out  = constrain_pwm(motor_left_pwm_out,  1000.0, 2000.0);
        motor_right_pwm_out = constrain_pwm(motor_right_pwm_out, 1000.0, 2000.0);

        /*
         * 最终输出左右推进器
         */
        //DEBUG_PRINTF("motor_left_pwm_out = %f, motor_right_pwm_out = %f \n", motor_left_pwm_out, motor_right_pwm_out);
        set_throttle_left_right(motor_left_pwm_out, motor_right_pwm_out, DEFAULT_DEVICE_NUM);


        global_bool_boatpilot.motor_left  = motor_left_pwm_out;
        global_bool_boatpilot.motor_right = motor_right_pwm_out;
	    break;
	case TURN_MODE_RUDDER:
	    break;
	default:
	    break;
	}

	return 0;
}

/*
 * Function:     get_ctrlpara
 * Description:  get parameters from ground control station
 *               all parameters are unsigned char type
 */
static int get_ctrlpara()
{
	ctrlpara.rudder_p = (float)gcs2ap_all_udp.rud_p * 0.1;
	ctrlpara.rudder_i = (float)gcs2ap_all_udp.rud_i * 0.01;
	ctrlpara.rudder_d = (float)gcs2ap_all_udp.rud_d * 0.1;

	ctrlpara.cruise_throttle=gcs2ap_all_udp.cruise_throttle_percent;

	ctrlpara.mmotor_on_pos=gcs2ap_all_udp.mmotor_on_pos;
	ctrlpara.mmotor_off_pos=gcs2ap_all_udp.mmotor_off_pos;
	ctrlpara.rudder_left_pos=gcs2ap_all_udp.rudder_left_pos;
	ctrlpara.rudder_right_pos=gcs2ap_all_udp.rudder_right_pos;
	ctrlpara.rudder_mid_pos=gcs2ap_all_udp.rudder_mid_pos;

	/*
	 * 如果油门通道的上限值比下限值大了 就颠倒过来
	 */
	int temp;
    if(ctrlpara.mmotor_off_pos>ctrlpara.mmotor_on_pos)
    {
        temp=ctrlpara.mmotor_on_pos;
        ctrlpara.mmotor_on_pos=ctrlpara.mmotor_off_pos;
        ctrlpara.mmotor_off_pos=temp;
    }

    ctrlpara.work_mode = gcs2ap_all_udp.workmode;
    //ctrlpara.throttle_change_time = gcs2ap_all_udp.throttle_change_time;
    ctrlpara.throttle_change_time = 5;

	return 0;
}

/*
 * Function:       get_ctrlinput
 * Description:    把从地面站传输过来的手控方向舵和油门量（unsigned char数值）转换为标准的1000-2000（浮点数值）
 *                        获取目标航向和当前航迹的方向（gps的航向）
 */
static int get_ctrlinput()
{
	ctrlinput.rudder_pwm = convert_to_pwm(ctrlpara.rudder_left_pos,ctrlpara.rudder_right_pos,gcs2ap_all_udp.cmd.rudder);
	ctrlinput.mmotor_onoff_pwm = convert_to_pwm(ctrlpara.mmotor_off_pos,ctrlpara.mmotor_on_pos,gcs2ap_all_udp.cmd.throttle);

    /*
     * 1. 获取期望航迹角course angle 或者 期望航向角heading angle
     */
    //ctrlinput.command_course_angle_radian = auto_navigation.command_course_angle_radian;
	ctrlinput.command_course_angle_radian = auto_navigation.out_command_course_radian;
    //ctrlinput.command_heading_angle_radian = auto_navigation.command_heading_angle_radian;

    /*
     * 2. 获取当前实际的航迹角course angle 或者 当前实际的航向角heading
     */
    ctrlinput.gps_course_angle_radian = auto_navigation.gps_course_angle_radian;//单位弧度

    return 0;
}

static int get_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput,struct CTRL_INPUT *ptr_ctrlinput,struct CTRL_PARA *ptr_ctrlpara)
{
    struct T_PID pid;

    static unsigned char final_manual_throttle_before_auto = 0;
    float command_throttle;

    switch(ptr_ctrlpara->work_mode)
    {
    case STOP_MODE:
        //推进器停止，方向舵停止
        set_motor_off();
        ptr_ctrloutput->mmotor_onoff_pwm =1000;
        ptr_ctrloutput->rudder_pwm =1500;
        break;
    case RC_MODE:
        //printf("进入手动驾驶\n");
        final_manual_throttle_before_auto = ptr_ctrlinput->mmotor_onoff_pwm;
        ptr_ctrloutput->mmotor_onoff_pwm = ptr_ctrlinput->mmotor_onoff_pwm;
        ptr_ctrloutput->rudder_pwm = ptr_ctrlinput->rudder_pwm;
        break;
    case RTL_MODE:
        //break;
    case AUTO_MODE:

        /*1. 计算方向舵输出*/
        pid.p = ctrlpara.rudder_p;
        pid.i = ctrlpara.rudder_i;
        pid.d = ctrlpara.rudder_d;
		ptr_ctrloutput->rudder_pwm = cal_rudder_control(ptr_ctrlinput->command_course_angle_radian,\
                                                        ptr_ctrlinput->gps_course_angle_radian,\
                                                        pid);

        /*
         * 2. 计算油门量输出
         * 如果巡航速度油门参数不等于0，那么就将油门设置为巡航油门
         * 如果巡航油门等于0了，那么将油门设置为 手动切到自动时瞬间的油门
         */
        if (ptr_ctrlpara->cruise_throttle!=0)
        {
            /*cruise_throttle是百分比*/
            command_throttle=1000.0+1000*(float)(ptr_ctrlpara->cruise_throttle)*0.01;
            /*控制油门的改变速率*/
            //ptr_ctrloutput->mmotor_onoff_pwm = cal_throttle_control(command_throttle,ptr_ctrloutput->mmotor_onoff_pwm);
            ptr_ctrloutput->mmotor_onoff_pwm = cal_throttle_control(command_throttle, ptr_ctrloutput->mmotor_onoff_pwm, ptr_ctrlpara->throttle_change_time);
        }
        else
        {
            ptr_ctrloutput->mmotor_onoff_pwm = final_manual_throttle_before_auto;
        }
        break;
    default:
        break;
    }

    /*
     *这里进行软件的输出限制
     */
    if (ptr_ctrloutput->rudder_pwm < convert_to_pwm(ptr_ctrlpara->rudder_left_pos,ptr_ctrlpara->rudder_right_pos,ptr_ctrlpara->rudder_left_pos))
    {
        ptr_ctrloutput->rudder_pwm = 1000.0;
    }
    else if (ptr_ctrloutput->rudder_pwm > convert_to_pwm(ptr_ctrlpara->rudder_left_pos,ptr_ctrlpara->rudder_right_pos,ptr_ctrlpara->rudder_right_pos))
    {
        ptr_ctrloutput->rudder_pwm = 2000.0;
    }

    ptr_ctrloutput->mmotor_onoff_pwm    = constrain_pwm(ptr_ctrloutput->mmotor_onoff_pwm,  1000.0, 2000.0);
    ptr_ctrloutput->rudder_pwm          = constrain_pwm(ptr_ctrloutput->rudder_pwm,        1000.0, 2000.0);
    ptr_ctrloutput->mmotor_fwdbwd_pwm   = constrain_pwm(ptr_ctrloutput->mmotor_fwdbwd_pwm, 1000.0, 2000.0);

    return 0;
}

/*
 * Function:       cal_rudder_control
 * Description:  先把error_head_track这个目标航向和实际航向的误差，化为-1--+1
 *                        然后get_pid函数，但是这个get_pid函数，仍然需要保证范围在-1--+1
 *                        最后把get_pid之后的rudder_ctrl，映射到1000-2000或者100-200，这个是针对舵机的pwm值
 *                        此函数目前输出1000-2000
 */
static float cal_rudder_control(float command_heading,float current_track_heading,struct T_PID pid)
{
	float rudder_ctrl = 0.0;
	float error_head_track = 0.0;
	float full_rudder_threshold=0.5;//180度的一半时，也就是90度，满舵
	static BIT_PID pid_yaw;

	error_head_track = command_heading - current_track_heading;

	/*
	 * 因为   command_heading范围为-pi--+pi
	 * current_track_heading范围为-pi--+pi
	 * 二者误差范围为-2*pi--+2*pi
	 * 所以需要改为-pi--+pi
	 * wrap_PI这个函数非常重要，保证无论如何都是从小于180度的方向转舵，转小圈
	 */
	error_head_track = wrap_PI(error_head_track);

	/*再由-pi--+pi转化为-1--+1*/
	error_head_track=error_head_track * M_1_PI; /*M_1_PI = 1/pi*/

	pid_yaw.set_kP(pid.p);
	pid_yaw.set_kI(pid.i);
	pid_yaw.set_kD(pid.d);
	rudder_ctrl = pid_yaw.get_pid(error_head_track, 20, 1);// 这个是PID控制器的
	//printf("rudder_ctrl PID = %f \n",rudder_ctrl);
	//rudder_ctrl = boat.pid_yaw.get_pid_finite(error_head_track, 20, 1);// 这个是有限时间控制器的
	//printf("rudder_ctrl Finite-time = %f \n",rudder_ctrl);

	if(rudder_ctrl>full_rudder_threshold)
	{
		rudder_ctrl=1.0;//右满舵 也就是pwm给为2000
	}
	else if(rudder_ctrl<-full_rudder_threshold)
	{
		rudder_ctrl=-1.0;//左满舵 也就是pwm给为1000
	}
	//printf("pid之后的rudder_ctrl 归一化之后=%f\n",rudder_ctrl);//20170508已测试

	/*
	 * 由-1--+1转化为1000--2000
	 * x=rudder_ctrl
	 * (x-(-1)) / (1-(-1)) = (y-1000) / (2000-1000)
	 * (x-(-1)) / (2) = (y-1000) / (1000)
	 */
	rudder_ctrl=500*(rudder_ctrl+1)+1000;

	return rudder_ctrl;
}

static float cal_throttle_control(float command_throttle,float current_throttle, float change_time)
{
	static float current_time=0.0;
	static float last_time=0.0;

	current_time=clock_gettime_s();

	/*
	 * 1秒钟油门变化百分之1
	 * 这个10秒还是1秒还挺重要的，因为貌似这个模拟量板子反应速度不够呀
	 * 如果是1秒钟改变百分之1，发现485总线总是接收超时，强制发送，也就是
	 * 模拟量板子在电压发生变化时，485传输数据会收到电平的影响，所以必须等待
	 * 模拟量输出电压稳定后才能再次更改需要发送的电压，至少2秒，我这里暂时用10秒钟，其实也够
	 * 而这个如果485设备出现强制发送就会干扰切换器和充电机，这说明充电机和切换器的健壮性还是不够，仍需要更改
	 */
	if( (current_time-last_time) > change_time )
	{
		if(current_throttle<(command_throttle-50))
		{
			current_throttle+=100;
			last_time=current_time;
		}
		else if(current_throttle>(command_throttle+50))
		{
			current_throttle-=100;
			last_time=current_time;
		}
		else
		{
			current_throttle+=0;
			last_time=current_time;
		}
	}
	else
	{
		current_throttle+=0;
	}

	return current_throttle;
}

/*
 * Function:       conver_to_pwm
 * Description:  将min和max之间的input转化为1000到2000的pwm值
 *                        也就是说目前使用的遥控器最小的值为9最大值为252，并不是对应的0-255
 *                        所以9以下的数值相当于浪费了，因此我们把9作为最小值，252作为最大值
 *                        相当于遥控器的输入其实就是9-252，然后对应了1000-2000
 */
static float convert_to_pwm(unsigned char min,unsigned char max,unsigned char input )
{
    unsigned int min_value;
    unsigned int max_value;
    unsigned int input_value;
    unsigned int temp;
    float ret;

    /*
     * 因为float和duoble总是有符号类型的，那么unsigned char 的130很有可能就转换成负数了
     * 为了确保转换成float型时不会出现负数，所以先转换为unsigned int型
     */
    min_value=(unsigned int)min;
    max_value=(unsigned int)max;
    input_value=(unsigned int)input;

    if(max_value<min_value)
    {
        temp=min_value;
        min_value=max_value;
        max_value=temp;
    }

    if(input_value>=max_value)
    {
        input_value=max_value;
    }
    else if(input_value<=min_value)
    {
        input_value=min_value;
    }

    /*
     * (x-a)/(b-a)=(y-1000)/(2000-1000)
     */
    if((max_value-min_value) !=0)
    {
        /*一定要注意除数为0的情况，否则就会出现not a number*/
        ret=(float)(input_value-min_value)*1000.0/(float)(max_value-min_value)+1000.0;
    }
    else
    {
        ret=(float)(input_value-min_value)*1000.0/255.0+1000.0;
    }

    return ret;
}

static float constrain_pwm(float input_pwm, float min_pwm, float max_pwm)
{
	if( input_pwm < min_pwm)
		return min_pwm;
	else if( input_pwm > max_pwm)
		return max_pwm;
	else
		return input_pwm;
}
