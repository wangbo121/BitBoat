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

#include "boatlink.h"
#include "global.h"
#include "location.h"
#include "navigation.h"
#include "gps.h"
#include "pid.h"
#include "radio.h"
#include "servo.h"
//#include "modbus_rotary_encoder.h"
//#include "modbus_relay_switch.h"
//#include "modbus_analog_ipam4404.h"
//#include "utilityfunctions.h"
#include "utility.h"
#include "save_data.h"
#include "control.h"

#define MIDDLE_RUDDER_PWM 1500
#define MIDDLE_THROTTLE_PWM 1500
#define REVERSE_STEER 1
#define REVERSE_THROTTLE 2

#define TURN_MODE_RUDDER 0
#define TURN_MODE_DIFFSPD 1
#define TURN_MODE_MIX 2

#define CTRL_TEST 0x55

#define CTRL_STEER_TEST 1
#define CTRL_STEER_IN_CL 0

#define ALLOW_ERROR 5

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#define ON_LMT 2000
#define OFF_LMT 1000
#define FWD_LMT 2000
#define BWD_LMT 1000

/*
 * 限制到左35度，右35度
 * 35/45*500=111.1约等于110
 * 35/45*500+1000=1110
 * 2000-35/45*500=1890
 */
//#define LEFT_LMT 1110
//#define RIGHT_LMT 1890
#define LEFT_LMT 1000
#define RIGHT_LMT 2000

struct CTRL_PARA ctrlpara;
struct CTRL_INPUT ctrlinput;
struct CTRL_OUTPUT ctrloutput;

static int get_ctrlpara(struct CTRL_PARA *ptr_ctrlpara, struct GCS2AP_RADIO *ptr_gcs2ap_radio_all);
static int get_ctrlinput(struct CTRL_INPUT *ptr_ctrlinput, struct GCS2AP_RADIO *ptr_gcs2ap_radio_all,struct T_NAVIGATION * ptr_auto_navigation);
static int get_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput,struct CTRL_INPUT *ptr_ctrlinput,struct CTRL_PARA *ptr_ctrlpara);

static float cal_rudder_control(float command_heading,float current_track_heading,struct T_PID pid);
static float cal_throttle_control(float command_throttle,float current_throttle);

static float convert_to_pwm(unsigned char min,unsigned char max,unsigned char input );

int control_loop(void)
{
    get_ctrlpara(&ctrlpara, &gcs2ap_radio_all);
    get_ctrlinput(&ctrlinput, &gcs2ap_radio_all,&auto_navigation);
    get_ctrloutput(&ctrloutput,&ctrlinput,&ctrlpara);

	return 0;
}

/*
 * Function:     execute_ctrloutput
 * Description:  利用ctrlinput，navigation，ctrlpara，gpsdata的数据，
 *               control_loop 只是计算出控制的量，但是并不输出，execute_ctrloutput才是输出
 *               二者的频率可以相同，也可以不同
 */
int execute_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput)
{
	static int execute_cnt=0;

	float motor_left_pwm_out=0.0;
	float motor_right_pwm_out=0.0;

	if(ptr_ctrloutput->rudder_pwm < 1000.0)
	{
	    ptr_ctrloutput->rudder_pwm = 1000.0;
	}
	else if(ptr_ctrloutput->rudder_pwm > 2000.0)
	{
	    ptr_ctrloutput->rudder_pwm = 2000.0;
	}

	if(ptr_ctrloutput->mmotor_onoff_pwm < 1000.0)
    {
        ptr_ctrloutput->mmotor_onoff_pwm = 1000.0;
    }
    else if(ptr_ctrloutput->mmotor_onoff_pwm > 2000.0)
    {
        ptr_ctrloutput->mmotor_onoff_pwm = 2000.0;
    }

	/*
	 * 1. 最终输出方向舵
	 */
    if((global_bool_boatpilot.turn_mode==TURN_MODE_RUDDER) \
       || (global_bool_boatpilot.turn_mode==TURN_MODE_MIX))
    {
        set_rudder(ptr_ctrloutput->rudder_pwm,DEFAULT_RUDDER_NUM);
    }
    else
    {

    }

	/*
	 * 2个推进器分别输出，差速控制方向，也就是“以车代舵”
	 */
	//左电机加速，左边比右边快，从而往右边转
    //增加百分之1*发送过来的增量数
    motor_left_pwm_out=ptr_ctrloutput->mmotor_onoff_pwm;

    //右电机加速，右边比左边快，从而往左边转
    //增加百分之1*发送过来的增量数
    motor_right_pwm_out=ptr_ctrloutput->mmotor_onoff_pwm;

    int delta_percent_abs;//改变的百分比绝对值
    int delta_percent;
    int delta_max;//最大改变百分比

    delta_max=(int)gcs2ap_radio_all.diffspd_lim;

    delta_percent=(int)gcs2ap_radio_all.diffspd_coef-100;
    delta_percent_abs=abs(delta_percent);
    if(delta_percent_abs>delta_max)
    {
        delta_percent_abs=delta_max;
    }

    if(delta_percent < 0)
    {
        //左边减小右边增加
        motor_left_pwm_out=motor_left_pwm_out - delta_percent_abs*10;
        motor_right_pwm_out=motor_right_pwm_out + delta_percent_abs*10;
    }
    else if(delta_percent > 0)
    {
        //左边增加右边减小
        motor_left_pwm_out=motor_left_pwm_out + delta_percent_abs*10;
        motor_right_pwm_out=motor_right_pwm_out - delta_percent_abs*10;
    }
    else
    {

    }

    if(motor_left_pwm_out < 1000.0)
    {
        motor_left_pwm_out = 1000.0;
    }
    if(motor_left_pwm_out > 2000.0)
    {
        motor_left_pwm_out = 2000.0;
    }

    if(motor_right_pwm_out < 1000.0)
    {
        motor_right_pwm_out = 1000.0;
    }
    if(motor_right_pwm_out > 2000.0)
    {
        motor_right_pwm_out = 2000.0;
    }

    float rudder2motor=0.0;
    int delta_rudder=0;

    if((global_bool_boatpilot.turn_mode==TURN_MODE_DIFFSPD)\
       || (global_bool_boatpilot.turn_mode==TURN_MODE_MIX))
    {
        /*
         * 增加差速，在油门的基础上，添加舵效，分为5个档位
         */
        rudder2motor=ptr_ctrloutput->rudder_pwm-1500;

        if(rudder2motor>50)
        {
            //printf("右舵，左推进器增加\n");
            //右舵
            delta_rudder=(int)rudder2motor;
            delta_rudder=fabs(delta_rudder);
            motor_left_pwm_out=motor_left_pwm_out+(delta_rudder/100)*100;
            motor_right_pwm_out=motor_right_pwm_out-(delta_rudder/100)*100;
        }
        else if(rudder2motor<-50)
        {
            //printf("左舵，右推进器增加\n");
            //左舵
            delta_rudder=(int)rudder2motor;
            delta_rudder=fabs(delta_rudder);
            motor_left_pwm_out=motor_left_pwm_out-(delta_rudder/100)*100;
            motor_right_pwm_out=motor_right_pwm_out+(delta_rudder/100)*100;
        }

        if(global_bool_boatpilot.turn_mode==TURN_MODE_DIFFSPD)
        {
            //如果只是差速控制，那就把方向舵关掉
            set_rudder_off();
        }
    }
    else
    {

    }

    if(motor_left_pwm_out < 1000.0)
    {
        motor_left_pwm_out = 1000.0;
    }
    if(motor_left_pwm_out > 2000.0)
    {
        motor_left_pwm_out = 2000.0;
    }

    if(motor_right_pwm_out < 1000.0)
    {
        motor_right_pwm_out = 1000.0;
    }
    if(motor_right_pwm_out > 2000.0)
    {
        motor_right_pwm_out = 2000.0;
    }

    /*
     * 2. 最终输出左右推进器
     */
    set_throttle_left_right(motor_left_pwm_out,motor_right_pwm_out,DEFAULT_THROTTLE_NUM);

    global_bool_boatpilot.left_motor_voltage=(short)((motor_left_pwm_out-1000)/20);//(left_motor-1000)/1000*5*10
    global_bool_boatpilot.right_motor_voltage=(short)((motor_right_pwm_out-1000)/20);//(right_motor-1000)/1000*5*10

    execute_cnt++;

	return 0;
}

/*
 * Function:     get_ctrlpara
 * Description:  从地面站获取控制参数，所有的参数均为unsigned char型，使用时需要进行类型转换
 *
 */
static int get_ctrlpara(struct CTRL_PARA *ptr_ctrlpara, struct GCS2AP_RADIO *ptr_gcs2ap_radio_all)
{
	ptr_ctrlpara->rudder_p = ptr_gcs2ap_radio_all->rud_p;
	ptr_ctrlpara->rudder_i = ptr_gcs2ap_radio_all->rud_i;
	ptr_ctrlpara->rudder_d = ptr_gcs2ap_radio_all->rud_d;
	ptr_ctrlpara->cruise_throttle=ptr_gcs2ap_radio_all->cruise_throttle_percent;

	ptr_ctrlpara->mmotor_on_pos=ptr_gcs2ap_radio_all->mmotor_on_pos;
	ptr_ctrlpara->mmotor_off_pos=ptr_gcs2ap_radio_all->mmotor_off_pos;
	ptr_ctrlpara->rudder_left_pos=ptr_gcs2ap_radio_all->rudder_left_pos;
	ptr_ctrlpara->rudder_right_pos=ptr_gcs2ap_radio_all->rudder_right_pos;
	ptr_ctrlpara->rudder_mid_pos=ptr_gcs2ap_radio_all->rudder_mid_pos;

	int temp;
    if(ctrlpara.mmotor_off_pos>ctrlpara.mmotor_on_pos)
    {
        temp=ctrlpara.mmotor_on_pos;
        ctrlpara.mmotor_on_pos=ctrlpara.mmotor_off_pos;
        ctrlpara.mmotor_off_pos=temp;
    }

	return 0;
}

/*
 * Function:     get_ctrlinput
 * Description:  把从地面站传输过来的手控方向舵和油门量（unsigned char数值）转换为标准的1000-2000（浮点数值）
 *               获取目标航向和当前航迹的方向（gps的航向）
 */
static int get_ctrlinput(struct CTRL_INPUT *ptr_ctrlinput, struct GCS2AP_RADIO *ptr_gcs2ap_radio_all,struct T_NAVIGATION * ptr_auto_navigation)
{
    ptr_ctrlinput->rudder_reverse = ptr_gcs2ap_radio_all->rudder_setup_reverse;
    ptr_ctrlinput->throttle_reverse = ptr_gcs2ap_radio_all->thruster_setup_reverse;

    if (ptr_ctrlinput->rudder_reverse)
    {
        ptr_ctrlinput->rudder_pwm = convert_to_pwm(ctrlpara.rudder_left_pos,ctrlpara.rudder_right_pos,(ctrlpara.rudder_right_pos - ptr_gcs2ap_radio_all->rc_rudder));
    }
    else
    {
        /*这里先直接用*/
        ptr_ctrlinput->rudder_pwm = convert_to_pwm(ctrlpara.rudder_left_pos,ctrlpara.rudder_right_pos,ptr_gcs2ap_radio_all->rc_rudder);
    }

    if (ptr_ctrlinput->throttle_reverse)
    {
        ptr_ctrlinput->mmotor_onoff_pwm = convert_to_pwm(ctrlpara.mmotor_off_pos,ctrlpara.mmotor_on_pos,(ctrlpara.mmotor_on_pos - ptr_gcs2ap_radio_all->rc_thruster));
    }
    else
    {
        ptr_ctrlinput->mmotor_onoff_pwm = convert_to_pwm(ctrlpara.mmotor_off_pos,ctrlpara.mmotor_on_pos,ptr_gcs2ap_radio_all->rc_thruster);
    }

    /*
     * 1. 获取期望航迹角course angle 或者 期望航向角heading angle
     */
    ptr_ctrlinput->command_course_angle_radian = ptr_auto_navigation->command_course_angle_radian;
    ptr_ctrlinput->command_heading_angle_radian = ptr_auto_navigation->command_heading_angle_radian;

    /*
     * 2. 获取当前实际的航迹角course angle 或者 当前实际的航向角heading
     */
    if(gcs2ap_radio_all.navigation_mode==NAVIGATION_COURSE_ANGLE)
    {
        ptr_ctrlinput->gps_course_angle_radian = ptr_auto_navigation->gps_course_angle_radian;//单位弧度
    }
    else
    {
        ptr_ctrlinput->gps_heading_angle_radian = ptr_auto_navigation->gps_heading_angle_radian;
    }

    return 0;
}

static int get_ctrloutput(struct CTRL_OUTPUT *ptr_ctrloutput,struct CTRL_INPUT *ptr_ctrlinput,struct CTRL_PARA *ptr_ctrlpara)
{
    struct T_PID pid;

    static unsigned char final_manual_throttle_before_auto = 0;
    float command_throttle;

    switch(gcs2ap_radio_all.workmode)
    {
    case STOP_MODE:
        //推进器停止，方向舵停止
        set_left_motor_off();
        set_right_motor_off();

        ptr_ctrloutput->mmotor_onoff_pwm =1000;
        ptr_ctrloutput->rudder_pwm =1500;
        break;
    case RC_MODE:
        //printf("进入手动驾驶\n");
        final_manual_throttle_before_auto = ptr_ctrlinput->mmotor_onoff_pwm;

        ptr_ctrloutput->mmotor_onoff_pwm = ptr_ctrlinput->mmotor_onoff_pwm;
        ptr_ctrloutput->rudder_pwm = ptr_ctrlinput->rudder_pwm;
        break;
    case MIX_MODE:
        break;
    case RTL_MODE:
        //break;
    case AUTO_MODE:
        //printf("进入自动驾驶\n");
        /*1. 计算方向舵输出*/
        pid.p = ((float)ctrlpara.rudder_p)*0.1;
        pid.i = ((float)ctrlpara.rudder_i)*0.000039215;
        pid.d = ((float)ctrlpara.rudder_d)*0.1;
        if(gcs2ap_radio_all.navigation_mode==NAVIGATION_COURSE_ANGLE)
        {
            ptr_ctrloutput->rudder_pwm = cal_rudder_control(ptr_ctrlinput->command_course_angle_radian,\
                                                            ptr_ctrlinput->gps_course_angle_radian,\
                                                            pid);
        }
        else
        {
            /*在风平浪静的情况下，其实是可以用 船头船尾连线与正北方向的夹角heading angle作为前进速度的方向*/
            ptr_ctrloutput->rudder_pwm = cal_rudder_control(ptr_ctrlinput->command_heading_angle_radian,\
                                                            ptr_ctrlinput->gps_heading_angle_radian,\
                                                            pid);
        }

        /*
         * 2. 计算油门量输出
         * 计算自动驾驶时的，
         * 如果巡航速度油门参数不等于0，那么就将油门设置为巡航油门
         * 如果巡航油门等于0了，那么将油门设置为手动时到自动时瞬间的油门
         */
        if (ptr_ctrlpara->cruise_throttle!=0)
        {
            /*cruise_throttle是百分比*/
            command_throttle=1000.0+1000*(float)(ptr_ctrlpara->cruise_throttle)*0.01;
            /*控制油门的改变速率*/
            ptr_ctrloutput->mmotor_onoff_pwm = cal_throttle_control(command_throttle,ptr_ctrloutput->mmotor_onoff_pwm);
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
        /*
         * 假如输入的方向舵要求达到最左边，
         * 例如但是遥控器却只能输入到8，所以设置left_pos=9，那么当输入为8时，直接达到最左边
         */
        //ptr_ctrloutput->rudder_pwm = ptr_ctrlpara->rudder_left_pos;
        ptr_ctrloutput->rudder_pwm = 1000;
    }
    else if (ptr_ctrloutput->rudder_pwm > convert_to_pwm(ptr_ctrlpara->rudder_left_pos,ptr_ctrlpara->rudder_right_pos,ptr_ctrlpara->rudder_right_pos))
    {
        /*
         * 假如输入的方向舵要求达到最右边，
         * 例如但是遥控器却只能输入到233，所以设置right_pos=230，那么当输入为233时，直接达到最右边
         * 而不是按照0-255对应1000-2000的映射
         * 这样的话，在最左边和最右边的时候可能会有个突变
         * 方向舵在中间时可能没有影响，但是往右打时，遥控器没有到最右边但是，方向舵可能已经达到最右了，
         * 因为我们对应的1000-2000对应的是-45--45度，但是方向舵只能是-35--+35度
         */
        //ptr_ctrloutput->rudder_pwm = ptr_ctrlpara->rudder_right_pos;
        ptr_ctrloutput->rudder_pwm = 2000;
    }

    /*
     * 限制输出幅度，进行保护
     * 按道理说这里应该是可以设置的，不能简单的设置为宏定义
     */
    if (ptr_ctrloutput->mmotor_onoff_pwm<OFF_LMT)
    {
    ptr_ctrloutput->mmotor_onoff_pwm = OFF_LMT;
    }
    else if (ptr_ctrloutput->mmotor_onoff_pwm>ON_LMT)
    {
    ptr_ctrloutput->mmotor_onoff_pwm = ON_LMT;
    }

    if (ptr_ctrloutput->mmotor_fwdbwd_pwm < BWD_LMT)
    {
        ptr_ctrloutput->mmotor_fwdbwd_pwm = BWD_LMT;
    }
    else if (ptr_ctrloutput->mmotor_fwdbwd_pwm > FWD_LMT)
    {
        ptr_ctrloutput->mmotor_fwdbwd_pwm = FWD_LMT;
    }

    if (ptr_ctrloutput->rudder_pwm < LEFT_LMT)
    {
        ptr_ctrloutput->rudder_pwm = LEFT_LMT;
    }
    else if (ptr_ctrloutput->rudder_pwm > RIGHT_LMT)
    {
        ptr_ctrloutput->rudder_pwm = RIGHT_LMT;
    }

    return 0;
}

/*
 * Function:     cal_rudder_control
 * Description:  先把error_head_track这个目标航向和实际航向的误差，化为-1--+1
 *               然后get_pid函数，但是这个get_pid函数，仍然需要保证范围在-1--+1
 *               最后把get_pid之后的rudder_ctrl，映射到1000-2000或者100-200，这个是针对舵机的pwm值
 *               此函数目前输出1000-2000
 */
static float cal_rudder_control(float command_heading,float current_track_heading,struct T_PID pid)
{
	float rudder_ctrl = 0.0;
	float error_head_track = 0.0;
	float full_rudder_threshold=0.5;//180度的一半时，也就是90度，满舵

	//printf("command_heading=:%f\ncurrent_track_heading=:%f\n",convert_radian_to_degree(command_heading),convert_radian_to_degree(current_track_heading));

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
	error_head_track=error_head_track * M_PI_RECIPROCAL;/*M_PI_RECIPROCAL=1/pi*/

//	rudder_ctrl = -get_pid(error_head_track, 1.0, pid.p, pid.i, pid.d);
	rudder_ctrl = get_pid(error_head_track, 1.0, pid.p, pid.i, pid.d);//这个是淮南船厂测试用，没有问题

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

static float cal_throttle_control(float command_throttle,float current_throttle)
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
	if((current_time-last_time)>gcs2ap_radio_all.throttle_change_time)
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
 * Function:     conver_to_pwm
 * Description:  将min和max之间的input转化为1000到2000的pwm值
 *               也就是说目前使用的遥控器最小的值为9最大值为252，并不是对应的0-255
 *               所以9以下的数值相当于浪费了，因此我们把9作为最小值，252作为最大值
 *               相当于遥控器的输入其实就是9-252，然后对应了1000-2000
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
