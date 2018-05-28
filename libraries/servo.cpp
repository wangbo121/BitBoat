/*
 * servo.c
 *
 *  Created on: Jul 16, 2016
 *      Author: wangbo
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "utility.h"
#include "II2C.h"

#include "servo.h"

/*目前油门最大电压为5伏特*/
#define MAX_THROTTLE 5

int set_rudder(float pwm, int rudder_num)
{
#if 0
    float pwm_normalize=0.0;
    float dead_zone=0.0;

    dead_zone=0.022*gcs2ap_radio_all.rudder_dead_zone_angle_degree;

    //printf("set servor pwm:%f\n",pwm);
    if(pwm<1000.0)
    {
        pwm=1000.0;
    }
    else if(pwm>2000.0)
    {
        pwm=2000.0;
    }

    /*pwm within 1000-2000 to -1.0--+1.0*/
    pwm_normalize=((float)pwm-1500)/500;/*-1.0--+1.0*/

    /*默认的方向舵通道数*/
    switch(rudder_num)
    {
    case DEFAULT_RUDDER_NUM:
        /*0xff00 继电器闭合状态 0x0000继电器断开状态*/

        set_switch.switch4_state=0x0000;

        /*如果读取的码盘的刻度大于计算输出的方向舵，那么就往左打舵
         * 如果读取的小于计算的值，则右舵
         * pwm值范围改为-1--+1,pwm其实对应的是期望的输出舵角
         * 码盘的值范围也改为-1--+1
         * */
        if(read_encoder.position_normalize > pwm_normalize + dead_zone)
        {
            //当前的舵面角度大于了期望的pwm舵面角度，所以要打左舵
            /*switch2_state左舵闭合*/
            set_switch.switch2_state=0xff00;
            /*switch3_state右舵断开*/
            set_switch.switch3_state=0x0000;
        }
        else if(read_encoder.position_normalize < pwm_normalize - dead_zone)
        {
            //当前的舵面角度小于了期望的pwm舵面角度，所以要打右舵
            set_switch.switch2_state=0x0000;
            set_switch.switch3_state=0xff00;
        }
        else
        {
            //停止 stop rudder
            set_switch.switch2_state=0x0000;
            set_switch.switch3_state=0x0000;
        }

        break;
    case SPARE_RUDDER_NUM:
        /*0xff00 继电器闭合状态 0x0000继电器断开状态*/

        set_switch.switch4_state=0xff00;

        if(read_encoder.position_normalize > pwm_normalize + dead_zone)
        {
            /*switch0_state左舵闭合*/
            set_switch.switch0_state=0xff00;
            /*switch3_state右舵断开*/
            set_switch.switch1_state=0x0000;

        }
        else if(read_encoder.position_normalize < pwm_normalize - dead_zone)
        {
            set_switch.switch0_state=0x0000;
            set_switch.switch1_state=0xff00;

        }
        else
        {
            set_switch.switch0_state=0x0000;
            set_switch.switch1_state=0x0000;
        }

        break;
    default:

        break;
    }
    global_bool_modbus.send_request_switch_cnt++;//请求发送继电器指令
#endif
    return 0;
}

int set_rudder_off()
{
#if 0
    //停止 stop rudder
    set_switch.switch2_state=0x0000;
    set_switch.switch3_state=0x0000;

    global_bool_modbus.send_request_switch_cnt++;//请求发送继电器指令
#endif
    return 0;
}

int set_throttle_left_right(float pwm_left, float pwm_right, int device_num)
{
    float pwm_left_normalize=0.0;
    float pwm_right_normalize=0.0;

    if(pwm_left < 1000.0)
    {
        pwm_left = 1000.0;
    }
    else if(pwm_left > 2000.0)
    {
        pwm_left = 2000.0;
    }

    if(pwm_right < 1000.0)
    {
        pwm_right = 1000.0;
    }
    else if(pwm_right > 2000.0)
    {
        pwm_right = 2000.0;
    }

    /*pwm within 1000-2000 to 0-1000 to 0--+1*/
    pwm_left_normalize=((float)pwm_left-1000)/1000;
    pwm_right_normalize=((float)pwm_right-1000)/1000;

    float voltage;
    switch(device_num)
    {
    case DEFAULT_DEVICE_NUM:
        voltage = pwm_left_normalize * MAX_THROTTLE;
        DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_0, voltage); // left motor

        voltage = pwm_right_normalize * MAX_THROTTLE;
        DAC7574_DA(DAC7574_DA1_ADDR, DA_CHANNEL_1, voltage); // right motor
        break;
    case SPARE_DEVICE_NUM:
        break;
    default:
        break;
    }

    return 0;
}

int set_motor_forward()
{
#if 0
    set_switch.switch9_state=0x0000;//前进
    set_switch.switch13_state=0x0000;//前进
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_motor_backward()
{
#if 0
    set_switch.switch9_state=0xff00;//后退
    set_switch.switch13_state=0xff00;//后退
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_motor_on()
{
#if 0
    set_switch.switch8_state=0xff00;
    set_switch.switch12_state=0xff00;
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_motor_off()
{
#if 0
    set_switch.switch8_state=0x0000;
    set_switch.switch12_state=0x0000;
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_left_motor_forward()
{
#if 0
    set_switch.switch9_state=0x0000;//前进
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_left_motor_backward()
{
#if 0
    set_switch.switch9_state=0xff00;//后退
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_right_motor_forward()
{
#if 0
    set_switch.switch13_state=0x0000;//前进
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_right_motor_backward()
{
#if 0
    set_switch.switch13_state=0xff00;//后退
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_left_motor_on()
{
#if 0
    set_switch.switch8_state=0xff00;
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_left_motor_off()
{
#if 0
    set_switch.switch8_state=0x0000;
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_right_motor_on()
{
#if 0
    set_switch.switch12_state=0xff00;
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int set_right_motor_off()
{
#if 0
    set_switch.switch12_state=0x0000;
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}

int start_third_small_motor()
{
#if 0
    //打开第三个小电机
    //printf("打开第三个小电机\n");
    set_switch.switch6_state=0xff00;
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}
int stop_third_small_motor()
{
#if 0
    //printf("关闭第三个小电机\n");
    set_switch.switch6_state=0x0000;
    //write_set_all_switch(&set_switch);
    global_bool_modbus.send_request_switch_cnt++;
#endif
    return 0;
}



