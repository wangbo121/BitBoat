/*
 * gps_XW5651.cpp
 *
 *  Created on: Jun 3, 2018
 *      Author: wangbo
 */


#include "gps_XW5651.h"

//static void XW5651_GPFPD_analysis(nmea_msg *gpsx, unsigned char *buf);
static void XW5651_GPFPD_analysis(void *gpsx, unsigned char *buf);



//void XW5651_GPFPD_analysis(nmea_msg *gpsx, unsigned char *buf)
void XW5651_GPFPD_analysis(void *gpsx, unsigned char *buf)
{
#ifdef XW_GPFPD_GPS
    unsigned char *p;
    unsigned char dx;
    unsigned char posx;
    unsigned int temp;

    /*计算speed*/
    float speed_east;
    float speed_north;
    float speed_u;/*向天的速度*/
    float speed_value;

    if ((p = (unsigned char*)strstr((const char *)buf, "GPFPD")) != NULL)
    {
        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)
        {
            /*
             * gps方向，这里应该不是航迹方向，是船尾与船头连线对正北的夹角，
             * 范围0--360，转换为整型扩大100倍
             */
            temp = nmea_str2num(p + posx, &dx);
            //gpsx->direction=convert_to_require(temp,dx,2);
            gpsx->yaw=convert_to_require(temp,dx,2);
        }
        /*20170410把yaw转为-180-180度*/
        if(gpsx->yaw>18000)
        {
            gpsx->yaw=gpsx->yaw-36000;
        }

        posx = nmea_comma_pos(p, 4);
        if (posx != 0xff)
        {
            /*俯仰*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->pitch=convert_to_require(temp,dx,2);
        }

        posx = nmea_comma_pos(p, 5);
        if (posx != 0xff)
        {
            /*滚转*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->roll=convert_to_require(temp,dx,2);
        }

        posx = nmea_comma_pos(p, 6);
        if (posx != 0xff)
        {
            /*纬度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->latitude=convert_to_require(temp,dx,5);
        }

        posx = nmea_comma_pos(p, 7);
        if (posx != 0xff)
        {
            /*经度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->longitude=convert_to_require(temp,dx,5);
        }

        posx = nmea_comma_pos(p, 8);
        if (posx != 0xff)
        {
            /*高度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->altitude=convert_to_require(temp,dx,2);
        }

        posx = nmea_comma_pos(p, 9);
        if (posx != 0xff)
        {
            /*东向速度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->velocity_east=convert_to_require(temp,dx,3);
        }

        posx = nmea_comma_pos(p, 10);
        if (posx != 0xff)
        {
            /*北向速度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->velocity_north=convert_to_require(temp,dx,3);
        }

        posx = nmea_comma_pos(p, 11);
        if (posx != 0xff)
        {
            /*天向速度*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->velocity_u=convert_to_require(temp,dx,3);
        }

        posx = nmea_comma_pos(p, 13);
        if (posx != 0xff)
        {   /*卫星数目1*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->posslnum1=convert_to_require(temp,dx,0);
        }

        posx = nmea_comma_pos(p, 14);
        if (posx != 0xff)
        {   /*卫星数目2*/
            temp = nmea_str2num(p + posx, &dx);
            gpsx->posslnum2=convert_to_require(temp,dx,0);
        }

        speed_east=gpsx->velocity_east;
        speed_north=gpsx->velocity_north;
        speed_u=gpsx->velocity_u;
        speed_value=sqrt(powf(speed_east*(1e-3),2) + powf(speed_north*(1e-3),2) +powf(speed_u*(1e-3),2));//这里的单位是m/s
        gpsx->velocity=(unsigned int)(speed_value*100);/*速度又扩大了100倍*/

        gpsx->course_radian = atan2(speed_east,speed_north);//[弧度]
        //printf("speed_east=%f,speed_north=%f,gpsx->course=%f",speed_east,speed_north,gpsx->course);
    }
#endif
}










