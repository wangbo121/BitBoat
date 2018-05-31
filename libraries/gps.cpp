/*
 * gps.c
 *
 *  Created on: 2016年5月10日
 *      Author: wangbo
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "global.h"
#include "uart.h"

#include "gps.h"

#define GPS_DATA_NMEA_LEN 1024
//#define GPS_DATA_NMEA_LEN 256

/*extern variable*/
nmea_msg gps_data;

nmea_msg gps_data_nmea;

/*static variable*/
static nmea_msg gpsx;

static void gps_analysis(nmea_msg *gpsx, unsigned char *buf);

static int convert_to_require(int x,int float_num,int require_num);

static struct T_UART_DEVICE uart_device_gps;

int gps_uart_init()
{
    uart_device_gps.uart_name = (char *)UART_GPS;

    uart_device_gps.baudrate=UART_GPS_BAUD;
    uart_device_gps.databits=UART_GPS_DATABITS;
    uart_device_gps.parity=UART_GPS_PARITY;
    uart_device_gps.stopbits=UART_GPS_STOPBITS;

    uart_device_gps.uart_num=open_uart_dev(uart_device_gps.uart_name);

    //uart_device_gps.ptr_fun=read_gps_data;

    set_uart_opt( uart_device_gps.uart_name, \
                  uart_device_gps.baudrate,\
                  uart_device_gps.databits,\
                  uart_device_gps.parity,\
                  uart_device_gps.stopbits);
//    之前串口接收都是创建线程接收的，后来改了，但是其实用线程接收很好，只是我还没弄明白怎么设置线程优先级
//    create_uart_pthread(&uart_device_gps);

    return 0;
}

int read_gps_data(unsigned char *buf, unsigned int len)
{

#if 1
	char buf_temp[GPS_DATA_NMEA_LEN + 1];
	memcpy(buf_temp, buf, len);
	buf_temp[len+1]='\0';
	printf("GPS收到的数据：%s\n", buf_temp);
#endif


	gps_analysis(&gpsx, (unsigned char*)buf);

	memcpy(&gps_data_nmea, &gpsx, sizeof(gpsx));


	/*
	 * 这里加个判断，如果速度太大，就认为是坏点，不赋值给gps_data
	 * 如果经纬度小于50度，也不赋值
	 */
	if(gpsx.speed<10000 && gpsx.longitude>5000000)
	{
	    /*
	     * 只有速度在小于100米每秒的情况下才赋值，声速是340米每秒
	     */
	    //memcpy(&gps_data, &gpsx, sizeof(gpsx));
	    //memcpy(&gps_data_nmea, &gpsx, sizeof(gpsx));
	}

#ifdef NMEA_GPS
	/*
	 * 这里是因为nmea的经纬度是度分秒的格式，所以需要转换为单位是[度]的格式
	 */
	int tmp_d,tmp_m;

	tmp_d = ((int)(gps_data.latitude * 0.000001)) * 1000000;
	tmp_m = gps_data.latitude - tmp_d;
	gps_data.latitude = (int)(tmp_d + tmp_m*1.66666667);
	tmp_d = ((int)(gps_data.longitude * 0.000001)) * 1000000;
	tmp_m = gps_data.longitude - tmp_d;
	gps_data.longitude = (int)(tmp_d + tmp_m*1.66666667);
#endif

	return 0;
}

int send_gps_data_NMEA()
{
    char send_buf[7]="wangbo";
    send_uart_data(uart_device_gps.uart_name, send_buf, sizeof(send_buf));



}


int read_gps_data_NMEA()
{
    static char recv_buf[GPS_DATA_NMEA_LEN];
    static int recv_buf_require_len = GPS_DATA_NMEA_LEN;// 请求接收的字节数目
    static int recv_buf_real_len = 0; //实际接收的字节数目
    static int max_wait_ms = 1000;//最大等待时间ms

    recv_buf_real_len = read_uart_data(uart_device_gps.uart_name, recv_buf, max_wait_ms, recv_buf_require_len);
    DEBUG_PRINTF("read_gps_data_NMEA    :    recv_buf_real_len = %d \n", recv_buf_real_len);

    if( recv_buf_real_len > 0)
    {
        DEBUG_PRINTF("read_gps_data_NMEA    :    receive %d bytes\n",recv_buf_real_len);
        read_gps_data((unsigned char*)recv_buf, recv_buf_real_len);
    }

    return 0;
}


int gps_uart_close()
{
    uart_device_gps.uart_name = (char *)UART_GPS;
    close_uart_dev(uart_device_gps.uart_name);

	return 0;
}

static unsigned char nmea_comma_pos(unsigned char *buf, unsigned char cx)
{
	unsigned char *p = buf;
	while (cx)
	{
		if (*buf == '*' || *buf<' ' || *buf>'z') return 0xff;
		if (*buf == ',') cx--;
		buf++;
	}
	return buf - p;
}

static unsigned int nmea_pow(unsigned char m, unsigned char n)
{
    unsigned int result = 1;
    while (n--) result *= m;
    return result;
}

/*
 * convert 39.12345 to 3912345
 * convert 39.123456 to 3912345
 * as for result, there is no more than five number after point
 * *dx : *float_num 保存了小数点后的位数是多少
 */
static int nmea_str2num(unsigned char *buf, unsigned char*dx)
{
    unsigned char *p = buf;
    unsigned int ires = 0;
    unsigned int fres = 0;
    unsigned char ilen = 0, flen = 0;
    unsigned char i;
    unsigned char mask = 0;
    int res;
    while (1)
    {
        if (*p == '-'){ mask |= 0x02; p++; }
        if (*p == ',' || (*p == '*'))break;
        if (*p == '.'){ mask |= 0x01; p++; }
        else if (*p>'9' || (*p<'0'))
        {
            ilen = 0;
            flen = 0;
            break;
        }
        if (mask & 0x01) flen++;
        else ilen++;
        p++;
    }

    if (mask & 0x02) buf++;
    for (i = 0; i<ilen; i++)
    {
        ires += nmea_pow(10, ilen - 1 - i)*(buf[i] - '0');
    }
    if (flen>5)flen = 5;
    *dx = flen;
    for (i = 0; i<flen; i++)
    {
        fres += nmea_pow(10, flen - 1 - i)*(buf[ilen + 1 + i] - '0');
    }
    res = ires*nmea_pow(10, flen) + fres;
    if (mask & 0x02) res = -res;
    return res;
}

/*
 * 因为一开始是把所有的浮点数转换为整型的数，但是每个传感器设备的位数不一定，而我们又需要固定精度的数据
 * 所以要把放大多了的和放大少了的转换为刚好放的精度
 * 比如实际纬度为39.123456789，但是如果转换为整型为39123456789，但是我们其实想要的是3912345，所以放大多了
 * 因此需要把剩下的去掉，那么就需要除以10^(9-5)=10^4
 * 而如果实际为39.12345，但是有的设备是39.123，那么就放大的小了，所以需要放到的3912300，需要乘以10^(5-3)=10^2
 */
static int convert_to_require(int x,int float_num,int require_num)
{
	if(float_num>=0 && float_num<require_num)
	{
		//return x*(10^(require_num-float_num));
	    return x*nmea_pow(10,(require_num-float_num));
	}
	else if(float_num>require_num)
	{
		return x/nmea_pow(10,(float_num-require_num));
	}
	else
	{
		return x;
	}
}

static void XW5651_GPFPD_analysis(nmea_msg *gpsx, unsigned char *buf)
{
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
		gpsx->speed=(unsigned int)(speed_value*100);/*速度又扩大了100倍*/

		gpsx->course=atan2(speed_east,speed_north);//[弧度]
		//printf("speed_east=%f,speed_north=%f,gpsx->course=%f",speed_east,speed_north,gpsx->course);
	}
}



#ifdef NMEA_GPS
static void nmea_GPGSV_analysis(nmea_msg *gpsx, unsigned char *buf);
static void nmea_GPGGA_analysis(nmea_msg *gpsx, unsigned char *buf);
static void nmea_GPRMC_analysis(nmea_msg *gpsx, unsigned char *buf);
static void nmea_GPVTG_analysis(nmea_msg *gpsx, unsigned char *buf);

static void nmea_GPGSV_analysis(nmea_msg *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    if ((p = (unsigned char*)strstr((const char *)buf, "$GPGSV")) != NULL)
    {
        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)gpsx->svnum = nmea_str2num(p + posx, &dx);
    }
}

static void nmea_GPGGA_analysis(nmea_msg *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    if ((p = (unsigned char*)strstr((const char *)buf, "$GPGGA")) != NULL)
    {
        posx = nmea_comma_pos(p, 6);
        if (posx != 0XFF)gpsx->gpssta = nmea_str2num(p + posx, &dx);
        posx = nmea_comma_pos(p, 7);
        if (posx != 0XFF)gpsx->posslnum = nmea_str2num(p + posx, &dx);
        posx = nmea_comma_pos(p, 9);
        if (posx != 0XFF)gpsx->altitude = nmea_str2num(p + posx, &dx);
    }
}

static void nmea_GPRMC_analysis(nmea_msg *gpsx, unsigned char *buf)
{
    unsigned char *p, dx;
    unsigned char posx;
    unsigned int temp;
    //if ((p = (unsigned char*)strstr((const char *)buf, "GPRMC")) != NULL)
    if ((p = (unsigned char*)strstr((const char *)buf, "GNRMC")) != NULL)
    {
        posx = nmea_comma_pos(p, 1);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx) / nmea_pow(10, dx);
            gpsx->utc.hour = temp / 10000;
            gpsx->utc.min = (temp / 100) % 100;
            gpsx->utc.sec = temp % 100;
        }

        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            gpsx->latitude = (int)((float)temp * 0.1+0.5);
            gpsx->latitude = (int)temp;
            gpsx->latitude = (int)3912345;
        }
        posx = nmea_comma_pos(p, 4);
        if (posx != 0xff)gpsx->nshemi = *(p + posx);

        posx = nmea_comma_pos(p, 5);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            gpsx->longitude = (int)((float)temp * 0.1+0.5);
            gpsx->longitude = (int)temp;
        }
        posx = nmea_comma_pos(p, 6);
        if (posx != 0xff)gpsx->ewhemi = *(p + posx);

        //posx = nmea_comma_pos(p, 7);
        //if (posx != 0xff)
        //{
        //  temp = nmea_str2num(p + posx, &dx);
        //  gpsx->speed = (int)((float)temp * 0.1 + 0.5);
        //}

        posx = nmea_comma_pos(p, 8);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            //gpsx->direction = (int)((float)temp * 0.1 + 0.5);
        }

        posx = nmea_comma_pos(p, 9);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            gpsx->utc.date = temp / 10000;
            gpsx->utc.month = (temp / 100) % 100;
            gpsx->utc.year = 2000 + temp % 100;
        }
    }
}

static void nmea_GPVTG_analysis(nmea_msg *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    if ((p = (unsigned char*)strstr((const char *)buf, "$GPVTG")) != NULL)
    {
        //posx = nmea_comma_pos(p, 7);
        posx = nmea_comma_pos(p, 5);
        if (posx != 0xff)
        {
            gpsx->speed = nmea_str2num(p + posx, &dx);
            if (dx<3)gpsx->speed *= nmea_pow(10, 3 - dx);
        }
    }
}
#endif


static void gps_analysis(nmea_msg *gpsx, unsigned char *buf)
{
#ifdef NMEA_GPS
    nmea_GPGSV_analysis(gpsx, buf);
    nmea_GPGGA_analysis(gpsx, buf);
    nmea_GPRMC_analysis(gpsx, buf);
    nmea_GPVTG_analysis(gpsx, buf);
#endif

#ifdef XW_GPFPD_GPS
    XW5651_GPFPD_analysis(gpsx, buf);
#endif
}
