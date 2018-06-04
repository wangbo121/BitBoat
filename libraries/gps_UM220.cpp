/*
 * gps_UM220.cpp
 *
 *  Created on: Jun 1, 2018
 *      Author: wangbo
 */


#include "gps_UM220.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>

#include "global.h"
#include "uart.h"

//#define GPS_DATA_LEN_UM220 1024   // UM220 每次发送的数据最大长度 其实有可能比1024少 这里表示串口每次读取时想要获取的数据字节个数
#define GPS_DATA_LEN_UM220 256   // UM220 每次发送的数据最大长度
/*
 * 我们要求读取1024个字节 但是不知道UM220什么时候能够发送够1024个字节来，不能一直傻等着，
 * 所以设置最大等待时间，如果在该时间内没有收够1024个字节，就不再继续等待
 */
#define GPS_DATA_WAIT_TIME_MS_UM220 3
#define Knot_2_Meter 0.51444// 速度转换 节 到 米每秒
#define DEG2RAD 0.017444

static struct T_UART_DEVICE uart_device_gps;


struct T_GPS_UM220 gps_data_UM220; // 当外部程序需要读取UM220设备时 从这个结构中获取数据
static struct T_GPS_UM220 gpsx;   // 静态变量 内部计算时使用

static int read_gps_data_UM220_GPS();
static int read_gps_data_UM220_BD();
static int read_gps_data_UM220_GPS_BD();

static int decode_data_gps_UM220_GPS(unsigned char *buf, int len);
static int decode_data_gps_UM220_BD(unsigned char *buf, int len);
static int decode_data_gps_UM220_GPS_BD(unsigned char *buf, int len);

/*
 * nmea_comma_pos返回的是 逗号 的下一个字节的地址 与最初字符串的地址的差
 */
static unsigned char nmea_comma_pos(unsigned char *buf, unsigned char cx);
static int64_t convert_to_require(int64_t x, unsigned char float_num, unsigned char require_num);
static int64_t nmea_pow(unsigned char m, unsigned char n);
static int64_t nmea_str2num(unsigned char *buf, unsigned char*dx);

static void nmea_GPGSV_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GPGGA_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GPRMC_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GPVTG_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);


static void nmea_GNGSV_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GNGGA_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GNRMC_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);
static void nmea_GNVTG_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf);


int gps_uart_init_UM220()
{
    uart_device_gps.uart_name = (char *)UART_GPS_UM220;

    uart_device_gps.baudrate        = UART_GPS_BAUD_UM220;
    uart_device_gps.databits        = UART_GPS_DATABITS_UM220;
    uart_device_gps.parity          = UART_GPS_PARITY_UM220;
    uart_device_gps.stopbits        = UART_GPS_STOPBITS_UM220;

    uart_device_gps.uart_num=open_uart_dev(uart_device_gps.uart_name);

    set_uart_opt( uart_device_gps.uart_name, \
                  uart_device_gps.baudrate,\
                  uart_device_gps.databits,\
                  uart_device_gps.parity,\
                  uart_device_gps.stopbits);

    return 0;
}


int write_gps_data_UM220()
{
    char send_buf[7]="wangbo";
    send_uart_data(uart_device_gps.uart_name, send_buf, sizeof(send_buf));

    return 0;
}

#define POSITION_SYS_GPS       0
#define POSITION_SYS_BD        1
#define POSITION_SYS_GPS_BD    2
int read_gps_data_UM220()
{
    //uint8_t positioning_system = 0; // 定位系统
    uint8_t positioning_system = 2; // 定位系统 GPS 和 北斗双定位

    switch(positioning_system)
    {
    case POSITION_SYS_GPS:
        read_gps_data_UM220_GPS();
        break;
    case POSITION_SYS_BD:
        read_gps_data_UM220_BD();
        break;
    case POSITION_SYS_GPS_BD:
        read_gps_data_UM220_GPS_BD();
        break;
    default:
        read_gps_data_UM220_GPS();
        break;
    }

    return 0;
}

/*
 * GPS定位
 */
int read_gps_data_UM220_GPS()
{
    static char        recv_buf[GPS_DATA_LEN_UM220];
    static uint16_t    recv_buf_require_len = GPS_DATA_LEN_UM220;// 请求接收的字节数目
    static uint16_t    recv_buf_real_len = 0; //实际接收的字节数目
    static uint8_t     max_wait_ms = GPS_DATA_WAIT_TIME_MS_UM220;//最大等待时间ms

    recv_buf_real_len = read_uart_data(uart_device_gps.uart_name, recv_buf, max_wait_ms, recv_buf_require_len);
    if( recv_buf_real_len > 0)
    {
        decode_data_gps_UM220_GPS((unsigned char*)recv_buf, recv_buf_real_len);
    }

    return 0;
}

/*
 * 北斗定位
 */
int read_gps_data_UM220_BD()
{
    static char        recv_buf[GPS_DATA_LEN_UM220];
    static uint16_t    recv_buf_require_len = GPS_DATA_LEN_UM220;// 请求接收的字节数目
    static uint16_t    recv_buf_real_len = 0; //实际接收的字节数目
    static uint8_t     max_wait_ms = GPS_DATA_WAIT_TIME_MS_UM220;//最大等待时间ms

    recv_buf_real_len = read_uart_data(uart_device_gps.uart_name, recv_buf, max_wait_ms, recv_buf_require_len);
    if( recv_buf_real_len > 0)
    {
        decode_data_gps_UM220_BD((unsigned char*)recv_buf, recv_buf_real_len);
    }

    return 0;
}

/*
 * GPS 和 北斗 混合
 */
int read_gps_data_UM220_GPS_BD()
{
    static char        recv_buf[GPS_DATA_LEN_UM220];
    static uint16_t    recv_buf_require_len = GPS_DATA_LEN_UM220;// 请求接收的字节数目
    static uint16_t    recv_buf_real_len = 0; //实际接收的字节数目
    static uint8_t     max_wait_ms = GPS_DATA_WAIT_TIME_MS_UM220;//最大等待时间ms

    recv_buf_real_len = read_uart_data(uart_device_gps.uart_name, recv_buf, max_wait_ms, recv_buf_require_len);
    recv_buf[recv_buf_real_len] = '\0';
//    DEBUG_PRINTF("GPS_UM220    :=\n");
//    DEBUG_PRINTF("%s \n", recv_buf);
    if( recv_buf_real_len > 0)
    {
        decode_data_gps_UM220_GPS_BD((unsigned char*)recv_buf, recv_buf_real_len);
    }

    return 0;
}

int gps_uart_close_UM220()
{
    uart_device_gps.uart_name = (char *)UART_GPS_UM220;
    close_uart_dev(uart_device_gps.uart_name);

    return 0;
}

int decode_data_gps_UM220_GPS(unsigned char *buf, int len)
{
//    nmea_GPVTG_analysis(&gpsx, buf);
//    nmea_GPGSV_analysis(&gpsx, buf);
    nmea_GPGGA_analysis(&gpsx, buf);
    nmea_GPRMC_analysis(&gpsx, buf);

    /*
     * 这里需要加个判断
     * 有这么几种数据错误的可能
     * 1 速度过大
     * 2 经纬度跳变 突然从116跳到了0
     * 如果经纬度小于50度，也不赋值
     */
    if((gpsx.latitude != 0) && (gpsx.longitude != 0) )
    {
        memcpy(&gps_data_UM220, &gpsx, sizeof(gpsx));
    }

    return 0;
}

int decode_data_gps_UM220_BD(unsigned char *buf, int len)
{

    return 0;
}

int decode_data_gps_UM220_GPS_BD(unsigned char *buf, int len)
{

//    nmea_GNVTG_analysis(&gpsx, buf);
//    nmea_GNGSV_analysis(&gpsx, buf);
    nmea_GNGGA_analysis(&gpsx, buf);
    nmea_GNRMC_analysis(&gpsx, buf);

    /*
     * 这里需要加个判断
     * 有这么几种数据错误的可能
     * 1 速度过大
     * 2 经纬度跳变 突然从116跳到了0
     * 如果经纬度小于50度，也不赋值
     */
    if((gpsx.latitude != 0) && (gpsx.longitude != 0) )
    {
        memcpy(&gps_data_UM220, &gpsx, sizeof(gpsx));
    }

    return 0;
}




static void nmea_GPGSV_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    if ((p = (unsigned char*)strstr((const char *)buf, "$GPGSV")) != NULL)
    {
        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)gpsx->svnum = nmea_str2num(p + posx, &dx); // 本系统可见卫星的总数 这个值和GPGGA中的 参与定位的星数不一样
    }
}

static void nmea_GPGGA_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    int64_t temp; // 小数点后有n位 就放大10的n次方倍
    int64_t temp_require; // 实际需要放大的倍数，因为temp也许放大的倍数太多，我们并不需要

    if ((p = (unsigned char*)strstr((const char *)buf, "$GPGGA")) != NULL) // 单独gps 勿删除
    {
        posx = nmea_comma_pos(p, 7);
        if (posx != 0XFF)gpsx->posslnum = nmea_str2num(p + posx, &dx); // 参与定位的卫星数量
        posx = nmea_comma_pos(p, 8);
        if (posx != 0XFF)gpsx->HDOP = nmea_str2num(p + posx, &dx); // HDOP 水平精度因子 0.0 ~ 99.999
        posx = nmea_comma_pos(p, 9);
        if (posx != 0XFF)
        {
            temp = nmea_str2num(p + posx, &dx);
            temp_require  = convert_to_require(temp, dx, 1);

            gpsx->altitude = (int64_t)((float)temp_require * 0.1);
        }
    }
}


static void nmea_GPRMC_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx;
    unsigned char posx;
    int64_t temp; // 小数点后有n位 就放大10的n次方倍
    int64_t temp_require; // 实际需要放大的倍数，因为temp也许放大的倍数太多，我们并不需要
    int64_t temp_require_int; // 计算时有可能会有负数，用这个


    int64_t tmp_d, tmp_m;

    if ((p = (unsigned char*)strstr((const char *)buf, "GPRMC")) != NULL) // 单独gps 勿删除
    {
        posx = nmea_comma_pos(p, 1);
        if (posx != 0xff)
        {
            temp          = nmea_str2num(p + posx, &dx);
            temp_require  = convert_to_require(temp, dx, 0);

            gpsx->hour    = (unsigned char)(temp / 1e4);
            gpsx->min     = (unsigned char)((temp / 100) % 100);
            gpsx->sec     = (unsigned char)(temp % 100);
        }

        posx = nmea_comma_pos(p, 2);
        if (posx != 0xff)
        {
            gpsx->gpssta = *(p + posx);
        }

        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)
        {
            temp          = nmea_str2num(p + posx, &dx);
            temp_require = convert_to_require(temp, dx, 7);

            tmp_d = (int64_t)(temp_require * 1e-9) * 1e9;
            tmp_m = temp_require - tmp_d;
        }

        posx = nmea_comma_pos(p, 4); // 北纬 还是 南纬度
        if (posx != 0xff)
        {
            gpsx->nshemi = *(p + posx);
        }
        if( gpsx->nshemi == 'N')
        {
            gpsx->latitude = (int64_t)((float)tmp_d * 0.01) + (float(tmp_m) * 0.01666667);
        }
        else if( gpsx->nshemi == 'S')
        {
            gpsx->latitude = - (int64_t)((float)tmp_d * 0.01) + (float(tmp_m) * 0.01666667);
        }

        posx = nmea_comma_pos(p, 5);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            temp_require = convert_to_require(temp, dx, 7);

            tmp_d = (int64_t)(temp_require * 1e-9) * 1e9;
            tmp_m = temp_require - tmp_d;
        }

        posx = nmea_comma_pos(p, 6); // 东经 还是 西经
        if (posx != 0xff)
        {
            gpsx->ewhemi = *(p + posx);
        }
        if( gpsx->ewhemi == 'E')
        {
            gpsx->longitude = (int64_t)((float)tmp_d * 0.01) + (float(tmp_m) * 0.01666667);
        }
        else if( gpsx->ewhemi == 'W')
        {
            gpsx->longitude = - (int64_t)((float)tmp_d * 0.01) + (float(tmp_m) * 0.01666667);
        }

        posx = nmea_comma_pos(p, 7);
        if (posx != 0xff)
        {
          temp = nmea_str2num(p + posx, &dx);
          temp_require = convert_to_require(temp, dx, 1); // 扩大10倍
          gpsx->velocity = (unsigned int)((float)temp_require * Knot_2_Meter);
        }

        posx = nmea_comma_pos(p, 8);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            temp_require = convert_to_require(temp, dx, 2); // 扩大100倍

            if(temp_require > 18000)
            {
                temp_require_int = (int64_t)temp_require - 36000;
            }
            else
            {
                temp_require_int = (int64_t)temp_require;
            }
            gpsx->course_radian = (float)temp_require_int * 0.01 * DEG2RAD;
        }

        posx = nmea_comma_pos(p, 9);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            gpsx->day        = (unsigned char)(temp / 10000);
            gpsx->month      = (unsigned char)((temp / 100) % 100);
            gpsx->year       = (unsigned short)(2000 + temp % 100);
        }
    }
}

static void nmea_GPVTG_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
//    unsigned char *p, dx,posx;
//    unsigned int temp;
//
//    if ((p = (unsigned char*)strstr((const char *)buf, "$GPVTG")) != NULL)
//    {
//
//    }
}

/*
 * nmea_comma_pos返回的是 逗号 的下一个字节的地址 与最初字符串的地址的差
 */
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

static int64_t nmea_pow(unsigned char m, unsigned char n)
{
    int64_t result = 1;
    while (n--) result *= m;
    return result;
}

/*
 * convert 39.12345 to 3912345
 * convert 39.123456 to 3912345
 * as for result, there is no more than five number after point
 * *dx : *float_num 保存了小数点后的位数是多少
 */
static int64_t nmea_str2num(unsigned char *buf, unsigned char*dx)
{
    unsigned char *p = buf;
    int64_t ires = 0;
    int64_t fres = 0;
    unsigned char ilen = 0, flen = 0;
    unsigned char i;
    unsigned char mask = 0;
    int64_t res;
    while (1)
    {
        if (*p == '-'){ mask |= 0x02; p++; } // 负号 负数

        if (*p == ',' || (*p == '*'))break;  // 该字段结束

        if (*p == '.'){ mask |= 0x01; p++; } // 小数点
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

    /*
     * 64位int类型 -9223372036854775808到9223372036854775807
     * 共19位
     */
    if(ilen + flen > 18)
    {
        printf("Error!!! the number is too large\n ");
    }

    if (mask & 0x02) buf++;

    for (i = 0; i < ilen; i++)
    {
        ires += nmea_pow(10, ilen - 1 - i) * (buf[i] - '0');
    }

    //if (flen > 5) flen = 5; // 小数部分有可能过多，这里限制放大倍数

    *dx = flen;
    for (i = 0; i<flen; i++)
    {
        fres += nmea_pow(10, flen - 1 - i)*(buf[ilen + 1 + i] - '0');
    }

    res = ires * nmea_pow(10, flen) + fres;

    if (mask & 0x02) res = -res;

    return res;
}

/*
 * 因为一开始是把所有的浮点数转换为整型的数，但是每个传感器设备的位数不一定，而我们又需要固定精度的数据
 * 所以要把放大多了的和放大少了的转换为刚好放的精度
 * 比如实际纬度为39.123456789，但是如果转换为整型为39123456789，但是我们其实想要的是3912345，所以放大多了
 * 因此需要把剩下的去掉，那么就需要除以10^(9-5)=10^4
 * 而如果实际为39.12345，但是有的设备是用39.123来表示39.12300，但其实我们应该放大到3912345的，如果只是计算小数点后面的则只能放大到39123，
 * 这样导致放大的倍数小了，所以需要放到的3912300，需要乘以10^(5-3)=10^2
 * float_num 输入值，实际的被放大的倍数，比如39.12345678 用nmea_str2num函数放大成了3912345678，那么float_num就是8
 * 但是我们需要的只是放大10的5次方倍，也就是3912345，所以require_num就是5
 */
static int64_t convert_to_require(int64_t x, unsigned char float_num, unsigned char require_num)
{
    if(float_num >= 0 && float_num < require_num)
    {
        //return x*(10^(require_num-float_num)); // 勿删除 这是曾经犯过的错误
        return x * nmea_pow(10, (require_num-float_num));
    }
    else if(float_num > require_num)
    {
        return x / nmea_pow(10, (float_num-require_num));
    }
    else
    {
        return x;
    }
}


static void nmea_GNGSV_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    if ((p = (unsigned char*)strstr((const char *)buf, "$GNGSV")) != NULL)
    {
        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)gpsx->svnum = nmea_str2num(p + posx, &dx); // 本系统可见卫星的总数 这个值和GPGGA中的 参与定位的星数不一样
    }
}

static void nmea_GNGGA_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx,posx;
    int64_t temp; // 小数点后有n位 就放大10的n次方倍
    int64_t temp_require; // 实际需要放大的倍数，因为temp也许放大的倍数太多，我们并不需要

    if ((p = (unsigned char*)strstr((const char *)buf, "$GNGGA")) != NULL) // 单独gps 勿删除
    {
        posx = nmea_comma_pos(p, 7);
        if (posx != 0XFF)gpsx->posslnum = nmea_str2num(p + posx, &dx); // 参与定位的卫星数量
        posx = nmea_comma_pos(p, 8);
        if (posx != 0XFF)gpsx->HDOP = nmea_str2num(p + posx, &dx); // HDOP 水平精度因子 0.0 ~ 99.999
        posx = nmea_comma_pos(p, 9);
        if (posx != 0XFF)
        {
            temp = nmea_str2num(p + posx, &dx);
            temp_require  = convert_to_require(temp, dx, 1);

            gpsx->altitude = (int64_t)((float)temp_require * 0.1);
        }
    }
}


static void nmea_GNRMC_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
    unsigned char *p, dx;
    unsigned char posx;
    int64_t temp; // 小数点后有n位 就放大10的n次方倍
    int64_t temp_require; // 实际需要放大的倍数，因为temp也许放大的倍数太多，我们并不需要
    int64_t temp_require_int; // 计算时有可能会有负数，用这个


    int64_t tmp_d, tmp_m;

    if ((p = (unsigned char*)strstr((const char *)buf, "GNRMC")) != NULL) // 单独gps 勿删除
    {
        posx = nmea_comma_pos(p, 1);
        if (posx != 0xff)
        {
            temp          = nmea_str2num(p + posx, &dx);
            temp_require  = convert_to_require(temp, dx, 0);

            gpsx->hour    = (unsigned char)(temp / 1e4);
            gpsx->min     = (unsigned char)((temp / 100) % 100);
            gpsx->sec     = (unsigned char)(temp % 100);
        }

        posx = nmea_comma_pos(p, 2);
        if (posx != 0xff)
        {
            gpsx->gpssta = *(p + posx);
        }

        posx = nmea_comma_pos(p, 3);
        if (posx != 0xff)
        {
            temp          = nmea_str2num(p + posx, &dx);
            temp_require  = convert_to_require(temp, dx, 7);

            tmp_d         = (int64_t)(temp_require * 1e-9) * 1e9; // 因为1e9 是double类型的，必须先转换为int64_t才能达到这条语句的目的
            tmp_m         = temp_require - tmp_d;
        }

        posx = nmea_comma_pos(p, 4); // 北纬 还是 南纬度
        if (posx != 0xff)
        {
            gpsx->nshemi = *(p + posx);
        }
        if( gpsx->nshemi == 'S')
        {
            gpsx->latitude = - (int64_t)((float)tmp_d * 0.01) + ((float)tmp_m * GPS_MINUTE_TO_DEGREE);
        }
        else
        {
            gpsx->latitude = (int64_t)((float)tmp_d * 0.01) + ((float)tmp_m * GPS_MINUTE_TO_DEGREE);
        }

        posx = nmea_comma_pos(p, 5);
        if (posx != 0xff)
        {
            temp            = nmea_str2num(p + posx, &dx);
            temp_require    = convert_to_require(temp, dx, 7);

            tmp_d = ((int64_t)(temp_require * 1e-9)) * 1e9;
            tmp_m = temp_require - tmp_d;
        }

        posx = nmea_comma_pos(p, 6); // 东经 还是 西经
        if (posx != 0xff)
        {
            gpsx->ewhemi = *(p + posx);
        }
        if( gpsx->ewhemi == 'W')
        {
            gpsx->longitude = - (int64_t)((float)tmp_d * 0.01) + ((float)tmp_m * 0.01666667);
        }
        else
        {
            gpsx->longitude = (int64_t)((float)tmp_d * 0.01) + ((float)tmp_m * 0.01666667);
        }

        posx = nmea_comma_pos(p, 7);
        if (posx != 0xff)
        {
          temp = nmea_str2num(p + posx, &dx);
          temp_require = convert_to_require(temp, dx, 1); // 扩大10倍
          gpsx->velocity = (unsigned int)((float)temp_require * Knot_2_Meter);
        }

        posx = nmea_comma_pos(p, 8);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            temp_require = convert_to_require(temp, dx, 2); // 扩大100倍

            if(temp_require > 18000)
            {
                temp_require_int = (int64_t)temp_require - 36000;
            }
            else
            {
                temp_require_int = (int64_t)temp_require;
            }
            gpsx->course_radian = (float)temp_require_int * 0.01 * DEG2RAD;
        }

        posx = nmea_comma_pos(p, 9);
        if (posx != 0xff)
        {
            temp = nmea_str2num(p + posx, &dx);
            gpsx->day        = (unsigned char)(temp / 10000);
            gpsx->month      = (unsigned char)((temp / 100) % 100);
            gpsx->year       = (unsigned short)(2000 + temp % 100);
        }
    }
}

static void nmea_GNVTG_analysis(struct T_GPS_UM220 *gpsx, unsigned char *buf)
{
//    unsigned char *p, dx,posx;
//    unsigned int temp;
//
//    if ((p = (unsigned char*)strstr((const char *)buf, "$GNVTG")) != NULL)
//    {
//
//    }
}










