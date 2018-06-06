/*
 *@File     : II2C.h
 *@Author   : wangbo
 *@Date     : May 28, 2018
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#ifndef LIBRARIES_II2C_H_
#define LIBRARIES_II2C_H_

#define DEV_I2C "/dev/i2c-1"

#define PCF8574_DO1_ADDR 0x24  //0x48
#define PCF8574_DO2_ADDR 0x25  //0x4a
#define DAC7574_DA1_ADDR 0x4c  //0x98
#define DAC7574_DA2_ADDR 0x4d  //0x9a

#define DAC7574_MSB_V(voltage) ((((int)(819.2*voltage/1.04))>>4) & 0xff)
#define DAC7574_LSB_V(voltage) ((((int)(819.2*voltage/1.04))&0xf) <<4)

#define DA_CHANNEL_0 0x10
#define DA_CHANNEL_1 0x12
#define DA_CHANNEL_2 0x14
#define DA_CHANNEL_3 0x16

int II2C_init();

int write_device_II2C(int chip, unsigned char *buf, unsigned int len);

int PFC8574_DO(int chip, unsigned char data);

int DAC7574_DA(int chip, int channel, float voltage);


#endif /* LIBRARIES_II2C_H_ */
