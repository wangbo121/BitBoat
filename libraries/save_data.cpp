/*
 * save_data.c
 *
 *  Created on: Jul 1, 2016
 *      Author: wangbo
 */

#include <stdio.h>
#include <stdlib.h>/*exit(1)*/
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#include <time.h>
#include <sys/time.h>

#include "save_data.h"

int create_log_file(char *log_name)
{
	int fd;
	char convert_to_string[200];

	time_t ptrtime;
	struct tm *time_of_file;

	time(&ptrtime);
	time_of_file=localtime(&ptrtime);

	sprintf(convert_to_string,"%d-%d-%d-%d-%d-%d",time_of_file->tm_year+1900,time_of_file->tm_mon+1,time_of_file->tm_mday,\
			                            time_of_file->tm_hour,time_of_file->tm_min,time_of_file->tm_sec);
	strcat(convert_to_string,log_name);

	if(-1==(fd=open(convert_to_string,O_RDWR |O_CREAT |O_APPEND)))
	{
		printf("Error:Can not open %s",convert_to_string);
		exit(1);
	}

	chmod(convert_to_string,0666);//改变文件权限为 任何人都可以修改

	//printf("create_log_file fd =%d\n",fd);// 20180208已测试

	return fd;
}

int save_data_to_string_log(int fd_log,char *string,int len)
{
	int write_len;
	int i=0;

	for(i=0;string[i]!='\0';i++)
		;
	len=i;

	if(fd_log)
	{
		write_len=write(fd_log,string,len);
	}

	return write_len;
}

int save_data_to_binary_log(int fd_log,void*buffer,int len)
{
    int write_len;

    if(fd_log)
    {
        write_len=write(fd_log,buffer,len);
    }

    return write_len;
}

int close_log_file(int fd_log)
{
	if(fd_log)
	{
		close(fd_log);
	}

	return 0;
}

int load_data_struct_from_binary(char* file_name,void* data_struct,int data_struct_size)
{
    struct stat f_stat;
    int fd=0;
    int read_len=0;

    if((access(file_name,F_OK))==0)
    {
        printf("文件存在，正在打开！\n");
        fd=open(file_name,O_RDWR |O_CREAT);

        if(-1== stat( file_name, &f_stat ) )
        {
            printf("文件存在，但是不能获取文件状态\n");

            return -1;
        }
        else if(f_stat.st_size>=data_struct_size)
        {
            printf("f_stat.st_size=%u\n",f_stat.st_size);
            //载入文件的最后一个数据结构
            lseek(fd,f_stat.st_size-data_struct_size,SEEK_SET);
            read_len = read(fd, data_struct, data_struct_size);
            printf("文件存在，载入文件%s,载入字节数=%d\n",file_name,read_len);

            return fd;
        }
        else
        {
            printf("文件存在，但是 %s 文件没有数据，载入数据失败\n",file_name);

            //return -1;//没有数据，就不载入了
            return fd;
        }
    }
    else
    {
        printf("文件不存在，正在创建文件 %s \n",file_name);
        if(-1==(fd=open(file_name,O_RDWR |O_CREAT)))
        {
            printf("文件不存在，并且创建文件 %s 失败！",file_name);
            exit(1);
        }
        else
        {
            printf("文件不存在，并且创建文件 %s 成功！",file_name);
            //chmod(file_name,0777);//改变文件权限为 任何人都可以修改
            chmod(file_name,0666);//改变文件权限为 任何人都可以修改
        }

        return fd;
    }

    return fd;
}
