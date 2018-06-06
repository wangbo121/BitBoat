/*
 *@File     : save_data.cpp
 *@Author   : wangbo
 *@Date     : Jul 1, 2016
 *@Copyright: 2018 Beijing Institute of Technology. All right reserved.
 *@Warning  : 本内容仅限于北京理工大学复杂工业控制实验室内部传阅-禁止外泄以及用于其他商业目的
 */
#include <stdio.h>
#include <stdlib.h>
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

	chmod(convert_to_string, 0666);//改变文件权限为 任何人都可以修改

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
        printf("文件 %s 存在，正在打开！\n", file_name);
        fd=open(file_name,O_RDWR |O_CREAT);

        if(-1== stat( file_name, &f_stat ) )
        {
            printf("文件 %s 存在，但是不能获取文件状态. \n", file_name);

            return -1;
        }
        else if(f_stat.st_size >= data_struct_size)
        {
            //printf("f_stat.st_size = %ld \n", f_stat.st_size);
            //载入文件的最后一个数据结构
            lseek(fd,f_stat.st_size-data_struct_size,SEEK_SET);
            read_len = read(fd, data_struct, data_struct_size);
            printf("文件 %s 存在，载入该文件，载入字节数 = %d. \n", file_name, read_len);

            return fd;
        }
        else
        {
            printf("文件 %s 存在，但是该文件没有数据，载入数据失败. \n",file_name);

            return fd;
        }
    }
    else
    {
        printf("文件不存在，正在创建文件 %s \n",file_name);
        if(-1 == (fd=open(file_name,O_RDWR |O_CREAT)))
        {
            printf("文件不存在，并且创建文件 %s 失败. \n",file_name);
            exit(1);
        }
        else
        {
            printf("文件不存在，并且创建文件 %s 成功. \n",file_name);
            chmod(file_name,0666);//改变文件权限为 任何人都可以修改
        }

        return fd;
    }

    return fd;
}
