/*
 * save_data.h
 *
 *  Created on: Jul 1, 2016
 *      Author: wangbo
 */

#ifndef HEADERS_SAVE_DATA_H_
#define HEADERS_SAVE_DATA_H_

int create_log_file(char *log_name);

/*
 * only for string, the end char must be \0' ,(len=0 or any one) is ok
 * 只能存储字符串也就是最后一定要有'\0'
 *
 */
int save_data_to_string_log(int fd_log, char *string, int len);

/*
 * 保存为二进制文件
 */
int save_data_to_binary_log(int fd_log, void*buffer, int len);

int close_log_file(int fd_log);

/*
 * 创建某种数据结构的二进制保存文件，比如航点数据结构以及日志数据结构
 * 如果本地目录中没有该二进制文件，则会创建，
 * 如果有的话，则会把文件中的内容加载到指定数据结构中
 */
int load_data_struct_from_binary(char* file_name,void* data_struct,int data_struct_size);

#endif /* HEADERS_SAVE_DATA_H_ */
