#ifndef __UpCom_H
#define __UpCom_H
#include "main.h"

#define UpCom_length 14		//接收数据的长度

struct Struct_UpCom_RxData{
	uint8_t		Frame_header;		//上位机发过来的帧头0x2A
	int 			yaw;						//上位机发过来的前进方向
	int  			pitch;					//上位机发过来的前进速度
	int 			flag1;					//上位机发过来的标志1
	uint8_t 	Frame_tail;   	//上位机发过来的帧尾
}__attribute__((packed));//使数据紧密排列，防止编译器自动填充

struct Struct_UpCom_ProcessData{
	int yaw;				//数据对应不过目前不清楚含义所以名字一样
	int pitch;			
	int flag1;
};

void UpCom_Data_Process(uint8_t* data);//处理数据的函数（可能要进行运算）
void UpCom_Clear(uint8_t* data);//清除缓冲区（Buffer）防止多次发送


#endif

