#include "Upcom.h"
uint8_t UpCom_RxData_Buffer[UpCom_length];  //最初用于接收数据

struct Struct_UpCom_RxData UpComRxData;		//对应过的数据
struct Struct_UpCom_ProcessData ProcessData;		//处理后的数据

void UpCom_Data_Process(uint8_t* data){
	
	struct Struct_UpCom_RxData* ptr = (struct Struct_UpCom_RxData*) data;		//转化数据类型
	UpComRxData = *ptr;			//对应到结构体上面
	
	ProcessData.yaw = (int)UpComRxData.yaw;				//转化类型（其实可以不用），可以在这里加上公式处理
	ProcessData.pitch = (int)UpComRxData.pitch;
	ProcessData.flag1 = (int)UpComRxData.flag1;
}
void UpCom_Clear(uint8_t* data){//清除缓冲区域
	for (int i = 0; i < UpCom_length; i++){
		UpCom_RxData_Buffer[i] = 0;
	}
}
