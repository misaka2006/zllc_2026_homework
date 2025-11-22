#include "Bluetooth.h"
#include "string.h"
uint8_t Bluetooth_RxData[Bluetooth_length];				//用于接收最初的数据

struct Struct_Bluetooth_RxData RxData;						//存对应后的数据
struct Struct_Bluetooth_RxData Pre_RxData;				//存Rx_Data之前的值
struct Struct_Bluetooth_PorcessData PorcessData;	//存处理过后的数据值

//判断按键所处的状态
void Judge_Key(enum Enum_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)  
{
    //带触发的判断
    switch (Pre_Status)
    {
    case (KEY_FREE):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = Key_Status_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = Key_Status_TRIG_FREE_PRESSED;
        }
        break;
        }
    }
    break;
    case (KEY_PRESSED):
    {
        switch (Status)
        {
        case (KEY_FREE):
        {
            *Key = Key_Status_TRIG_PRESSED_FREE;
        }
        break;
        case (KEY_PRESSED):
        {
            *Key = Key_Status_PRESSED;
        }
        break;
        }
    }
    break;
    } 
}

//处理数据，将数据转化成（-1,1）的范围
void Bluetooth_Data_Process(uint8_t *data)
{
    struct Struct_Bluetooth_RxData* ptr = (struct Struct_Bluetooth_RxData*)data; 
		RxData = *ptr;
    
    PorcessData.Remote_Left_X = (float)RxData.Remote_Left_X / 100.0f;
    PorcessData.Remote_Left_Y = (float)RxData.Remote_Left_Y / 100.0f;
    PorcessData.Remote_Right_X = (float)RxData.Remote_Right_X / 100.0f;
    PorcessData.Remote_Right_Y = (float)RxData.Remote_Right_Y / 100.0f;
}

//扫描按键
void Key_Scan()
{
    for (int i = 0; i < 8; i++)
    {
        Judge_Key(&PorcessData.Key[i], (RxData.Button_One >> i) & (0x1), (Pre_RxData.Button_One >> i) & (0x1));
    }
    for (int i = 0; i < 8; i++)
    {
        Judge_Key(&PorcessData.Key[i+8], (RxData.Button_Two >> i) & (0x1), (Pre_RxData.Button_Two >> i) & (0x1));
    }

    memcpy(&Pre_RxData,&RxData,Bluetooth_length);
}

