#include "hx711.h"


void HX711_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = HX711_PinSCK;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HX711_PortSCK, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = HX711_PinDT;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HX711_PortDT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_RESET);

}

//int HX711_Average_Value(uint8_t times)
//{
//    int sum = 0;
//    for (int i = 0; i < times; i++)
//    {
//        sum += HX711_Value();
//    }
//
//    return sum / times;
//}

int HX711_Value(int gain)
{
    int buffer;
    buffer = 0;

    while (HAL_GPIO_ReadPin(HX711_PortDT, HX711_PinDT)==1)
    ;

    for (uint8_t i = 0; i < 24; i++)
    {
    	HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_SET);

        buffer = buffer << 1 ;

        if (HAL_GPIO_ReadPin(HX711_PortDT, HX711_PinDT))
        {
            buffer ++;
        }

        HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_RESET);
    }

    for (int i = 0; i < gain; i++)
    {
    	HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_SET);
    	HAL_GPIO_WritePin(HX711_PortSCK, HX711_PinSCK, GPIO_PIN_RESET);
    }



    if ((buffer & 0x800000) == 0x800000)
    {
    		buffer = buffer ^ 0xff000000;
    }
    return buffer;
}



