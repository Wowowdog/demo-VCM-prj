#include "key.h"
#include "delay.h"
#include "led.h"
#include "myiic.h"
//////////////////////////////////////////////////////////////////////////////////	 

//////////////////////////////////////////////////////////////////////////////////
uint8_t PCB_NUM;
//按键初始化函数
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟
		__HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();           //开启GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOE时钟
	  __HAL_RCC_GPIOG_CLK_ENABLE();           //开启GPIOG时钟

    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15;            //PA0
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    
    GPIO_Initure.Pin=GPIO_PIN_11;           //PC13
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
    
    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15; //PH2,3
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
	  GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4; //PH2,3
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);
		
		GPIO_Initure.Pin=GPIO_PIN_8;            //KEY1
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		
		GPIO_Initure.Pin=GPIO_PIN_8;            //KEY0
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
		ALLKEY_ON();
}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//0，没有任何按键按下
//1，WKUP按下 WK_UP
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(void)
{ 
		u8 key0=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8);
    u8 key1=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
    if(!key0)
			if(PCB_NUM<9)
				PCB_NUM++;
			else;
    else if(!key1) 
			if(PCB_NUM>0)
				PCB_NUM--;
	  
		if(!key0||!key1) {
		IIC_Write_GT24(0X1212,1,&PCB_NUM);
		LED_Show(PCB_NUM);
		HAL_Delay(500);
		}
	  return PCB_NUM;   //无按键按下
}

//开关处理函数
void ALLKEY_ON(void)
{
OL_ON;
CL_ON;
OIS_ON;
CL_CURR_ON;
OIS_CURR_ON;
};











