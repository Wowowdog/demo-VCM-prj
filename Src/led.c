#include "led.h"
#include "myiic.h"
	

//初始化PB0,PB1为输出.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOE_CLK_ENABLE();					//开启GPIOB时钟
	
    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;			//PB0，1
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
    GPIO_Initure.Pull=GPIO_NOPULL;         			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);     		//初始化GPIOB.0和GPIOB.1
	
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	//PB0置0
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	//PB1置1 
	  
		
	
	  IIC_Read_GT24(0X1212,1);
	  PCB_NUM=my_iic[0];
	  if(PCB_NUM==0xFF){PCB_NUM=0x00;IIC_Write_GT24(0X1212,1,&PCB_NUM);}
	  LED_Show(PCB_NUM);
}



void LED1(u8 n)	
{
	if(n)
  {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);}
  else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);}
};

void LED2(u8 n)
{if(n)
 {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);}
};

void LED3(u8 n)	
	{if(n)
  {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);}
};
	
void LED4(u8 n)	
	{if(n)
  {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);}
};
	
void LED5(u8 n)	
	{if(n)
  {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);}
};
	
void LED6(u8 n)	
	{if(n)
  {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);}
};
	
void LED7(u8 n)	
	{
		if(n){HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);}
 else {HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);}
};


void LED_Show(u8 i)
{
		switch(i)
			{
				case 0x00:LED1(1);LED2(1);LED3(1);LED4(1);LED5(1);LED6(0);LED7(1);break;
				case 0x01:LED1(0);LED2(1);LED3(0);LED4(1);LED5(0);LED6(0);LED7(0);break;
				case 0x02:LED1(1);LED2(0);LED3(0);LED4(1);LED5(1);LED6(1);LED7(1);break;
				case 0x03:LED1(1);LED2(1);LED3(0);LED4(1);LED5(0);LED6(1);LED7(1);break;
				case 0x04:LED1(0);LED2(1);LED3(1);LED4(1);LED5(0);LED6(1);LED7(0);break;
				case 0x05:LED1(1);LED2(1);LED3(1);LED4(0);LED5(0);LED6(1);LED7(1);break;
				case 0x06:LED1(1);LED2(1);LED3(1);LED4(0);LED5(1);LED6(1);LED7(1);break;
				case 0x07:LED1(0);LED2(1);LED3(0);LED4(1);LED5(0);LED6(0);LED7(1);break;
				case 0x08:LED1(1);LED2(1);LED3(1);LED4(1);LED5(1);LED6(1);LED7(1);break;
				case 0x09:LED1(1);LED2(1);LED3(1);LED4(1);LED5(0);LED6(1);LED7(1);break;
				default:break;
			}
};
