#include "myiic.h"
#include "delay.h"
#include "usbd_cdc_if.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//IIC驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/12/28
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

uint8_t my_iic[256]={0};    //receve_buffer

void  IIC_SCL(u8 n,GPIO_TypeDef * gpio_type,u16 gpio)
{
		(n?HAL_GPIO_WritePin(gpio_type,gpio,GPIO_PIN_SET):HAL_GPIO_WritePin(gpio_type,gpio,GPIO_PIN_RESET));
};

void  IIC_SDA(u8 n,GPIO_TypeDef * gpio_type,u16 gpio)
{
		(n?HAL_GPIO_WritePin(gpio_type,gpio,GPIO_PIN_SET):HAL_GPIO_WritePin(gpio_type,gpio,GPIO_PIN_RESET));
};

//IIC初始化
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //使能GPIOH时钟
	  __HAL_RCC_GPIOD_CLK_ENABLE(); 
	  __HAL_RCC_GPIOG_CLK_ENABLE(); 
    
    //PH4,5初始化设置
    GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_4;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //快速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
			
	  GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11;
	  HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	
	  GPIO_Initure.Pin=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
	  HAL_GPIO_Init(GPIOG,&GPIO_Initure);
	
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
    IIC_SDA(0,GPIOB,GPIO_PIN_3);
    IIC_SCL(0,GPIOB,GPIO_PIN_4);  
		
		IIC_SDA(0,GPIOD,GPIO_PIN_0);
    IIC_SCL(0,GPIOD,GPIO_PIN_1);  
		IIC_SDA(0,GPIOD,GPIO_PIN_2);
    IIC_SCL(0,GPIOD,GPIO_PIN_3);  
		IIC_SDA(0,GPIOD,GPIO_PIN_4);
    IIC_SCL(0,GPIOD,GPIO_PIN_5);  
		IIC_SDA(0,GPIOD,GPIO_PIN_6);
    IIC_SCL(0,GPIOD,GPIO_PIN_7);  
		IIC_SDA(0,GPIOD,GPIO_PIN_8);
    IIC_SCL(0,GPIOD,GPIO_PIN_9);  
		
		IIC_SDA(0,GPIOD,GPIO_PIN_13);
    IIC_SCL(0,GPIOD,GPIO_PIN_12); 
		
		IIC_SDA(0,GPIOG,GPIO_PIN_9);
    IIC_SCL(0,GPIOG,GPIO_PIN_10);  
		IIC_SDA(0,GPIOG,GPIO_PIN_11);
    IIC_SCL(0,GPIOG,GPIO_PIN_12);  
		IIC_SDA(0,GPIOG,GPIO_PIN_13);
    IIC_SCL(0,GPIOG,GPIO_PIN_14); 
}

//产生IIC起始信号
void IIC_Start(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	SDA_OUT(gpio_type,sda);     //sda线输出
	IIC_SDA(1,gpio_type,sda);	  	  
	IIC_SCL(1,gpio_type,scl);
	delay_us(2);
 	IIC_SDA(0,gpio_type,sda);//START:when CLK is high,DATA change form high to low 
	delay_us(2);
	IIC_SCL(0,gpio_type,scl);//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	SDA_OUT(gpio_type,sda);//sda线输出
	IIC_SCL(0,gpio_type,scl);
	IIC_SDA(0,gpio_type,sda);//STOP:when CLK is high DATA change form low to high
 	delay_us(2);
	IIC_SCL(1,gpio_type,scl); 
	IIC_SDA(1,gpio_type,sda);//发送I2C总线结束信号
	delay_us(2);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	u16 ucErrTime=0;
	SDA_IN(gpio_type,sda);      //SDA设置为输入  
	IIC_SDA(1,gpio_type,sda);delay_us(1);	  
  //delay_us(1);	
	IIC_SCL(1,gpio_type,scl);delay_us(1);	 

	while(HAL_GPIO_ReadPin(gpio_type,sda))
	{
		ucErrTime++;
		if(ucErrTime>25)
		{
			//IIC_Stop(gpio_type,scl,sda);
			IIC_SCL(0,gpio_type,scl);
			return 1;
		}
	}

	IIC_SCL(0,gpio_type,scl);//时钟输出0 	 
  //delay_us(2);	 
	delay_us(1);  
	return 0;  
} 
//产生ACK应答
void IIC_Ack(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	IIC_SCL(0,gpio_type,scl);
	SDA_OUT(gpio_type,sda);
	IIC_SDA(0,gpio_type,sda);
	delay_us(2);
	IIC_SCL(1,gpio_type,scl);
	delay_us(2);
	IIC_SCL(0,gpio_type,scl);
}
//不产生ACK应答		    
void IIC_NAck(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	IIC_SCL(0,gpio_type,scl);
	SDA_OUT(gpio_type,sda);
	IIC_SDA(1,gpio_type,sda);
	delay_us(2);
	IIC_SCL(1,gpio_type,scl);
	delay_us(2);
	IIC_SCL(0,gpio_type,scl);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd,GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{                        
    u8 t;   
	  SDA_OUT(gpio_type,sda); 	    
	
	
    IIC_SCL(0,gpio_type,scl);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7,gpio_type,sda);
        txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL(1,gpio_type,scl);
		delay_us(2); 
		IIC_SCL(0,gpio_type,scl);	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack,GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	unsigned char i,receive=0;
	
	
	SDA_IN(gpio_type,sda);//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0,gpio_type,scl); 
        delay_us(2);
		IIC_SCL(1,gpio_type,scl);
        receive<<=1;
        if(HAL_GPIO_ReadPin(gpio_type,sda))receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck(gpio_type,scl,sda);//发送nACK
    else
        IIC_Ack(gpio_type,scl,sda); //发送ACK   
    return receive;
}

void SDA_IN(GPIO_TypeDef * gpio_type,u16 gpio)
{ 
	u8 i=0;
   	switch(gpio)
			{
				case GPIO_PIN_3:i=3;break;
				case GPIO_PIN_13:i=13;break;
				case GPIO_PIN_11:i=11;break;
				case GPIO_PIN_9:i=9;break;
				case GPIO_PIN_6:i=6;break;
				case GPIO_PIN_4:i=4;break;
				case GPIO_PIN_2:i=2;break;
				case GPIO_PIN_0:i=0;break;
				case GPIO_PIN_8:i=8;break;
				default:break;
			} 
		gpio_type->MODER&=~(3<<(i*2));gpio_type->MODER|=0<<i*2;
};
	
void SDA_OUT(GPIO_TypeDef * gpio_type,u16 gpio)
{
	  u8 i=0;
   	switch(gpio)
			{
				case GPIO_PIN_3:i=3;break;
				case GPIO_PIN_13:i=13;break;
				case GPIO_PIN_11:i=11;break;
				case GPIO_PIN_9:i=9;break;
				case GPIO_PIN_6:i=6;break;
				case GPIO_PIN_4:i=4;break;
				case GPIO_PIN_2:i=2;break;
				case GPIO_PIN_0:i=0;break;
				case GPIO_PIN_8:i=8;break;
				default:break;
			} 
		gpio_type->MODER&=~(3<<(i*2));gpio_type->MODER|=1<<i*2;
};

void IIC_Write_Arr(u8 iic_ord,u8 iic_addr,u8 reg_addr,u8 length,u8* data)
{
		u16 sda,scl;
	  GPIO_TypeDef * gpio;
		switch(iic_ord)
			{
				case 0x00:gpio=GPIOB;scl=GPIO_PIN_4;sda=GPIO_PIN_3;break;
				case 0x01:gpio=GPIOG;scl=GPIO_PIN_14;sda=GPIO_PIN_13;break;
				case 0x02:gpio=GPIOG;scl=GPIO_PIN_12;sda=GPIO_PIN_11;break;
				case 0x03:gpio=GPIOG;scl=GPIO_PIN_10;sda=GPIO_PIN_9;break;
				case 0x04:gpio=GPIOD;scl=GPIO_PIN_7;sda=GPIO_PIN_6;break;
				case 0x05:gpio=GPIOD;scl=GPIO_PIN_5;sda=GPIO_PIN_4;break;
				case 0x06:gpio=GPIOD;scl=GPIO_PIN_3;sda=GPIO_PIN_2;break;
				case 0x07:gpio=GPIOD;scl=GPIO_PIN_1;sda=GPIO_PIN_0;break;
				case 0x08:gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;break;
				case 0x09:gpio=GPIOD;scl=GPIO_PIN_9;sda=GPIO_PIN_8;break;
				default:break;
			}
		IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(iic_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Send_Byte(reg_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		for(u8 i=0;i<length;i++)
			{
				IIC_Send_Byte(data[i],gpio,scl,sda);     //发送字节							   
      	IIC_Wait_Ack(gpio,scl,sda);
				 				
			};
		IIC_Stop(gpio,scl,sda);	
		
};
void IIC_Read_Arr(u8 iic_ord,u8 iic_addr,u8 reg_addr,u8 length)
{
    u16 sda,scl;
	  GPIO_TypeDef * gpio;
		switch(iic_ord)
			{
				case 0x00:gpio=GPIOB;scl=GPIO_PIN_4;sda=GPIO_PIN_3;break;
				case 0x01:gpio=GPIOG;scl=GPIO_PIN_14;sda=GPIO_PIN_13;break;
				case 0x02:gpio=GPIOG;scl=GPIO_PIN_12;sda=GPIO_PIN_11;break;
				case 0x03:gpio=GPIOG;scl=GPIO_PIN_10;sda=GPIO_PIN_9;break;
				case 0x04:gpio=GPIOD;scl=GPIO_PIN_7;sda=GPIO_PIN_6;break;
				case 0x05:gpio=GPIOD;scl=GPIO_PIN_5;sda=GPIO_PIN_4;break;
				case 0x06:gpio=GPIOD;scl=GPIO_PIN_3;sda=GPIO_PIN_2;break;
				case 0x07:gpio=GPIOD;scl=GPIO_PIN_1;sda=GPIO_PIN_0;break;
				case 0x08:gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;break;
				case 0x09:gpio=GPIOD;scl=GPIO_PIN_9;sda=GPIO_PIN_8;break;
				default:break;
			}
		IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(iic_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Send_Byte(reg_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Stop(gpio,scl,sda);	
		IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(iic_addr+1,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		for(u8 i=0;i<length;i++)
			{
				if(i<length-1){
				my_iic[i]=IIC_Read_Byte(1,gpio,scl,sda);}
        else 	my_iic[i]=IIC_Read_Byte(0,gpio,scl,sda);			//receive_buffer
			};
		IIC_Stop(gpio,scl,sda);	
};	

void IIC_Write_GT24(u16 reg_addr,u8 length,u8* data)
{
	
	  u16 sda,scl;
	  GPIO_TypeDef * gpio;
	  gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;  //IIC8路
		IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(0XA0,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Send_Byte(reg_addr>>8,gpio,scl,sda);
	  IIC_Wait_Ack(gpio,scl,sda);
	  IIC_Send_Byte(reg_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		for(u8 i=0;i<length;i++)
			{
				IIC_Send_Byte(data[i],gpio,scl,sda);     //发送字节							   
      	IIC_Wait_Ack(gpio,scl,sda);
				 				
			};
		IIC_Stop(gpio,scl,sda);	
};

void IIC_Read_GT24(u16 reg_addr,u8 length)
{
		u16 sda,scl;
	  GPIO_TypeDef * gpio;
	  gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;  //IIC8路
	  IIC_Start(gpio,scl,sda);
	  IIC_Send_Byte(0XA0,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Send_Byte(reg_addr>>8,gpio,scl,sda);
	  IIC_Wait_Ack(gpio,scl,sda);
	  IIC_Send_Byte(reg_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
	  IIC_Stop(gpio,scl,sda);	
	  IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(0XA0+1,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		for(u8 i=0;i<length;i++)
			{
				my_iic[i]=IIC_Read_Byte(1,gpio,scl,sda);   //receive_buffer
			};
		IIC_Stop(gpio,scl,sda);	
	 //CDC_Transmit_HS(my_iic,length);
};

void IIC_Write(void)
{
		if(my_RxBuf[3]==0X10) 
    {
		  IIC_Write_Arr(0,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
		  IIC_Write_Arr(1,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
		  IIC_Write_Arr(2,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
			IIC_Write_Arr(3,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
			IIC_Write_Arr(4,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
			IIC_Write_Arr(5,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
			IIC_Write_Arr(6,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
			IIC_Write_Arr(7,my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
		}
		else IIC_Write_Arr(my_RxBuf[3],my_RxBuf[4],my_RxBuf[5],my_RxBuf[6],my_RxBuf+7);
};   

void IIC_Read(void)
{
		IIC_Read_Arr(my_RxBuf[3],my_RxBuf[4],my_RxBuf[5],my_RxBuf[6]);
	  CDC_Transmit_HS(my_iic,my_RxBuf[6]);
};	

u8 check_device(GPIO_TypeDef * gpio_type,u16 scl,u16 sda,u8 addr)
{
	
	IIC_Start(gpio_type,scl,sda);
	IIC_Send_Byte(addr,gpio_type,scl,sda);
	if(IIC_Wait_Ack(gpio_type,scl,sda)==0)
	{ 
	IIC_Stop(gpio_type,scl,sda);
	return addr;}
	else {
  IIC_Stop(gpio_type,scl,sda);
	return 0;}
	
}

void IICboardcast(void)
{
	 u16 sda,scl;
	 GPIO_TypeDef * gpio;
	 u8 addr;
	 u8 cnt=0;
	 u8 send[2];
	 u8 err[3]={0xaa,0xaa,0xaa};
		switch(my_RxBuf[3])
			{
				case 0x00:gpio=GPIOB;scl=GPIO_PIN_4;sda=GPIO_PIN_3;break;
				case 0x01:gpio=GPIOG;scl=GPIO_PIN_14;sda=GPIO_PIN_13;break;
				case 0x02:gpio=GPIOG;scl=GPIO_PIN_12;sda=GPIO_PIN_11;break;
				case 0x03:gpio=GPIOG;scl=GPIO_PIN_10;sda=GPIO_PIN_9;break;
				case 0x04:gpio=GPIOD;scl=GPIO_PIN_7;sda=GPIO_PIN_6;break;
				case 0x05:gpio=GPIOD;scl=GPIO_PIN_5;sda=GPIO_PIN_4;break;
				case 0x06:gpio=GPIOD;scl=GPIO_PIN_3;sda=GPIO_PIN_2;break;
				case 0x07:gpio=GPIOD;scl=GPIO_PIN_1;sda=GPIO_PIN_0;break;
				case 0x08:gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;break;
				case 0x09:gpio=GPIOD;scl=GPIO_PIN_9;sda=GPIO_PIN_8;break;
				default:break;
			}
	for(u8 i=0;i<128;i++)
	{addr=check_device(gpio,scl,sda,i*2);
	if(addr!=0)
  { send[0]=i*2;
		send[1]=i*2+1;
		
		CDC_Transmit_HS(send,1);
		cnt++;		
	} 	
	}
	if(0==cnt){CDC_Transmit_HS(err,3);}
};


void balltest(void)
{
		u8 ball[6]={my_RxBuf[3],my_RxBuf[3]>>1,my_RxBuf[3]>>2,my_RxBuf[3]>>3,my_RxBuf[3]>>4,my_RxBuf[3]>>5};
		u32 time=my_RxBuf[4];
		time=(time<<8) + my_RxBuf[5];
		time=(time<<8) + my_RxBuf[6];
		u8 begin=0x00;
		u8 end=0xff;
		if(ball[0]&0x01){IIC_Write_Arr(0,0x1c,0x02,1,&begin);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x1c,0x02,1,&begin);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x1c,0x02,1,&begin);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x1c,0x02,1,&begin);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x1c,0x02,1,&begin);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x1c,0x02,1,&begin);}
		if(ball[0]&0x01){IIC_Write_Arr(0,0x9c,0x02,1,&begin);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x9c,0x02,1,&begin);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x9c,0x02,1,&begin);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x9c,0x02,1,&begin);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x9c,0x02,1,&begin);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x9c,0x02,1,&begin);}
		HAL_Delay(5);
		for(u16 i=0;i<time;i++)
		{
	  if(ball[0]&0x01){IIC_Write_Arr(0,0x1c,0x00,1,&begin);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x1c,0x00,1,&begin);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x1c,0x00,1,&begin);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x1c,0x00,1,&begin);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x1c,0x00,1,&begin);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x1c,0x00,1,&begin);}
		
		if(ball[0]&0x01){IIC_Write_Arr(0,0x9c,0x00,1,&begin);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x9c,0x00,1,&begin);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x9c,0x00,1,&begin);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x9c,0x00,1,&begin);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x9c,0x00,1,&begin);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x9c,0x00,1,&begin);}
		HAL_Delay(my_RxBuf[7]*10);
		if(ball[0]&0x01){IIC_Write_Arr(0,0x1c,0x00,1,&end);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x1c,0x00,1,&end);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x1c,0x00,1,&end);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x1c,0x00,1,&end);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x1c,0x00,1,&end);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x1c,0x00,1,&end);}
		
		if(ball[0]&0x01){IIC_Write_Arr(0,0x9c,0x00,1,&end);}
		if(ball[1]&0x01){IIC_Write_Arr(1,0x9c,0x00,1,&end);}
		if(ball[2]&0x01){IIC_Write_Arr(2,0x9c,0x00,1,&end);}
		if(ball[3]&0x01){IIC_Write_Arr(3,0x9c,0x00,1,&end);}
		if(ball[4]&0x01){IIC_Write_Arr(4,0x9c,0x00,1,&end);}
		if(ball[5]&0x01){IIC_Write_Arr(5,0x9c,0x00,1,&end);}
		HAL_Delay(my_RxBuf[7]*10);
		
		
	  }
};

void sinroute(void)
{   
	  u8 begin=0x00;
		u8 channel=my_RxBuf[3];
	  u8 amp=100;
	  double amp_d=amp;
	  u8 pos=0;
	  double pos_d=pos;
	  double j_d=0;
	  double PI=3.1415926535;
		IIC_Write_Arr(channel,0x18,0x02,1,&begin);
	  HAL_Delay(5);
	  u16 ped=1000/my_RxBuf[4];
	  u16 zhouqi=30;
	  if(my_RxBuf[5]!=0){zhouqi=my_RxBuf[5];}
	  for(u16 i=0;i<zhouqi;i++)	
			{
					for(u16 j=0;j<ped;j++)
				  {
				  j_d=j;
					pos_d=amp_d*sin(2*PI*j_d/ped);
					pos=pos_d+amp;
					IIC_Write_Arr(channel,0x18,0x00,1,&pos);
					HAL_Delay(1);
					}
			}
};


