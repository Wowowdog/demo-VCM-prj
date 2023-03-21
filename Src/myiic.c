#include "myiic.h"
#include "delay.h"
#include "usbd_cdc_if.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F7������
//IIC��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/28
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
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

//IIC��ʼ��
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOHʱ��
	  __HAL_RCC_GPIOD_CLK_ENABLE(); 
	  __HAL_RCC_GPIOG_CLK_ENABLE(); 
    
    //PH4,5��ʼ������
    GPIO_Initure.Pin=GPIO_PIN_3|GPIO_PIN_4;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;    //����
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

//����IIC��ʼ�ź�
void IIC_Start(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	SDA_OUT(gpio_type,sda);     //sda�����
	IIC_SDA(1,gpio_type,sda);	  	  
	IIC_SCL(1,gpio_type,scl);
	delay_us(2);
 	IIC_SDA(0,gpio_type,sda);//START:when CLK is high,DATA change form high to low 
	delay_us(2);
	IIC_SCL(0,gpio_type,scl);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	SDA_OUT(gpio_type,sda);//sda�����
	IIC_SCL(0,gpio_type,scl);
	IIC_SDA(0,gpio_type,sda);//STOP:when CLK is high DATA change form low to high
 	delay_us(2);
	IIC_SCL(1,gpio_type,scl); 
	IIC_SDA(1,gpio_type,sda);//����I2C���߽����ź�
	delay_us(2);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	u16 ucErrTime=0;
	SDA_IN(gpio_type,sda);      //SDA����Ϊ����  
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

	IIC_SCL(0,gpio_type,scl);//ʱ�����0 	 
  //delay_us(2);	 
	delay_us(1);  
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd,GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{                        
    u8 t;   
	  SDA_OUT(gpio_type,sda); 	    
	
	
    IIC_SCL(0,gpio_type,scl);//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7,gpio_type,sda);
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL(1,gpio_type,scl);
		delay_us(2); 
		IIC_SCL(0,gpio_type,scl);	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack,GPIO_TypeDef * gpio_type,u16 scl,u16 sda)
{
	unsigned char i,receive=0;
	
	
	SDA_IN(gpio_type,sda);//SDA����Ϊ����
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
        IIC_NAck(gpio_type,scl,sda);//����nACK
    else
        IIC_Ack(gpio_type,scl,sda); //����ACK   
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
				IIC_Send_Byte(data[i],gpio,scl,sda);     //�����ֽ�							   
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
	  gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;  //IIC8·
		IIC_Start(gpio,scl,sda);
		IIC_Send_Byte(0XA0,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		IIC_Send_Byte(reg_addr>>8,gpio,scl,sda);
	  IIC_Wait_Ack(gpio,scl,sda);
	  IIC_Send_Byte(reg_addr,gpio,scl,sda);
		IIC_Wait_Ack(gpio,scl,sda);
		for(u8 i=0;i<length;i++)
			{
				IIC_Send_Byte(data[i],gpio,scl,sda);     //�����ֽ�							   
      	IIC_Wait_Ack(gpio,scl,sda);
				 				
			};
		IIC_Stop(gpio,scl,sda);	
};

void IIC_Read_GT24(u16 reg_addr,u8 length)
{
		u16 sda,scl;
	  GPIO_TypeDef * gpio;
	  gpio=GPIOD;scl=GPIO_PIN_12;sda=GPIO_PIN_13;  //IIC8·
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


