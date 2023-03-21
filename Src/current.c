#include "key.h"
#include "current.h"
#include "myiic.h"
#include "usbd_cdc_if.h"

void CURR_Init(void)
{
	  u8 buf[2]={0X50,0X00};
		u8 ad_1[3]={0X00,0X00};
		u8 ad_2[3]={0Xa9,0X00};
		IIC_Write_Arr(8,0X1a,0X7f,2,ad_1);  //3.3V供电源
		IIC_Write_Arr(8,0X1a,0X1f,2,ad_2);  //3.3V供电
		IIC_Write_Arr(8,0X80,0X05,2,buf);  //AFУ׼
		IIC_Write_Arr(8,0X82,0X05,2,buf);  //OISУ׼

};

void CL_AF_CURR(void)
{
	  if(my_RxBuf[3]==0x00){
		OL_OFF;
	  CL_ON;
		OIS_OFF;
	  CL_CURR_ON;
		HAL_Delay(5);
	  IIC_Read_Arr(8,0X80,0X01,2);
	  CDC_Transmit_HS(my_iic,2);
		}
	  if(my_RxBuf[3]==0x01){
		OL_OFF;
	  CL_ON;
		OIS_OFF;
	  CL_CURR_OFF;
		HAL_Delay(5);
	  IIC_Read_Arr(8,0X80,0X01,2);
	  CDC_Transmit_HS(my_iic,2);
		}
};
	
void CL_OIS_CURR(void)
{
	if(my_RxBuf[3]==0x00){
		OL_OFF;
	  CL_OFF;
		OIS_ON;	
	  OIS_CURR_ON;
		HAL_Delay(5);
	  IIC_Read_Arr(8,0X82,0X01,2);
	  CDC_Transmit_HS(my_iic,2);
	}
	if(my_RxBuf[3]==0x01){
		OL_OFF;
	  CL_OFF;
		OIS_ON;	
	  OIS_CURR_OFF;
		HAL_Delay(5);
	  IIC_Read_Arr(8,0X82,0X01,2);
	  CDC_Transmit_HS(my_iic,2);
	}
};











