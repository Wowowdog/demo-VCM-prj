/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

u8 timflag=0;
u32 cnt=0;
int flag=1;
u16 curr9800=0;
u8 curr[2]={0};
u8 curr0[2]={0};
u8 curr1[2]={0x03,0xff};
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 19999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
				if(htim==(&htim3)) 
					{ 
							cnt++;
		if((cnt%1000)==0){flag=-flag;}
		if((cnt%1000)==0){
		rtU.in=rtU.in+(flag*200);
//		rtU.in_o=rtU.in_o+(flag*500);
		}
//		rtU.in=0;
		rtU.hallcode=hall_dect_y();
		rtU.hallcode_d=(hall_dect_x1()+hall_dect_x2())/2;
		rtU.hallcode_g=rtU.hallcode_d;
		ois195y_step();
		curr9800=rtY.Out1;
		curr[0]=curr9800>>8;
		curr[1]=curr9800;
		IIC_Write_Arr(0,0x18,0x03,2,curr);
		curr9800=rtY.Out2*0.8;
		curr[0]=curr9800>>8;
		curr[1]=curr9800;
//		IIC_Write_Arr(6,0x18,0x03,2,curr);
//		IIC_Write_Arr(3,0x18,0x03,2,curr);
//		curr9800=rtY.Out3;
//		curr[0]=curr9800>>8;
//		curr[1]=curr9800;
		
					}
}

void TIM3_Init(u16 arr,u16 psc) 
{ 
	htim3.Instance=TIM3; 
	htim3.Init.Prescaler=psc; //分频系数 
	htim3.Init.CounterMode=TIM_COUNTERMODE_UP; //向上计数器 向上计数器 
	htim3.Init.Period=arr; //自动装载值 自动装载值 
	htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; //时钟分频因子 时钟分频因子 
	HAL_TIM_Base_Init(&htim3); //初始化定时器 初始化定时器 3 
	HAL_TIM_Base_Start_IT(&htim3);
}
/* USER CODE END 1 */
